/**
 * @file cr20_manual_jog_logger.cpp
 * @brief Passive 50 Hz data logger for manually-jogged drilling trials on
 * the CR20. Prints one human-readable line per sample to stdout; capture it
 * with `script` (see docs/drill-test-quickstart.md).
 *
 * STRICTLY READ-ONLY WITH RESPECT TO MOTION. This program connects, asks
 * the controller to stream its state, reads it, and writes it to a file.
 * It never powers the arm on or off, never changes the operate mode or the
 * motion-control mode, never calls fcInit()/fcStart(), and never commands
 * motion. You drive the arm from the pendant exactly as you normally
 * would; this just watches. The only controller-side writes are
 * setToolset() (selects which already-configured tool this SDK session
 * reports poses against - it does not create or modify a tool) and
 * startReceiveRobotState()/stopReceiveRobotState().
 *
 * WHERE THE DATA COMES FROM - AND WHY IT IS TWO SOURCES
 *
 * 1. The controller's UDP state stream (startReceiveRobotState). This is
 *    the pose / joint angle / joint torque source and it is controller-
 *    paced, so its timestamps are exact and do not drift with host load.
 *    Requested at 4 ms; every 5th frame is written, giving exactly 50 Hz.
 *    4 ms is chosen because the SDK only accepts 1/2/4/8 ms or 1 s, and 4
 *    is the only one of those that divides 20 ms evenly.
 *
 *    Note which fields are NOT in here: data_types.h marks tau_ext_base,
 *    tau_ext_stiff, and every "_c" (commanded) field as valid only once
 *    real-time control mode is open. Opening RT mode means taking control
 *    of the arm, which is the one thing this program must not do while
 *    somebody is jogging it. So the external-wrench estimate has to come
 *    from somewhere else, which is source 2.
 *
 * 2. ForceControl::getEndTorque(), polled over the non-RT request/response
 *    channel. This is the force estimate: the controller's own model-based
 *    external joint torques plus the Cartesian force/torque they resolve
 *    to, in the TOOL frame. Being an RPC it has unbounded latency, so it
 *    runs on its own thread and publishes its latest answer into a
 *    snapshot; the 50 Hz printer takes whatever is current and prints its
 *    age alongside it. That way a slow controller reply degrades the
 *    freshness of six numbers instead of stalling the whole log.
 *
 *    Every run probes getEndTorque() once before it starts and says
 *    plainly whether force data will be present, because the alternative
 *    is finding out from a column of NaNs after the concrete dust has
 *    settled.
 *
 * THE FORCE NUMBERS ARE ONLY AS GOOD AS THE TOOLSET
 *
 * The controller subtracts a model of the tool's own weight to get
 * "external" force. If the payload configured on the pendant does not
 * match what is actually bolted on, that error appears as a constant
 * offset on every force reading here. Take a bias reading with the bit in
 * free air at the start of each session and subtract it later - the run
 * prints the active tool's mass and CoG so you can tell what model was in
 * use.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;

namespace
{

constexpr const char* kRobotIp = "192.168.2.160";
constexpr const char* kLocalIp = "192.168.2.22";

/// Which already-configured tool/workobject this SDK session reports poses
/// against. A fresh session inherits the IDENTITY toolset, not whatever the
/// pendant is showing - so without this the logged pose is the FLANGE pose
/// and the force frame is the flange, not the bit. Name your real drill TCP
/// here once it exists on the controller; "g_tool_0"/"g_wobj_0" are the
/// global defaults and are identity. Leave empty to log against whatever
/// the session already has.
///
/// See cr20_toolset_query.cpp - it prints the difference without moving
/// anything, and is worth running once before a session.
constexpr const char* kToolName = "g_tool_0";
constexpr const char* kWorkobjectName = "g_wobj_0";

/// Controller state-stream interval. Allowed: 1/2/4/8 ms or 1 s.
constexpr auto kStreamInterval = std::chrono::milliseconds(4);

/// Write one row per this many streamed frames. 5 x 4 ms = 20 ms = 50 Hz.
constexpr int kFrameDecimation = 5;

constexpr double kStreamIntervalSec = 0.004;
constexpr double kSampleIntervalSec = kStreamIntervalSec * kFrameDecimation;

/// How long to wait for each streamed frame before calling the stream
/// stalled. Generous relative to the 4 ms interval - this is a fault
/// detector, not a pacing mechanism.
constexpr auto kStreamTimeout = std::chrono::milliseconds(500);

/// Target rate for the getEndTorque() polling thread. Matched to the log
/// rate so force columns are fresh; the achieved rate is measured and
/// reported, since the RPC may not keep up.
constexpr auto kForcePollInterval = std::chrono::milliseconds(20);

/// Disk backstop. At ~50 rows/s and ~250 bytes/row this is about 450 MB,
/// which is more than any single drilling trial should ever need. The run
/// stops cleanly and closes the file rather than filling the LattePanda.
constexpr int kMaxRunSeconds = 3600;

/// Set by SIGINT/SIGTERM. Ctrl+C is the normal way to end a run.
std::atomic<bool> g_stop_requested{false};

void requestStop(int /*signal*/)
{
    g_stop_requested = true;
}

/// Latest answer from the force-polling thread, with the time it was taken
/// so the writer can record how stale it is.
struct ForceSnapshot
{
    std::array<double, 6> joint_torque{};
    std::array<double, 6> external_torque{};
    std::array<double, 3> cart_torque{};
    std::array<double, 3> cart_force{};
    std::chrono::steady_clock::time_point taken_at{};
    bool valid = false;
};

std::mutex g_force_mutex;
ForceSnapshot g_force_snapshot;

std::string nowTimestampIso()
{
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S%z");
    return oss.str();
}

template <size_t N> std::string formatArray(const std::array<double, N>& values)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < N; ++i)
    {
        if (i > 0)
        {
            oss << ' ';
        }
        oss << values[i];
    }
    return oss.str();
}

/// Poll getEndTorque() until told to stop, publishing each answer. Runs on
/// its own thread so RPC latency cannot pace the 50 Hz writer.
void pollForceLoop(ForceControl_T<6>& force_control,
                   std::atomic<bool>& running,
                   std::atomic<long>& poll_count,
                   std::atomic<long>& poll_error_count)
{
    while (running)
    {
        const auto poll_started = std::chrono::steady_clock::now();

        ForceSnapshot snapshot;
        std::error_code ec;
        // FrameType::tool - the bit's own frame, so cart_force Z is force
        // along the drill axis rather than along some base axis.
        force_control.getEndTorque(FrameType::tool,
                                   snapshot.joint_torque,
                                   snapshot.external_torque,
                                   snapshot.cart_torque,
                                   snapshot.cart_force,
                                   ec);
        if (ec)
        {
            ++poll_error_count;
        }
        else
        {
            snapshot.taken_at = std::chrono::steady_clock::now();
            snapshot.valid = true;
            ++poll_count;
            std::lock_guard<std::mutex> lock(g_force_mutex);
            g_force_snapshot = snapshot;
        }

        const auto elapsed = std::chrono::steady_clock::now() - poll_started;
        if (elapsed < kForcePollInterval)
        {
            std::this_thread::sleep_for(kForcePollInterval - elapsed);
        }
    }
}

} // namespace

int main()
{
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    std::cout << "cr20_manual_jog_logger - run started " << nowTimestampIso() << std::endl;

    xMateRobot robot;
    std::error_code ec;
    try
    {
        robot.connectToRobot(kRobotIp, kLocalIp);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Connect to robot failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Connected to " << kRobotIp << " (local " << kLocalIp << ")." << std::endl;
    std::cout << "PASSIVE: the arm is not powered on, no mode is changed, nothing is commanded to move.\n"
              << "Jog from the pendant as normal." << std::endl;

    // Select the tool. Everything downstream - the logged pose, and the
    // frame getEndTorque() resolves force into - hangs off this.
    Toolset active_toolset = robot.toolset(ec);
    ec.clear();
    if (kToolName[0] != '\0')
    {
        const Toolset selected = robot.setToolset(kToolName, kWorkobjectName, ec);
        if (ec)
        {
            std::cerr << "WARNING: setToolset(" << kToolName << ", " << kWorkobjectName
                      << ") failed: " << ec.message() << "\n"
                      << "         Continuing with the session's inherited toolset. Pose and force\n"
                      << "         are then reported against THAT frame, not the drill TCP." << std::endl;
            ec.clear();
        }
        else
        {
            active_toolset = selected;
            std::cout << "setToolset(" << kToolName << ", " << kWorkobjectName << ") OK." << std::endl;
        }
    }
    const std::array<double, 3> tool_trans{
        active_toolset.end.trans[0], active_toolset.end.trans[1], active_toolset.end.trans[2]};
    const std::array<double, 3> tool_rpy{
        active_toolset.end.rpy[0], active_toolset.end.rpy[1], active_toolset.end.rpy[2]};
    const std::array<double, 3> tool_cog{
        active_toolset.load.cog[0], active_toolset.load.cog[1], active_toolset.load.cog[2]};
    std::cout << "Active tool: mass " << active_toolset.load.mass << " kg, TCP offset [" << formatArray(tool_trans)
              << "] m.\n"
              << "  A payload that does not match what is actually fitted biases every force reading." << std::endl;

    auto force_control = robot.forceControl();

    // Probe once, before the run, so "will there be force data?" is
    // answered now rather than by a column of NaNs afterwards. This is a
    // query only - no fcInit(), which would put the controller into force
    // control while somebody is holding the pendant.
    bool force_available = false;
    {
        std::array<double, 6> probe_joint_torque{}, probe_external_torque{};
        std::array<double, 3> probe_cart_torque{}, probe_cart_force{};
        std::error_code probe_ec;
        force_control.getEndTorque(FrameType::tool,
                                   probe_joint_torque,
                                   probe_external_torque,
                                   probe_cart_torque,
                                   probe_cart_force,
                                   probe_ec);
        if (probe_ec)
        {
            std::cerr << "WARNING: getEndTorque() probe failed: " << probe_ec.message() << "\n"
                      << "         Force/external-torque columns will be empty. Pose, joint angles,\n"
                      << "         joint velocity and joint torque are unaffected and still logged." << std::endl;
        }
        else
        {
            force_available = true;
            std::cout << "getEndTorque() probe OK. Bias now (bit in free air): force [" << formatArray(probe_cart_force)
                      << "] N, torque [" << formatArray(probe_cart_torque) << "] Nm.\n"
                      << "  Write that down - it is the offset to subtract from this session's numbers."
                      << std::endl;
        }
    }

    // Measured fields only. data_types.h marks the commanded ("_c") fields
    // and both tau_ext variants as valid only under RT control mode, which
    // this program deliberately never opens.
    using namespace RtSupportedFields;
    const std::vector<std::string> fields{tcpPoseAbc_m, jointPos_m, tau_m};
    try
    {
        robot.startReceiveRobotState(kStreamInterval, fields);
    }
    catch (const std::exception& e)
    {
        std::cerr << "startReceiveRobotState failed: " << e.what() << "\n"
                  << "Nothing was changed on the controller; it is safe to retry." << std::endl;
        return 1;
    }
    std::cout << "State stream started at " << kStreamInterval.count() << " ms." << std::endl;

    // The received-frame queue is not self-clearing, so a fresh run would
    // otherwise start by writing however many stale frames accumulated
    // between the stream starting and the loop below being reached.
    while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()) != 0)
    {
    }

    std::atomic<bool> force_thread_running{force_available};
    std::atomic<long> force_poll_count{0};
    std::atomic<long> force_poll_error_count{0};
    std::thread force_thread;
    if (force_available)
    {
        force_thread = std::thread(pollForceLoop,
                                   std::ref(force_control),
                                   std::ref(force_thread_running),
                                   std::ref(force_poll_count),
                                   std::ref(force_poll_error_count));
    }

    // Printed once, before the stream of numbers: which tool model produced
    // the force figures below is not recoverable from the figures themselves,
    // and it is the first thing you will want when reading the transcript back.
    std::cout << "\n=== Run configuration ===\n"
              << "  robot:        " << kRobotIp << '\n'
              << "  tool:         " << (kToolName[0] != '\0' ? kToolName : "(inherited)") << " / "
              << (kToolName[0] != '\0' ? kWorkobjectName : "(inherited)") << '\n'
              << "  tool offset:  [" << formatArray(tool_trans) << "] m, rpy [" << formatArray(tool_rpy) << "] rad\n"
              << "  tool load:    " << active_toolset.load.mass << " kg, cog [" << formatArray(tool_cog) << "] m\n"
              << "  sample rate:  " << (1.0 / kSampleIntervalSec) << " Hz (" << kStreamInterval.count()
              << " ms stream / every " << kFrameDecimation << "th frame)\n"
              << "  force:        " << (force_available ? "available, tool frame" : "NOT AVAILABLE") << '\n'
              << "  pose frame:   endInRef (tool relative to workobject)\n"
              << "\n=== Column legend ===\n"
              << "  t       controller time [s] - frames x " << kStreamIntervalSec
              << " s, exact and unaffected by host load\n"
              << "  dt      wall-clock gap since the previous line [ms]. Should track 20 ms; if it\n"
              << "          does not, the host fell behind and drained a backlog.\n"
              << "  tcp     TCP position [mm] and orientation [deg] in the workobject frame\n"
              << "  q       joint angles [deg]\n"
              << "  tau     measured joint torque [Nm]\n"
              << "  F / T   external force [N] / torque [Nm] at the TCP, in the TOOL frame - the\n"
              << "          controller's own estimate, with the tool's modelled weight removed\n"
              << "  age     how stale the F/T reading is [ms]; it is polled on a separate thread\n";

    std::cout << "\nRecording at " << (1.0 / kSampleIntervalSec) << " Hz. Press Ctrl+C to stop.\n" << std::endl;

    std::array<double, 6> tcp_pose{};
    std::array<double, 6> joint_pos{};
    std::array<double, 6> joint_torque{};

    long frame_index = 0;
    long rows_written = 0;
    long stream_stall_count = 0;
    const auto run_started = std::chrono::steady_clock::now();
    auto previous_row_at = run_started;
    std::string stop_reason = "ctrl-c";

    while (!g_stop_requested)
    {
        if (std::chrono::steady_clock::now() - run_started >= std::chrono::seconds(kMaxRunSeconds))
        {
            stop_reason = "max-run-seconds reached";
            break;
        }

        // Blocks until the controller's next frame arrives, so the loop is
        // paced by the stream rather than by a sleep.
        if (robot.updateRobotState(kStreamTimeout) == 0)
        {
            ++stream_stall_count;
            std::cerr << "WARNING: no state frame within " << kStreamTimeout.count() << " ms (stall #"
                      << stream_stall_count << ")." << std::endl;
            continue;
        }
        ++frame_index;
        if (frame_index % kFrameDecimation != 0)
        {
            continue;
        }

        // getStateData() returns -1 for a field that was never requested or
        // whose type does not match, and leaves the destination untouched -
        // which would quietly fill a column with the previous row's value
        // forever. Checked once, on the first row, so a misconfigured field
        // list is caught before the hole is drilled rather than after.
        const int read_status = robot.getStateData(tcpPoseAbc_m, tcp_pose) +
                                robot.getStateData(jointPos_m, joint_pos) +
                                robot.getStateData(tau_m, joint_torque);
        if (read_status != 0 && rows_written == 0)
        {
            std::cerr << "WARNING: one or more state fields could not be read (getStateData sum "
                      << read_status << ").\n"
                      << "         Those columns will repeat a stale value. Check the field list." << std::endl;
        }

        ForceSnapshot force;
        {
            std::lock_guard<std::mutex> lock(g_force_mutex);
            force = g_force_snapshot;
        }

        const auto row_at = std::chrono::steady_clock::now();
        const double host_dt_ms =
            std::chrono::duration<double, std::milli>(row_at - previous_row_at).count();
        previous_row_at = row_at;

        // One line per sample. Fixed widths so the columns stay aligned as
        // values gain and lose digits - at 50 lines a second an unaligned
        // column is unreadable, and reading it live is the whole point of
        // printing rather than writing a file.
        std::ostringstream line;
        line << std::fixed << std::setprecision(2);
        line << "t=" << std::setw(8) << (frame_index * kStreamIntervalSec) << "  dt=" << std::setw(6) << host_dt_ms;

        line << "  tcp[mm]";
        for (int axis = 0; axis < 3; ++axis)
        {
            line << ' ' << std::setw(8) << (tcp_pose[axis] * 1000.0);
        }
        line << " [deg]";
        for (int axis = 3; axis < 6; ++axis)
        {
            line << ' ' << std::setw(7) << (tcp_pose[axis] * 180.0 / M_PI);
        }

        line << "  q[deg]";
        for (const double value : joint_pos)
        {
            line << ' ' << std::setw(8) << (value * 180.0 / M_PI);
        }

        line << "  tau[Nm]";
        for (const double value : joint_torque)
        {
            line << ' ' << std::setw(8) << value;
        }

        if (force.valid)
        {
            line << "  F[N]";
            for (const double value : force.cart_force)
            {
                line << ' ' << std::setw(8) << value;
            }
            line << "  T[Nm]";
            for (const double value : force.cart_torque)
            {
                line << ' ' << std::setw(7) << value;
            }
            line << "  age=" << std::setw(6)
                 << std::chrono::duration<double, std::milli>(row_at - force.taken_at).count();
        }
        else
        {
            // Said outright rather than printed as zeros: a zero force is a
            // real, meaningful reading and must not be confused with a
            // missing one.
            line << "  F/T unavailable";
        }
        std::cout << line.str() << '\n';

        ++rows_written;
    }

    force_thread_running = false;
    if (force_thread.joinable())
    {
        force_thread.join();
    }
    robot.stopReceiveRobotState();

    const double run_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_started).count();

    std::cout << "\n=== Run summary ===\n"
              << "  stop reason:       " << stop_reason << '\n'
              << "  duration:          " << std::setprecision(1) << run_seconds << " s\n"
              << "  rows written:      " << rows_written << " (" << std::setprecision(2)
              << (run_seconds > 0.0 ? rows_written / run_seconds : 0.0) << " Hz achieved)\n"
              << "  frames received:   " << frame_index << '\n'
              << "  stream stalls:     " << stream_stall_count << '\n'
              << "  force polls ok:    " << force_poll_count.load() << " (" << std::setprecision(2)
              << (run_seconds > 0.0 ? force_poll_count.load() / run_seconds : 0.0) << " Hz achieved)\n"
              << "  force poll errors: " << force_poll_error_count.load() << std::endl;

    return 0;
}
