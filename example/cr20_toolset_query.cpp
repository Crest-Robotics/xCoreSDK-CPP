/**
 * @file cr20_toolset_query.cpp
 * @brief Read-only dump of the active SDK toolset and the current arm pose,
 * for checking what TCP cr20_drill/drill_test.cpp will actually push
 * along before running it.
 *
 * STRICTLY READ-ONLY. This program connects, asks questions, and
 * disconnects. It never powers the arm on, never changes the operate mode
 * or motion-control mode, and never commands motion. setToolset() is the
 * one thing it writes, and it only selects which already-configured tool
 * the SDK session uses - it does not create or modify a tool.
 *
 * WHY THIS EXISTS
 *
 * cr20_drill/drill_test.cpp advances along a TOOL axis, taken as a
 * column of the live TCP pose's rotation block. Which direction that
 * actually is depends entirely on the active toolset, and the SDK's
 * toolset is NOT the pendant's - robot.toolset() reports whatever the SDK
 * session currently has, which is the identity default until setToolset()
 * is called. That difference is invisible when the bit happens to point
 * straight down, because a downward tool +Z and base -Z are numerically
 * the same vector. This program makes it visible without moving anything.
 *
 * The number that settles it is the angle between flange Z and tool Z,
 * printed at the bottom. Zero means the tool frame is not rotated relative
 * to the flange, so "tool Z" and "flange Z" are the same direction and an
 * identity toolset would be indistinguishable from a correctly calibrated
 * coaxial one.
 */

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <system_error>

#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

namespace
{

constexpr const char* kToolName = "g_tool_0";
constexpr const char* kWorkobjectName = "g_wobj_0";

constexpr const char* kRobotIp = "192.168.2.160";
constexpr const char* kLocalIp = "192.168.2.22";

template <size_t N> std::string formatArray(const std::array<double, N>& values)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "[";
    for (size_t i = 0; i < N; ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

void printFrame(const std::string& label, const Frame& frame)
{
    const std::array<double, 3> trans{frame.trans[0], frame.trans[1], frame.trans[2]};
    const std::array<double, 3> rpy{frame.rpy[0], frame.rpy[1], frame.rpy[2]};
    const std::array<double, 3> rpy_deg{
        rpy[0] * 180.0 / M_PI, rpy[1] * 180.0 / M_PI, rpy[2] * 180.0 / M_PI};
    std::cout << "  " << label << "\n"
              << "    trans (m):   " << formatArray(trans) << "\n"
              << "    rpy (rad):   " << formatArray(rpy) << "\n"
              << "    rpy (deg):   " << formatArray(rpy_deg) << std::endl;
}

void printToolset(const std::string& label, const Toolset& toolset)
{
    const std::array<double, 3> cog{toolset.load.cog[0], toolset.load.cog[1], toolset.load.cog[2]};
    const std::array<double, 3> inertia{
        toolset.load.inertia[0], toolset.load.inertia[1], toolset.load.inertia[2]};
    std::cout << "\n" << label << "\n";
    printFrame("end (TCP relative to FLANGE) - this is the TCP:", toolset.end);
    printFrame("ref (reference frame relative to WORLD):", toolset.ref);
    std::cout << "  load\n"
              << "    mass (kg):   " << toolset.load.mass << "\n"
              << "    cog (m):     " << formatArray(cog) << "\n"
              << "    inertia:     " << formatArray(inertia) << std::endl;

    const double translation_magnitude =
        std::abs(toolset.end.trans[0]) + std::abs(toolset.end.trans[1]) + std::abs(toolset.end.trans[2]);
    const double rotation_magnitude =
        std::abs(toolset.end.rpy[0]) + std::abs(toolset.end.rpy[1]) + std::abs(toolset.end.rpy[2]);
    if (translation_magnitude + rotation_magnitude < 1e-9)
    {
        std::cout << "  -> end is IDENTITY: no TCP offset and no TCP rotation are configured\n"
                  << "     for this tool. tcpPose_m would report the FLANGE pose." << std::endl;
    }
    else if (rotation_magnitude < 1e-9)
    {
        std::cout << "  -> end has an offset but NO rotation: the TCP is a translated flange\n"
                  << "     frame, so tool axes are parallel to flange axes." << std::endl;
    }
}

/// Column N of a row-major 4x4's rotation block - i.e. that frame's axis N
/// expressed in the parent frame. Hand indexing is fine here because this is
/// a read-only query that never commands motion; cr20_drill/drill_test.cpp
/// deliberately has no equivalent, and routes every commanded pose through
/// drill_frames.hpp's advanceAlongBit() instead.
std::array<double, 3> axisFromPose(const std::array<double, 16>& pose, int column)
{
    return {pose[column], pose[column + 4], pose[column + 8]};
}

std::string describeDirectionInBase(const std::array<double, 3>& axis)
{
    const double angle_from_down_deg = std::acos(std::max(-1.0, std::min(1.0, -axis[2]))) * 180.0 / M_PI;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    if (angle_from_down_deg < 20.0)
    {
        oss << "points DOWN (" << angle_from_down_deg << " deg off vertical)";
    }
    else if (angle_from_down_deg > 160.0)
    {
        oss << "points UP (" << (180.0 - angle_from_down_deg) << " deg off vertical)";
    }
    else
    {
        oss << "points at " << angle_from_down_deg << " deg from straight down";
    }
    return oss.str();
}

} // namespace

int main()
{
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
    std::cout << "READ-ONLY: the arm is not powered on and nothing is commanded to move." << std::endl;

    // Before: whatever this fresh SDK session inherited. This is the state
    // cr20_drill/drill_test.cpp would run in if its setToolset() call
    // were removed, and the thing its identity-warning is about.
    const Toolset inherited = robot.toolset(ec);
    if (ec)
    {
        std::cerr << "toolset() query failed: " << ec.message() << std::endl;
        return 1;
    }
    printToolset("=== Toolset INHERITED by a fresh SDK session (before setToolset) ===", inherited);

    // After: what selecting the tool by name actually resolves to. The
    // by-name overload returns the resolved toolset directly.
    const Toolset selected = robot.setToolset(kToolName, kWorkobjectName, ec);
    if (ec)
    {
        std::cerr << "\nsetToolset(" << kToolName << ", " << kWorkobjectName << ") failed: " << ec.message()
                  << "\nThe tool must already exist on the controller under that exact name." << std::endl;
        return 1;
    }
    printToolset(std::string("=== Toolset after setToolset(\"") + kToolName + "\", \"" + kWorkobjectName +
                     "\") - what the drill test would use ===",
                 selected);

    // Current pose, both ways. flangeInBase is independent of the toolset;
    // endInRef moves with it, so the two together show what the toolset is
    // actually doing.
    const std::array<double, 6> flange_in_base = robot.posture(CoordinateType::flangeInBase, ec);
    if (ec)
    {
        std::cerr << "posture(flangeInBase) failed: " << ec.message() << std::endl;
        return 1;
    }
    const std::array<double, 6> end_in_ref = robot.posture(CoordinateType::endInRef, ec);
    if (ec)
    {
        std::cerr << "posture(endInRef) failed: " << ec.message() << std::endl;
        return 1;
    }

    std::array<double, 16> flange_pose{};
    std::array<double, 16> end_pose{};
    Utils::postureToTransArray(flange_in_base, flange_pose);
    Utils::postureToTransArray(end_in_ref, end_pose);

    const std::array<double, 3> flange_z = axisFromPose(flange_pose, 2);
    const std::array<double, 3> tool_z = axisFromPose(end_pose, 2);
    const std::array<double, 3> tool_x = axisFromPose(end_pose, 0);
    const std::array<double, 3> tool_y = axisFromPose(end_pose, 1);

    std::cout << "\n=== Current pose (arm wherever it is now) ===\n"
              << std::fixed << std::setprecision(6)
              << "  flangeInBase [x,y,z,rx,ry,rz]: " << formatArray(flange_in_base) << "\n"
              << "  endInRef     [x,y,z,rx,ry,rz]: " << formatArray(end_in_ref) << "\n"
              << "\n  TCP orientation in base (columns = tool X, Y, Z axes):\n"
              << "    tool X: " << formatArray(tool_x) << "\n"
              << "    tool Y: " << formatArray(tool_y) << "\n"
              << "    tool Z: " << formatArray(tool_z) << "  <- the drill test's push axis\n"
              << "      -> " << describeDirectionInBase(tool_z) << std::endl;

    const double dot = flange_z[0] * tool_z[0] + flange_z[1] * tool_z[1] + flange_z[2] * tool_z[2];
    const double tool_vs_flange_deg = std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / M_PI;
    std::cout << "\n=== The number that settles it ===\n"
              << "  flange Z in base: " << formatArray(flange_z) << "\n"
              << "  tool   Z in base: " << formatArray(tool_z) << "\n"
              << "  angle between them: " << std::setprecision(3) << tool_vs_flange_deg << " deg" << std::endl;
    if (tool_vs_flange_deg < 0.01)
    {
        std::cout << "  -> Tool Z is COAXIAL with flange Z. The tool frame is not rotated, so the\n"
                  << "     drill test's push direction is the same whether or not a TCP is active.\n"
                  << "     A TCP offset still matters for where the pose is reported FROM, but not\n"
                  << "     for which way the feed goes." << std::endl;
    }
    else
    {
        std::cout << "  -> Tool Z is TILTED from flange Z by the angle above. The active toolset\n"
                  << "     therefore changes the push direction, and running the drill test with\n"
                  << "     the wrong tool would feed " << std::setprecision(3) << tool_vs_flange_deg
                  << " deg off the bit axis." << std::endl;
    }

    std::cout << "\nDone. Nothing was powered on and nothing moved." << std::endl;
    return 0;
}
