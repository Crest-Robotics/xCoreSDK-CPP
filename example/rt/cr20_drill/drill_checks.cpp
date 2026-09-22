/**
 * @file drill_checks.cpp
 * @brief Off-robot verification for drill_frames.hpp and drill_cycle.hpp.
 *
 * Links against `eigen` alone - no SDK, no controller, no threads. That link
 * line is the point: the two headers under test are the pieces where a
 * mistake moves a spinning bit somewhere unintended, and they are also pure
 * functions of numbers, so they can be proven wrong on a laptop. If either
 * header ever acquires an SDK dependency, this target stops building, which
 * is the signal that the split has been broken.
 *
 * Run it before every drilling session, and after touching either header:
 *
 *     ./drill_checks     ->  "ALL CHECKS PASSED", exit 0
 *
 * It proves control-law and geometry properties only. It says nothing about
 * whether the forces are right for the material - that is what the on-robot
 * commissioning ladder in the quickstart is for.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "drill_cycle.hpp"
#include "drill_frames.hpp"

using cr20_drill::AbortReason;
using cr20_drill::Command;
using cr20_drill::DrillConfig;
using cr20_drill::DrillCycle;
using cr20_drill::Feedback;
using cr20_drill::Phase;
using cr20_drill::Pose;
using cr20_drill::PoseMatrix;
using cr20_drill::ToolDelta;

namespace
{

int g_checks = 0;
int g_failures = 0;

void check(bool condition, const std::string& what)
{
    ++g_checks;
    if (!condition)
    {
        ++g_failures;
        std::printf("  FAIL  %s\n", what.c_str());
    }
}

void checkNear(double actual, double expected, double tolerance, const std::string& what)
{
    ++g_checks;
    if (!(std::abs(actual - expected) <= tolerance))
    {
        ++g_failures;
        std::printf("  FAIL  %s: got %.9f, expected %.9f (tol %.2e)\n", what.c_str(), actual, expected, tolerance);
    }
}

void section(const char* name)
{
    std::printf("\n%s\n", name);
}

// --------------------------------------------------------------------------
// Pose construction helpers. Deliberately built with Eigen directly rather
// than with anything from drill_frames.hpp, so the tests do not validate the
// header against itself.
// --------------------------------------------------------------------------

Pose poseFrom(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
    PoseMatrix matrix = PoseMatrix::Identity();
    matrix.topLeftCorner<3, 3>() = rotation;
    matrix.topRightCorner<3, 1>() = translation;

    Pose pose{};
    Eigen::Map<PoseMatrix>(pose.data()) = matrix;
    return pose;
}

/// ZYX intrinsic, the convention the xCore SDK uses for postures. Only needed
/// to rebuild the real logged baseline; the header itself never touches it.
Eigen::Matrix3d rotationFromRpy(double rx, double ry, double rz)
{
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

double deg(double degrees)
{
    return degrees * M_PI / 180.0;
}

// --------------------------------------------------------------------------
// 1. Frame math
// --------------------------------------------------------------------------

void checkFrames()
{
    section("Frame math");

    std::mt19937 rng{12345};
    std::uniform_real_distribution<double> angle{-M_PI, M_PI};
    std::uniform_real_distribution<double> offset{-1.0, 1.0};
    std::uniform_real_distribution<double> depth{-0.1, 0.1};

    // (1) Zero advance is the identity. This is what guarantees the first
    // pose the control loop commands is bit-for-bit the pose the arm is
    // already in, without a runtime special case.
    double worst_zero = 0.0;
    for (int i = 0; i < 200; ++i)
    {
        const Pose baseline =
            poseFrom(rotationFromRpy(angle(rng), angle(rng), angle(rng)),
                     Eigen::Vector3d{offset(rng), offset(rng), offset(rng)});
        const Pose advanced = cr20_drill::advanceAlongBit(baseline, 0.0);
        for (std::size_t k = 0; k < 16; ++k)
        {
            worst_zero = std::max(worst_zero, std::abs(advanced[k] - baseline[k]));
        }
    }
    check(worst_zero < 1e-12, "advanceAlongBit(B, 0) == B over 200 random poses");

    // (2) advanceAlongBit and toolDeltaBetween are inverses, and the advance
    // is purely along the bit axis with no lateral or rotational component.
    double worst_round_trip = 0.0;
    for (int i = 0; i < 200; ++i)
    {
        const Pose baseline =
            poseFrom(rotationFromRpy(angle(rng), angle(rng), angle(rng)),
                     Eigen::Vector3d{offset(rng), offset(rng), offset(rng)});
        const double d = depth(rng);
        const ToolDelta delta = cr20_drill::toolDeltaBetween(baseline, cr20_drill::advanceAlongBit(baseline, d));

        worst_round_trip = std::max({worst_round_trip, std::abs(delta.z_m - d), std::abs(delta.x_m),
                                     std::abs(delta.y_m), std::abs(delta.rx_rad), std::abs(delta.ry_rad),
                                     std::abs(delta.rz_rad)});
    }
    check(worst_round_trip < 1e-12, "toolDeltaBetween(B, advanceAlongBit(B, d)) == {0,0,d,0,0,0}");

    // (3) Identity baseline. Pins the row-major index convention: if the
    // Eigen map were reading the array as column-major, this lands in X.
    {
        const Pose baseline = poseFrom(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        const Pose advanced = cr20_drill::advanceAlongBit(baseline, 0.010);
        checkNear(advanced[3], 0.000, 1e-12, "identity baseline, advance 10mm -> base x");
        checkNear(advanced[7], 0.000, 1e-12, "identity baseline, advance 10mm -> base y");
        checkNear(advanced[11], 0.010, 1e-12, "identity baseline, advance 10mm -> base z");
    }

    // (4) Tool pointing at the floor. A sign flip anywhere in the
    // composition sends base z UP instead of down.
    {
        const Pose baseline = poseFrom(rotationFromRpy(M_PI, 0.0, 0.0), Eigen::Vector3d{0.3, 0.4, 0.5});
        const Pose advanced = cr20_drill::advanceAlongBit(baseline, 0.010);
        checkNear(advanced[11], 0.490, 1e-12, "tool Z pointing down, advance 10mm -> base z DECREASES");
    }

    // (5) The direct regression test for the bug this rewrite exists to
    // prevent. A baseline tilted 30 degrees about base Y must advance along
    // the TOOL's Z, which is 30 degrees off base Z. An implementation that
    // translated along a base axis instead would give (0, 0, 10) mm.
    {
        const Pose baseline = poseFrom(rotationFromRpy(0.0, deg(30.0), 0.0), Eigen::Vector3d::Zero());
        const Pose advanced = cr20_drill::advanceAlongBit(baseline, 0.010);
        checkNear(advanced[3] * 1000.0, 5.000, 1e-6, "30deg tilt, advance 10mm -> base x (mm)");
        checkNear(advanced[7] * 1000.0, 0.000, 1e-6, "30deg tilt, advance 10mm -> base y (mm)");
        checkNear(advanced[11] * 1000.0, 8.660, 1e-3, "30deg tilt, advance 10mm -> base z (mm)");
    }

    // (6) The real logged approach pose from drill_log.csv. First confirm the
    // bit axis comes out where the logged run measured it, then confirm a
    // 10 mm advance lands where the plan predicts.
    {
        const Pose baseline = poseFrom(rotationFromRpy(deg(-106.35), deg(-12.50), deg(-1.21)),
                                       Eigen::Vector3d{-0.2732, 0.5768, 0.4843});

        const auto axis = cr20_drill::bitAxisInParentForDisplay(baseline);
        checkNear(axis[0], 0.081, 1e-3, "logged baseline bit axis, base x");
        checkNear(axis[1], 0.958, 1e-3, "logged baseline bit axis, base y");
        checkNear(axis[2], -0.275, 1e-3, "logged baseline bit axis, base z");

        const Pose advanced = cr20_drill::advanceAlongBit(baseline, 0.010);
        checkNear(advanced[3] * 1000.0, -272.4, 0.1, "logged baseline, advance 10mm -> base x (mm)");
        checkNear(advanced[7] * 1000.0, 586.4, 0.1, "logged baseline, advance 10mm -> base y (mm)");
        checkNear(advanced[11] * 1000.0, 481.6, 0.1, "logged baseline, advance 10mm -> base z (mm)");

        // The orientation must survive untouched, or the hole curves.
        const ToolDelta delta = cr20_drill::toolDeltaBetween(baseline, advanced);
        check(std::abs(delta.rx_rad) + std::abs(delta.ry_rad) + std::abs(delta.rz_rad) < 1e-12,
              "advancing does not rotate the tool");
    }

    // bitAxisAngleRad on a constructed pair.
    {
        const Pose a = poseFrom(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        const Pose b = poseFrom(rotationFromRpy(0.0, deg(5.0), 0.0), Eigen::Vector3d::Zero());
        checkNear(cr20_drill::bitAxisAngleRad(a, b) * 180.0 / M_PI, 5.0, 1e-9, "bitAxisAngleRad on a 5deg pair");
    }
}

// --------------------------------------------------------------------------
// 2. A crude plant, enough to exercise the state machine.
//
// Not a force model - it makes no claim about what the material really does.
// It exists so that the phase transitions, the aborts and the limiter can be
// driven through a whole run. Force predictions are the on-robot ladder's
// job.
// --------------------------------------------------------------------------

struct Plant
{
    double gap_m = 0.0015;              ///< standoff before the bit touches
    double cut_gain_m_per_s_per_n = 1.7e-5; ///< ~1.5 mm/s at 100 N
    double cut_threshold_n = 10.0;      ///< below this it rubs rather than cuts
    double breakthrough_m = 1.0;        ///< depth past which resistance vanishes
    /// How fast the tool can follow the command in free air. Kept below the
    /// depth-runaway abort (25 mm/s): the real arm managed 1.6-8 mm/s, and a
    /// plant that snaps to the command faster than that trips the runaway
    /// detector on its own and never exercises the stroke at all.
    double free_rate_m_per_s = 0.008;
    double push_stiffness_n_per_m = 3000.0;
    /// Static friction at the TCP, split by direction because the real arm
    /// is: 8.3 N to break away going forwards, 19.7 N holding it back from
    /// the baseline on the way home (both measured 2026-09-22). That
    /// asymmetry matters - a single figure either deadlocks the outward
    /// stroke, since the scheduled spotting lead is only 12 N, or is too weak
    /// to park the tool short on the way back. With both set, the tool parks
    /// at reverse_friction/stiffness from the commanded point, which is
    /// exactly the condition the retract overshoot exists to beat.
    double static_friction_n = 0.0;
    double reverse_friction_n = 0.0;

    double depth_m = 0.0;
    double force_n = 0.0;

    void step(double commanded_depth_m, double dt_s)
    {
        const double lead = commanded_depth_m - depth_m;

        if (depth_m < gap_m || depth_m > breakthrough_m)
        {
            // Nothing to push against, so the tool chases the command - but
            // only once the spring beats stiction.
            force_n = 0.0;
            const double friction = lead >= 0.0 ? static_friction_n : reverse_friction_n;
            if (std::abs(lead) * push_stiffness_n_per_m < friction)
            {
                return;
            }
            depth_m += std::clamp(lead, -free_rate_m_per_s * dt_s, free_rate_m_per_s * dt_s);
            return;
        }

        force_n = std::max(0.0, lead) * push_stiffness_n_per_m;

        // A negative lead means the commanded point is BEHIND the tool, i.e.
        // it is being pulled back out of the hole. The bit does not have to
        // cut to come out, so it simply follows.
        if (lead < 0.0)
        {
            depth_m += std::max(lead, -free_rate_m_per_s * dt_s);
            return;
        }
        depth_m += cut_gain_m_per_s_per_n * std::max(0.0, force_n - cut_threshold_n) * dt_s;
    }
};

struct RunResult
{
    Phase final_phase = Phase::kSettle;
    AbortReason abort = AbortReason::kNone;
    double final_commanded_m = 0.0;
    double final_depth_m = 0.0;
    double min_commanded_m = 0.0;
    double max_depth_m = 0.0;
    double max_hole_depth_m = 0.0;
    double contact_datum_m = 0.0;
    bool contact_found = false;
    double max_lead_m = 0.0;
    double first_advance_step_m = 0.0;
    int cycles = 0;
    std::vector<Phase> phase_sequence;
    bool completed = false;
};

/// Drives a cycle against a plant until it finishes or the budget runs out.
/// `inject` is called each cycle so a test can force an abort at a chosen
/// phase without reaching into the class.
RunResult run(DrillConfig config, Plant plant, int max_cycles = 400000,
              const std::function<void(Feedback&, Phase)>& inject = nullptr)
{
    DrillCycle cycle{config};
    RunResult result{};

    const double dt = 0.001;
    double previous_depth = 0.0;
    double previous_commanded = 0.0;
    bool seen_first_advance = false;

    for (int i = 0; i < max_cycles; ++i)
    {
        Feedback fb{};
        fb.depth_m = plant.depth_m;
        fb.axial_force_n = plant.force_n;
        fb.depth_rate_m_per_s = (plant.depth_m - previous_depth) / dt;
        fb.dt_s = dt;
        previous_depth = plant.depth_m;

        if (inject)
        {
            inject(fb, cycle.phase());
        }

        const Command command = cycle.step(fb);

        if (result.phase_sequence.empty() || result.phase_sequence.back() != command.phase)
        {
            result.phase_sequence.push_back(command.phase);
        }

        if (!seen_first_advance && command.depth_m > 0.0)
        {
            result.first_advance_step_m = command.depth_m - previous_commanded;
            seen_first_advance = true;
        }
        previous_commanded = command.depth_m;

        result.max_lead_m = std::max(result.max_lead_m, command.lead_m);
        result.max_depth_m = std::max(result.max_depth_m, plant.depth_m);
        result.max_hole_depth_m = std::max(result.max_hole_depth_m, cycle.holeDepth(plant.depth_m));
        result.final_commanded_m = command.depth_m;
        result.min_commanded_m = std::min(result.min_commanded_m, command.depth_m);
        result.final_phase = command.phase;
        result.cycles = i + 1;

        if (command.finished)
        {
            result.completed = true;
            break;
        }

        plant.step(command.depth_m, dt);
    }

    result.final_depth_m = plant.depth_m;
    result.abort = cycle.abortReason();
    result.contact_datum_m = cycle.contactDatum();
    result.contact_found = cycle.contactFound();
    return result;
}

int phaseEntryCount(const std::vector<Phase>& sequence, Phase phase)
{
    int count = 0;
    for (const Phase p : sequence)
    {
        if (p == phase)
        {
            ++count;
        }
    }
    return count;
}

// --------------------------------------------------------------------------
// 3. State machine
// --------------------------------------------------------------------------

void checkStateMachine()
{
    section("State machine");

    const DrillConfig nominal{};

    // Nominal run: reaches depth, dwells, retracts, finishes at zero.
    {
        const RunResult r = run(nominal, Plant{});
        check(r.completed, "nominal run finishes");
        check(r.final_phase == Phase::kDone, "nominal run ends in kDone");
        check(r.abort == AbortReason::kNone, "nominal run does not abort");
        checkNear(r.final_commanded_m, 0.0, 1e-6, "nominal run returns the command to the baseline");
        check(r.max_hole_depth_m >= nominal.hole_depth_m, "nominal run reaches the target HOLE depth");

        // Every regime is visited, once each.
        for (const Phase p :
             {Phase::kSettle, Phase::kSpotting, Phase::kEntry, Phase::kDrilling, Phase::kSlowFinish, Phase::kDwell,
              Phase::kRetract, Phase::kDone})
        {
            check(phaseEntryCount(r.phase_sequence, p) == 1,
                  std::string{"nominal run enters "} + cr20_drill::phaseName(p) + " exactly once");
        }

        // The lead cap is the force ceiling and the lunge bound at once.
        check(r.max_lead_m <= nominal.lead_max_m + 1e-9, "lead never exceeds the cap");

        // Ramp from rest: the first advancing cycle moves one cycle of feed
        // rate, not a step.
        check(r.first_advance_step_m <= nominal.spotting_feed_m_per_s * 0.001 + 1e-12,
              "first advancing cycle steps by at most one cycle of feed rate");
    }

    // The hole is datumed from CONTACT, not from the pose the loop started
    // in. The operator leaves an eyeballed standoff, so measuring from the
    // start pose would silently take that gap out of the hole. Raw travel
    // must therefore end up a whole standoff deeper than the hole target.
    for (const double gap_mm : {0.5, 1.0, 2.0, 3.0})
    {
        Plant plant{};
        plant.gap_m = gap_mm / 1000.0;

        const RunResult r = run(nominal, plant);
        const std::string tag = " (standoff " + std::to_string(static_cast<int>(gap_mm * 10)) + "/10 mm)";

        check(r.contact_found, "contact datum is found" + tag);
        checkNear(r.contact_datum_m * 1000.0, gap_mm, 0.2, "contact datum lands at the standoff" + tag);
        check(r.max_hole_depth_m >= nominal.hole_depth_m, "hole reaches its target from the datum" + tag);
        checkNear((r.max_depth_m - r.max_hole_depth_m) * 1000.0, gap_mm, 0.2,
                  "raw travel exceeds hole depth by exactly the standoff" + tag);
    }

    // Determinism: two cycles driven identically agree. This is the
    // observable consequence of evaluate() being pure.
    {
        const RunResult a = run(nominal, Plant{});
        const RunResult b = run(nominal, Plant{});
        check(a.cycles == b.cycles && a.final_phase == b.final_phase && a.phase_sequence == b.phase_sequence,
              "identical inputs produce identical runs");
    }

    // Monotonic regime advance. Depth is driven backwards and forwards
    // across every boundary; each phase must still be entered exactly once.
    {
        // require_contact off so the datum stays at zero and hole depth is
        // exactly the depth fed in - this test is about regime ordering, not
        // about where the hole is datumed from.
        DrillConfig no_datum = nominal;
        no_datum.require_contact = false;

        DrillCycle cycle{no_datum};
        std::vector<Phase> sequence;

        // Get past settle first.
        for (int i = 0; i < 2100; ++i)
        {
            Feedback fb{};
            fb.dt_s = 0.001;
            fb.axial_force_n = 20.0;
            cycle.step(fb);
        }

        const double sweep[] = {0.002, 0.004, 0.002, 0.011, 0.008, 0.012, 0.046, 0.044, 0.047, 0.030};
        for (const double d : sweep)
        {
            for (int i = 0; i < 50; ++i)
            {
                Feedback fb{};
                fb.depth_m = d;
                fb.axial_force_n = 20.0;
                fb.dt_s = 0.001;
                const Command c = cycle.step(fb);
                if (sequence.empty() || sequence.back() != c.phase)
                {
                    sequence.push_back(c.phase);
                }
            }
        }

        for (const Phase p : {Phase::kSpotting, Phase::kEntry, Phase::kDrilling, Phase::kSlowFinish})
        {
            check(phaseEntryCount(sequence, p) <= 1,
                  std::string{"oscillating depth enters "} + cr20_drill::phaseName(p) + " at most once");
        }
        check(cycle.phase() == Phase::kSlowFinish, "regimes never step backwards when depth retreats");
    }
}

// --------------------------------------------------------------------------
// 4. Aborts
// --------------------------------------------------------------------------

void checkAborts()
{
    section("Aborts");

    const DrillConfig nominal{};

    // An abort raised in each phase must still land at kDone with the
    // command returned to the baseline.
    const Phase trigger_phases[] = {Phase::kSettle, Phase::kSpotting, Phase::kEntry,
                                    Phase::kDrilling, Phase::kSlowFinish, Phase::kDwell};
    for (const Phase trigger : trigger_phases)
    {
        const RunResult r = run(nominal, Plant{}, 400000, [trigger](Feedback& fb, Phase phase) {
            if (phase == trigger)
            {
                fb.operator_abort = true;
            }
        });
        check(r.completed && r.final_phase == Phase::kDone,
              std::string{"abort during "} + cr20_drill::phaseName(trigger) + " reaches kDone");
        checkNear(r.final_commanded_m, 0.0, 1e-6,
                  std::string{"abort during "} + cr20_drill::phaseName(trigger) + " returns to baseline");
        check(r.abort == AbortReason::kOperator,
              std::string{"abort during "} + cr20_drill::phaseName(trigger) + " records the operator reason");
    }

    // Retract is never gated. Every advance-blocking condition is held at a
    // tripping value throughout; the command must still reach zero.
    {
        const RunResult r = run(nominal, Plant{}, 400000, [](Feedback& fb, Phase phase) {
            if (phase == Phase::kDrilling)
            {
                fb.operator_abort = true;
            }
            if (phase == Phase::kRetract)
            {
                fb.axial_force_n = 500.0;
                fb.lateral_force_n = 500.0;
                fb.reaction_torque_nm = 100.0;
                fb.lateral_m = 0.050;
                fb.axis_drift_rad = 1.0;
                fb.depth_rate_m_per_s = 5.0;
            }
        });
        check(r.completed, "retract completes with every abort condition held true");
        checkNear(r.final_commanded_m, 0.0, 1e-6, "retract reaches the baseline despite held aborts");
    }

    // A rigid wall stalls rather than running away, and the force ceiling
    // holds the lead at the cap.
    {
        Plant wall{};
        wall.cut_gain_m_per_s_per_n = 0.0;
        const RunResult r = run(nominal, wall, 200000, [](Feedback& fb, Phase phase) {
            // Nothing cuts, so nothing would ever end the run on its own;
            // the watchdog is the caller's job in the real program.
            if (phase == Phase::kSpotting && fb.depth_m >= 0.0015)
            {
                fb.watchdog_expired = true;
            }
        });
        check(r.max_lead_m <= nominal.lead_max_m + 1e-9, "rigid wall: lead still respects the cap");
        checkNear(r.final_commanded_m, 0.0, 1e-6, "rigid wall: retract still returns to baseline");
    }

    // Breakthrough must trip the runaway detector, not be ridden down.
    {
        Plant breakthrough{};
        breakthrough.breakthrough_m = 0.040;
        // Past breakthrough the tool is released into open space and really
        // does run - that is the whole point, and it is what the runaway
        // detector has to catch.
        breakthrough.free_rate_m_per_s = 0.5;
        const RunResult r = run(nominal, breakthrough);
        check(r.abort == AbortReason::kDepthRunaway || r.abort == AbortReason::kDepthOverrun,
              "breakthrough trips a depth abort");
        check(r.completed && r.final_phase == Phase::kDone, "breakthrough still returns to the baseline");
    }

    // The bit pointing away from the work: nothing is ever touched, so no
    // force develops. The contact check must stop it in spotting.
    {
        Plant no_contact{};
        no_contact.gap_m = 10.0; // never reaches material
        const RunResult r = run(nominal, no_contact);
        check(r.abort == AbortReason::kNoContact, "no contact during spotting aborts");
        check(phaseEntryCount(r.phase_sequence, Phase::kDrilling) == 0,
              "no contact never reaches the drilling regime");
        check(r.completed && r.final_phase == Phase::kDone, "no contact still returns to the baseline");
    }

    // The same free-air motion is legitimate when contact is not required,
    // which is how the commissioning ladder's free-air stroke runs.
    {
        DrillConfig free_air = nominal;
        free_air.require_contact = false;
        free_air.hole_depth_m = 0.005;
        free_air.lead_max_m = 0.003;

        Plant air{};
        air.gap_m = 10.0;

        const RunResult r = run(free_air, air);
        check(r.abort == AbortReason::kNone, "free-air stroke with require_contact=false does not abort");
        check(r.max_lead_m <= free_air.lead_max_m + 1e-9, "free-air stroke respects its lead cap");
        check(r.completed, "free-air stroke finishes");

        // The tool genuinely travels the whole stroke in air, so the retract
        // must wait for it to come back - ending on the commanded point alone
        // would stop the loop with the arm still a full stroke out.
        check(std::abs(r.final_depth_m) <= free_air.retract_settle_m + 1e-6,
              "free-air stroke returns the TOOL to the baseline, not just the command");
    }

    // The retract overshoot, against a plant that actually has stiction.
    // Without the overshoot the tool parks at friction/stiffness short of the
    // baseline - 6.6 mm on the real arm - and no timeout can fix that,
    // because the restoring force has already faded to nothing.
    {
        DrillConfig cfg = nominal;
        cfg.require_contact = false;
        cfg.hole_depth_m = 0.055;
        cfg.lead_max_m = 0.010;

        Plant sticky{};
        sticky.gap_m = 10.0;
        sticky.static_friction_n = 8.3;     // forward breakaway, measured
        sticky.reverse_friction_n = 19.7;   // at-rest on the way home, measured

        const RunResult r = run(cfg, sticky);
        check(r.completed, "sticky free-air stroke finishes");
        check(std::abs(r.final_depth_m) <= cfg.retract_settle_m + 1e-6,
              "retract overshoot brings the tool home against 20 N of stiction");
        check(std::abs(r.final_commanded_m) <= 1e-6,
              "and unwinds the command back to the baseline, leaving nothing loaded");
        check(r.min_commanded_m < -1e-6,
              "the commanded point really does go past the baseline to do it");
    }

    // The same thing at full depth, which is the configuration used to watch
    // the whole cycle run in air.
    {
        DrillConfig full_air = nominal;
        full_air.require_contact = false;
        full_air.hole_depth_m = 0.055;
        full_air.lead_max_m = 0.010;

        Plant air{};
        air.gap_m = 10.0;

        const RunResult r = run(full_air, air);
        check(r.completed && r.abort == AbortReason::kNone, "55 mm free-air stroke completes cleanly");
        check(r.max_depth_m >= 0.055, "55 mm free-air stroke reaches full depth");
        check(std::abs(r.final_depth_m) <= full_air.retract_settle_m + 1e-6,
              "55 mm free-air stroke brings the tool all the way home");
    }

    // A failed force calibration must be caught at the end of settle,
    // before anything advances.
    {
        const RunResult r = run(nominal, Plant{}, 400000, [](Feedback& fb, Phase phase) {
            if (phase == Phase::kSettle)
            {
                fb.axial_force_n = 35.0; // the uncalibrated bias measured on this arm
            }
        });
        check(r.abort == AbortReason::kCalibrationResidual, "settle residual above the limit aborts");
        check(phaseEntryCount(r.phase_sequence, Phase::kSpotting) == 0,
              "failed calibration never reaches spotting");
    }
}

// --------------------------------------------------------------------------
// 5. The schedule table, printed from the live constants so that the plan
// document and the code cannot quietly drift apart.
// --------------------------------------------------------------------------

void printSchedule()
{
    const DrillConfig config{};
    const DrillCycle cycle{config};

    section("Lead schedule (from the live constants)");
    std::printf("  hole depth %.1f mm, lead cap %.1f mm, stiffness %.0f N/m\n\n", config.hole_depth_m * 1000.0,
                config.lead_max_m * 1000.0, config.push_stiffness_n_per_m);
    std::printf("  %8s  %10s  %10s  %12s\n", "depth", "lead", "thrust", "needed*");
    std::printf("  %8s  %10s  %10s  %12s\n", "[mm]", "[mm]", "[N]", "[N]");

    for (const double depth_mm : {0.0, 3.0, 10.0, 20.0, 30.0, 45.0, 55.0, 60.0})
    {
        const double depth_m = depth_mm / 1000.0;
        // Measured smooth-feed demand from the logged reference run.
        const double needed_n = 2.52 * depth_mm + 0.5;
        std::printf("  %8.0f  %10.1f  %10.1f  %12.1f\n", depth_mm, cycle.leadForDepth(depth_m) * 1000.0,
                    cycle.thrustForDepth(depth_m), needed_n);
    }
    std::printf("\n  * measured demand, 2.52 N/mm x depth + 0.5 N, from drill_log.csv\n");
    std::printf("    The lead is also the lunge distance if the bit breaks through.\n");
}

} // namespace

int main()
{
    std::printf("cr20 drill off-robot checks\n");

    checkFrames();
    checkStateMachine();
    checkAborts();
    printSchedule();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        std::printf("ALL CHECKS PASSED\n");
        return 0;
    }
    std::printf("CHECKS FAILED\n");
    return 1;
}
