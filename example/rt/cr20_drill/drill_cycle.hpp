/**
 * @file drill_cycle.hpp
 * @brief The drill plunge state machine and depth-scheduled feed law, with no
 * SDK, robot, Eigen or threading dependency - plain scalars only.
 *
 * Everything here operates on a single scalar depth along the bit axis.
 * Turning that scalar into a 4x4 pose is drill_frames.hpp's job, and talking
 * to a controller is drill_test.cpp's. Keeping this file free of both is what
 * lets drill_checks.cpp exercise every phase transition, every abort and the
 * whole feed schedule on a laptop, before any of it reaches a real arm.
 *
 * HOW THE FEED WORKS
 *
 * The controller is a spring: it takes a position command and pushes with
 * force proportional to how far that command is from where the tool actually
 * is. So thrust is commanded indirectly, by deliberately commanding a point
 * some distance AHEAD of the measured depth. On top of that sits a constant
 * feedforward preload, applied outside the position spring:
 *
 *     thrust = preload + cartesian_stiffness_z * lead
 *
 * The preload exists because this arm has a friction deadband of about 20 N
 * at the TCP: below it, a commanded offset produces neither motion nor a
 * readable external force. See DrillConfig::preload_n for the measurements
 * and for why a preload under breakaway is safe to put underneath the full
 * lead schedule. Everything below concerns the spring term.
 *
 * That lead is the one number that matters, for two reasons at once. It sets
 * the working force, and it is also exactly how far the tool lunges if the
 * bit breaks through or slips - the stored spring releases into open space.
 * A fixed lead big enough for the bottom of a 55 mm hole (50 mm, 150 N) would
 * therefore carry a 50 mm lunge risk from the very first millimetre, when the
 * bit is least captured and most likely to skate.
 *
 * Hence the depth schedule: lead grows with measured depth, roughly
 * `lead = depth + 4 mm`, tracking what the material was measured to demand
 * (about 2.5 N per mm of depth, against a 3 N/mm stiffness). At 10 mm depth
 * the stored lunge is 14 mm; the full 50 mm exists only below 45 mm, by which
 * point the bit is captured laterally by its own bore.
 *
 * The cap, `lead_max_m`, is the primary safety dial: it bounds both the
 * spring thrust and the lunge distance. It is no longer the WHOLE dial,
 * though - the preload sits underneath it and does not scale, so
 * `lead_max_m = 0.003` now means 9 N of spring on top of the preload rather
 * than 9 N in total. Low-force rungs below the deadband are not available at
 * any setting of it, because they were never observable in the first place.
 */

#ifndef CR20_DRILL_CYCLE_HPP_
#define CR20_DRILL_CYCLE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cr20_drill
{

/// The plunge progression. Ordering is significant: regime selection compares
/// these as integers and only ever moves forwards (see DrillCycle::evaluate).
enum class Phase
{
    kSettle = 0,
    kPreload,
    kSpotting,
    kEntry,
    kDrilling,
    kSlowFinish,
    kDwell,
    kRetract,
    kDone,
};

inline const char* phaseName(Phase phase)
{
    switch (phase)
    {
        case Phase::kSettle:
            return "settle";
        case Phase::kPreload:
            return "preload";
        case Phase::kSpotting:
            return "spotting";
        case Phase::kEntry:
            return "entry";
        case Phase::kDrilling:
            return "drilling";
        case Phase::kSlowFinish:
            return "slow_finish";
        case Phase::kDwell:
            return "dwell";
        case Phase::kRetract:
            return "retract";
        case Phase::kDone:
            return "done";
    }
    return "unknown";
}

inline int phaseIndex(Phase phase)
{
    return static_cast<int>(phase);
}

/// Why the run stopped advancing. kNone means it did not.
enum class AbortReason
{
    kNone = 0,
    kOperator,
    kCalibrationResidual,
    kPreloadCreep,
    kNoContact,
    kMotionReversed,
    kAxialForce,
    kLateralForce,
    kReactionTorque,
    kLateralExcursion,
    kAxisDrift,
    kDepthRunaway,
    kDepthOverrun,
    kWatchdog,
};

inline const char* abortReasonName(AbortReason reason)
{
    switch (reason)
    {
        case AbortReason::kNone:
            return "none";
        case AbortReason::kOperator:
            return "operator";
        case AbortReason::kCalibrationResidual:
            return "calibration_residual";
        case AbortReason::kPreloadCreep:
            return "preload_creep";
        case AbortReason::kNoContact:
            return "no_contact";
        case AbortReason::kMotionReversed:
            return "motion_reversed";
        case AbortReason::kAxialForce:
            return "axial_force";
        case AbortReason::kLateralForce:
            return "lateral_force";
        case AbortReason::kReactionTorque:
            return "reaction_torque";
        case AbortReason::kLateralExcursion:
            return "lateral_excursion";
        case AbortReason::kAxisDrift:
            return "axis_drift";
        case AbortReason::kDepthRunaway:
            return "depth_runaway";
        case AbortReason::kDepthOverrun:
            return "depth_overrun";
        case AbortReason::kWatchdog:
            return "watchdog";
    }
    return "unknown";
}

/// What bounded the commanded depth this cycle. Logged per row; more useful
/// than a bare event counter because it says WHICH limit was binding and for
/// how long, which is how you tell "the schedule is right but the bit is
/// cutting faster than expected" from "the force ceiling is holding it back".
enum class Limit
{
    kNone = 0,
    kFeedRate,
    kLeadCap,
    kForceHold,
    kDepthTarget,
};

inline const char* limitName(Limit limit)
{
    switch (limit)
    {
        case Limit::kNone:
            return "none";
        case Limit::kFeedRate:
            return "feed_rate";
        case Limit::kLeadCap:
            return "lead_cap";
        case Limit::kForceHold:
            return "force_hold";
        case Limit::kDepthTarget:
            return "depth_target";
    }
    return "unknown";
}

/**
 * @struct DrillConfig
 * @brief Every knob, with the defaults for a 55 mm blind hole.
 *
 * For the commissioning ladder only two of these normally change:
 * `hole_depth_m` and `lead_max_m`. The lead cap alone flattens the schedule,
 * so a shallow low-force run needs no other edits.
 */
struct DrillConfig
{
    // --- the two knobs the ladder turns ---------------------------------

    /// Blind-hole target, measured from the CONTACT DATUM - the depth at
    /// which the bit was detected to have stalled against the work - and not
    /// from the pose the loop started in. The operator sets up with a 1-2 mm
    /// standoff that they have only eyeballed, so measuring from the start
    /// pose would take that unknown gap straight out of the hole.
    ///
    /// Note that even from the datum, this still overstates the hole: at the
    /// measured environment stiffness of ~13 N/mm, 150 N of thrust is taken
    /// up by roughly 11 mm of deflection and material yield. Gauge the first
    /// hole rather than trusting this number.
    double hole_depth_m = 0.055;

    /// Ceiling on the commanded lead. THE safety dial: it sets both the
    /// maximum thrust (lead * stiffness) and the maximum lunge distance.
    /// 0.050 m = 150 N at 3000 N/m.
    double lead_max_m = 0.050;

    // --- the lead schedule ----------------------------------------------

    /// Lead at zero depth, i.e. the spotting force. 0.004 m = 12 N.
    double lead_at_surface_m = 0.004;

    /// Growth of lead per metre of depth. Unity because the material's
    /// measured demand (~2.5 N/mm) and the stiffness ceiling (3 N/mm) are
    /// near enough the same number, which is what makes the schedule
    /// memorable: lead is depth plus a constant.
    double lead_slope = 1.0;

    /// Push-axis stiffness, N/m. Not commanded from here - drill_test.cpp
    /// owns setCartesianImpedance - but the predicted thrust printed in the
    /// plan block and logged per row is computed from it, so it must match.
    double push_stiffness_n_per_m = 3000.0;

    // --- the preload ------------------------------------------------------
    //
    // A constant feedforward force along the bit axis, applied through
    // setCartesianImpedanceDesiredTorque (fc frame, Z) rather than through the
    // position spring. Thrust is therefore TWO terms from here on:
    //
    //     thrust = preload_n + push_stiffness_n_per_m * lead
    //
    // WHY IT EXISTS. The 2026-09-22 free-air runs measured a friction deadband
    // of about 20 N at the TCP. Free-air velocity is flat at 0.15-0.3 mm/s
    // from 9 N all the way to 18 N, then steps to 1.34 mm/s at 19 N; the
    // reverse direction parks the tool 6.56 mm short of its command at rest,
    // which is 19.7 N. Below that band a commanded offset produces neither
    // motion nor a readable external force - the estimator reported 6.16 N
    // against a 9 N spring load with nothing touching the tool. Every force
    // target under ~20 N was therefore unobservable, which is what killed the
    // contact detector and what made the bottom of the commissioning ladder
    // meaningless. Stiffness offers no relief: 3000 N/m is the SDK ceiling and
    // is already set.
    //
    // WHY IT IS SAFE TO KEEP THE FULL LEAD SCHEDULE UNDERNEATH IT. A preload
    // BELOW breakaway cannot drive motion on its own. If the bit breaks
    // through, the lead collapses as the tool lunges, net force falls to the
    // preload alone, and the arm stops because the preload is under the knee.
    // The spring term is self-limiting because force decays with distance; the
    // preload is self-limiting because it sits under the floor. So the stored
    // lunge at any depth is exactly what it was before.
    //
    // WHICH MAKES "UNDER BREAKAWAY" A HARD CONSTRAINT, NOT A PREFERENCE. Above
    // it the arm creeps forward continuously at zero lead with nothing
    // commanding it. Breakaway varies with pose and has been measured in ONE
    // pose, so this is set well under the single figure we have rather than
    // shaved close to it. kPreload exists to catch the case where it is still
    // too high - see preload_creep_abort_m.
    //
    // NOTE FOR THE LADDER: lead_max_m is no longer the whole safety dial. It
    // still bounds the lunge, but it no longer bounds the thrust, because this
    // term sits underneath it and does not scale.
    double preload_n = 15.0;

    /// Rate the preload is ramped in and out at. Applied from a non-RT thread,
    /// so this is a target rather than a guarantee; the trace logs what was
    /// actually asked for each cycle.
    double preload_ramp_n_per_s = 20.0;

    /// Held at full preload and zero lead before spotting begins. This is the
    /// measurement window the whole design rests on: preload applied, nothing
    /// else commanded, so the trace shows plainly whether the arm holds
    /// station and whether tauExt_inStiff reports the feedforward or subtracts
    /// it as commanded internal torque. That second question decides whether
    /// the axial force thresholds need re-basing, and it cannot be answered
    /// from the SDK headers.
    double preload_hold_sec = 2.0;

    /// Travel during kPreload BEYOND the standing offset the preload itself
    /// creates, at which the preload is judged to be driving the arm rather
    /// than merely loading it.
    ///
    /// The offset is not a fault and cannot be designed away. With the
    /// command held at the baseline, the tool moves forward until the spring
    /// cancels the feedforward - preload_n/stiffness, or 5 mm at 15 N against
    /// 3000 N/m - and then stops. An ABSOLUTE threshold below that figure
    /// aborts on the preload working correctly, which is what a 1 mm limit
    /// did on 2026-09-23: the tool converged on 1.4 mm and the run stopped
    /// calling it a runaway.
    ///
    /// So the check is on the excess over that offset. Converging anywhere
    /// inside it means friction and the spring found a balance; travelling
    /// past it means neither is holding, which is the real failure.
    double preload_creep_margin_m = 0.003;

    // --- regime boundaries, on measured depth ---------------------------

    double entry_begins_m = 0.003;
    double drilling_begins_m = 0.010;
    double slow_finish_begins_m = 0.045;

    // --- commanded feed rates, per regime -------------------------------
    //
    // These bound the COMMANDED point, which advances at roughly twice the
    // rate the bit actually cuts: the command chases `depth + lead`, and lead
    // itself grows with depth, so d(depth + lead)/dt = 2 * cut rate. A
    // commanded 2.5 mm/s therefore permits about 1.25 mm/s of cutting, close
    // to the 1.47 mm/s achieved by hand in the logged reference run.

    double spotting_feed_m_per_s = 0.0010;
    double entry_feed_m_per_s = 0.0030;
    double drilling_feed_m_per_s = 0.0050;
    double slow_finish_feed_m_per_s = 0.0020;

    // --- retract --------------------------------------------------------

    /// Rate used while the commanded point is still AHEAD of the tool.
    /// Retreating there only bleeds off stored spring - the tool barely moves
    /// - so it should be fast. At 55 mm depth the command is 50 mm ahead, and
    /// unwinding that at the withdrawal rate would mean five further seconds
    /// of full-thrust drilling before the force even reached zero, which is
    /// precisely wrong when the operator has just pressed abort.
    double bleed_rate_m_per_s = 0.050;

    /// Rate used once the commanded point is BEHIND the tool and is therefore
    /// pulling the bit out of the hole, which is real motion.
    double withdraw_rate_m_per_s = 0.010;

    // --- timers ---------------------------------------------------------

    /// Held at exactly zero offset before anything advances. Long enough to
    /// average the force residual over, and to let the controller's own
    /// command filter settle on a stationary command.
    double settle_sec = 2.0;

    /// Held at depth before retracting, so the stored spring bleeds off
    /// through cutting rather than through withdrawal.
    double dwell_sec = 0.3;

    /// How close to the baseline the TOOL must get before the retract stops
    /// pulling and unwinds. Friction gives the tool a deadband of roughly
    /// +-friction/stiffness (~6.6 mm here) around the commanded point, so it
    /// will not stop dead on the line; this is the band in which it counts as
    /// home.
    double retract_settle_m = 0.003;

    /// Hard limit on how far past the baseline the commanded point may go.
    ///
    /// This is a SAFETY STOP, not a target. The retract ramps the command
    /// negative for exactly as long as the tool has not arrived, which is
    /// self-tuning: it goes as far as friction demands and no further,
    /// without this file needing to know what the friction is. Treating the
    /// limit as a target instead is what an earlier version did, and it
    /// dragged the tool 8.4 mm PAST the baseline - the command reached the
    /// bound whether or not the tool had arrived, then held it there.
    ///
    /// Reaching this bound means the tool is not following at all; the
    /// timeout then ends the run.
    double retract_command_limit_m = 0.030;

    /// Backstop for a bit that will not come out. Once this expires the run
    /// unwinds the command to the baseline and finishes regardless.
    ///
    /// Generous on purpose: this is a backstop, not a performance target, and
    /// the wall-clock watchdog still bounds the whole run. The real arm took
    /// 5.6 s to bring the tool back 47 mm, but it does so against friction
    /// that varies with pose, and a timeout that fires early leaves the tool
    /// parked out - which is the failure this whole mechanism exists to
    /// avoid.
    double retract_timeout_sec = 60.0;

    // --- contact and direction checks ------------------------------------

    /// Require contact to have been found during spotting before any deeper
    /// regime is entered, and datum the hole depth from it. Set false only
    /// for a deliberate free-air stroke, where there is nothing to touch and
    /// depth is therefore measured from the start pose.
    bool require_contact = true;

    /// Lead at which the bit is judged to have stalled against the work.
    ///
    /// In free space the tool tracks the commanded point, so the lead stays
    /// near zero. The moment the bit touches something it stops following
    /// and the lead begins to open up. That transition is both the contact
    /// detector and the depth datum, which is why one threshold serves both.
    ///
    /// 0.3 mm is ~1 N of spring force, and at the measured environment
    /// stiffness it corresponds to about 0.02 mm of penetration - close
    /// enough to first touch to datum from, and far enough above position
    /// noise to be unambiguous.
    double contact_datum_lead_m = 0.0003;

    /// Commanded depth by which contact must have been found. Just under the
    /// spotting lead, so the check fires once the lead is essentially fully
    /// developed and there is nothing left to wait for.
    ///
    /// This also bounds the standoff the operator may leave: the gap must be
    /// less than this, or the bit never reaches the work before the check
    /// fires. With the default 4 mm spotting lead that means a gap under
    /// 3.5 mm, and the 1-2 mm the quickstart asks for sits comfortably
    /// inside it.
    double contact_check_arm_depth_m = 0.0035;

    /// Grace period after the arm depth is reached, so a transient does not
    /// decide it.
    double contact_check_hold_sec = 0.5;

    // --- aborts ----------------------------------------------------------

    /// Mean force residual over the settle window, above which
    /// calibrateForceSensor() is judged not to have taken. The uncalibrated
    /// bias measured on this arm was 35 N, so this catches the failure that
    /// matters while clearing the noise floor comfortably.
    double calibration_residual_abort_n = 25.0;
    double calibration_residual_warn_n = 10.0;

    /// Measured retreat, against an advancing command, that means the tool is
    /// moving opposite to the command rather than merely failing to follow.
    double motion_reversed_m = 0.001;

    /// Freezes advance only. Lead growth stops; retract is never gated.
    double axial_force_hold_n = 180.0;

    /// Sensor-side backstop, independent of the stiffness model above.
    double axial_force_abort_n = 230.0;

    double lateral_force_abort_n = 80.0;
    double reaction_torque_abort_nm = 15.0;
    /// Raised from 5 mm after the 2026-09-22 free-air stroke: with nothing
    /// touching the tool at all, the arm's own tracking error reached 4.8 mm
    /// on the way out and 10.9 mm on the faster retract. A 5 mm limit would
    /// have tripped on the arm's own path, not on the hole wandering.
    double lateral_excursion_abort_m = 0.015;
    double axis_drift_abort_rad = 0.0873; // 5 degrees

    /// Catches breakthrough. At 1 kHz this fires within tens of milliseconds,
    /// i.e. well under a millimetre of travel, and puts the cycle straight
    /// into the fast bleed.
    double depth_runaway_m_per_s = 0.025;

    /// Depth past the target at which something has clearly gone wrong.
    double depth_overrun_margin_m = 0.005;
};

/**
 * @struct Feedback
 * @brief Everything the state machine is allowed to know, computed by the
 * caller from one pose reading and one wrench reading.
 *
 * Note that the depth, lateral and drift figures all come from a single
 * toolDeltaBetween(baseline, measured) call - see drill_frames.hpp.
 */
struct Feedback
{
    /// Travel along the bit axis from the baseline, metres. Positive is into
    /// the work.
    double depth_m = 0.0;

    /// Wander perpendicular to the bit axis, metres, magnitude.
    double lateral_m = 0.0;

    /// Tool tilt since the baseline, radians.
    double axis_drift_rad = 0.0;

    /// Force along the bit axis, newtons, magnitude. Sign convention of
    /// tauExt_inStiff on this controller is not yet confirmed, so every
    /// comparison here is on magnitude; the signed value is logged.
    double axial_force_n = 0.0;

    double lateral_force_n = 0.0;
    double reaction_torque_nm = 0.0;

    /// Rate of change of depth, metres per second, already smoothed by the
    /// caller.
    double depth_rate_m_per_s = 0.0;

    /// This cycle's period, seconds.
    double dt_s = 0.001;

    bool operator_abort = false;
    bool watchdog_expired = false;
};

/// What the state machine produced this cycle.
struct Command
{
    /// Commanded depth along the bit axis, metres. Hand this to
    /// advanceAlongBit().
    double depth_m = 0.0;

    /// The lead this implies, metres. Logged so that `lead * stiffness`
    /// can be checked against the measured force - the single most useful
    /// consistency check in the trace.
    double lead_m = 0.0;

    /// Feedforward force to apply along the bit axis this cycle, newtons.
    /// The caller is responsible for getting this to the controller; it is
    /// produced here so that the ramp sequencing is part of the state machine
    /// that drill_checks.cpp can exercise on a laptop, rather than living in
    /// the thread that happens to own the SDK handle.
    double preload_n = 0.0;

    Phase phase = Phase::kSettle;
    Limit limit = Limit::kNone;
    bool finished = false;
};

/**
 * @class DrillCycle
 * @brief Turns a measured depth and force into the next commanded depth, one
 *   control cycle at a time.
 *
 * The per-cycle contract is deliberately explicit, because the thing that
 * made the previous implementation hard to reason about was that phase
 * selection, one-shot latching and the motion update were tangled together:
 *
 *     1. checkAborts()  - pure. Has anything gone wrong?
 *     2. evaluate()     - pure. Which state SHOULD I be in?
 *     3. onExit/onEntry - at most once per transition. Latches and timers.
 *     4. execute()      - every cycle. The only thing that moves the command.
 *
 * Single-threaded by construction: step() is the only mutator. An abort from
 * another thread arrives through Feedback::operator_abort rather than this
 * class holding an atomic, keeping it free of any threading policy.
 */
class DrillCycle
{
  public:
    explicit DrillCycle(DrillConfig config) : config_(config)
    {
    }

    Command step(const Feedback& feedback)
    {
        const AbortReason abort = checkAborts(feedback);
        const Phase next = evaluate(feedback, abort);

        if (next != phase_)
        {
            onExit(feedback);
            phase_ = next;
            onEntry(abort);
        }

        time_in_phase_s_ += feedback.dt_s;
        return execute(feedback);
    }

    Phase phase() const
    {
        return phase_;
    }
    bool finished() const
    {
        return phase_ == Phase::kDone;
    }
    AbortReason abortReason() const
    {
        return abort_reason_;
    }
    double commandedDepth() const
    {
        return commanded_depth_;
    }

    /// Mean force residual measured over the settle window. Meaningful only
    /// after settle has ended.
    double settleResidualN() const
    {
        return settle_samples_ > 0 ? settle_force_sum_ / static_cast<double>(settle_samples_) : 0.0;
    }

    /// The lead the schedule asks for at a given HOLE depth (measured from
    /// the contact datum, not the start pose). Exposed so the plan block
    /// prints the same numbers the loop will use, rather than a second copy
    /// of the formula that can drift out of step with this one.
    double leadForDepth(double hole_depth_m) const
    {
        const double scheduled = config_.lead_at_surface_m + config_.lead_slope * hole_depth_m;
        return std::clamp(scheduled, config_.lead_at_surface_m, config_.lead_max_m);
    }

    /// Thrust the schedule implies at a given hole depth, newtons. Both
    /// terms: the preload floor plus what the spring adds on top of it.
    double thrustForDepth(double hole_depth_m) const
    {
        return config_.preload_n + leadForDepth(hole_depth_m) * config_.push_stiffness_n_per_m;
    }


    /// Depth of the hole itself: travel beyond the point where the bit was
    /// detected to have stalled against the work. Zero until contact is
    /// found. This - not the raw travel from the start pose - is what the
    /// regime boundaries, the lead schedule and the depth target all use, so
    /// that the operator's eyeballed standoff does not come out of the hole.
    /// How far the preload pushes the tool forward when the command is held
    /// at the baseline: the displacement at which the spring exactly cancels
    /// the feedforward. Zero without a preload.
    double preloadStandingOffsetM() const
    {
        return config_.push_stiffness_n_per_m > 0.0
                   ? std::abs(config_.preload_n) / config_.push_stiffness_n_per_m
                   : 0.0;
    }

    double holeDepth(double measured_depth_m) const
    {
        return measured_depth_m - contact_datum_m_;
    }

    /// Where the bit was found. Zero if contact was never required.
    double contactDatum() const
    {
        return contact_datum_m_;
    }
    bool contactFound() const
    {
        return contact_found_;
    }

    const DrillConfig& config() const
    {
        return config_;
    }

  private:
    // -- 1. aborts -------------------------------------------------------
    //
    // Pure: reads feedback and latched state, returns a reason, mutates
    // nothing. Checked before any regime logic so that no phase can mask an
    // abort. Every force comparison is on magnitude - see Feedback.

    AbortReason checkAborts(const Feedback& fb) const
    {
        if (fb.operator_abort)
        {
            return AbortReason::kOperator;
        }
        if (fb.watchdog_expired)
        {
            return AbortReason::kWatchdog;
        }
        // Both of these are evaluated here rather than latched on a phase
        // exit, so that they are seen BEFORE the transition that would
        // otherwise let the run advance one phase on a condition that should
        // already have stopped it.
        if (phase_ == Phase::kSettle && time_in_phase_s_ >= config_.settle_sec &&
            std::abs(settleResidualN()) > config_.calibration_residual_abort_n)
        {
            return AbortReason::kCalibrationResidual;
        }
        // Travel past the point where the spring can cancel the feedforward
        // means nothing is holding the tool and the preload is driving it.
        if (phase_ == Phase::kPreload &&
            std::abs(fb.depth_m) > preloadStandingOffsetM() + config_.preload_creep_margin_m)
        {
            return AbortReason::kPreloadCreep;
        }
        if (contact_check_failed_)
        {
            return AbortReason::kNoContact;
        }

        // The tool moving backwards while the command advances means the
        // command is not reaching the arm the way this file believes it is.
        // Nothing downstream is trustworthy if that is true.
        if (commanded_depth_ > config_.motion_reversed_m && fb.depth_m < -config_.motion_reversed_m)
        {
            return AbortReason::kMotionReversed;
        }

        if (std::abs(fb.axial_force_n) > config_.axial_force_abort_n)
        {
            return AbortReason::kAxialForce;
        }
        if (std::abs(fb.lateral_force_n) > config_.lateral_force_abort_n)
        {
            return AbortReason::kLateralForce;
        }
        if (std::abs(fb.reaction_torque_nm) > config_.reaction_torque_abort_nm)
        {
            return AbortReason::kReactionTorque;
        }
        if (fb.lateral_m > config_.lateral_excursion_abort_m)
        {
            return AbortReason::kLateralExcursion;
        }
        if (fb.axis_drift_rad > config_.axis_drift_abort_rad)
        {
            return AbortReason::kAxisDrift;
        }
        if (fb.depth_rate_m_per_s > config_.depth_runaway_m_per_s)
        {
            return AbortReason::kDepthRunaway;
        }
        if (holeDepth(fb.depth_m) > config_.hole_depth_m + config_.depth_overrun_margin_m)
        {
            return AbortReason::kDepthOverrun;
        }

        return AbortReason::kNone;
    }

    // -- 2. evaluation ----------------------------------------------------

    /**
     * Pure. Which phase should we be in, given this feedback?
     *
     * Regime selection is monotonic - it only ever moves forwards. Measured
     * depth is noisy and can genuinely decrease as material yields, so a
     * plain threshold comparison would chatter across every boundary,
     * re-running onEntry and onExit repeatedly. Only kRetract and kDone may
     * be entered out of order.
     */
    Phase evaluate(const Feedback& fb, AbortReason abort) const
    {
        if (phase_ == Phase::kDone)
        {
            return Phase::kDone;
        }

        if (phase_ == Phase::kRetract)
        {
            // The TOOL has to be home, not just the command. When drilling,
            // the two are nearly the same - the bit is stalled in its hole,
            // so commanded and measured arrive together. In free air they are
            // not: the tool genuinely travels the whole stroke, the command
            // bleeds to zero in about a second, and ending on the command
            // alone would stop the control loop with the arm still a full
            // stroke out and a large spring pulling it back.
            //
            // The timeout is the backstop for a bit that will not come out at
            // all; the run then ends with the command at the baseline, which
            // is the most retracting force this design can apply anyway.
            const bool command_home = std::abs(commanded_depth_) <= kDepthEpsilonM;
            const bool gave_up = time_in_phase_s_ >= config_.retract_timeout_sec;

            // Nothing waits on the feedforward here: it cannot be released
            // from inside the loop at all, so the caller clears it once the
            // loop has stopped.
            return (command_home && (retract_tool_home_ || gave_up)) ? Phase::kDone : Phase::kRetract;
        }

        if (abort != AbortReason::kNone)
        {
            return Phase::kRetract;
        }

        if (phase_ == Phase::kSettle)
        {
            return time_in_phase_s_ >= config_.settle_sec ? Phase::kPreload : Phase::kSettle;
        }

        // Leave only once the feedforward has been held AT its target, timed
        // from the moment it got there rather than from phase entry, so the
        // ramp does not eat into the measurement window.
        if (phase_ == Phase::kPreload)
        {
            return preload_hold_s_ >= config_.preload_hold_sec ? Phase::kSpotting : Phase::kPreload;
        }

        if (phase_ == Phase::kDwell)
        {
            return time_in_phase_s_ >= config_.dwell_sec ? Phase::kRetract : Phase::kDwell;
        }

        // Until the bit has been found there is no datum, so no depth
        // threshold below means anything yet - raw travel would otherwise be
        // mistaken for hole depth and march the run straight through the
        // regimes on a standoff gap it had not even closed. Finding the work
        // is spotting's whole job, so stay there until it has.
        if (config_.require_contact && !contact_found_)
        {
            return phase_;
        }

        // Everything below is on HOLE depth - travel beyond the contact
        // datum - so that the standoff the operator left is not counted as
        // part of the hole.
        const double hole_depth = holeDepth(fb.depth_m);

        if (hole_depth >= config_.hole_depth_m)
        {
            return Phase::kDwell;
        }

        const Phase by_depth = regimeForDepth(hole_depth);
        return phaseIndex(by_depth) > phaseIndex(phase_) ? by_depth : phase_;
    }

    Phase regimeForDepth(double hole_depth_m) const
    {
        if (hole_depth_m >= config_.slow_finish_begins_m)
        {
            return Phase::kSlowFinish;
        }
        if (hole_depth_m >= config_.drilling_begins_m)
        {
            return Phase::kDrilling;
        }
        if (hole_depth_m >= config_.entry_begins_m)
        {
            return Phase::kEntry;
        }
        return Phase::kSpotting;
    }

    // -- 3. transitions ---------------------------------------------------

    void onExit(const Feedback& fb)
    {
        // Nothing needs unwinding on the way out of any phase today. The
        // hook is kept because the per-cycle contract is the readable part
        // of this class, and a transition with no exit action should look
        // like an empty case rather than like a missing concept.
        (void)fb;
    }

    void onEntry(AbortReason abort)
    {
        time_in_phase_s_ = 0.0;

        switch (phase_)
        {
            case Phase::kSettle:
                commanded_depth_ = 0.0;
                settle_force_sum_ = 0.0;
                settle_samples_ = 0;
                break;

            case Phase::kPreload:
                preload_hold_s_ = 0.0;
                break;

            case Phase::kSpotting:
                contact_hold_s_ = 0.0;
                contact_checked_ = false;
                break;

            case Phase::kDwell:
                // Freeze wherever the command happens to be; nothing more is
                // commanded until the retract.
                dwell_depth_m_ = commanded_depth_;
                break;

            case Phase::kRetract:
                if (abort_reason_ == AbortReason::kNone)
                {
                    abort_reason_ = abort;
                }
                break;

            case Phase::kEntry:
            case Phase::kDrilling:
            case Phase::kSlowFinish:
            case Phase::kDone:
                break;
        }
    }

    // -- 4. execution -----------------------------------------------------

    Command execute(const Feedback& fb)
    {
        Command command{};
        command.phase = phase_;
        command.finished = (phase_ == Phase::kDone);

        updatePreload(fb);

        switch (phase_)
        {
            case Phase::kSettle:
                // Exactly zero offset, so the first pose commanded to the
                // controller is bit-for-bit the pose the arm is already in.
                // The preload is still zero here, which is what makes the
                // residual below a measurement of the force sensor rather
                // than of the feedforward.
                commanded_depth_ = 0.0;
                settle_force_sum_ += fb.axial_force_n;
                ++settle_samples_;
                break;

            case Phase::kPreload:
                // Command held at the baseline while the feedforward comes up
                // underneath it. Nothing here moves the commanded point.
                commanded_depth_ = 0.0;
                break;

            case Phase::kSpotting:
                updateContactSearch(fb);
                command.limit = advance(fb);
                break;

            case Phase::kEntry:
            case Phase::kDrilling:
            case Phase::kSlowFinish:
                command.limit = advance(fb);
                break;

            case Phase::kDwell:
                commanded_depth_ = dwell_depth_m_;
                command.limit = Limit::kDepthTarget;
                break;

            case Phase::kRetract:
                command.limit = retract(fb);
                break;

            case Phase::kDone:
                commanded_depth_ = 0.0;
                break;
        }

        command.depth_m = commanded_depth_;
        command.lead_m = commanded_depth_ - fb.depth_m;
        command.preload_n = preload_applied_n_;
        return command;
    }

    /**
     * The feedforward is constant for the whole run, and this reports it.
     *
     * It cannot be anything else. setCartesianImpedanceDesiredTorque is a
     * pre-loop configuration call - every in-loop write is rejected by the
     * controller - so the force is live from the first cycle, through the
     * retract, until the loop stops and the caller clears it. There is no
     * ramp to own and no phase that can turn it off.
     *
     * Two consequences worth stating where they cannot be missed. The settle
     * residual is measured with the preload applied, so it is no longer a
     * clean calibration check. And the retract has to beat this force on top
     * of 19.7 N of reverse stiction - which it does only because the retract
     * command is driven to -30 mm, i.e. -90 N of spring, that swamps it.
     */
    void updatePreload(const Feedback& fb)
    {
        preload_applied_n_ = config_.preload_n;
        preload_hold_s_ += fb.dt_s;
    }

    /**
     * The whole feed law: chase `measured depth + scheduled lead`, rate
     * limited.
     *
     * Position is continuous under a rate limiter, so there is no step in
     * commanded force - only a force rate, and at 3000 N/m times 2.5 mm/s
     * that is 7.5 N/s, which a drill does not notice. That is why this
     * replaces the previous trapezoid-plus-jerk-limiter entirely; the
     * controller's own setFilterFrequency() low-pass rounds what is left.
     *
     * On the first cycle commanded_depth_ is zero and the step is one cycle
     * of feed rate - a few microns - so ramping from rest is a property of
     * the limiter rather than a special case anyone has to maintain.
     */
    Limit advance(const Feedback& fb)
    {
        const double scheduled_lead = leadForDepth(holeDepth(fb.depth_m));
        const bool lead_capped = scheduled_lead >= config_.lead_max_m - kLeadEpsilonM;

        const double target = fb.depth_m + scheduled_lead;
        const double feed_rate = feedRateForPhase();

        // The force hold freezes advance without touching retreat: an
        // overforce must always be able to relieve itself.
        const bool force_held = std::abs(fb.axial_force_n) > config_.axial_force_hold_n;
        const double max_forward = force_held ? 0.0 : feed_rate * fb.dt_s;
        const double max_backward = -config_.bleed_rate_m_per_s * fb.dt_s;

        const double requested = target - commanded_depth_;
        const double applied = std::clamp(requested, max_backward, max_forward);
        commanded_depth_ = std::max(0.0, commanded_depth_ + applied);

        if (force_held && requested > 0.0)
        {
            return Limit::kForceHold;
        }
        if (requested > max_forward)
        {
            return Limit::kFeedRate;
        }
        if (lead_capped)
        {
            return Limit::kLeadCap;
        }
        return Limit::kNone;
    }

    /**
     * Retract in two stages, then unwind.
     *
     * The commanded point is driven PAST the baseline while the tool is still
     * out, which is the part that is not obvious. The spring's restoring pull
     * is stiffness x distance-from-command, so it fades to nothing exactly as
     * the tool approaches the command. Against static friction the tool
     * therefore parks short, at distance friction/stiffness - measured on
     * 2026-09-22 as 6.56 mm, from 19.7 N of stiction against 3 N/mm. Stopping
     * the command at the baseline cannot bring the tool to the baseline, for
     * the same reason you overshoot any setpoint you are dragging against
     * friction.
     *
     * Once the tool is home the target returns to the baseline, so the run
     * ends with BOTH the tool and the command there and the pull unwound
     * gradually rather than released as a step.
     */
    Limit retract(const Feedback& fb)
    {
        if (std::abs(fb.depth_m) <= config_.retract_settle_m)
        {
            // Latched: without this the target would flip back and forth
            // every time noise nudged the tool across the tolerance band.
            retract_tool_home_ = true;
        }

        // Keep ramping the command negative for as long as the tool has not
        // arrived, then unwind to the baseline. Open-loop overshoot cannot
        // work here: the right amount is friction/stiffness, friction varies
        // with pose and direction, and guessing high drags the tool past the
        // baseline instead of stopping on it.
        const bool unwinding = retract_tool_home_ || time_in_phase_s_ >= config_.retract_timeout_sec;
        const double target = unwinding ? 0.0 : -config_.retract_command_limit_m;

        // While the command is still ahead of the tool, retreating only
        // bleeds off stored spring and the tool barely moves, so it can be
        // fast. Once the command is behind the tool it is pulling the bit out
        // of the hole, which is real motion and gets the slow rate.
        const double rate =
            commanded_depth_ > fb.depth_m ? config_.bleed_rate_m_per_s : config_.withdraw_rate_m_per_s;

        const double step = rate * fb.dt_s;
        commanded_depth_ += std::clamp(target - commanded_depth_, -step, step);
        return Limit::kFeedRate;
    }

    double feedRateForPhase() const
    {
        switch (phase_)
        {
            case Phase::kSpotting:
                return config_.spotting_feed_m_per_s;
            case Phase::kEntry:
                return config_.entry_feed_m_per_s;
            case Phase::kDrilling:
                return config_.drilling_feed_m_per_s;
            case Phase::kSlowFinish:
                return config_.slow_finish_feed_m_per_s;
            default:
                return 0.0;
        }
    }

    /**
     * Find the work, datum the hole to it, and refuse to go deeper if it was
     * never found.
     *
     * The failure this guards against is subtle, and it is the one the
     * previous test never got far enough to rule out. If the tool frame's +Z
     * is not the direction the bit physically points, then commanding an
     * advance still moves the tool along +Z and measured depth still climbs -
     * command and measurement flip together, so depth on its own says
     * nothing. What distinguishes the two cases is whether the tool STALLS:
     *
     *   pointed at the work   the bit touches down, stops following the
     *                         commanded point, and the lead opens up
     *   pointed at open air   the tool tracks the command indefinitely and
     *                         the lead stays near zero
     *
     * So the detector is the lead, not the force. It is the same physical
     * quantity - force is stiffness times lead - but read from position,
     * which on this arm is far quieter than the joint-transducer force
     * estimate, so no averaging or noise margin is needed.
     *
     * The instant the lead first opens is also the best estimate of first
     * touch, which is exactly the datum the hole depth should be measured
     * from. One mechanism, both jobs.
     */
    void updateContactSearch(const Feedback& fb)
    {
        if (!config_.require_contact || contact_checked_)
        {
            return;
        }

        if (!contact_found_ && (commanded_depth_ - fb.depth_m) >= config_.contact_datum_lead_m)
        {
            contact_found_ = true;
            contact_datum_m_ = fb.depth_m;
        }

        if (commanded_depth_ < config_.contact_check_arm_depth_m)
        {
            contact_hold_s_ = 0.0;
            return;
        }

        contact_hold_s_ += fb.dt_s;
        if (contact_hold_s_ < config_.contact_check_hold_sec)
        {
            return;
        }

        // The commanded lead is fully developed and has stayed there. If the
        // bit still has not stalled against anything, it is not going to:
        // either the tool axis is wrong or the standoff was larger than the
        // spotting lead, and going deeper is the wrong move in both cases.
        contact_checked_ = true;
        contact_check_failed_ = !contact_found_;
    }

    static constexpr double kDepthEpsilonM = 1e-6;
    static constexpr double kLeadEpsilonM = 1e-9;
    static constexpr double kPreloadEpsilonN = 1e-6;

    DrillConfig config_;

    Phase phase_ = Phase::kSettle;
    AbortReason abort_reason_ = AbortReason::kNone;

    double commanded_depth_ = 0.0;
    double time_in_phase_s_ = 0.0;
    double dwell_depth_m_ = 0.0;

    double settle_force_sum_ = 0.0;
    std::size_t settle_samples_ = 0;

    bool retract_tool_home_ = false;

    /// Feedforward force currently asked for, newtons. Ramped by
    /// updatePreload() and reported through Command::preload_n; this class
    /// never talks to the controller itself.
    double preload_applied_n_ = 0.0;
    double preload_hold_s_ = 0.0;

    double contact_hold_s_ = 0.0;
    bool contact_checked_ = false;
    bool contact_check_failed_ = false;

    /// Raw depth at which the bit was found to have stalled against the
    /// work. Hole depth is measured from here, not from the start pose.
    double contact_datum_m_ = 0.0;
    bool contact_found_ = false;
};

} // namespace cr20_drill

#endif // CR20_DRILL_CYCLE_HPP_
