/**
 * @file admittance_feed.hpp
 * @brief The position-mode admittance drill feed, with no SDK, robot, Eigen
 * or threading dependency - plain scalars only.
 *
 * Design: docs/drill-anchor-control-design.md. This file is sections 3 and 6
 * of that document, and nothing else.
 *
 * HOW THE FEED WORKS
 *
 * Every command lies on one line, fixed at the baseline:
 *
 *     P_cmd = advanceAlongBit(P0, s)
 *
 * so the five non-feed axes are held by the position servos, and this file
 * decides only the scalar s. It does that by integrating a velocity:
 *
 *     push = kFzToPushSign * (fz - fz_tare)          newtons, + = pushing on the work
 *
 * (every wrench axis is tared over the settle window, not just fz)
 *     v    = V_des + (F_des - push) / B              the whole admittance law
 *     s   += v * dt
 *
 * with v clamped and acceleration-limited. s is the controller's own state:
 * it is integrated from the previous COMMAND, never from the measured pose.
 * Under load the command runs ahead of the tool by F/K, and that gap is the
 * push - restarting from the measured pose each cycle would throw it away.
 *
 * With V_des = 0 the feed comes only from force error, so while cutting the
 * steady push is F_des - B * v_cut, not F_des. That is deliberate for the
 * first holes; see the design doc, section 5.
 */

#ifndef CR20_DRILL_ADMITTANCE_FEED_HPP_
#define CR20_DRILL_ADMITTANCE_FEED_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace cr20_drill
{
namespace admittance
{

/// Fixed control period. The controller consumes one pose per cycle, so the
/// integration uses this rather than the measured dt, which jitters around it.
constexpr double kCyclePeriodSec = 0.001;

/// tauExt_inStiff is the force the WORLD applies on the ROBOT, in tool axes,
/// +Z along the bit into the hole. Pushing into the work therefore reads as
/// NEGATIVE fz. Established from the impedance-mode logs (fz against lead has
/// slope -3.13 N/mm against a +3 N/mm spring); see the design doc, section 4.
/// Every conversion from fz to push goes through this one constant.
constexpr double kFzToPushSign = -1.0;

/// Close enough to the baseline to call the retract home.
constexpr double kHomeToleranceM = 1e-6;

enum class Phase
{
    kSettle = 0,
    kApproach,
    kCut,
    kHold,
    kRetract,
    kDone,
};

inline const char* phaseName(Phase phase)
{
    switch (phase)
    {
        case Phase::kSettle:
            return "settle";
        case Phase::kApproach:
            return "approach";
        case Phase::kCut:
            return "cut";
        case Phase::kHold:
            return "hold";
        case Phase::kRetract:
            return "retract";
        case Phase::kDone:
            return "done";
    }
    return "unknown";
}

/// Why the run stopped advancing. kNone means it did not. Only the first
/// reason is kept.
enum class AbortReason
{
    kNone = 0,
    kOperator,
    kWatchdog,
    kNoContact,
    kUnexpectedContact,
    kAxialForce,
    kLateralForce,
    kReactionTorque,
    kLateralExcursion,
    kAxisDrift,
    kStall,
    kRetractPull,
    kReversedPush,
};

inline const char* abortReasonName(AbortReason reason)
{
    switch (reason)
    {
        case AbortReason::kNone:
            return "none";
        case AbortReason::kOperator:
            return "operator";
        case AbortReason::kWatchdog:
            return "watchdog";
        case AbortReason::kNoContact:
            return "no_contact";
        case AbortReason::kUnexpectedContact:
            return "unexpected_contact";
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
        case AbortReason::kStall:
            return "stall";
        case AbortReason::kRetractPull:
            return "retract_pull";
        case AbortReason::kReversedPush:
            return "reversed_push";
    }
    return "unknown";
}

/**
 * @struct AdmittanceConfig
 * @brief Every knob, with the defaults for the first real hole.
 *
 * The rationale for each default is in the plan that introduced this file and
 * in the design doc. The short version is on each field.
 */
struct AdmittanceConfig
{
    // --- run shape ------------------------------------------------------

    /// Free-air stroke instead of a hole: approach for free_air_stroke_m,
    /// hold, retract. Any contact aborts. The first run on a new setup.
    bool free_air = false;
    double free_air_stroke_m = 0.010;

    /// Blind-hole depth, measured from the CONTACT DEPTH (bit seated), not
    /// from the start pose.
    double hole_depth_m = 0.015;

    /// Tare window. Averages the ~5 N per-sample noise to well under 1 N.
    double settle_sec = 2.0;

    /// Time at the bottom before retracting, so the command-to-hole gap
    /// (F/K) can cut out.
    double hold_sec = 1.0;

    // --- approach and contact -------------------------------------------

    /// Plain velocity until contact. Decoupled from F_des on purpose.
    double approach_feed_m_per_s = 0.002;

    /// Travel past this with no contact means something is wrong. Covers the
    /// standoff plus the ~10 mm SDS slide plus margin.
    double max_approach_m = 0.030;

    /// Contact is declared when fast-filtered push crosses this - at bit
    /// SEATING, not first touch, because the SDS bit slides back into the
    /// chuck at almost zero force first.
    double contact_push_n = 25.0;

    // --- the admittance law ---------------------------------------------

    /// F_des. The push at ZERO feed; while cutting, push settles at
    /// F_des - B * v_cut when V_des is zero.
    double desired_push_n = 100.0;

    /// V_des. Zero for the first holes: pure force regulation.
    double feed_bias_m_per_s = 0.0;

    /// B. Chosen from B >= 2 * K * tau_f; holds zeta >= 0.7 for chain
    /// stiffness K up to ~1.9e5 N/m at the 3 Hz control filter.
    double damping_n_s_per_m = 2.0e4;

    /// Feed ceiling: just above the ~2.7 mm/s the real holes cut at.
    double max_feed_m_per_s = 0.003;

    /// Backoff ceiling when push exceeds F_des. Slow, so the bit stays near
    /// the face rather than pulling off and re-impacting.
    double max_backoff_m_per_s = 0.001;

    /// Normal acceleration limit: keeps the pose stream smooth and stops a
    /// single noisy sample stepping the velocity. 0 -> 3 mm/s in 30 ms, so it
    /// stays faster than the loop itself (B/K = 0.1 s at K = 2e5 N/m). At
    /// 20 mm/s^2 it was slower, and hard contact overshot F_des by 11%.
    double max_accel_m_per_s2 = 0.100;

    /// Deceleration used only to freeze the command on a retract abort: the
    /// retract stops in ~4 ms rather than ~20 ms, because against a jam every
    /// extra millimetre of pull adds K * 1 mm of tension.
    double stop_accel_m_per_s2 = 0.500;

    /// Control filter on push. Far below the ~70-80 Hz hammer; sets tau_f.
    double control_filter_hz = 3.0;

    /// Abort and contact filter on push and lateral force: reacts in ~10 ms
    /// but ignores single-sample spikes.
    double abort_filter_hz = 20.0;

    /// Reaction-torque filter time constant. Raw tz swings +-10 Nm sample to
    /// sample under the spindle; 100 ms is what the impedance test settled on.
    double torque_filter_sec = 0.100;

    // --- retract --------------------------------------------------------

    double retract_feed_m_per_s = 0.002;

    /// Pull ceiling. The jammed retract of 2026-09-24 twisted the tool at
    /// 123 N; this is half that. Exceeding it freezes the command - stop,
    /// don't pull harder.
    double retract_pull_cap_n = 60.0;

    // --- aborts ---------------------------------------------------------

    /// Cut-phase feed below this for stall_sec is a stall, not slow cutting.
    double stall_feed_m_per_s = 0.0001;
    double stall_sec = 2.0;

    /// On the MAGNITUDE of fast-filtered push, so it also catches a sign
    /// error (the law driving into the work while push reads negative).
    double axial_force_abort_n = 150.0;

    /// Push reading this far NEGATIVE while advancing. Drilling cannot pull
    /// the tool forward, so this is a sign error (or a wrong frame). Without
    /// it, a flipped sign never registers contact and the approach just keeps
    /// driving until max_approach_m.
    double reversed_push_abort_n = 25.0;

    double lateral_force_abort_n = 80.0;
    /// On tared, 100 ms-filtered tz. 20 rather than drill_test's 15: in
    /// position mode tz reads ~5 Nm of joint friction just from moving (free
    /// air, 2026-09-25), on top of the drill's 4.9 Nm mean / 9 Nm peak.
    double reaction_torque_abort_nm = 20.0;
    double lateral_excursion_abort_m = 0.015;
    double axis_drift_abort_rad = 0.0873; // 5 degrees
};

/// Index of each component in a wrench: [fx, fy, fz, tx, ty, tz].
constexpr int kFx = 0;
constexpr int kFy = 1;
constexpr int kFz = 2;
constexpr int kTz = 5;

using Wrench = std::array<double, 6>;

/// Everything the feed needs from one control cycle. Geometry comes from
/// toolDeltaBetween(baseline, measured) and bitAxisAngleRad in the robot
/// program.
struct Feedback
{
    /// Raw tauExt_inStiff [fx, fy, fz, tx, ty, tz], tool axes, as the
    /// controller reports it. In cartesianPosition this still carries the
    /// tool's full weight (26.3 N measured against m*g = 26.6 N, free-air run
    /// 2026-09-25), which is why every axis is tared, not just fz.
    Wrench wrench{};

    /// Measured travel along the bit axis since the baseline. Logged, and
    /// used for nothing that moves the arm.
    double measured_depth_m = 0.0;

    double lateral_m = 0.0;
    double axis_drift_rad = 0.0;


    bool operator_abort = false;
    bool watchdog_expired = false;
};

/// What the robot program should do this cycle.
struct Command
{
    /// s: commanded travel along the bit axis from the baseline.
    double depth_m = 0.0;
    double feed_m_per_s = 0.0;

    /// Tared push, control-filtered and abort-filtered.
    double push_control_n = 0.0;
    double push_fast_n = 0.0;

    Phase phase = Phase::kSettle;
    bool finished = false;
};

/**
 * @class AdmittanceFeed
 * @brief One step() per 1 ms cycle; returns the commanded depth.
 */
class AdmittanceFeed
{
  public:
    explicit AdmittanceFeed(const AdmittanceConfig& config) : config_(config)
    {
    }

    Command step(const Feedback& fb)
    {
        time_in_phase_s_ += kCyclePeriodSec;
        updateFilters(fb);

        switch (phase_)
        {
            case Phase::kSettle:
                stepSettle(fb);
                break;
            case Phase::kApproach:
                stepApproach(fb);
                break;
            case Phase::kCut:
                stepCut(fb);
                break;
            case Phase::kHold:
                stepHold(fb);
                break;
            case Phase::kRetract:
                stepRetract(fb);
                break;
            case Phase::kDone:
                target_feed_ = 0.0;
                feed_ = 0.0;
                break;
        }

        integrate();

        Command command;
        command.depth_m = commanded_depth_;
        command.feed_m_per_s = feed_;
        command.push_control_n = push_control_;
        command.push_fast_n = push_fast_;
        command.phase = phase_;
        command.finished = phase_ == Phase::kDone;
        return command;
    }

    Phase phase() const
    {
        return phase_;
    }

    AbortReason abortReason() const
    {
        return abort_reason_;
    }

    /// Mean raw wrench over the settle window, subtracted from every axis
    /// afterwards. Valid for the whole run because orientation is locked, so
    /// the tool's weight never changes direction in tool axes.
    const Wrench& wrenchTare() const
    {
        return wrench_tare_;
    }

    bool contactFound() const
    {
        return contact_found_;
    }

    double contactDepth() const
    {
        return contact_depth_;
    }

    /// Hole depth for a given commanded depth: zero until contact.
    double holeDepth(double commanded_depth_m) const
    {
        return contact_found_ ? std::max(0.0, commanded_depth_m - contact_depth_) : 0.0;
    }

    const AdmittanceConfig& config() const
    {
        return config_;
    }

  private:
    // --- phases ---------------------------------------------------------

    void stepSettle(const Feedback& fb)
    {
        target_feed_ = 0.0;
        if (fb.operator_abort || fb.watchdog_expired)
        {
            abort_reason_ = fb.operator_abort ? AbortReason::kOperator : AbortReason::kWatchdog;
            enterPhase(Phase::kDone);
            return;
        }

        for (int i = 0; i < 6; ++i)
        {
            wrench_sum_[i] += fb.wrench[i];
        }
        ++wrench_samples_;
        if (time_in_phase_s_ >= config_.settle_sec)
        {
            for (int i = 0; i < 6; ++i)
            {
                wrench_tare_[i] = wrench_sum_[i] / wrench_samples_;
            }
            tared_ = true;
            push_control_ = 0.0;
            push_fast_ = 0.0;
            enterPhase(Phase::kApproach);
        }
    }

    void stepApproach(const Feedback& fb)
    {
        target_feed_ = config_.approach_feed_m_per_s;
        if (checkAdvancingAborts(fb))
        {
            return;
        }

        const bool touching = push_fast_ > config_.contact_push_n;

        if (config_.free_air)
        {
            if (touching)
            {
                abortToRetract(AbortReason::kUnexpectedContact);
            }
            else if (commanded_depth_ >= config_.free_air_stroke_m)
            {
                enterPhase(Phase::kHold);
            }
            return;
        }

        if (touching)
        {
            contact_found_ = true;
            contact_depth_ = commanded_depth_;
            enterPhase(Phase::kCut);
        }
        else if (commanded_depth_ >= config_.max_approach_m)
        {
            abortToRetract(AbortReason::kNoContact);
        }
    }

    void stepCut(const Feedback& fb)
    {
        // THE admittance law.
        target_feed_ = config_.feed_bias_m_per_s +
                       (config_.desired_push_n - push_control_) / config_.damping_n_s_per_m;

        if (checkAdvancingAborts(fb))
        {
            return;
        }

        stall_time_s_ = feed_ < config_.stall_feed_m_per_s ? stall_time_s_ + kCyclePeriodSec : 0.0;
        if (stall_time_s_ >= config_.stall_sec)
        {
            abortToRetract(AbortReason::kStall);
            return;
        }

        if (commanded_depth_ >= contact_depth_ + config_.hole_depth_m)
        {
            enterPhase(Phase::kHold);
        }
    }

    void stepHold(const Feedback& fb)
    {
        target_feed_ = 0.0;
        if (checkAdvancingAborts(fb))
        {
            return;
        }
        if (time_in_phase_s_ >= config_.hold_sec)
        {
            enterPhase(Phase::kRetract);
        }
    }

    void stepRetract(const Feedback& fb)
    {
        if (frozen_)
        {
            // A retract abort: stop hard, then finish once stopped.
            target_feed_ = 0.0;
            if (feed_ == 0.0)
            {
                enterPhase(Phase::kDone);
            }
            return;
        }

        // Full retract speed, slowing into the baseline at the normal
        // acceleration limit so the stream never steps.
        const double braking_speed = std::sqrt(2.0 * config_.max_accel_m_per_s2 * std::max(0.0, commanded_depth_));
        target_feed_ = -std::min(config_.retract_feed_m_per_s, braking_speed);

        // Stop, don't pull harder: on a jam the pull only grows.
        AbortReason reason = AbortReason::kNone;
        if (-push_fast_ > config_.retract_pull_cap_n)
        {
            reason = AbortReason::kRetractPull;
        }
        else if (fb.axis_drift_rad > config_.axis_drift_abort_rad)
        {
            reason = AbortReason::kAxisDrift;
        }
        else if (fb.lateral_m > config_.lateral_excursion_abort_m)
        {
            reason = AbortReason::kLateralExcursion;
        }
        if (reason != AbortReason::kNone)
        {
            recordAbort(reason);
            frozen_ = true;
            target_feed_ = 0.0;
            return;
        }

        if (commanded_depth_ <= kHomeToleranceM)
        {
            target_feed_ = 0.0;
            if (std::abs(feed_) <= config_.max_accel_m_per_s2 * kCyclePeriodSec)
            {
                enterPhase(Phase::kDone);
            }
        }
    }

    // --- aborts ---------------------------------------------------------

    /// Checks shared by approach, cut and hold. Any of them sends the run to
    /// retract. Returns true when one fired.
    bool checkAdvancingAborts(const Feedback& fb)
    {
        AbortReason reason = AbortReason::kNone;
        if (fb.operator_abort)
        {
            reason = AbortReason::kOperator;
        }
        else if (fb.watchdog_expired)
        {
            reason = AbortReason::kWatchdog;
        }
        else if (std::abs(push_fast_) > config_.axial_force_abort_n)
        {
            reason = AbortReason::kAxialForce;
        }
        else if (push_fast_ < -config_.reversed_push_abort_n)
        {
            reason = AbortReason::kReversedPush;
        }
        else if (lateral_force_fast_ > config_.lateral_force_abort_n)
        {
            reason = AbortReason::kLateralForce;
        }
        else if (std::abs(reaction_torque_filtered_) > config_.reaction_torque_abort_nm)
        {
            reason = AbortReason::kReactionTorque;
        }
        else if (fb.lateral_m > config_.lateral_excursion_abort_m)
        {
            reason = AbortReason::kLateralExcursion;
        }
        else if (fb.axis_drift_rad > config_.axis_drift_abort_rad)
        {
            reason = AbortReason::kAxisDrift;
        }

        if (reason == AbortReason::kNone)
        {
            return false;
        }
        abortToRetract(reason);
        return true;
    }

    void abortToRetract(AbortReason reason)
    {
        recordAbort(reason);
        enterPhase(Phase::kRetract);
        target_feed_ = -config_.retract_feed_m_per_s;
    }

    void recordAbort(AbortReason reason)
    {
        if (abort_reason_ == AbortReason::kNone)
        {
            abort_reason_ = reason;
        }
    }

    // --- plumbing -------------------------------------------------------

    void enterPhase(Phase phase)
    {
        phase_ = phase;
        time_in_phase_s_ = 0.0;
        stall_time_s_ = 0.0;
    }

    static double onePoleAlpha(double time_constant_s)
    {
        return kCyclePeriodSec / (time_constant_s + kCyclePeriodSec);
    }

    static double timeConstantForHz(double hz)
    {
        return 1.0 / (2.0 * M_PI * hz);
    }

    void updateFilters(const Feedback& fb)
    {
        // Before the tare exists, the forces are meaningless: hold them at zero.
        Wrench tared{};
        if (tared_)
        {
            for (int i = 0; i < 6; ++i)
            {
                tared[i] = fb.wrench[i] - wrench_tare_[i];
            }
        }
        const double push_raw = kFzToPushSign * tared[kFz];
        const double lateral_force = std::hypot(tared[kFx], tared[kFy]);

        push_control_ += onePoleAlpha(timeConstantForHz(config_.control_filter_hz)) * (push_raw - push_control_);
        const double fast_alpha = onePoleAlpha(timeConstantForHz(config_.abort_filter_hz));
        push_fast_ += fast_alpha * (push_raw - push_fast_);
        lateral_force_fast_ += fast_alpha * (lateral_force - lateral_force_fast_);
        reaction_torque_filtered_ +=
            onePoleAlpha(config_.torque_filter_sec) * (tared[kTz] - reaction_torque_filtered_);
    }

    void integrate()
    {
        double target = target_feed_;
        if (phase_ == Phase::kCut)
        {
            target = std::clamp(target, -config_.max_backoff_m_per_s, config_.max_feed_m_per_s);
        }

        const double accel = frozen_ ? config_.stop_accel_m_per_s2 : config_.max_accel_m_per_s2;
        const double max_step = accel * kCyclePeriodSec;
        feed_ += std::clamp(target - feed_, -max_step, max_step);
        commanded_depth_ += feed_ * kCyclePeriodSec;
    }

    AdmittanceConfig config_;

    Phase phase_ = Phase::kSettle;
    AbortReason abort_reason_ = AbortReason::kNone;
    double time_in_phase_s_ = 0.0;

    double commanded_depth_ = 0.0;
    double feed_ = 0.0;
    double target_feed_ = 0.0;
    bool frozen_ = false;

    Wrench wrench_sum_{};
    int wrench_samples_ = 0;
    Wrench wrench_tare_{};
    bool tared_ = false;

    double push_control_ = 0.0;
    double push_fast_ = 0.0;
    double lateral_force_fast_ = 0.0;
    double reaction_torque_filtered_ = 0.0;

    bool contact_found_ = false;
    double contact_depth_ = 0.0;
    double stall_time_s_ = 0.0;
};

} // namespace admittance
} // namespace cr20_drill

#endif // CR20_DRILL_ADMITTANCE_FEED_HPP_
