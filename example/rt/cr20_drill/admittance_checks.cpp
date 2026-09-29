/**
 * @file admittance_checks.cpp
 * @brief Off-robot verification for admittance_feed.hpp.
 *
 * No SDK, no controller, no threads: the feed is driven against a 1-D plant
 * along the bit axis. The plant is the model from the design doc
 * (docs/drill-anchor-control-design.md, section 5):
 *
 *   - the SDS bit hangs slide_m out of the chuck and slides back at zero
 *     force, so first touch and seating are different points;
 *   - once seated, push = K * (command - hole bottom), a spring;
 *   - the hole bottom advances at cut_rate * (push - cut_threshold);
 *   - the controller reports fz = -push (world-on-robot), plus an offset and
 *     noise.
 *
 * The concrete model is invented. These checks prove the feed's structure -
 * signs, phases, limits, aborts, the steady-state formula - not whether the
 * numbers suit real concrete. That is what the on-robot ladder is for.
 *
 *     ./cr20_admittance_checks   ->  "ALL CHECKS PASSED", exit 0
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "admittance_feed.hpp"

using cr20_drill::admittance::AbortReason;
using cr20_drill::admittance::AdmittanceConfig;
using cr20_drill::admittance::AdmittanceFeed;
using cr20_drill::admittance::Command;
using cr20_drill::admittance::Feedback;
using cr20_drill::admittance::kCyclePeriodSec;
using cr20_drill::admittance::kFx;
using cr20_drill::admittance::kFy;
using cr20_drill::admittance::kFz;
using cr20_drill::admittance::kTz;
using cr20_drill::admittance::Wrench;
using cr20_drill::admittance::Phase;
using cr20_drill::admittance::phaseName;

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
    if (std::abs(actual - expected) > tolerance)
    {
        ++g_failures;
        std::printf("  FAIL  %s: got %.6g, expected %.6g +- %.3g\n", what.c_str(), actual, expected, tolerance);
    }
}

/// The 1-D world along the bit axis. Positions are in the command's
/// coordinate: s = 0 at the baseline, + into the work.
struct Plant
{
    double stiffness_n_per_m = 2.0e5;
    double standoff_m = 0.005;        // gap from the extended bit tip to the face
    double slide_m = 0.010;           // SDS float
    double cut_rate_m_per_n_s = 7.5e-5; // ~2.7 mm/s at ~46 N
    double cut_threshold_n = 10.0;
    double fz_offset_n = 0.0;

    /// Constant offsets on the other axes, e.g. the tool's weight, which
    /// cartesianPosition does not compensate.
    double fx_offset_n = 0.0;
    double fy_offset_n = 0.0;
    double tz_offset_nm = 0.0;

    /// Hammer vibration on fx: a zero-mean sine. The 2026-09-28 holes shook
    /// fx by about 27 N sd; the hammer runs at about 70-80 Hz.
    double fx_vibration_amplitude_n = 0.0;
    double vibration_hz = 75.0;

    double fz_noise_sd_n = 0.0;
    bool wrong_sign = false;          // report fz = +push instead of -push

    /// Hole depth (from the face) past which the material stops cutting.
    double rebar_at_hole_depth_m = 1e9;

    /// How fast the bit still cuts once at the rebar. Zero is a bit that
    /// makes no progress; a masonry bit grinding on steel creeps.
    double rebar_cut_rate_m_per_n_s = 0.0;

    /// When set, a bit that has stopped cutting is also held in the hole, so
    /// retracting pulls against it once the slide is used up.
    bool jam_when_stalled = false;

    // --- state ---
    double hole_bottom_m = 0.0; // seated-TCP coordinate of the hole bottom
    double push_n = 0.0;
    double time_s = 0.0;
    std::mt19937 rng{12345};

    void reset()
    {
        hole_bottom_m = standoff_m + slide_m;
        push_n = 0.0;
        time_s = 0.0;
    }

    double faceM() const
    {
        return standoff_m + slide_m;
    }

    bool atRebar() const
    {
        return hole_bottom_m - faceM() >= rebar_at_hole_depth_m;
    }

    /// Advance one cycle with the command at s; returns the wrench as the
    /// controller would report it.
    Wrench step(double s)
    {
        const double compression = s - hole_bottom_m;
        const double tension = hole_bottom_m - slide_m - s;

        if (compression > 0.0)
        {
            push_n = stiffness_n_per_m * compression;
        }
        else if (jam_when_stalled && atRebar() && tension > 0.0)
        {
            push_n = -stiffness_n_per_m * tension;
        }
        else
        {
            push_n = 0.0;
        }

        const double cut_rate = atRebar() ? rebar_cut_rate_m_per_n_s : cut_rate_m_per_n_s;
        if (push_n > cut_threshold_n)
        {
            hole_bottom_m += cut_rate * (push_n - cut_threshold_n) * kCyclePeriodSec;
        }

        double noise = 0.0;
        if (fz_noise_sd_n > 0.0)
        {
            std::normal_distribution<double> dist{0.0, fz_noise_sd_n};
            noise = dist(rng);
        }
        const double reported_push = push_n + noise;
        Wrench wrench{};
        wrench[kFz] = (wrong_sign ? reported_push : -reported_push) + fz_offset_n;
        wrench[kFx] = fx_offset_n + fx_vibration_amplitude_n * std::sin(2.0 * M_PI * vibration_hz * time_s);
        wrench[kFy] = fy_offset_n;
        wrench[kTz] = tz_offset_nm;
        time_s += kCyclePeriodSec;
        return wrench;
    }
};

struct Row
{
    double t_s;
    Phase phase;
    double depth_m;
    double feed_m_per_s;
    double push_control_n;
    double push_fast_n;
    double true_push_n;
};

using Hook = std::function<void(double t_s, const Command& last, Feedback& fb)>;

std::vector<Row> run(AdmittanceFeed& feed, Plant& plant, double max_time_s, const Hook& hook = nullptr)
{
    plant.reset();
    std::vector<Row> rows;
    Command last{};
    double t = 0.0;
    while (t < max_time_s)
    {
        Feedback fb{};
        fb.wrench = plant.step(last.depth_m);
        fb.measured_depth_m = last.depth_m;
        if (hook)
        {
            hook(t, last, fb);
        }
        last = feed.step(fb);
        rows.push_back(Row{t, last.phase, last.depth_m, last.feed_m_per_s, last.push_control_n, last.push_fast_n,
                           plant.push_n});
        t += kCyclePeriodSec;
        if (last.finished)
        {
            break;
        }
    }
    return rows;
}

bool visited(const std::vector<Row>& rows, Phase phase)
{
    return std::any_of(rows.begin(), rows.end(), [&](const Row& r) { return r.phase == phase; });
}

double firstTimeIn(const std::vector<Row>& rows, Phase phase)
{
    for (const Row& r : rows)
    {
        if (r.phase == phase)
        {
            return r.t_s;
        }
    }
    return -1.0;
}

/// Mean true push over the middle of the cut phase, well away from the
/// contact transient and the switch to hold.
double steadyCutPush(const std::vector<Row>& rows, double* mean_feed = nullptr)
{
    std::vector<const Row*> cut;
    for (const Row& r : rows)
    {
        if (r.phase == Phase::kCut)
        {
            cut.push_back(&r);
        }
    }
    if (cut.size() < 10)
    {
        return std::nan("");
    }
    const std::size_t from = cut.size() / 2;
    const std::size_t to = cut.size() * 9 / 10;
    double push = 0.0;
    double feed = 0.0;
    for (std::size_t i = from; i < to; ++i)
    {
        push += cut[i]->true_push_n;
        feed += cut[i]->feed_m_per_s;
    }
    const double n = static_cast<double>(to - from);
    if (mean_feed)
    {
        *mean_feed = feed / n;
    }
    return push / n;
}

/// Closed-form steady push: v = V + (F - p)/B and v = c (p - F0). If that
/// feed would exceed max_feed, the clamp binds instead and the push is
/// whatever cuts at exactly max_feed: p = F0 + max_feed / c.
double predictedSteadyPush(const AdmittanceConfig& config, const Plant& plant)
{
    const double b = config.damping_n_s_per_m;
    const double c = plant.cut_rate_m_per_n_s;
    const double unclamped =
        (config.feed_bias_m_per_s * b + config.desired_push_n + b * c * plant.cut_threshold_n) / (1.0 + b * c);
    const double clamped = plant.cut_threshold_n + config.max_feed_m_per_s / c;
    return c * (unclamped - plant.cut_threshold_n) > config.max_feed_m_per_s ? clamped : unclamped;
}

// ---------------------------------------------------------------------------

void checkFreeAir()
{
    std::printf("free-air stroke\n");
    AdmittanceConfig config;
    config.free_air = true;
    Plant plant;
    plant.standoff_m = 1.0; // nothing to touch
    plant.fz_noise_sd_n = 5.0;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 60.0);

    double max_depth = 0.0;
    double max_feed = 0.0;
    for (const Row& r : rows)
    {
        max_depth = std::max(max_depth, r.depth_m);
        max_feed = std::max(max_feed, r.feed_m_per_s);
    }

    check(rows.back().phase == Phase::kDone, "free air finishes");
    check(feed.abortReason() == AbortReason::kNone, "free air: no abort (noise alone never trips anything)");
    check(visited(rows, Phase::kHold) && !visited(rows, Phase::kCut), "free air: approach -> hold, never cut");
    checkNear(max_feed, config.approach_feed_m_per_s, 1e-9, "free air: approaches at approach_feed");
    checkNear(max_depth, config.free_air_stroke_m, 0.0002, "free air: turns round at the stroke");
    checkNear(rows.back().depth_m, 0.0, 1e-5, "free air: returns to the baseline");
}

void checkUnexpectedContact()
{
    std::printf("free air, but something is there\n");
    AdmittanceConfig config;
    config.free_air = true;
    Plant plant;
    plant.standoff_m = 0.002;
    plant.slide_m = 0.0;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 60.0);
    check(feed.abortReason() == AbortReason::kUnexpectedContact, "free air: contact aborts as unexpected_contact");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "free air: then retracts home");
}

void checkHole(double desired_push_n, double feed_bias_m_per_s)
{
    std::printf("hole at F_des = %.0f N, V_des = %.1f mm/s\n", desired_push_n, feed_bias_m_per_s * 1000.0);
    AdmittanceConfig config;
    config.desired_push_n = desired_push_n;
    config.feed_bias_m_per_s = feed_bias_m_per_s;
    Plant plant;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);
    const std::string tag = " (F_des " + std::to_string(static_cast<int>(desired_push_n)) + ")";

    check(rows.back().phase == Phase::kDone, "hole finishes" + tag);
    check(feed.abortReason() == AbortReason::kNone, "hole: no abort" + tag);
    check(feed.contactFound(), "hole: contact found" + tag);

    // Contact at SEATING: past the standoff by the whole slide, not at first touch.
    checkNear(feed.contactDepth(), plant.faceM(), 0.0005, "hole: contact is at seating, not first touch" + tag);

    double mean_feed = 0.0;
    const double push = steadyCutPush(rows, &mean_feed);
    checkNear(push, predictedSteadyPush(config, plant), 1.0,
              "hole: steady push = (V B + F_des + B c F0) / (1 + B c)" + tag);
    check(mean_feed <= config.max_feed_m_per_s + 1e-12, "hole: steady feed within max_feed" + tag);

    // Reaches the target measured from contact.
    double deepest = 0.0;
    for (const Row& r : rows)
    {
        deepest = std::max(deepest, r.depth_m);
    }
    checkNear(deepest - feed.contactDepth(), config.hole_depth_m, 0.0005, "hole: depth from contact" + tag);
    checkNear(rows.back().depth_m, 0.0, 1e-5, "hole: retracts to the baseline" + tag);

    // Push positive under compression: the sign is right end to end.
    bool sign_ok = true;
    for (const Row& r : rows)
    {
        if (r.phase == Phase::kCut && r.true_push_n > 40.0 && r.push_control_n <= 0.0)
        {
            sign_ok = false;
        }
    }
    check(sign_ok, "hole: compressed plant reads positive push" + tag);
}

void checkTare()
{
    std::printf("tare\n");
    AdmittanceConfig config;
    Plant clean;
    Plant offset;
    offset.fz_offset_n = 8.0;
    offset.fz_noise_sd_n = 5.0;

    AdmittanceFeed feed_clean{config};
    AdmittanceFeed feed_offset{config};
    const double push_clean = steadyCutPush(run(feed_clean, clean, 120.0));
    const double push_offset = steadyCutPush(run(feed_offset, offset, 120.0));

    checkNear(feed_offset.wrenchTare()[kFz], 8.0, 0.5, "tare: settle measures the +8 N offset");
    checkNear(push_offset, push_clean, 1.0, "tare: offset and noise leave the steady push unchanged");
}

void checkToolWeight()
{
    std::printf("tool weight on every axis\n");
    AdmittanceConfig config;

    // The free-air run of 2026-09-25 at rest: fz +14.3 N, fy -21.9 N, i.e.
    // the tool's full weight. Untared, fy alone is a quarter of the lateral
    // abort. Tared, it must vanish.
    Plant plant;
    plant.fz_offset_n = 14.3;
    plant.fy_offset_n = -21.9;
    plant.tz_offset_nm = 0.7;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);
    checkNear(feed.wrenchTare()[kFy], -21.9, 1e-6, "weight: fy tared");
    checkNear(feed.wrenchTare()[kTz], 0.7, 1e-6, "weight: tz tared");
    check(feed.abortReason() == AbortReason::kNone && rows.back().phase == Phase::kDone,
          "weight: a full hole completes with the weight present");

    // A real lateral load still trips the abort once the weight is tared out:
    // 90 N extra on fy for 0.2 s, 1 s after contact.
    AdmittanceFeed loaded{config};
    double cut_started = -1.0;
    run(loaded, plant, 120.0, [&](double t, const Command& last, Feedback& fb) {
        if (last.phase == Phase::kCut && cut_started < 0.0)
        {
            cut_started = t;
        }
        if (cut_started >= 0.0 && t > cut_started + 1.0 && t < cut_started + 1.2)
        {
            fb.wrench[kFy] += 90.0;
        }
    });
    check(loaded.abortReason() == AbortReason::kLateralForce, "weight: a real 90 N side load still aborts");
}

void checkHardContact()
{
    std::printf("hard contact (nothing cuts)\n");
    AdmittanceConfig config;
    Plant plant;
    plant.rebar_at_hole_depth_m = 0.0; // stops cutting at the face

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);

    const double contact_t = firstTimeIn(rows, Phase::kCut);
    double peak = 0.0;
    double settled = 0.0;
    for (const Row& r : rows)
    {
        if (r.phase != Phase::kCut)
        {
            continue;
        }
        peak = std::max(peak, r.true_push_n);
        if (r.t_s <= contact_t + 1.5)
        {
            settled = r.true_push_n;
        }
    }
    check(peak < 1.10 * config.desired_push_n, "hard contact: overshoot under 10% of F_des");
    checkNear(settled, config.desired_push_n, 2.0, "hard contact: push settles at F_des within 1.5 s");
    check(feed.abortReason() == AbortReason::kStall, "hard contact: stall abort once the feed stops");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "hard contact: then retracts home");
}

void checkMeasuredOperatingPoint()
{
    std::printf("measured operating point (touch-off K, B = 1e4)\n");

    // The touch-off measured K = 2.2-2.8e4 N/m. Hard contact must stay
    // overdamped across that range, and still settle without ringing at 4x
    // it, in case the real wall or the running spindle is stiffer.
    for (const double stiffness : {2.2e4, 2.8e4, 1.0e5})
    {
        AdmittanceConfig config;
        config.damping_n_s_per_m = 1.0e4;
        Plant plant;
        plant.stiffness_n_per_m = stiffness;
        plant.rebar_at_hole_depth_m = 0.0;

        AdmittanceFeed feed{config};
        const auto rows = run(feed, plant, 120.0);
        const double contact_t = firstTimeIn(rows, Phase::kCut);
        double peak = 0.0;
        double settled = 0.0;
        for (const Row& r : rows)
        {
            if (r.phase != Phase::kCut)
            {
                continue;
            }
            peak = std::max(peak, r.true_push_n);
            if (r.t_s <= contact_t + 1.8)
            {
                settled = r.true_push_n;
            }
        }
        const std::string tag = " (K " + std::to_string(static_cast<int>(stiffness)) + " N/m)";
        check(peak < 1.10 * config.desired_push_n, "operating point: overshoot under 10%" + tag);
        checkNear(settled, config.desired_push_n, 2.0, "operating point: settles at F_des within 1.8 s" + tag);
    }

    // And while cutting at the measured K, the shortfall is B * v_cut.
    AdmittanceConfig config;
    config.damping_n_s_per_m = 1.0e4;
    Plant plant;
    plant.stiffness_n_per_m = 2.5e4;
    AdmittanceFeed feed{config};
    const double push = steadyCutPush(run(feed, plant, 120.0));
    checkNear(push, predictedSteadyPush(config, plant), 1.0, "operating point: steady cutting push matches formula (feed clamp binding)");
}

void checkRebarMidHole()
{
    std::printf("rebar mid-hole\n");
    AdmittanceConfig config;
    Plant plant;
    plant.rebar_at_hole_depth_m = 0.008;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);
    check(feed.abortReason() == AbortReason::kStall, "rebar: stall abort");
    check(!visited(rows, Phase::kHold), "rebar: never reached the target");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "rebar: retracts home");
}

void checkRebarCreep()
{
    std::printf("rebar the bit creeps through\n");

    // A masonry bit on steel still grinds forward at ~0.1-0.2 mm/s. That is
    // a stall too: without it the feed holds F_des until the watchdog.
    AdmittanceConfig config;
    config.damping_n_s_per_m = 1.0e4;
    Plant plant;
    plant.stiffness_n_per_m = 2.5e4;
    plant.rebar_at_hole_depth_m = 0.008;
    plant.rebar_cut_rate_m_per_n_s = 1.8e-6; // 0.16 mm/s at 100 N

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);
    check(feed.abortReason() == AbortReason::kStall, "rebar creep: stall abort");
    check(!visited(rows, Phase::kHold), "rebar creep: never reached the target");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "rebar creep: retracts home");
    check(rows.back().t_s < 30.0, "rebar creep: done well before the watchdog");
}

void checkHammerVibration()
{
    std::printf("hammer vibration on the lateral axes\n");

    // Zero-mean shaking with peaks about the 136 N single samples of the
    // 2026-09-28 lateral aborts, on top of a 5 N real side load, must not
    // trip the lateral abort. Taking the magnitude before filtering
    // rectified this into a false trip (from about 120 N of shaking).
    AdmittanceConfig config;
    config.damping_n_s_per_m = 1.0e4;
    Plant shaken;
    shaken.stiffness_n_per_m = 2.5e4;
    shaken.fx_offset_n = 5.0;
    shaken.fx_vibration_amplitude_n = 150.0;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, shaken, 120.0);
    check(feed.abortReason() == AbortReason::kNone && rows.back().phase == Phase::kDone,
          "vibration: a full hole completes under 150 N of fx shaking");

    // The same shaking with a real 90 N side load on fx still aborts.
    AdmittanceFeed loaded{config};
    double cut_started = -1.0;
    run(loaded, shaken, 120.0, [&](double t, const Command& last, Feedback& fb) {
        if (last.phase == Phase::kCut && cut_started < 0.0)
        {
            cut_started = t;
        }
        if (cut_started >= 0.0 && t > cut_started + 1.0 && t < cut_started + 1.2)
        {
            fb.wrench[kFx] += 90.0;
        }
    });
    check(loaded.abortReason() == AbortReason::kLateralForce, "vibration: a real 90 N side load still aborts");
}

void checkJammedRetract()
{
    std::printf("jammed retract\n");
    AdmittanceConfig config;
    Plant plant;
    // The measured chain, not the 2e5 default. The pull overshoot past the
    // cap is K * (v * tau_filter + v^2 / 2a): at the 15 mm/s retract it is
    // ~9 N here but ~69 N at 2e5 N/m. Accepted on the assumption a jammed
    // bit stays near the buffer's stiffness (2026-09-29).
    plant.stiffness_n_per_m = 2.5e4;
    plant.rebar_at_hole_depth_m = 0.008;
    plant.jam_when_stalled = true;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);

    double peak_pull = 0.0;
    for (const Row& r : rows)
    {
        peak_pull = std::max(peak_pull, -r.true_push_n);
    }
    check(feed.abortReason() == AbortReason::kStall, "jam: the stall is recorded first");
    check(rows.back().phase == Phase::kDone, "jam: finishes");
    check(rows.back().depth_m > 0.0, "jam: stops in the hole rather than pulling home");
    check(peak_pull > config.retract_pull_cap_n, "jam: the pull cap was what stopped it");
    check(peak_pull < config.retract_pull_cap_n + 20.0, "jam: pull overshoots the cap by under 20 N");
    check(rows.back().feed_m_per_s == 0.0, "jam: command frozen at the end");
}

void checkWrongSign()
{
    std::printf("wrong force sign\n");
    AdmittanceConfig config;

    // Once against concrete that cuts, once against a face that doesn't.
    for (const double rebar_at : {1e9, 0.0})
    {
        Plant plant;
        plant.wrong_sign = true;
        plant.rebar_at_hole_depth_m = rebar_at;

        AdmittanceFeed feed{config};
        const auto rows = run(feed, plant, 120.0);

        double peak = 0.0;
        for (const Row& r : rows)
        {
            peak = std::max(peak, r.true_push_n);
        }
        const std::string tag = rebar_at > 1.0 ? " (cutting)" : " (hard face)";
        // With the sign flipped, push reads negative under compression, so
        // contact is never seen; the reversed-push guard must catch it.
        check(feed.abortReason() == AbortReason::kReversedPush, "wrong sign: reversed_push abort" + tag);
        check(peak < 2.0 * config.reversed_push_abort_n, "wrong sign: true push stays under 50 N" + tag);
        check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "wrong sign: retracts home" + tag);
    }
}

void checkOverload()
{
    std::printf("overload\n");
    AdmittanceConfig config;
    config.desired_push_n = 140.0; // steady push near the 150 N abort
    Plant plant;
    plant.rebar_at_hole_depth_m = 0.0;
    plant.fz_noise_sd_n = 10.0;

    // A burst of load the law cannot shed: an extra 60 N for 0.2 s, 1 s after
    // contact, stands in for a bit that grabs.
    AdmittanceFeed feed{config};
    double cut_started = -1.0;
    const auto rows = run(feed, plant, 120.0, [&](double t, const Command& last, Feedback& fb) {
        if (last.phase == Phase::kCut && cut_started < 0.0)
        {
            cut_started = t;
        }
        if (cut_started >= 0.0 && t > cut_started + 1.0 && t < cut_started + 1.2)
        {
            fb.wrench[kFz] -= 60.0;
        }
    });
    check(feed.abortReason() == AbortReason::kAxialForce, "overload: axial force abort");
    check(rows.back().phase == Phase::kDone, "overload: finishes");
}

void checkNoContact()
{
    std::printf("no contact\n");
    AdmittanceConfig config;
    Plant plant;
    plant.standoff_m = 0.100;

    AdmittanceFeed feed{config};
    const auto rows = run(feed, plant, 120.0);
    check(feed.abortReason() == AbortReason::kNoContact, "no contact: aborts");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "no contact: retracts home");
}

void checkOperatorAbort()
{
    std::printf("operator abort mid-cut\n");
    AdmittanceConfig config;
    Plant plant;

    AdmittanceFeed feed{config};
    double cut_started = -1.0;
    const auto rows = run(feed, plant, 120.0, [&](double t, const Command& last, Feedback& fb) {
        if (last.phase == Phase::kCut && cut_started < 0.0)
        {
            cut_started = t;
        }
        fb.operator_abort = cut_started >= 0.0 && t > cut_started + 1.0;
    });
    check(feed.abortReason() == AbortReason::kOperator, "operator: abort recorded");
    check(!visited(rows, Phase::kHold), "operator: never reached hold");
    check(rows.back().phase == Phase::kDone && rows.back().depth_m <= 1e-5, "operator: retracts home");
}

void checkLimitsEveryCycle()
{
    std::printf("limits on every cycle\n");
    AdmittanceConfig config;

    std::vector<Plant> plants(4);
    plants[1].fz_noise_sd_n = 10.0;
    plants[2].rebar_at_hole_depth_m = 0.005;
    plants[3].rebar_at_hole_depth_m = 0.005;
    plants[3].jam_when_stalled = true;

    bool feed_ok = true;
    bool accel_ok = true;
    for (Plant& plant : plants)
    {
        AdmittanceFeed feed{config};
        const auto rows = run(feed, plant, 120.0);
        for (std::size_t i = 1; i < rows.size(); ++i)
        {
            const Row& r = rows[i];
            if (r.phase == Phase::kCut &&
                (r.feed_m_per_s > config.max_feed_m_per_s + 1e-12 ||
                 r.feed_m_per_s < -config.max_backoff_m_per_s - 1e-12))
            {
                feed_ok = false;
            }
            const double step = std::abs(r.feed_m_per_s - rows[i - 1].feed_m_per_s);
            if (step > config.stop_accel_m_per_s2 * kCyclePeriodSec + 1e-12)
            {
                accel_ok = false;
            }
        }
    }
    check(feed_ok, "limits: cut feed within [-max_backoff, max_feed]");
    check(accel_ok, "limits: velocity step never exceeds the stop deceleration");
}

} // namespace

int main()
{
    checkFreeAir();
    checkUnexpectedContact();
    checkHole(100.0, 0.0);
    checkHole(50.0, 0.0);
    checkHole(50.0, 0.0005);
    checkTare();
    checkToolWeight();
    checkHardContact();
    checkMeasuredOperatingPoint();
    checkRebarMidHole();
    checkRebarCreep();
    checkHammerVibration();
    checkJammedRetract();
    checkWrongSign();
    checkOverload();
    checkNoContact();
    checkOperatorAbort();
    checkLimitsEveryCycle();

    if (g_failures == 0)
    {
        std::printf("\nALL CHECKS PASSED (%d)\n", g_checks);
        return 0;
    }
    std::printf("\n%d of %d CHECKS FAILED\n", g_failures, g_checks);
    return 1;
}
