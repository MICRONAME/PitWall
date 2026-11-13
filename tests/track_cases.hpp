#pragma once

#include <pitwall/track/lap_tracker.hpp>
#include <pitwall/simulation/vehicle.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace track_tests {
using namespace pitwall::track;
using namespace std::chrono_literals;
using pitwall::domain::Vec2;

inline void require(bool condition, std::string_view message) {
    if (!condition) { throw std::runtime_error(std::string{message}); }
}
inline void near(double actual, double expected, double tolerance, std::string_view message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            std::string{message} + ": expected " + std::to_string(expected) +
                ", got " + std::to_string(actual));
}
template <typename Function> void invalid(Function function) {
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("Expected invalid_argument");
}

inline Track square() {
    return Track{"square", {{0, 0}, {100, 0}, {100, 100}, {0, 100}}, 4.0};
}

struct Trace {
    Track track;
    LapTracker tracker;
    double progress{-2.0};
    std::chrono::microseconds time{};

    explicit Trace(Track value = square(), double max_displacement = 10.0)
        : track(std::move(value)), tracker(track, {}, max_displacement) {
        (void)tracker.update(track.pose_at(progress).position_m, time);
    }

    void move_to(double target, double speed_mps = 20.0) {
        while (std::abs(target - progress) > 1e-9) {
            const double delta = std::clamp(target - progress, -1.0, 1.0);
            progress += delta;
            time += std::chrono::microseconds{static_cast<std::chrono::microseconds::rep>(
                std::llround(std::abs(delta) / speed_mps * 1'000'000.0))};
            (void)tracker.update(track.pose_at(progress).position_m, time);
        }
    }

    void sample(Vec2 position, std::chrono::microseconds delta = 50ms) {
        time += delta;
        (void)tracker.update(position, time);
    }
};

inline void geometry() {
    const auto track = square();
    near(track.length_m(), 400.0, 0.0, "Closed perimeter");
    const auto start = track.pose_at(0.0);
    near(start.position_m.x, 50.0, 0.0, "Start at first edge midpoint");
    near(start.position_m.y, 0.0, 0.0, "Start Y");
    near(start.forward.x, 1.0, 0.0, "Driving direction");
    require(track.centerline().size() == 4, "Keep supplied geometry");
}

inline void projection() {
    const auto track = square();
    const auto result = track.project({75.0, 1.0});
    near(result.pose.progress_m, 25.0, 1e-12, "Distance from start");
    near(result.pose.position_m.x, 75.0, 1e-12, "Projection X");
    near(result.distance_to_centerline_m, 1.0, 1e-12, "Road distance");
    near(result.signed_lateral_offset_m, 1.0, 1e-12, "Left offset");
    require(result.on_track, "Inside road");
    near(track.project({75.0, -1.0}).signed_lateral_offset_m, -1.0, 1e-12, "Right offset");
}

inline void wrapping() {
    const auto track = square();
    for (const double progress : {-801.0, -400.0, -1.0, 0.0, 400.0, 801.0}) {
        const auto pose = track.pose_at(progress);
        require(pose.progress_m >= 0.0 && pose.progress_m < 400.0, "Canonical progress");
        const auto projected = track.project(pose.position_m);
        near(projected.pose.progress_m, pose.progress_m, 1e-10, "Wrapped round trip");
    }
}

inline void boundaries() {
    const auto track = square();
    require(track.project({50.0, 2.0}).on_track, "Boundary is included");
    require(!track.project({50.0, 2.001}).on_track, "Beyond boundary");
    require(!track.project({50.0, 50.0}).on_track, "Infield is outside");
}

inline void swept_shortcut() {
    const auto track = square();
    require(track.project({50, 0}).on_track && track.project({50, 100}).on_track,
            "Both shortcut endpoints are on road");
    require(!track.contains_motion({50, 0}, {50, 100}), "Infield crossing must be rejected");
    require(track.contains_motion({40, 0}, {60, 0}), "Straight road motion");
    require(track.contains_motion({40, 2}, {60, 2}), "Boundary motion");
}

inline void rounded_corner() {
    const auto track = square();
    require(track.project({101.0, -1.0}).on_track, "Endpoint disk rounds the corner");
    require(track.contains_motion({99, -1}, {101, 1}), "Motion covered by adjacent road capsules");
    require(track.contains_motion({100, 0}, {100, 0}), "Stationary on road");
    require(!track.contains_motion({50, 50}, {50, 50}), "Stationary infield");
}

inline void bad_geometry() {
    invalid([] { (void)Track{"", {{0, 0}, {10, 0}, {0, 10}}, 2}; });
    invalid([] { (void)Track{"x", {{0, 0}, {1, 1}}, 2}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 0}, {10, 10}, {0, 0}}, 2}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 10}, {0, 10}, {10, 0}}, 2}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 0}, {5, 0}, {0, 10}}, 2}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 0}, {0, 10}}, 0}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 0}, {0, 10}},
                             std::numeric_limits<double>::quiet_NaN()}; });
    invalid([] { (void)Track{"x", {{0, 0}, {10, 0}, {0, 2e6}}, 2}; });
}

inline void bad_queries() {
    const auto track = square();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    invalid([&] { (void)track.pose_at(nan); });
    invalid([&] { (void)track.project({nan, 0}); });
    invalid([&] { (void)track.contains_motion({0, 0}, {0, nan}); });
}

inline void oval_roundtrip() {
    const auto track = make_oval_track();
    require(track.centerline().size() == 64 && track.length_m() > 350.0 &&
                track.length_m() < 450.0, "Default oval dimensions");
    for (int i = 0; i < 100; ++i) {
        const double progress = track.length_m() * static_cast<double>(i) / 100.0;
        const auto result = track.project(track.pose_at(progress).position_m);
        near(result.distance_to_centerline_m, 0.0, 1e-10, "Oval centreline distance");
        near(result.pose.progress_m, progress, 1e-8, "Oval progress round trip");
    }
}

inline void wait_for_start() {
    Trace trace;
    require(!trace.tracker.status().running, "First sample must not start timing");
    trace.move_to(-1.0);
    require(!trace.tracker.status().running, "Still waiting before line");
    trace.move_to(1.0);
    require(trace.tracker.status().running && trace.tracker.completed_laps().empty(),
            "First forward crossing starts timing, not a completed lap");
}

inline void directed_start() {
    const auto track = square();
    LapTracker tracker{track};
    (void)tracker.update(track.pose_at(1).position_m, 0us);
    (void)tracker.update(track.pose_at(-1).position_m, 100ms);
    require(!tracker.status().running, "Backward start crossing cannot start a lap");
    (void)tracker.update(track.pose_at(1).position_m, 200ms);
    require(tracker.status().running, "Forward recrossing can start a lap");
}

inline void ordered_lap() {
    Trace trace;
    trace.move_to(101.0);
    require(trace.tracker.status().passed_checkpoints == 1, "First checkpoint");
    trace.move_to(201.0);
    require(trace.tracker.status().passed_checkpoints == 2, "Second checkpoint");
    trace.move_to(301.0);
    require(trace.tracker.status().passed_checkpoints == 3, "Third checkpoint");
    trace.move_to(401.0);
    require(trace.tracker.completed_laps().size() == 1, "One completed lap");
    const auto& lap = trace.tracker.completed_laps().front();
    require(lap.valid && lap.reason == LapInvalidReason::none, "Correct lap is valid");
    require(lap.duration == 20s && lap.number == 1, "Timed full lap");
    require(trace.tracker.status().current_lap_number == 2, "Next lap starts at finish");
}

inline void best_lap() {
    Trace trace;
    trace.move_to(400.0);
    trace.move_to(800.0, 40.0);
    require(trace.tracker.completed_laps().size() == 2, "Two laps");
    require(trace.tracker.best_lap_time() == 10s, "Best valid lap");
}

inline void off_track() {
    Trace trace;
    trace.move_to(5.0);
    trace.sample({55, 3});
    require(!trace.tracker.status().current_lap_valid &&
                trace.tracker.status().reason == LapInvalidReason::off_track, "Off-track invalidation");
    trace.sample(trace.track.pose_at(5).position_m);
    trace.move_to(401.0);
    require(!trace.tracker.completed_laps().front().valid, "Off-track lap stays invalid");
    require(!trace.tracker.best_lap_time(), "Invalid lap excluded from best");
}

inline void shortcut_lap() {
    Trace trace{square(), 200.0};
    trace.move_to(5.0);
    trace.sample({55, 100});
    require(trace.tracker.status().reason == LapInvalidReason::off_track,
            "Swept infield shortcut invalidates even with on-road endpoints");
}

inline void teleport() {
    Trace trace;
    trace.move_to(1.0);
    trace.sample(trace.track.pose_at(30).position_m);
    require(trace.tracker.status().reason == LapInvalidReason::discontinuity,
            "Large on-road jump invalidates lap");
}

inline void reverse_gate() {
    Trace trace;
    trace.move_to(101.0);
    trace.move_to(99.0);
    require(trace.tracker.status().reason == LapInvalidReason::wrong_direction,
            "Reverse checkpoint crossing invalidates");
}

inline void line_endpoints() {
    Trace trace;
    trace.move_to(0.0);
    require(trace.tracker.status().running, "Landing exactly on line starts once");
    trace.move_to(1.0);
    require(trace.tracker.completed_laps().empty(), "Leaving line forward must not complete a lap");
    trace.move_to(100.0);
    require(trace.tracker.status().passed_checkpoints == 1, "Landing on checkpoint");
    trace.move_to(99.0);
    require(trace.tracker.status().reason == LapInvalidReason::wrong_direction,
            "Leaving exact checkpoint backwards must be detected");
}

inline void missing_checkpoints() {
    Trace trace;
    trace.move_to(1.0);
    trace.sample(trace.track.pose_at(399.0).position_m);
    trace.progress = 399.0;
    trace.move_to(401.0);
    require(trace.tracker.completed_laps().size() == 1 &&
                !trace.tracker.completed_laps().front().valid, "Jumping past checkpoints cannot score");
    require(!trace.tracker.best_lap_time(), "Missing checkpoint lap excluded");
}

inline void recovery() {
    Trace trace;
    trace.move_to(5.0);
    trace.sample({55, 3});
    trace.sample(trace.track.pose_at(5.0).position_m);
    trace.move_to(401.0);
    require(!trace.tracker.completed_laps().front().valid, "First lap invalid");
    require(trace.tracker.status().current_lap_valid, "New lap recovers at legal finish");
    trace.move_to(801.0);
    require(trace.tracker.completed_laps().back().valid, "Second lap valid after recovery");
    require(trace.tracker.best_lap_time() == 20s, "Only recovered valid lap is best");
}

inline void reset() {
    Trace trace;
    trace.move_to(401.0);
    trace.tracker.reset();
    require(!trace.tracker.status().running && trace.tracker.completed_laps().empty() &&
                !trace.tracker.best_lap_time(), "Reset clears session timing and history");
    (void)trace.tracker.update(trace.track.pose_at(-1.0).position_m, 0us);
    (void)trace.tracker.update(trace.track.pose_at(1.0).position_m, 100ms);
    require(trace.tracker.status().running, "Reset accepts new clock epoch");
}

inline void bad_time_atomic() {
    Trace trace;
    trace.move_to(1.0);
    const auto before = trace.tracker.status();
    invalid([&] { (void)trace.tracker.update({0, 0}, trace.time); });
    invalid([&] { (void)trace.tracker.update({0, 0}, -1us); });
    invalid([&] { (void)trace.tracker.update({std::numeric_limits<double>::quiet_NaN(), 0},
                                           trace.time + 1ms); });
    const auto after = trace.tracker.status();
    require(before.elapsed == after.elapsed && after.current_lap_valid,
            "Rejected timestamp/position must not alter tracker");
    trace.move_to(401.0);
    require(trace.tracker.completed_laps().front().valid, "Continue after rejected sample");
}

inline void bad_rules() {
    invalid([] { (void)LapTracker{square(), {0.0}}; });
    invalid([] { (void)LapTracker{square(), {100.0, 100.0}}; });
    invalid([] { (void)LapTracker{square(), {200.0, 100.0}}; });
    invalid([] { (void)LapTracker{square(), {400.0}}; });
    invalid([] { (void)LapTracker{square(), {std::numeric_limits<double>::quiet_NaN()}}; });
    invalid([] { (void)LapTracker{square(), {}, 0.0}; });
}

inline void interpolated_time() {
    const auto track = square();
    LapTracker tracker{track};
    (void)tracker.update(track.pose_at(-1.0).position_m, 0us);
    const auto result = tracker.update(track.pose_at(3.0).position_m, 400ms);
    require(result.started && tracker.status().elapsed == 300ms,
            "Start timestamp is interpolated within sample interval");
}

inline void multiple_gates() {
    const auto track = square();
    LapTracker tracker{track, {1.0, 2.0, 3.0}};
    (void)tracker.update(track.pose_at(-1.0).position_m, 0us);
    (void)tracker.update(track.pose_at(4.0).position_m, 250ms);
    require(tracker.status().running && tracker.status().passed_checkpoints == 3 &&
                tracker.status().current_lap_valid, "Process gates in movement order inside one sample");
}

inline void high_timestamp() {
    const auto track = square();
    LapTracker tracker{track};
    const auto maximum = std::chrono::microseconds{
        std::numeric_limits<std::chrono::microseconds::rep>::max()};
    (void)tracker.update(track.pose_at(-1).position_m, 0us);
    (void)tracker.update(track.pose_at(1).position_m, maximum);
    require(tracker.status().running && tracker.status().elapsed.count() >= 0,
            "Interpolating a large timestamp must not overflow");
}

inline void vehicle_integration() {
    constexpr double radius = 30.0;
    std::vector<Vec2> vertices;
    for (int i = 0; i < 64; ++i) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) / 64.0;
        vertices.push_back({radius * std::cos(angle), radius * std::sin(angle)});
    }
    LapTracker tracker{Track{"physics_circle", std::move(vertices), 6.0}};
    pitwall::simulation::VehicleConfig config;
    config.drag_coefficient_n_per_mps2 = 0.0;
    config.rolling_resistance_n = 0.0;
    pitwall::simulation::VehicleModel model{config};
    pitwall::simulation::VehicleState state;
    constexpr double initial_angle = -0.1;
    state.position_m = {radius * std::cos(initial_angle), radius * std::sin(initial_angle)};
    state.speed_mps = 10.0;
    state.heading_rad = initial_angle + std::numbers::pi / 2.0;
    state.steering_rad = std::atan(config.wheelbase_m / radius);
    const pitwall::simulation::ControlInput controls{0.0, 0.0,
                                                     state.steering_rad / config.max_steering_rad};
    (void)tracker.update(state.position_m, 0us);
    for (int tick = 1; tick <= 4000; ++tick) {
        state = model.step(state, controls, 10ms);
        (void)tracker.update(state.position_m, 10ms * tick);
    }
    require(tracker.completed_laps().size() == 2, "Physical model completes two laps");
    for (const auto& lap : tracker.completed_laps()) {
        require(lap.valid, "Actual vehicle positions stay inside road and pass all gates");
        near(std::chrono::duration<double>(lap.duration).count(),
             2.0 * std::numbers::pi * radius / state.speed_mps, 1e-4,
             "Physical circle lap time matches analytic period");
    }
}

struct Case { std::string_view name; void (*run)(); };
inline constexpr std::array cases{
    Case{"geometry", geometry}, Case{"projection", projection}, Case{"wrapping", wrapping},
    Case{"boundaries", boundaries}, Case{"swept_shortcut", swept_shortcut},
    Case{"rounded_corner", rounded_corner}, Case{"bad_geometry", bad_geometry},
    Case{"bad_queries", bad_queries}, Case{"oval_roundtrip", oval_roundtrip},
    Case{"wait_for_start", wait_for_start}, Case{"directed_start", directed_start},
    Case{"ordered_lap", ordered_lap}, Case{"best_lap", best_lap}, Case{"off_track", off_track},
    Case{"shortcut_lap", shortcut_lap}, Case{"teleport", teleport}, Case{"reverse_gate", reverse_gate},
    Case{"line_endpoints", line_endpoints}, Case{"missing_checkpoints", missing_checkpoints},
    Case{"recovery", recovery}, Case{"reset", reset}, Case{"bad_time_atomic", bad_time_atomic},
    Case{"bad_rules", bad_rules}, Case{"interpolated_time", interpolated_time},
    Case{"multiple_gates", multiple_gates}, Case{"high_timestamp", high_timestamp},
    Case{"vehicle_integration", vehicle_integration}};

} // namespace track_tests
