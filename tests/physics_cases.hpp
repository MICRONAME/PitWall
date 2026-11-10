#pragma once

#include <pitwall/simulation/fixed_step.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace physics_tests {
using namespace pitwall::simulation;
using namespace std::chrono_literals;

inline void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

inline void near(double actual, double expected, double tolerance, std::string_view message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            std::string{message} + ": expected " + std::to_string(expected) +
                ", got " + std::to_string(actual));
}

template <typename Function> void invalid(Function function) {
    try {
        function();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("Expected std::invalid_argument");
}

inline VehicleConfig ideal_config() {
    VehicleConfig config;
    config.mass_kg = 1000.0;
    config.engine_force_n = 2000.0;
    config.brake_force_n = 10000.0;
    config.drag_coefficient_n_per_mps2 = 0.0;
    config.rolling_resistance_n = 0.0;
    return config;
}

inline void rest() {
    VehicleModel model;
    VehicleState state;
    for (int i = 0; i < 100; ++i) {
        state = model.step(state, {}, 10ms);
    }
    near(state.speed_mps, 0.0, 0.0, "Rest speed");
    near(state.distance_m, 0.0, 0.0, "Rest distance");
    near(state.position_m.x, 0.0, 0.0, "Rest position");
    state = model.step(state, {0.0, 1.0, 1.0}, 10ms);
    near(state.speed_mps, 0.0, 0.0, "Braking at rest");
    near(state.heading_rad, 0.0, 0.0, "Steering at rest must not rotate vehicle");
}

inline void analytic_acceleration() {
    VehicleModel model{ideal_config()};
    VehicleState state;
    for (int i = 0; i < 100; ++i) {
        state = model.step(state, {1.0, 0.0, 0.0}, 10ms);
    }
    near(state.speed_mps, 2.0, 1e-12, "v=a*t");
    near(state.position_m.x, 1.0, 1e-12, "x=a*t*t/2");
    near(state.position_m.y, 0.0, 0.0, "Straight path Y");
    near(state.longitudinal_acceleration_mps2, 2.0, 1e-12, "F/m");
}

inline void braking() {
    VehicleModel model{ideal_config()};
    VehicleState state;
    state.speed_mps = 10.0;
    for (int i = 0; i < 200; ++i) {
        state = model.step(state, {0.0, 1.0, 0.0}, 10ms);
    }
    near(state.speed_mps, 0.0, 0.0, "No reverse after stopping");
    near(state.distance_m, 5.0, 1e-10, "Analytic stopping distance");
}

inline void partial_tick_stop() {
    VehicleModel model{ideal_config()};
    VehicleState state;
    state.speed_mps = 0.03;
    state = model.step(state, {0.0, 1.0, 0.0}, 10ms);
    near(state.speed_mps, 0.0, 0.0, "Stop inside a tick");
    near(state.distance_m, 0.000045, 1e-15, "Integrate only until stop");
}

inline void resistance() {
    VehicleModel model;
    VehicleState state;
    state.speed_mps = 20.0;
    const auto next = model.step(state, {}, 10ms);
    require(next.speed_mps < state.speed_mps, "Coasting should decelerate");
    const double expected = -(140.0 + 0.42 * 400.0) / 1200.0;
    near(next.longitudinal_acceleration_mps2, expected, 1e-10, "Rolling plus air drag");
}

inline void speed_limit() {
    auto config = ideal_config();
    config.max_speed_mps = 1.005;
    VehicleModel model{config};
    VehicleState state;
    for (int i = 0; i < 100; ++i) {
        state = model.step(state, {1.0, 0.0, 0.0}, 10ms);
    }
    near(state.speed_mps, config.max_speed_mps, 0.0, "Speed clamp");
    const double time_to_limit = config.max_speed_mps / 2.0;
    near(state.distance_m, time_to_limit * time_to_limit +
                               config.max_speed_mps * (1.0 - time_to_limit),
         1e-12, "Integrate the partial tick at speed limit");
    near(state.longitudinal_acceleration_mps2, 0.0, 0.0, "Acceleration at speed cap");
}

inline void turn_symmetry() {
    VehicleModel model{ideal_config()};
    VehicleState left;
    left.speed_mps = 10.0;
    VehicleState right = left;
    for (int i = 0; i < 100; ++i) {
        left = model.step(left, {0.0, 0.0, 0.5}, 10ms);
        right = model.step(right, {0.0, 0.0, -0.5}, 10ms);
    }
    require(left.position_m.y > 0.0 && right.position_m.y < 0.0, "Turn direction");
    near(left.position_m.x, right.position_m.x, 1e-12, "Mirror X");
    near(left.position_m.y, -right.position_m.y, 1e-12, "Mirror Y");
    near(left.heading_rad, -right.heading_rad, 1e-12, "Mirror heading");
}

inline void analytic_circle() {
    auto config = ideal_config();
    config.max_lateral_acceleration_mps2 = 1000.0;
    VehicleModel model{config};
    VehicleState state;
    state.speed_mps = 10.0;
    state.steering_rad = 0.2;
    for (int i = 0; i < 100; ++i) {
        state = model.step(state, {0.0, 0.0, 0.2 / config.max_steering_rad}, 10ms);
    }
    const double radius = config.wheelbase_m / std::tan(0.2);
    const double heading = 10.0 / radius;
    near(state.heading_rad, heading, 1e-12, "Circle heading");
    near(state.position_m.x, radius * std::sin(heading), 1e-11, "Circle X");
    near(state.position_m.y, radius * (1.0 - std::cos(heading)), 1e-11, "Circle Y");
    near(state.distance_m, 10.0, 1e-12, "Arc length");
}

inline void steering_rate() {
    VehicleModel model{ideal_config()};
    const auto next = model.step({}, {0.0, 0.0, 1.0}, 10ms);
    near(next.steering_rad, model.config().steering_rate_rad_per_s * 0.01, 1e-12,
         "Steering rate limit");
}

inline void grip_limit() {
    VehicleModel model{ideal_config()};
    VehicleState state;
    state.speed_mps = 60.0;
    for (int i = 0; i < 100; ++i) {
        state = model.step(state, {0.0, 0.0, 1.0}, 10ms);
        const double lateral = state.speed_mps * state.speed_mps *
                               std::abs(std::tan(state.steering_rad)) / model.config().wheelbase_m;
        require(lateral <= model.config().max_lateral_acceleration_mps2 + 1e-10,
                "High speed curvature bound");
    }
}

inline void bad_controls() {
    VehicleModel model;
    for (const auto input : {ControlInput{-0.1, 0.0, 0.0}, ControlInput{1.1, 0.0, 0.0},
                             ControlInput{0.0, -0.1, 0.0}, ControlInput{0.0, 1.1, 0.0},
                             ControlInput{0.0, 0.0, -1.1}, ControlInput{0.0, 0.0, 1.1}}) {
        invalid([&] { (void)model.step({}, input, 10ms); });
    }
    for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()}) {
        invalid([&] { (void)model.step({}, {value, 0.0, 0.0}, 10ms); });
        invalid([&] { (void)model.step({}, {0.0, value, 0.0}, 10ms); });
        invalid([&] { (void)model.step({}, {0.0, 0.0, value}, 10ms); });
    }
}

inline void bad_config() {
    constexpr std::array members{
        &VehicleConfig::mass_kg, &VehicleConfig::wheelbase_m, &VehicleConfig::engine_force_n,
        &VehicleConfig::brake_force_n, &VehicleConfig::drag_coefficient_n_per_mps2,
        &VehicleConfig::rolling_resistance_n, &VehicleConfig::max_speed_mps,
        &VehicleConfig::max_steering_rad, &VehicleConfig::steering_rate_rad_per_s,
        &VehicleConfig::max_lateral_acceleration_mps2};
    for (const auto member : members) {
        auto config = ideal_config();
        config.*member = std::numeric_limits<double>::quiet_NaN();
        invalid([&] { (void)VehicleModel{config}; });
        config.*member = -1.0;
        invalid([&] { (void)VehicleModel{config}; });
    }
    auto config = ideal_config();
    config.mass_kg = 0.0;
    invalid([&] { (void)VehicleModel{config}; });
    config = ideal_config();
    config.max_steering_rad = std::numbers::pi / 2.0;
    invalid([&] { (void)VehicleModel{config}; });
    config = ideal_config();
    config.mass_kg = std::numeric_limits<double>::min();
    invalid([&] { (void)VehicleModel{config}; });
}

inline void bad_state_and_duration() {
    VehicleModel model;
    VehicleState state;
    state.speed_mps = -1.0;
    invalid([&] { (void)model.step(state, {}, 10ms); });
    state.speed_mps = model.config().max_speed_mps + 1.0;
    invalid([&] { (void)model.step(state, {}, 10ms); });
    state = {};
    state.position_m.x = std::numeric_limits<double>::infinity();
    invalid([&] { (void)model.step(state, {}, 10ms); });
    for (const auto duration : {0us, -1us, 50'001us}) {
        invalid([&] { (void)model.step({}, {}, duration); });
    }
}

inline FixedStepSimulation run_at_fps(std::int64_t fps) {
    FixedStepSimulation simulation;
    for (std::int64_t frame = 0; frame < fps * 4; ++frame) {
        const auto start_us = frame * 1'000'000 / fps;
        const auto end_us = (frame + 1) * 1'000'000 / fps;
        const auto result = simulation.advance(std::chrono::microseconds{end_us - start_us},
                                               {0.7, 0.0, 0.1});
        require(result.dropped_time == 0us, "No overload in normal frame schedule");
    }
    return simulation;
}

inline void fps_independence() {
    const auto slow = run_at_fps(30);
    const auto fast = run_at_fps(144);
    require(slow.tick_count() == 400 && fast.tick_count() == 400, "100 Hz tick count");
    near(slow.state().position_m.x, fast.state().position_m.x, 0.0, "FPS independent X");
    near(slow.state().position_m.y, fast.state().position_m.y, 0.0, "FPS independent Y");
    near(slow.state().speed_mps, fast.state().speed_mps, 0.0, "FPS independent speed");
    near(slow.state().heading_rad, fast.state().heading_rad, 0.0, "FPS independent heading");
}

inline void accumulator() {
    FixedStepSimulation simulation;
    auto result = simulation.advance(6ms, {1.0, 0.0, 0.0});
    require(result.steps == 0, "Partial tick must wait");
    near(result.interpolation_alpha, 0.6, 1e-15, "First remainder");
    result = simulation.advance(7ms, {1.0, 0.0, 0.0});
    require(result.steps == 1 && simulation.simulation_time() == 10ms, "Accumulated tick");
    near(result.interpolation_alpha, 0.3, 1e-15, "Preserve leftover time");
}

inline void overload() {
    FixedStepSimulation simulation;
    const auto result = simulation.advance(1005ms, {1.0, 0.0, 0.0});
    require(result.steps == 8, "Catch-up work is bounded");
    require(result.dropped_time == 920ms, "Report discarded whole ticks");
    require(simulation.simulation_time() == 80ms, "Clock counts actual simulated time");
    near(result.interpolation_alpha, 0.5, 1e-15, "Keep fractional time on overload");
    invalid([] { (void)FixedStepSimulation{{}, 0}; });
    invalid([] { (void)FixedStepSimulation{{}, 1001}; });
}

inline void reset() {
    FixedStepSimulation simulation;
    (void)simulation.advance(25ms, {1.0, 0.0, 0.5});
    simulation.reset();
    require(simulation.tick_count() == 0 && simulation.simulation_time() == 0us, "Reset clock");
    near(simulation.state().distance_m, 0.0, 0.0, "Reset distance");
    require(simulation.telemetry(2).throttle == 0.0, "Reset controls");
    const auto result = simulation.advance(5ms, {});
    require(result.steps == 0, "Reset accumulator");
}

inline void telemetry() {
    FixedStepSimulation simulation;
    (void)simulation.advance(20ms, {0.5, 0.0, 0.2});
    const auto sample = simulation.telemetry(42);
    require(sample.session_id == 42 && sample.sequence == 2 && sample.session_time == 20ms,
            "Telemetry identity and tick timestamp");
    near(sample.speed_mps, simulation.state().speed_mps, 0.0, "Telemetry speed");
    near(sample.steering_rad, simulation.state().steering_rad, 0.0, "Applied steering angle");
    near(sample.throttle, 0.5, 0.0, "Last simulated input");
    require(!pitwall::domain::validate(sample), "Simulation emits valid domain data");
    invalid([&] { (void)simulation.telemetry(0); });
}

inline void invalid_frame_is_atomic() {
    FixedStepSimulation simulation;
    (void)simulation.advance(15ms, {1.0, 0.0, 0.0});
    const auto before = simulation.telemetry(1);
    invalid([&] { (void)simulation.advance(-1us, {}); });
    invalid([&] { (void)simulation.advance(61s, {}); });
    invalid([&] { (void)simulation.advance(0us, {2.0, 0.0, 0.0}); });
    const auto after = simulation.telemetry(1);
    require(before.sequence == after.sequence && before.session_time == after.session_time,
            "Rejected frame must not modify clock");
    near(before.position_m.x, after.position_m.x, 0.0, "Rejected frame must not modify state");
    const auto result = simulation.advance(5ms, {});
    require(result.steps == 1, "Rejected frame must preserve remainder");
}

inline void heading_wrap_and_long_run() {
    VehicleModel model{ideal_config()};
    VehicleState state;
    state.speed_mps = 5.0;
    for (int i = 0; i < 10'000; ++i) {
        state = model.step(state, {0.0, 0.0, 1.0}, 10ms);
        require(std::abs(state.heading_rad) <= std::numbers::pi, "Heading normalized");
        require(std::isfinite(state.position_m.x) && std::isfinite(state.position_m.y),
                "Long run must remain finite");
    }
}

struct Case {
    std::string_view name;
    void (*run)();
};

inline constexpr std::array cases{
    Case{"rest", rest}, Case{"analytic_acceleration", analytic_acceleration},
    Case{"braking", braking}, Case{"partial_tick_stop", partial_tick_stop},
    Case{"resistance", resistance}, Case{"speed_limit", speed_limit},
    Case{"turn_symmetry", turn_symmetry}, Case{"analytic_circle", analytic_circle},
    Case{"steering_rate", steering_rate}, Case{"grip_limit", grip_limit},
    Case{"bad_controls", bad_controls}, Case{"bad_config", bad_config},
    Case{"bad_state_and_duration", bad_state_and_duration},
    Case{"fps_independence", fps_independence}, Case{"accumulator", accumulator},
    Case{"overload", overload}, Case{"reset", reset}, Case{"telemetry", telemetry},
    Case{"invalid_frame_is_atomic", invalid_frame_is_atomic},
    Case{"heading_wrap_and_long_run", heading_wrap_and_long_run}};

} // namespace physics_tests
