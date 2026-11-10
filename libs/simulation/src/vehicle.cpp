#include <pitwall/simulation/vehicle.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace pitwall::simulation {
namespace {

void check_config(const VehicleConfig& c) {
    const std::array values{c.mass_kg, c.wheelbase_m, c.engine_force_n, c.brake_force_n,
                            c.drag_coefficient_n_per_mps2, c.rolling_resistance_n,
                            c.max_speed_mps, c.max_steering_rad, c.steering_rate_rad_per_s,
                            c.max_lateral_acceleration_mps2};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("Vehicle configuration must be finite");
        }
    }
    if (c.mass_kg <= 0.0 || c.wheelbase_m <= 0.0 || c.engine_force_n < 0.0 ||
        c.brake_force_n < 0.0 || c.drag_coefficient_n_per_mps2 < 0.0 ||
        c.rolling_resistance_n < 0.0 || c.max_speed_mps <= 0.0 ||
        c.max_steering_rad <= 0.0 || c.max_steering_rad >= std::numbers::pi / 2.0 ||
        c.steering_rate_rad_per_s <= 0.0 || c.max_lateral_acceleration_mps2 <= 0.0) {
        throw std::invalid_argument("Invalid vehicle parameter range");
    }
    // Guard derived expressions, not merely the individual finite inputs.
    const double max_force = c.engine_force_n + c.brake_force_n + c.rolling_resistance_n +
                             c.drag_coefficient_n_per_mps2 * c.max_speed_mps * c.max_speed_mps;
    if (!std::isfinite(max_force / c.mass_kg) ||
        !std::isfinite(std::tan(c.max_steering_rad) / c.wheelbase_m) ||
        !std::isfinite(c.max_speed_mps * c.max_speed_mps)) {
        throw std::invalid_argument("Vehicle parameter combinations overflow");
    }
}

bool finite_state(const VehicleState& s) {
    return std::isfinite(s.position_m.x) && std::isfinite(s.position_m.y) &&
           std::isfinite(s.speed_mps) && std::isfinite(s.heading_rad) &&
           std::isfinite(s.steering_rad) &&
           std::isfinite(s.longitudinal_acceleration_mps2) && std::isfinite(s.distance_m);
}

} // namespace

void validate_controls(const ControlInput& input) {
    if (!std::isfinite(input.throttle) || !std::isfinite(input.brake) ||
        !std::isfinite(input.steering) || input.throttle < 0.0 || input.throttle > 1.0 ||
        input.brake < 0.0 || input.brake > 1.0 || input.steering < -1.0 ||
        input.steering > 1.0) {
        throw std::invalid_argument("Controls must be finite: pedals [0,1], steering [-1,1]");
    }
}

VehicleModel::VehicleModel(VehicleConfig config) : config_(config) {
    check_config(config_);
}

VehicleState VehicleModel::step(const VehicleState& state, const ControlInput& input,
                                std::chrono::microseconds duration) const {
    validate_controls(input);
    if (!finite_state(state) || state.speed_mps < 0.0 ||
        state.speed_mps > config_.max_speed_mps || state.distance_m < 0.0 ||
        std::abs(state.steering_rad) > config_.max_steering_rad) {
        throw std::invalid_argument("Invalid vehicle state");
    }
    if (duration.count() <= 0 || duration > std::chrono::milliseconds{50}) {
        throw std::invalid_argument("Physics step must be in (0, 50ms]");
    }
    const double dt = std::chrono::duration<double>(duration).count();
    const double v = state.speed_mps;
    const double driving_force = input.throttle * config_.engine_force_n;
    const double resistance = input.brake * config_.brake_force_n +
                              config_.rolling_resistance_n +
                              config_.drag_coefficient_n_per_mps2 * v * v;
    const double acceleration = (driving_force - resistance) / config_.mass_kg;
    const double next_speed = std::clamp(v + acceleration * dt, 0.0, config_.max_speed_mps);

    // Integrate until reaching zero/the speed limit, then hold that speed.
    // A stopping car must not travel backwards or move while already at rest.
    double active_time = dt;
    if (acceleration != 0.0) {
        active_time = std::clamp((next_speed - v) / acceleration, 0.0, dt);
    }
    const double distance = v * active_time + 0.5 * acceleration * active_time * active_time +
                            next_speed * (dt - active_time);

    const double requested_angle = input.steering * config_.max_steering_rad;
    const double max_change = config_.steering_rate_rad_per_s * dt;
    double angle = state.steering_rad +
                   std::clamp(requested_angle - state.steering_rad, -max_change, max_change);
    double curvature = std::tan(angle) / config_.wheelbase_m;
    const double peak_speed = std::max(v, next_speed);
    if (peak_speed > 0.0) {
        // A geometric grip approximation, not tire slip or a tire-force model.
        const double limit = config_.max_lateral_acceleration_mps2 / (peak_speed * peak_speed);
        curvature = std::clamp(curvature, -limit, limit);
        angle = std::atan(curvature * config_.wheelbase_m);
    }

    VehicleState next = state;
    const double heading = std::remainder(state.heading_rad, 2.0 * std::numbers::pi);
    const double turn = curvature * distance;
    const double half_turn = turn * 0.5;
    // Exact circular-arc displacement for constant curvature; stable near zero.
    const double sinc = std::abs(half_turn) < 1e-8
                            ? 1.0 - half_turn * half_turn / 6.0
                            : std::sin(half_turn) / half_turn;
    next.position_m.x += distance * sinc * std::cos(heading + half_turn);
    next.position_m.y += distance * sinc * std::sin(heading + half_turn);
    next.speed_mps = next_speed;
    next.heading_rad = std::remainder(heading + turn, 2.0 * std::numbers::pi);
    next.steering_rad = angle;
    next.longitudinal_acceleration_mps2 = (next_speed - v) / dt;
    next.distance_m += distance;
    if (!finite_state(next) || distance < 0.0) {
        throw std::overflow_error("Vehicle step produced unrepresentable values");
    }
    return next;
}

} // namespace pitwall::simulation
