#pragma once

#include <pitwall/domain/telemetry.hpp>

#include <chrono>

namespace pitwall::simulation {

// SI units. These are illustrative vehicle parameters, not a calibrated real car.
struct VehicleConfig {
    double mass_kg{1200.0};
    double wheelbase_m{2.6};
    double engine_force_n{8000.0};
    double brake_force_n{14000.0};
    double drag_coefficient_n_per_mps2{0.42};
    double rolling_resistance_n{140.0};
    double max_speed_mps{70.0};
    double max_steering_rad{0.6};
    double steering_rate_rad_per_s{1.5};
    double max_lateral_acceleration_mps2{8.0};
};

struct ControlInput {
    double throttle{}; // [0, 1]
    double brake{};    // [0, 1]
    double steering{}; // [-1, 1]; positive is counterclockwise (left)
};

// Position is the centre of the rear axle. Heading zero points along +X;
// +Y is up in world space. Rendering must handle its own screen coordinates.
struct VehicleState {
    domain::Vec2 position_m{};
    double speed_mps{};
    double heading_rad{};
    double steering_rad{}; // Applied front-wheel angle after the curvature limit
    double longitudinal_acceleration_mps2{};
    double distance_m{}; // Actual travelled distance, NOT centreline progress
};

// Throws invalid_argument for invalid parameters/state/input/duration.
// Throws overflow_error if a calculation cannot be represented as finite doubles.
// No global state or IO; the input state is never modified.
class VehicleModel {
public:
    explicit VehicleModel(VehicleConfig config = {});
    [[nodiscard]] const VehicleConfig& config() const noexcept { return config_; }
    [[nodiscard]] VehicleState step(const VehicleState& state, const ControlInput& input,
                                    std::chrono::microseconds duration) const;

private:
    VehicleConfig config_;
};

void validate_controls(const ControlInput& input);

} // namespace pitwall::simulation
