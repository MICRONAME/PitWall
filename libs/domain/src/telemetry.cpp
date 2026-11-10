#include <pitwall/domain/telemetry.hpp>

#include <array>
#include <cmath>

namespace pitwall::domain {

std::optional<ValidationError> validate(const TelemetrySample& sample) noexcept {
    if (sample.session_id == 0) {
        return ValidationError::invalid_session;
    }
    if (sample.session_time.count() < 0) {
        return ValidationError::negative_time;
    }
    const std::array values{sample.position_m.x, sample.position_m.y, sample.speed_mps,
                            sample.longitudinal_acceleration_mps2, sample.heading_rad,
                            sample.steering_rad, sample.throttle, sample.brake};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return ValidationError::non_finite_value;
        }
    }
    if (sample.speed_mps < 0.0) {
        return ValidationError::negative_speed;
    }
    if (sample.throttle < 0.0 || sample.throttle > 1.0 || sample.brake < 0.0 ||
        sample.brake > 1.0) {
        return ValidationError::invalid_pedal;
    }
    return std::nullopt;
}

std::string_view describe(ValidationError error) noexcept {
    switch (error) {
    case ValidationError::invalid_session:
        return "Session ID must be nonzero";
    case ValidationError::negative_time:
        return "Simulation time must be nonnegative";
    case ValidationError::non_finite_value:
        return "Telemetry values must be finite";
    case ValidationError::negative_speed:
        return "Speed must be nonnegative; reverse is not supported yet";
    case ValidationError::invalid_pedal:
        return "Pedal values must be in [0, 1]";
    }
    return "Unknown validation error";
}

} // namespace pitwall::domain
