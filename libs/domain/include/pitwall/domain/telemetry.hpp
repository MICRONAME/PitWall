#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>

namespace pitwall::domain {

struct Vec2 {
    double x{};
    double y{};
};

// Domain values, never a memory layout for network messages.
// Position: metres; speed: m/s; acceleration: m/s^2; angles: radians.
// Session time is simulation time, independent of the wall clock.
struct TelemetrySample {
    std::uint64_t session_id{1};
    std::uint64_t sequence{};
    std::chrono::microseconds session_time{};
    Vec2 position_m{};
    double speed_mps{};
    double longitudinal_acceleration_mps2{};
    double heading_rad{};
    double steering_rad{};
    double throttle{}; // [0, 1]
    double brake{};    // [0, 1]
};

enum class ValidationError {
    invalid_session,
    negative_time,
    non_finite_value,
    negative_speed,
    invalid_pedal
};

// Stateless field validation; ordering and session transitions belong to the source.
[[nodiscard]] std::optional<ValidationError> validate(const TelemetrySample& sample) noexcept;
[[nodiscard]] std::string_view describe(ValidationError error) noexcept;

} // namespace pitwall::domain
