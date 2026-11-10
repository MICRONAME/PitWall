#pragma once

#include <pitwall/domain/telemetry.hpp>

#include <limits>
#include <stdexcept>

// Both runners execute the same checks, including in Release builds.
inline void verify_domain_contract() {
    using namespace pitwall::domain;
    const auto expect = [](const TelemetrySample& sample, std::optional<ValidationError> error) {
        if (validate(sample) != error) {
            throw std::runtime_error("Telemetry validation contract failed");
        }
    };
    TelemetrySample sample{};
    expect(sample, std::nullopt);
    sample.session_id = 0;
    expect(sample, ValidationError::invalid_session);
    sample = {};
    sample.session_time = std::chrono::microseconds{-1};
    expect(sample, ValidationError::negative_time);
    sample = {};
    sample.speed_mps = -0.1;
    expect(sample, ValidationError::negative_speed);
    sample = {};
    sample.throttle = 1.01;
    expect(sample, ValidationError::invalid_pedal);
    sample = {};
    sample.brake = -0.01;
    expect(sample, ValidationError::invalid_pedal);
    sample = {};
    sample.throttle = 1.0;
    sample.brake = 1.0;
    sample.position_m = {-10.0, -20.0};
    sample.longitudinal_acceleration_mps2 = -5.0;
    expect(sample, std::nullopt);

    // Check every floating-point field: NaN can bypass ordinary range comparisons.
    for (int field = 0; field < 8; ++field) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
            sample = {};
            double* fields[]{&sample.position_m.x, &sample.position_m.y, &sample.speed_mps,
                             &sample.longitudinal_acceleration_mps2, &sample.heading_rad,
                             &sample.steering_rad, &sample.throttle, &sample.brake};
            *fields[field] = invalid;
            expect(sample, ValidationError::non_finite_value);
        }
    }
}
