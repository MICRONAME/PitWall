#pragma once

#include <pitwall/simulation/vehicle.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace pitwall::simulation {

struct FrameResult {
    std::size_t steps{};
    std::chrono::microseconds dropped_time{};
    double interpolation_alpha{}; // Remainder / fixed step, in [0, 1)
};

class FixedStepSimulation {
public:
    static constexpr auto fixed_step = std::chrono::microseconds{10'000};

    explicit FixedStepSimulation(VehicleConfig config = {}, std::size_t max_steps_per_frame = 8);

    // Controls are held for this frame's physics ticks. Frame time must be [0, 60s].
    // If overloaded, discard whole unsimulated ticks and report their duration;
    // preserve the fractional remainder. Simulation time counts executed ticks only.
    [[nodiscard]] FrameResult advance(std::chrono::microseconds frame_time,
                                       const ControlInput& controls);
    void reset() noexcept;

    [[nodiscard]] const VehicleState& state() const noexcept { return state_; }
    [[nodiscard]] std::chrono::microseconds simulation_time() const noexcept { return time_; }
    [[nodiscard]] std::uint64_t tick_count() const noexcept { return ticks_; }
    [[nodiscard]] domain::TelemetrySample telemetry(std::uint64_t session_id) const;

private:
    VehicleModel model_;
    std::size_t max_steps_per_frame_;
    VehicleState state_{};
    ControlInput last_controls_{};
    std::chrono::microseconds remainder_{};
    std::chrono::microseconds time_{};
    std::uint64_t ticks_{};
};

} // namespace pitwall::simulation
