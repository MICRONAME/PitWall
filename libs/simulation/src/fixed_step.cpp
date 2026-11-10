#include <pitwall/simulation/fixed_step.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace pitwall::simulation {

FixedStepSimulation::FixedStepSimulation(VehicleConfig config, std::size_t max_steps_per_frame)
    : model_(config), max_steps_per_frame_(max_steps_per_frame) {
    if (max_steps_per_frame_ == 0 || max_steps_per_frame_ > 1000) {
        throw std::invalid_argument("Catch-up limit must be in [1, 1000]");
    }
}

FrameResult FixedStepSimulation::advance(std::chrono::microseconds frame_time,
                                         const ControlInput& controls) {
    validate_controls(controls); // Reject bad controls even when there are no complete ticks.
    if (frame_time.count() < 0 || frame_time > std::chrono::seconds{60}) {
        throw std::invalid_argument("Frame duration must be in [0, 60s]");
    }
    const auto available = remainder_ + frame_time;
    const auto whole_ticks = available.count() / fixed_step.count();
    const auto steps = std::min(static_cast<std::size_t>(whole_ticks), max_steps_per_frame_);
    const auto simulated = fixed_step * static_cast<std::chrono::microseconds::rep>(steps);
    if (time_.count() > std::numeric_limits<std::chrono::microseconds::rep>::max() -
                            simulated.count() ||
        ticks_ > std::numeric_limits<std::uint64_t>::max() - steps) {
        throw std::overflow_error("Simulation clock overflow");
    }

    // Commit only after all steps succeed: invalid arithmetic leaves the frame unchanged.
    VehicleState next = state_;
    for (std::size_t i = 0; i < steps; ++i) {
        next = model_.step(next, controls, fixed_step);
    }
    const auto next_remainder = available % fixed_step;
    const auto dropped = available - simulated - next_remainder;
    state_ = next;
    remainder_ = next_remainder;
    time_ += simulated;
    ticks_ += steps;
    if (steps != 0) {
        last_controls_ = controls;
    }
    return {steps, dropped, static_cast<double>(remainder_.count()) /
                                static_cast<double>(fixed_step.count())};
}

void FixedStepSimulation::reset() noexcept {
    state_ = {};
    last_controls_ = {};
    remainder_ = {};
    time_ = {};
    ticks_ = 0;
}

domain::TelemetrySample FixedStepSimulation::telemetry(std::uint64_t session_id) const {
    if (session_id == 0) {
        throw std::invalid_argument("Telemetry session ID must be nonzero");
    }
    domain::TelemetrySample sample{};
    sample.session_id = session_id;
    sample.sequence = ticks_;
    sample.session_time = time_;
    sample.position_m = state_.position_m;
    sample.speed_mps = state_.speed_mps;
    sample.longitudinal_acceleration_mps2 = state_.longitudinal_acceleration_mps2;
    sample.heading_rad = state_.heading_rad;
    sample.steering_rad = state_.steering_rad;
    sample.throttle = last_controls_.throttle;
    sample.brake = last_controls_.brake;
    return sample;
}

} // namespace pitwall::simulation
