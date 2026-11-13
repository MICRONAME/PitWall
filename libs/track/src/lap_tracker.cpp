#include <pitwall/track/lap_tracker.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pitwall::track {
namespace {
double side(domain::Vec2 point, const TrackPose& gate) {
    return (point.x - gate.position_m.x) * gate.forward.x +
           (point.y - gate.position_m.y) * gate.forward.y;
}

std::chrono::microseconds interpolate_time(std::chrono::microseconds from,
                                          std::chrono::microseconds to, double fraction) {
    if (fraction <= 0.0) { return from; }
    if (fraction >= 1.0) { return to; }
    const auto delta = to - from;
    const long double offset = static_cast<long double>(fraction) *
                               static_cast<long double>(delta.count());
    if (offset >= static_cast<long double>(delta.count())) { return to; }
    return from + std::chrono::microseconds{
                      static_cast<std::chrono::microseconds::rep>(offset)};
}
} // namespace

std::string_view describe(LapInvalidReason reason) noexcept {
    switch (reason) {
    case LapInvalidReason::none: return "valid";
    case LapInvalidReason::off_track: return "off track or shortcut";
    case LapInvalidReason::discontinuity: return "position discontinuity";
    case LapInvalidReason::wrong_direction: return "reverse gate crossing";
    case LapInvalidReason::checkpoint_order: return "missing or out-of-order checkpoint";
    }
    return "unknown lap status";
}

LapTracker::LapTracker(Track track, std::vector<double> checkpoints_m, double max_sample_distance_m)
    : track_(std::move(track)), max_sample_distance_m_(max_sample_distance_m) {
    if (!std::isfinite(max_sample_distance_m_) || max_sample_distance_m_ <= 0.0) {
        throw std::invalid_argument("Maximum sample displacement must be finite and positive");
    }
    if (checkpoints_m.empty()) {
        checkpoints_m = {track_.length_m() * 0.25, track_.length_m() * 0.5,
                         track_.length_m() * 0.75};
    }
    gates_.push_back({track_.pose_at(0.0)});
    double previous = 0.0;
    for (const double progress : checkpoints_m) {
        if (!std::isfinite(progress) || progress <= previous || progress >= track_.length_m()) {
            throw std::invalid_argument("Checkpoints must be strictly ordered inside (0, length)");
        }
        gates_.push_back({track_.pose_at(progress)});
        previous = progress;
    }
}

void LapTracker::invalidate(LapInvalidReason reason) noexcept {
    if (running_ && reason_ == LapInvalidReason::none) { reason_ = reason; }
}

LapUpdate LapTracker::update(domain::Vec2 position, std::chrono::microseconds timestamp) {
    LapUpdate result{track_.project(position), false, std::nullopt};
    if (timestamp.count() < 0 || (previous_position_ && timestamp <= previous_time_)) {
        throw std::invalid_argument("Sample time must be nonnegative and strictly increasing");
    }
    if (!previous_position_) {
        previous_position_ = position;
        previous_time_ = timestamp;
        return result;
    }
    const auto from = *previous_position_;
    const double displacement = std::hypot(position.x - from.x, position.y - from.y);
    const bool discontinuity = displacement > max_sample_distance_m_;
    const bool road_motion = !discontinuity && track_.contains_motion(from, position);
    if (!road_motion) {
        invalidate(discontinuity ? LapInvalidReason::discontinuity : LapInvalidReason::off_track);
    } else {
        struct Crossing { std::size_t gate; double fraction; bool forward; };
        std::vector<Crossing> crossings;
        for (std::size_t i = 0; i < gates_.size(); ++i) {
            const auto& gate = gates_[i].pose;
            const double before = side(from, gate);
            const double after = side(position, gate);
            const bool forward = before < 0.0 && after >= 0.0;
            // Assign an exact-on-line point to the forward side. This counts a
            // forward arrival once and detects leaving that point backwards.
            const bool backward = before >= 0.0 && after < 0.0;
            if (!forward && !backward) { continue; }
            const double fraction = std::clamp(-before / (after - before), 0.0, 1.0);
            const domain::Vec2 crossing{from.x + fraction * (position.x - from.x),
                                        from.y + fraction * (position.y - from.y)};
            const double lateral = -(crossing.x - gate.position_m.x) * gate.forward.y +
                                    (crossing.y - gate.position_m.y) * gate.forward.x;
            if (std::abs(lateral) <= track_.road_width_m() * 0.5 + 1e-9) {
                crossings.push_back({i, fraction, forward});
            }
        }
        std::sort(crossings.begin(), crossings.end(), [](const Crossing& a, const Crossing& b) {
            if (a.fraction != b.fraction) { return a.fraction < b.fraction; }
            return a.gate < b.gate;
        });
        for (const auto crossing : crossings) {
            if (!crossing.forward) {
                invalidate(LapInvalidReason::wrong_direction);
                continue;
            }
            if (crossing.gate == 0) {
                const auto event_time = interpolate_time(previous_time_, timestamp, crossing.fraction);
                if (running_) {
                    if (next_gate_ != gates_.size()) { invalidate(LapInvalidReason::checkpoint_order); }
                    LapResult lap{laps_.size() + 1, start_time_, event_time, event_time - start_time_,
                                  reason_ == LapInvalidReason::none, reason_};
                    if (lap.duration.count() <= 0) {
                        lap.valid = false;
                        lap.reason = LapInvalidReason::discontinuity;
                    }
                    laps_.push_back(lap);
                    result.completed_lap = lap;
                    if (lap.valid && (!best_time_ || lap.duration < *best_time_)) {
                        best_time_ = lap.duration;
                    }
                }
                running_ = true;
                reason_ = LapInvalidReason::none;
                next_gate_ = 1;
                start_time_ = event_time;
                result.started = true;
            } else if (running_) {
                if (crossing.gate == next_gate_) { ++next_gate_; }
                else { invalidate(LapInvalidReason::checkpoint_order); }
            }
        }
    }
    previous_position_ = position;
    previous_time_ = timestamp;
    return result;
}

LapStatus LapTracker::status() const noexcept {
    return {running_, laps_.size() + 1, running_ ? next_gate_ - 1 : 0,
            running_ && reason_ == LapInvalidReason::none, reason_,
            running_ ? previous_time_ - start_time_ : std::chrono::microseconds{0}};
}

void LapTracker::reset() noexcept {
    previous_position_.reset();
    previous_time_ = {};
    start_time_ = {};
    running_ = false;
    reason_ = LapInvalidReason::none;
    next_gate_ = 1;
    laps_.clear();
    best_time_.reset();
}

} // namespace pitwall::track
