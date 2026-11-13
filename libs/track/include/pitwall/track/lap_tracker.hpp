#pragma once

#include <pitwall/track/track.hpp>

#include <chrono>
#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace pitwall::track {

enum class LapInvalidReason { none, off_track, discontinuity, wrong_direction, checkpoint_order };
[[nodiscard]] std::string_view describe(LapInvalidReason reason) noexcept;

struct LapResult {
    std::size_t number{};
    std::chrono::microseconds start_time{};
    std::chrono::microseconds finish_time{};
    std::chrono::microseconds duration{};
    bool valid{};
    LapInvalidReason reason{LapInvalidReason::none};
};

struct LapUpdate {
    TrackProjection projection{};
    bool started{};
    std::optional<LapResult> completed_lap{};
};

struct LapStatus {
    bool running{};
    std::size_t current_lap_number{1};
    std::size_t passed_checkpoints{};
    bool current_lap_valid{}; // False while waiting for the first forward start crossing
    LapInvalidReason reason{LapInvalidReason::none};
    std::chrono::microseconds elapsed{};
};

class LapTracker {
public:
    // Owns the track by value: no dangling reference to a renderer's track.
    // Empty checkpoints select quarter/half/three-quarter positions.
    explicit LapTracker(Track track, std::vector<double> checkpoints_m = {},
                        double max_sample_distance_m = 10.0);

    // Time is simulation time, nonnegative and strictly increasing after the first sample.
    // Invalid positions/timestamps throw without changing tracker state.
    [[nodiscard]] LapUpdate update(domain::Vec2 position, std::chrono::microseconds timestamp);
    void reset() noexcept;

    [[nodiscard]] const Track& track() const noexcept { return track_; }
    [[nodiscard]] LapStatus status() const noexcept;
    [[nodiscard]] std::span<const LapResult> completed_laps() const noexcept { return laps_; }
    [[nodiscard]] std::optional<std::chrono::microseconds> best_lap_time() const noexcept {
        return best_time_;
    }

private:
    struct Gate { TrackPose pose; };
    void invalidate(LapInvalidReason reason) noexcept;

    Track track_;
    std::vector<Gate> gates_;
    double max_sample_distance_m_;
    std::optional<domain::Vec2> previous_position_{};
    std::chrono::microseconds previous_time_{};
    std::chrono::microseconds start_time_{};
    bool running_{};
    LapInvalidReason reason_{LapInvalidReason::none};
    std::size_t next_gate_{1};
    std::vector<LapResult> laps_;
    std::optional<std::chrono::microseconds> best_time_{};
};

} // namespace pitwall::track
