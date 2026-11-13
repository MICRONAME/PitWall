#pragma once

#include <pitwall/domain/telemetry.hpp>

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace pitwall::track {

struct TrackPose {
    domain::Vec2 position_m{};
    domain::Vec2 forward{}; // Unit tangent in the declared driving direction
    double progress_m{};   // [0, length), measured from the start line
};

struct TrackProjection {
    TrackPose pose{};
    double distance_to_centerline_m{};
    double signed_lateral_offset_m{}; // Positive on the left of the selected segment
    std::size_t segment_index{};
    bool on_track{};
};

// Immutable simple closed centreline, with a constant-width road formed by the
// union of segment capsules. Start is at the midpoint of the first segment.
// Vertices are not repeated at the end. No IO, graphics or vehicle dependency.
class Track {
public:
    Track(std::string id, std::vector<domain::Vec2> centerline, double road_width_m);

    [[nodiscard]] const std::string& id() const noexcept { return id_; }
    [[nodiscard]] double length_m() const noexcept { return length_m_; }
    [[nodiscard]] double road_width_m() const noexcept { return road_width_m_; }
    [[nodiscard]] std::span<const domain::Vec2> centerline() const noexcept { return points_; }
    [[nodiscard]] TrackPose pose_at(double progress_m) const;
    [[nodiscard]] TrackProjection project(domain::Vec2 position_m) const;
    [[nodiscard]] bool contains_motion(domain::Vec2 from, domain::Vec2 to) const;

private:
    std::string id_;
    std::vector<domain::Vec2> points_;
    std::vector<double> segment_lengths_;
    std::vector<double> cumulative_lengths_;
    double road_width_m_{};
    double length_m_{};
    double start_offset_m_{};
};

[[nodiscard]] Track make_oval_track();

} // namespace pitwall::track
