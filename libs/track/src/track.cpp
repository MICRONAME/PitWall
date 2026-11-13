#include <pitwall/track/track.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace pitwall::track {
namespace {
using domain::Vec2;
constexpr double epsilon = 1e-9;

Vec2 subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
double magnitude(Vec2 value) { return std::hypot(value.x, value.y); }

void check_point(Vec2 point) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        std::abs(point.x) > 1e6 || std::abs(point.y) > 1e6) {
        throw std::invalid_argument("Track coordinates must be finite and within +/-1,000,000 m");
    }
}

double wrap(double value, double length) {
    double result = std::fmod(value, length);
    if (result < 0.0) { result += length; }
    return result >= length ? 0.0 : result;
}

bool on_segment(Vec2 a, Vec2 b, Vec2 p) {
    return std::abs(cross(subtract(b, a), subtract(p, a))) <= epsilon &&
           p.x >= std::min(a.x, b.x) - epsilon && p.x <= std::max(a.x, b.x) + epsilon &&
           p.y >= std::min(a.y, b.y) - epsilon && p.y <= std::max(a.y, b.y) + epsilon;
}

bool intersects(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double ab_c = cross(subtract(b, a), subtract(c, a));
    const double ab_d = cross(subtract(b, a), subtract(d, a));
    const double cd_a = cross(subtract(d, c), subtract(a, c));
    const double cd_b = cross(subtract(d, c), subtract(b, c));
    if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
        ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon))) {
        return true;
    }
    return on_segment(a, b, c) || on_segment(a, b, d) ||
           on_segment(c, d, a) || on_segment(c, d, b);
}

struct Interval { double low; double high; };

bool clip_linear(double origin, double delta, double low, double high, Interval& interval) {
    if (std::abs(delta) < 1e-14) {
        return origin >= low && origin <= high;
    }
    const double t1 = (low - origin) / delta;
    const double t2 = (high - origin) / delta;
    interval.low = std::max(interval.low, std::min(t1, t2));
    interval.high = std::min(interval.high, std::max(t1, t2));
    return interval.low <= interval.high;
}

} // namespace

Track::Track(std::string id, std::vector<Vec2> centerline, double road_width_m)
    : id_(std::move(id)), points_(std::move(centerline)), road_width_m_(road_width_m) {
    if (id_.empty() || points_.size() < 3 || points_.size() > 4096 ||
        !std::isfinite(road_width_m_) || road_width_m_ < 1e-4 || road_width_m_ > 1e4) {
        throw std::invalid_argument("Track requires an ID, 3..4096 vertices and width [0.0001,10000] m");
    }
    for (const auto point : points_) { check_point(point); }
    const auto size = points_.size();
    for (std::size_t i = 0; i < size; ++i) {
        const double segment_length = magnitude(subtract(points_[(i + 1) % size], points_[i]));
        if (segment_length < 1e-5) {
            throw std::invalid_argument("Track has a degenerate segment; omit repeated closing vertex");
        }
        cumulative_lengths_.push_back(length_m_);
        segment_lengths_.push_back(segment_length);
        length_m_ += segment_length;
    }
    for (std::size_t i = 0; i < size; ++i) {
        // Adjacent edges may join but must not reverse and overlap.
        const auto incoming = subtract(points_[i], points_[(i + size - 1) % size]);
        const auto outgoing = subtract(points_[(i + 1) % size], points_[i]);
        if (std::abs(cross(incoming, outgoing)) <= epsilon && dot(incoming, outgoing) < 0.0) {
            throw std::invalid_argument("Adjacent track segments overlap");
        }
        for (std::size_t j = i + 1; j < size; ++j) {
            if (j == i + 1 || (i == 0 && j == size - 1)) { continue; }
            if (intersects(points_[i], points_[(i + 1) % size],
                           points_[j], points_[(j + 1) % size])) {
                throw std::invalid_argument("Track centreline must not intersect itself");
            }
        }
    }
    start_offset_m_ = segment_lengths_.front() * 0.5;
}

TrackPose Track::pose_at(double progress_m) const {
    if (!std::isfinite(progress_m)) { throw std::invalid_argument("Progress must be finite"); }
    const double progress = wrap(progress_m, length_m_);
    const double absolute = wrap(progress + start_offset_m_, length_m_);
    const auto next = std::upper_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), absolute);
    const auto index = static_cast<std::size_t>(next - cumulative_lengths_.begin() - 1);
    const auto edge = subtract(points_[(index + 1) % points_.size()], points_[index]);
    const double fraction = (absolute - cumulative_lengths_[index]) / segment_lengths_[index];
    return {{points_[index].x + edge.x * fraction, points_[index].y + edge.y * fraction},
            {edge.x / segment_lengths_[index], edge.y / segment_lengths_[index]}, progress};
}

TrackProjection Track::project(Vec2 position_m) const {
    check_point(position_m);
    TrackProjection result;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const auto edge = subtract(points_[(i + 1) % points_.size()], points_[i]);
        const Vec2 tangent{edge.x / segment_lengths_[i], edge.y / segment_lengths_[i]};
        const double offset = std::clamp(dot(subtract(position_m, points_[i]), tangent),
                                         0.0, segment_lengths_[i]);
        const Vec2 closest{points_[i].x + tangent.x * offset, points_[i].y + tangent.y * offset};
        const auto delta = subtract(position_m, closest);
        const double distance = magnitude(delta);
        if (distance < best) {
            best = distance;
            result.pose = {closest, tangent,
                           wrap(cumulative_lengths_[i] + offset - start_offset_m_, length_m_)};
            result.distance_to_centerline_m = distance;
            result.signed_lateral_offset_m = std::copysign(distance, cross(tangent, delta));
            result.segment_index = i;
        }
    }
    result.on_track = best <= road_width_m_ * 0.5 + epsilon;
    return result;
}

bool Track::contains_motion(Vec2 from, Vec2 to) const {
    check_point(from);
    check_point(to);
    const auto motion = subtract(to, from);
    const double square_length = dot(motion, motion);
    if (square_length < 1e-20) { return project(from).on_track && project(to).on_track; }

    // Cover the motion parameter [0,1] by intervals inside road capsules.
    // Each capsule consists of an oriented rectangle and its endpoint disks.
    std::vector<Interval> intervals;
    intervals.reserve(points_.size() * 2);
    const double radius = road_width_m_ * 0.5 + epsilon;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const auto edge = subtract(points_[(i + 1) % points_.size()], points_[i]);
        const Vec2 tangent{edge.x / segment_lengths_[i], edge.y / segment_lengths_[i]};
        const auto offset = subtract(from, points_[i]);
        Interval strip{0.0, 1.0};
        if (clip_linear(dot(offset, tangent), dot(motion, tangent), 0.0, segment_lengths_[i], strip) &&
            clip_linear(cross(tangent, offset), cross(tangent, motion), -radius, radius, strip)) {
            intervals.push_back(strip);
        }
        const double b = dot(offset, motion);
        const double c = dot(offset, offset) - radius * radius;
        const double discriminant = b * b - square_length * c;
        if (discriminant >= 0.0) {
            const double root = std::sqrt(discriminant);
            Interval disk{std::max(0.0, (-b - root) / square_length),
                          std::min(1.0, (-b + root) / square_length)};
            if (disk.low <= disk.high) { intervals.push_back(disk); }
        }
    }
    std::sort(intervals.begin(), intervals.end(), [](Interval a, Interval b) {
        return a.low < b.low;
    });
    double covered = 0.0;
    for (const auto interval : intervals) {
        if (interval.low > covered + 1e-12) { return false; }
        covered = std::max(covered, interval.high);
        if (covered >= 1.0 - 1e-12) { return true; }
    }
    return false;
}

Track make_oval_track() {
    std::vector<Vec2> points;
    constexpr std::size_t vertices = 64;
    points.reserve(vertices);
    for (std::size_t i = 0; i < vertices; ++i) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) /
                             static_cast<double>(vertices);
        points.push_back({80.0 * std::cos(angle), 45.0 * std::sin(angle)});
    }
    return Track{"pitwall_oval", std::move(points), 10.0};
}

} // namespace pitwall::track
