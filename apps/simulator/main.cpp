#include <pitwall/simulation/fixed_step.hpp>
#include <pitwall/track/lap_tracker.hpp>

#include <exception>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

namespace {

int run_track_demo() {
    auto track = pitwall::track::make_oval_track();
    pitwall::track::LapTracker tracker{track};
    std::cout << "PitWall track geometry demo (scripted positions, not vehicle physics/autopilot)\n"
              << "track=" << track.id() << ", length_m=" << std::fixed << std::setprecision(3)
              << track.length_m() << ", width_m=" << track.road_width_m() << '\n';
    constexpr double speed_mps = 20.0;
    constexpr double start_progress = -2.0;
    (void)tracker.update(track.pose_at(start_progress).position_m, std::chrono::microseconds{0});
    const double end_progress = 2.0 * track.length_m() + 2.0;
    const int samples = static_cast<int>(std::ceil((end_progress - start_progress) / 0.5));
    for (int i = 1; i <= samples; ++i) {
        const double progress = std::min(end_progress, start_progress + static_cast<double>(i) * 0.5);
        const auto timestamp = std::chrono::microseconds{
            static_cast<std::chrono::microseconds::rep>(
                std::llround((progress - start_progress) / speed_mps * 1'000'000.0))};
        const auto update = tracker.update(track.pose_at(progress).position_m, timestamp);
        if (update.completed_lap) {
            const auto& lap = *update.completed_lap;
            std::cout << "lap=" << lap.number << ", time_s="
                      << std::chrono::duration<double>(lap.duration).count()
                      << ", status=" << pitwall::track::describe(lap.reason) << '\n';
        }
    }
    std::size_t valid = 0;
    for (const auto& lap : tracker.completed_laps()) { if (lap.valid) { ++valid; } }
    std::cout << "track demo completed: laps=" << tracker.completed_laps().size()
              << ", valid=" << valid << '\n';
    if (tracker.completed_laps().size() != 2 || valid != 2) {
        throw std::runtime_error("Scripted track demo did not complete two valid laps");
    }
    return 0;
}

int run_demo() {
    using namespace std::chrono_literals;
    pitwall::simulation::FixedStepSimulation simulation;
    std::cout << "PitWall vehicle physics demo: 12 simulated seconds, 100 Hz\n"
                 "0-4s: accelerate; 4-7s: turn left; 7-12s: brake\n"
                 " time_s   speed_kmh        x_m        y_m  heading_rad\n";
    std::cout << std::fixed << std::setprecision(3);
    const auto print = [&simulation] {
        const auto sample = simulation.telemetry(1);
        if (const auto error = pitwall::domain::validate(sample)) {
            throw std::runtime_error(std::string{pitwall::domain::describe(*error)});
        }
        std::cout << std::setw(7) << std::chrono::duration<double>(sample.session_time).count()
                  << std::setw(12) << sample.speed_mps * 3.6 << std::setw(11) << sample.position_m.x
                  << std::setw(11) << sample.position_m.y << std::setw(13) << sample.heading_rad
                  << '\n';
    };
    print();
    for (int frame = 0; frame < 600; ++frame) {
        pitwall::simulation::ControlInput controls{};
        if (frame < 200) {
            controls.throttle = 1.0;
        } else if (frame < 350) {
            controls.throttle = 0.25;
            controls.steering = 0.25;
        } else {
            controls.brake = 1.0;
        }
        const auto result = simulation.advance(20ms, controls);
        if (result.dropped_time.count() != 0) {
            throw std::runtime_error("Unexpected dropped time in the scripted demo");
        }
        if ((frame + 1) % 50 == 0) {
            print();
        }
    }
    std::cout << "demo completed: steps=" << simulation.tick_count()
              << ", distance_m=" << simulation.state().distance_m << '\n';
    return 0;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string_view{argv[1]} == "--version") {
        std::cout << "PitWall Simulator " << PITWALL_VERSION << '\n';
        return 0;
    }
    if (argc == 2 && std::string_view{argv[1]} == "--help") {
        std::cout << "Usage: pitwall_simulator [--help | --version | --demo | --track-demo]\n"
                     "--demo runs a scripted headless vehicle simulation without waiting.\n"
                     "--track-demo checks two laps using scripted centreline positions.\n"
                     "Graphics, keyboard driving and UDP are future milestones.\n";
        return 0;
    }
    if (argc == 2 && std::string_view{argv[1]} == "--demo") {
        try {
            return run_demo();
        } catch (const std::exception& error) {
            std::cerr << "Simulation error: " << error.what() << '\n';
            return 1;
        }
    }
    if (argc == 2 && std::string_view{argv[1]} == "--track-demo") {
        try {
            return run_track_demo();
        } catch (const std::exception& error) {
            std::cerr << "Track error: " << error.what() << '\n';
            return 1;
        }
    }
    if (argc != 1) {
        std::cerr << "Unsupported arguments. Use --help.\n";
        return 2;
    }
    std::cout << "PitWall Simulator " << PITWALL_VERSION << " - vehicle physics ready\n"
                 "Track geometry and lap tracking ready. Run --demo or --track-demo.\n"
                 "No graphical window yet.\n";
}
