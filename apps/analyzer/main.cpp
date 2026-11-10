#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string_view{argv[1]} == "--version") {
        std::cout << "PitWall Analyzer " << PITWALL_VERSION << '\n';
        return 0;
    }
    if (argc == 2 && std::string_view{argv[1]} == "--help") {
        std::cout << "Usage: pitwall_analyzer [--help | --version]\n"
                     "Project foundation. Telemetry dashboard is a future milestone.\n";
        return 0;
    }
    if (argc != 1) {
        std::cerr << "Unsupported arguments. Use --help.\n";
        return 2;
    }
    std::cout << "PitWall Analyzer " << PITWALL_VERSION << " - foundation ready\n"
                 "No UDP receiver, dashboard, storage or replay implemented yet.\n";
}
