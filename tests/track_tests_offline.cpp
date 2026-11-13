#include "track_cases.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    if (argc != 2) { std::cerr << "Supply one track test case\n"; return 2; }
    for (const auto& test : track_tests::cases) {
        if (test.name == std::string_view{argv[1]}) {
            try {
                test.run();
                std::cout << test.name << " passed\n";
                return 0;
            } catch (const std::exception& error) {
                std::cerr << test.name << ": " << error.what() << '\n';
                return 1;
            }
        }
    }
    std::cerr << "Unknown track test case\n";
    return 2;
}
