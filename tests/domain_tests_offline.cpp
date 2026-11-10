#include "domain_contract.hpp"

#include <exception>
#include <iostream>

int main() {
    try {
        verify_domain_contract();
        std::cout << "Telemetry domain contract passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
