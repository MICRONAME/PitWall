#include "physics_cases.hpp"

#include <gtest/gtest.h>

class VehiclePhysics : public testing::TestWithParam<physics_tests::Case> {};

TEST_P(VehiclePhysics, Contract) {
    EXPECT_NO_THROW(GetParam().run());
}

INSTANTIATE_TEST_SUITE_P(
    Physics, VehiclePhysics, testing::ValuesIn(physics_tests::cases),
    [](const testing::TestParamInfo<physics_tests::Case>& info) {
        return std::string{info.param.name};
    });
