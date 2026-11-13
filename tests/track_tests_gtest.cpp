#include "track_cases.hpp"

#include <gtest/gtest.h>

class TrackAndLaps : public testing::TestWithParam<track_tests::Case> {};
TEST_P(TrackAndLaps, Contract) { EXPECT_NO_THROW(GetParam().run()); }
INSTANTIATE_TEST_SUITE_P(
    Track, TrackAndLaps, testing::ValuesIn(track_tests::cases),
    [](const testing::TestParamInfo<track_tests::Case>& info) {
        return std::string{info.param.name};
    });
