#include "domain_contract.hpp"

#include <gtest/gtest.h>

TEST(TelemetryDomain, ValidationContract) {
    EXPECT_NO_THROW(verify_domain_contract());
}
