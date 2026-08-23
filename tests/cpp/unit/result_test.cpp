#include <gtest/gtest.h>

#include "policy_runtime/common/result.hpp"

TEST(ResultTest, CarriesTypedError) {
  auto result = policy_runtime::Result<int>::failure(
      {policy_runtime::ErrorCode::invalid_argument, "bad value"});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, policy_runtime::ErrorCode::invalid_argument);
}

TEST(ResultTest, VoidSuccessHasValue) {
  const auto result = policy_runtime::Result<void>::success();

  EXPECT_TRUE(result.has_value());
}

TEST(ResultTest, VoidFailureCarriesError) {
  const auto result = policy_runtime::Result<void>::failure(
      {policy_runtime::ErrorCode::io, "transport unavailable"});

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, policy_runtime::ErrorCode::io);
  EXPECT_EQ(result.error().message, "transport unavailable");
}
