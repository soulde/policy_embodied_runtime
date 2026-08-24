#pragma once

#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/policy_profile.hpp"

namespace policy_runtime {

class ProcessorPipeline {
 public:
  static Result<ProcessorPipeline> create(
      const std::vector<profiles::ProcessorSpec>& specifications,
      std::string_view stage);

  Result<nlohmann::json> run(const nlohmann::json& value) const;

 private:
  explicit ProcessorPipeline(std::size_t noop_count) noexcept
      : noop_count_(noop_count) {}

  std::size_t noop_count_{};
};

}  // namespace policy_runtime
