#include "policy_runtime/policy/pipeline.hpp"

#include <string>

namespace policy_runtime {

Result<ProcessorPipeline> ProcessorPipeline::create(
    const std::vector<profiles::ProcessorSpec>& specifications,
    std::string_view stage) {
  for (const auto& specification : specifications) {
    if (specification.name != "noop") {
      return Result<ProcessorPipeline>::failure(
          {ErrorCode::invalid_argument,
           "unknown " + std::string(stage) + " processor: " +
               specification.name});
    }
    if (!specification.params.empty()) {
      return Result<ProcessorPipeline>::failure(
          {ErrorCode::invalid_argument,
           "noop " + std::string(stage) +
               " processor does not accept parameters"});
    }
  }
  return Result<ProcessorPipeline>::success(
      ProcessorPipeline(specifications.empty() ? 1U : specifications.size()));
}

Result<nlohmann::json> ProcessorPipeline::run(
    const nlohmann::json& value) const {
  if (noop_count_ == 0) {
    return Result<nlohmann::json>::failure(
        {ErrorCode::internal, "processor pipeline is not initialized"});
  }
  return Result<nlohmann::json>::success(value);
}

}  // namespace policy_runtime
