#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/policy_profile.hpp"

namespace policy_runtime {

struct PolicySession {
  std::string session_id;
  std::uint64_t step_id{};
  nlohmann::json metadata = nlohmann::json::object();
};

class Policy {
 public:
  virtual ~Policy() = default;
  virtual Result<nlohmann::json> infer(const nlohmann::json& observation,
                                       const PolicySession& session) = 0;
  virtual void reset(const std::string& session_id) noexcept = 0;
  virtual nlohmann::json capabilities() const = 0;
};

Result<std::unique_ptr<Policy>> make_policy(
    const profiles::PolicyProfile& profile);

}  // namespace policy_runtime
