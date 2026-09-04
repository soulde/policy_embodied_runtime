#pragma once

#include <cstdint>

#include "policy_runtime/protocol/damiao/protocol.hpp"

namespace policy_runtime {

class MITMotorDevice {
 public:
  explicit MITMotorDevice(DamiaoLimits limits) : limits_(limits) {}
  virtual ~MITMotorDevice() = default;

  const DamiaoLimits& limits() const noexcept { return limits_; }
  bool fault_latched() const noexcept { return fault_latched_; }
  const DamiaoFeedback& feedback() const noexcept { return feedback_; }

  void latch_fault() noexcept { fault_latched_ = true; }
  void clear_fault() noexcept { fault_latched_ = false; }

 protected:
  void update_feedback(const DamiaoFeedback& feedback) noexcept {
    feedback_ = feedback;
    if (feedback.error_code != 0U) {
      fault_latched_ = true;
    }
  }

 private:
  DamiaoLimits limits_;
  DamiaoFeedback feedback_{};
  bool fault_latched_{false};
};

}  // namespace policy_runtime
