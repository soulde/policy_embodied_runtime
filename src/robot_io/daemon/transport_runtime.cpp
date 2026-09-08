#include "policy_runtime/robot_io/daemon/transport_runtime.hpp"

#include <utility>

namespace policy_runtime::robot_io {

Result<TransportRuntime> TransportRuntime::create(
    Transport& transport, std::chrono::nanoseconds period) {
  if (period.count() <= 0) {
    return Result<TransportRuntime>::failure(
        {ErrorCode::invalid_argument, "transport runtime period must be positive"});
  }
  return Result<TransportRuntime>::success(TransportRuntime{transport, period});
}

TransportRuntime::TransportRuntime(Transport& transport,
                                   std::chrono::nanoseconds period) noexcept
    : transport_(&transport), period_(period) {}

TransportRuntime::TransportRuntime(TransportRuntime&& other) noexcept
    : transport_(other.transport_), period_(other.period_),
      running_(other.running_.load(std::memory_order_acquire)),
      worker_(std::move(other.worker_)) {
  other.transport_ = nullptr;
  other.running_.store(false, std::memory_order_release);
}

TransportRuntime& TransportRuntime::operator=(TransportRuntime&& other) noexcept {
  if (this == &other) return *this;
  stop();
  transport_ = other.transport_;
  period_ = other.period_;
  running_.store(other.running_.load(std::memory_order_acquire),
                 std::memory_order_release);
  worker_ = std::move(other.worker_);
  other.transport_ = nullptr;
  other.running_.store(false, std::memory_order_release);
  return *this;
}

TransportRuntime::~TransportRuntime() { stop(); }

Result<void> TransportRuntime::start() {
  if (transport_ == nullptr || running_.exchange(true, std::memory_order_acq_rel)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "transport runtime is not startable"});
  }
  auto opened = transport_->open();
  if (!opened.has_value()) {
    running_.store(false, std::memory_order_release);
    return opened;
  }
  try {
    worker_ = std::jthread([this](std::stop_token stop) { cycle_loop(stop); });
  } catch (...) {
    transport_->close();
    running_.store(false, std::memory_order_release);
    return Result<void>::failure(
        {ErrorCode::internal, "failed to start transport runtime worker"});
  }
  return Result<void>::success();
}

void TransportRuntime::stop() noexcept {
  running_.store(false, std::memory_order_release);
  if (worker_.joinable()) worker_.request_stop();
  worker_ = std::jthread{};
  if (transport_ != nullptr) transport_->close();
}

bool TransportRuntime::running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

void TransportRuntime::cycle_loop(std::stop_token stop) {
  auto next = std::chrono::steady_clock::now();
  std::uint64_t sequence{};
  while (!stop.stop_requested() && running()) {
    const auto now = std::chrono::steady_clock::now();
    transport_->cycle(CycleContext{sequence++, now, period_});
    next += period_;
    std::this_thread::sleep_until(next);
  }
}

}  // namespace policy_runtime::robot_io
