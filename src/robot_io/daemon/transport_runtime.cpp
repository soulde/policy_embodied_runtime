#include "policy_runtime/robot_io/daemon/transport_runtime.hpp"

#include <algorithm>
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

Result<TransportRuntime> TransportRuntime::create(
    SocketCanTransport transport, ReceiveCallback callback,
    std::size_t actuator_slots) {
  if (!transport.valid() || !callback || actuator_slots == 0U) {
    return Result<TransportRuntime>::failure(
        {ErrorCode::invalid_argument,
         "transport runtime requires an open transport and callback"});
  }
  std::vector<std::unique_ptr<Slot>> slots;
  slots.reserve(actuator_slots);
  for (std::size_t index = 0; index < actuator_slots; ++index) {
    slots.emplace_back(std::make_unique<Slot>());
  }
  return Result<TransportRuntime>::success(
      TransportRuntime{std::move(transport), std::move(callback),
                       std::move(slots)});
}

TransportRuntime::TransportRuntime(Transport& transport,
                                   std::chrono::nanoseconds period) noexcept
    : mode_(Mode::generic),
      transport_(&transport),
      asynchronous_transport_(dynamic_cast<AsynchronousTransport*>(&transport)),
      period_(period) {}

TransportRuntime::TransportRuntime(SocketCanTransport transport,
                                   ReceiveCallback callback,
                                   std::vector<std::unique_ptr<Slot>> slots)
    : mode_(Mode::can),
      can_transport_(std::make_unique<SocketCanTransport>(std::move(transport))),
      asynchronous_transport_(can_transport_.get()),
      period_(std::chrono::milliseconds{1}),
      callback_(std::move(callback)),
      slots_(std::move(slots)) {
  can_transport_->set_receive_handler([this](const CanFrame& can) {
    DeviceFrame frame;
    frame.address = can.id;
    frame.size = can.dlc;
    std::copy_n(can.data.begin(), can.dlc, frame.bytes.begin());
    callback_(frame, ++receive_sequence_);
  });
}

TransportRuntime::TransportRuntime(TransportRuntime&& other) noexcept
    : mode_(other.mode_),
      transport_(other.transport_),
      can_transport_(std::move(other.can_transport_)),
      asynchronous_transport_(other.mode_ == Mode::can
                                  ? static_cast<AsynchronousTransport*>(
                                        can_transport_.get())
                                  : other.asynchronous_transport_),
      period_(other.period_),
      callback_(std::move(other.callback_)), slots_(std::move(other.slots_)),
      running_(other.running_.load(std::memory_order_acquire)),
      worker_(std::move(other.worker_)),
      receive_worker_(std::move(other.receive_worker_)),
      poll_period_(other.poll_period_), receive_sequence_(other.receive_sequence_) {
  if (can_transport_ != nullptr) {
    can_transport_->set_receive_handler([this](const CanFrame& can) {
      DeviceFrame frame;
      frame.address = can.id;
      frame.size = can.dlc;
      std::copy_n(can.data.begin(), can.dlc, frame.bytes.begin());
      callback_(frame, ++receive_sequence_);
    });
  }
  other.transport_ = nullptr;
  other.asynchronous_transport_ = nullptr;
  other.running_.store(false, std::memory_order_release);
}

TransportRuntime& TransportRuntime::operator=(TransportRuntime&& other) noexcept {
  if (this == &other) return *this;
  stop();
  mode_ = other.mode_;
  transport_ = other.transport_;
  can_transport_ = std::move(other.can_transport_);
  asynchronous_transport_ =
      mode_ == Mode::can
          ? static_cast<AsynchronousTransport*>(can_transport_.get())
          : other.asynchronous_transport_;
  period_ = other.period_;
  callback_ = std::move(other.callback_);
  slots_ = std::move(other.slots_);
  running_.store(other.running_.load(std::memory_order_acquire),
                 std::memory_order_release);
  worker_ = std::move(other.worker_);
  receive_worker_ = std::move(other.receive_worker_);
  poll_period_ = other.poll_period_;
  receive_sequence_ = other.receive_sequence_;
  if (can_transport_ != nullptr) {
    can_transport_->set_receive_handler([this](const CanFrame& can) {
      DeviceFrame frame;
      frame.address = can.id;
      frame.size = can.dlc;
      std::copy_n(can.data.begin(), can.dlc, frame.bytes.begin());
      callback_(frame, ++receive_sequence_);
    });
  }
  other.transport_ = nullptr;
  other.asynchronous_transport_ = nullptr;
  other.running_.store(false, std::memory_order_release);
  return *this;
}

TransportRuntime::~TransportRuntime() { stop(); }

Result<void> TransportRuntime::start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "transport runtime is already running"});
  }
  try {
    if (mode_ == Mode::generic) {
      if (transport_ == nullptr) {
        running_.store(false, std::memory_order_release);
        return Result<void>::failure(
            {ErrorCode::invalid_argument, "generic transport is missing"});
      }
      auto opened = transport_->open();
      if (!opened.has_value()) {
        running_.store(false, std::memory_order_release);
        return opened;
      }
      worker_ = std::jthread([this](std::stop_token stop) { cycle_loop(stop); });
      if (asynchronous_transport_ != nullptr) {
        receive_worker_ = std::jthread([this](std::stop_token stop) {
          while (!stop.stop_requested() && running()) {
            asynchronous_transport_->receive_once();
            std::this_thread::sleep_for(poll_period_);
          }
        });
      }
    } else {
      receive_worker_ = std::jthread([this](std::stop_token stop) {
        while (!stop.stop_requested() && running()) {
          can_transport_->receive_once();
          std::this_thread::sleep_for(poll_period_);
        }
      });
      worker_ = std::jthread([this](std::stop_token stop) {
        cycle_loop(stop);
      });
    }
  } catch (...) {
    stop();
    return Result<void>::failure(
        {ErrorCode::internal, "failed to start transport runtime workers"});
  }
  return Result<void>::success();
}

void TransportRuntime::stop() noexcept {
  running_.store(false, std::memory_order_release);
  if (asynchronous_transport_ != nullptr) {
    asynchronous_transport_->request_stop();
  }
  worker_.request_stop();
  receive_worker_.request_stop();
  worker_ = std::jthread{};
  receive_worker_ = std::jthread{};
  if (mode_ == Mode::generic && transport_ != nullptr) transport_->close();
}

Result<void> TransportRuntime::stage(std::size_t slot,
                                     const DeviceFrame& frame) noexcept {
  if (mode_ != Mode::can || slot >= slots_.size() ||
      frame.size > frame.bytes.size()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "transport frame slot or size is invalid"});
  }
  slots_[slot]->mailbox.publish(frame);
  return Result<void>::success();
}

bool TransportRuntime::running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

void TransportRuntime::cycle_loop(std::stop_token stop) {
  auto next = std::chrono::steady_clock::now();
  std::uint64_t sequence{};
  while (!stop.stop_requested() && running()) {
    const auto now = std::chrono::steady_clock::now();
    if (mode_ == Mode::generic) {
      transport_->cycle(CycleContext{sequence++, now, period_});
    } else {
      for (auto& slot : slots_) {
        auto frame = slot->mailbox.read_latest(slot->consumed_sequence);
        if (!frame.has_value()) continue;
        slot->consumed_sequence = frame->sequence;
        CanFrame can;
        can.id = frame->address;
        can.dlc = static_cast<std::uint8_t>(frame->size);
        std::copy_n(frame->bytes.begin(), can.dlc, can.data.begin());
        static_cast<void>(can_transport_->send_frame(can));
      }
    }
    next += period_;
    std::this_thread::sleep_until(next);
  }
}

}  // namespace policy_runtime::robot_io
