#include "policy_runtime/robot_io/transport_runtime.hpp"

#include <algorithm>
#include <thread>
#include <utility>

namespace policy_runtime::robot_io {

Result<TransportRuntime> TransportRuntime::create(SocketCanTransport transport,
                                                   ReceiveCallback callback,
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
      TransportRuntime(std::move(transport), std::move(callback),
                       std::move(slots)));
}

TransportRuntime::TransportRuntime(SocketCanTransport transport,
                                   ReceiveCallback callback,
                                   std::vector<std::unique_ptr<Slot>> slots)
    : transport_(std::move(transport)),
      callback_(std::move(callback)),
      slots_(std::move(slots)) {}

TransportRuntime::TransportRuntime(TransportRuntime&& other) noexcept
    : transport_(std::move(other.transport_)),
      callback_(std::move(other.callback_)),
      slots_(std::move(other.slots_)),
      running_(other.running_.load(std::memory_order_acquire)),
      poll_period_(other.poll_period_),
      receive_sequence_(other.receive_sequence_) {
  other.running_.store(false, std::memory_order_release);
}

TransportRuntime& TransportRuntime::operator=(TransportRuntime&& other) noexcept {
  if (this != &other) {
    stop();
    transport_ = std::move(other.transport_);
    callback_ = std::move(other.callback_);
    slots_ = std::move(other.slots_);
    running_.store(other.running_.load(std::memory_order_acquire),
                   std::memory_order_release);
    poll_period_ = other.poll_period_;
    receive_sequence_ = other.receive_sequence_;
    other.running_.store(false, std::memory_order_release);
  }
  return *this;
}

TransportRuntime::~TransportRuntime() { stop(); }

Result<void> TransportRuntime::start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "transport runtime is already running"});
  }
  try {
    receive_worker_ = std::jthread([this](std::stop_token stop) {
      receive_loop(stop);
    });
    send_worker_ = std::jthread([this](std::stop_token stop) {
      send_loop(stop);
    });
  } catch (...) {
    stop();
    return Result<void>::failure(
        {ErrorCode::internal, "failed to start transport workers"});
  }
  return Result<void>::success();
}

void TransportRuntime::stop() noexcept {
  running_.store(false, std::memory_order_release);
  receive_worker_.request_stop();
  send_worker_.request_stop();
  if (receive_worker_.joinable()) receive_worker_.join();
  if (send_worker_.joinable()) send_worker_.join();
}

Result<void> TransportRuntime::stage(std::size_t slot,
                                     const DeviceFrame& frame) noexcept {
  if (slot >= slots_.size() || frame.size > frame.bytes.size()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "transport frame slot or size is invalid"});
  }
  slots_[slot]->mailbox.publish(frame);
  return Result<void>::success();
}

bool TransportRuntime::running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

void TransportRuntime::receive_loop(std::stop_token stop) {
  while (!stop.stop_requested() && running()) {
    auto received = transport_.receive_frame();
    if (received.has_value() && received.value().has_value()) {
      const auto& can = received.value().value();
      DeviceFrame frame;
      frame.address = can.id;
      frame.size = can.dlc;
      std::copy_n(can.data.begin(), can.dlc, frame.bytes.begin());
      ++receive_sequence_;
      callback_(frame, receive_sequence_);
    }
    std::this_thread::sleep_for(poll_period_);
  }
}

void TransportRuntime::send_loop(std::stop_token stop) {
  while (!stop.stop_requested() && running()) {
    for (auto& slot : slots_) {
      auto frame = slot->mailbox.read_latest(slot->consumed_sequence);
      if (!frame.has_value()) continue;
      slot->consumed_sequence = frame->sequence;
      CanFrame can;
      can.id = frame->address;
      can.dlc = static_cast<std::uint8_t>(frame->size);
      std::copy_n(frame->bytes.begin(), can.dlc, can.data.begin());
      static_cast<void>(transport_.send_frame(can));
    }
    std::this_thread::sleep_for(poll_period_);
  }
}

}  // namespace policy_runtime::robot_io
