#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include "policy_runtime/transport/frame_transport.hpp"

namespace policy_runtime {

struct SerialConfig {
  std::string path;
  std::uint32_t baud_rate{};
  std::size_t read_buffer_size{256U};
  std::size_t maximum_frame_size{259U};
  std::chrono::milliseconds read_timeout{20};
  std::chrono::milliseconds write_timeout{20};
};

enum class SerialError : std::uint8_t {
  none,
  timeout,
  framing,
  io,
  closed,
  queue_full,
  internal,
};

class SerialTransport final : public FrameTransport {
 public:
  explicit SerialTransport(SerialConfig config);
  ~SerialTransport() override;

  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  Result<void> open() override;
  void close() noexcept override;
  void request_stop() noexcept override;
  TransportHealth health() const noexcept override;
  SchedulingClass scheduling_class() const noexcept override;
  void cycle(const CycleContext& context) noexcept override;

  Result<void> write(ChannelId channel,
                     std::span<const std::byte> data) override;
  Result<std::size_t> read(ChannelId channel,
                           std::span<std::byte> buffer) override;

  SerialError last_error() const noexcept;
  std::uint64_t timeout_count() const noexcept;
  std::uint64_t io_error_count() const noexcept;
  bool is_open() const noexcept;
  const SerialConfig& config() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace policy_runtime
