#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include "policy_runtime/transport/asynchronous_transport.hpp"

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

using SerialChannelId = std::uint32_t;

class SerialTransport : public AsynchronousTransport {
 public:
  explicit SerialTransport(SerialConfig config);
  ~SerialTransport() override;

  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  virtual Result<void> open() override;
  virtual void close() noexcept override;
  virtual void request_stop() noexcept override;
  virtual void receive_once() noexcept override;
  virtual TransportHealth health() const noexcept override;
  virtual SchedulingClass scheduling_class() const noexcept override;
  virtual void cycle(const CycleContext& context) noexcept override;

  virtual Result<void> write(SerialChannelId channel,
                             std::span<const std::byte> data);
  virtual Result<std::size_t> read(SerialChannelId channel,
                                   std::span<std::byte> buffer);

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
