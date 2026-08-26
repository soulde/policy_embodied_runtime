#include "policy_runtime/transport/serial/serial_transport.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <tuple>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/eventfd.h>
#include <termios.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

constexpr std::size_t kMaximumFrameStorage = 259U;
constexpr std::size_t kMaximumReadBuffer = 4096U;
constexpr std::size_t kQueueCapacity = 64U;
constexpr auto kMaximumIoTimeout = std::chrono::milliseconds{100};

using PortIdentity =
    std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>;

std::mutex open_ports_mutex;
std::set<PortIdentity> open_ports;

Error system_error(const char* operation) {
  return {ErrorCode::io,
          std::string(operation) + ": " +
              std::error_code(errno, std::generic_category()).message()};
}

std::optional<speed_t> serial_speed(std::uint32_t baud_rate) {
  switch (baud_rate) {
    case 9'600U:
      return B9600;
    case 19'200U:
      return B19200;
    case 38'400U:
      return B38400;
    case 57'600U:
      return B57600;
    case 115'200U:
      return B115200;
#ifdef B230400
    case 230'400U:
      return B230400;
#endif
#ifdef B460800
    case 460'800U:
      return B460800;
#endif
#ifdef B500000
    case 500'000U:
      return B500000;
#endif
#ifdef B576000
    case 576'000U:
      return B576000;
#endif
#ifdef B921600
    case 921'600U:
      return B921600;
#endif
#ifdef B1000000
    case 1'000'000U:
      return B1000000;
#endif
#ifdef B1152000
    case 1'152'000U:
      return B1152000;
#endif
#ifdef B2000000
    case 2'000'000U:
      return B2000000;
#endif
    default:
      return std::nullopt;
  }
}

int remaining_timeout_ms(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
  const auto rounded = remaining + std::chrono::milliseconds{1};
  return static_cast<int>(std::min<std::int64_t>(
      rounded.count(), std::numeric_limits<int>::max()));
}

}  // namespace

class SerialTransport::Impl {
 public:
  explicit Impl(SerialConfig value) : config(std::move(value)) {}

  struct Frame {
    ChannelId channel{};
    std::size_t size{};
    std::array<std::byte, kMaximumFrameStorage> data{};
  };

  template <class Queue>
  static bool push(Queue& queue, std::size_t& head, std::size_t& size,
                   const Frame& frame) noexcept {
    if (size >= queue.size()) {
      return false;
    }
    queue[(head + size) % queue.size()] = frame;
    ++size;
    return true;
  }

  void request_stop() noexcept {
    const int descriptor = wake_fd.load(std::memory_order_acquire);
    if (descriptor < 0) {
      return;
    }
    const std::uint64_t signal{1U};
    static_cast<void>(::write(descriptor, &signal, sizeof(signal)));
  }

  bool stop_requested() noexcept {
    const int descriptor = wake_fd.load(std::memory_order_acquire);
    if (descriptor < 0) {
      return false;
    }
    std::uint64_t signal{};
    const auto result = ::read(descriptor, &signal, sizeof(signal));
    return result == static_cast<ssize_t>(sizeof(signal)) ||
           (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK);
  }

  template <class Queue>
  static bool pop(Queue& queue, std::size_t& head, std::size_t& size,
                  Frame& frame) noexcept {
    if (size == 0U) {
      return false;
    }
    frame = queue[head];
    head = (head + 1U) % queue.size();
    --size;
    return true;
  }

  SerialError write_all(const Frame& frame) noexcept {
    std::size_t offset{};
    const auto deadline = std::chrono::steady_clock::now() + config.write_timeout;
    while (offset < frame.size) {
      const auto written =
          ::write(fd, frame.data.data() + offset, frame.size - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        return SerialError::io;
      }
      std::array<struct pollfd, 2> descriptors{{
          {fd, POLLOUT, 0},
          {wake_fd.load(std::memory_order_acquire), POLLIN, 0},
      }};
      int ready{};
      for (;;) {
        const int timeout = remaining_timeout_ms(deadline);
        if (timeout == 0) {
          return SerialError::timeout;
        }
        ready = ::poll(descriptors.data(), static_cast<nfds_t>(descriptors.size()), timeout);
        if (ready >= 0 || errno != EINTR) {
          break;
        }
      }
      if (ready == 0) {
        return SerialError::timeout;
      }
      if (ready < 0 ||
          (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        return SerialError::io;
      }
      if ((descriptors[1].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) !=
          0) {
        static_cast<void>(stop_requested());
        return SerialError::closed;
      }
    }
    return SerialError::none;
  }

  void discard_stream_prefix(std::size_t count) noexcept {
    count = std::min(count, stream_size);
    std::move(stream.begin() + count, stream.begin() + stream_size,
              stream.begin());
    stream_size -= count;
  }

  bool extract_frame(Frame& frame, bool& saw_framing_error) noexcept {
    std::size_t header{};
    while (header + 1U < stream_size &&
           !(stream[header] == std::byte{0xff} &&
             stream[header + 1U] == std::byte{0xff})) {
      ++header;
    }
    if (header > 0U) {
      discard_stream_prefix(header);
    }
    if (stream_size < 4U) {
      return false;
    }
    const auto length = std::to_integer<std::uint8_t>(stream[3]);
    const auto frame_size = static_cast<std::size_t>(length) + 4U;
    if (length < 2U || frame_size > config.maximum_frame_size ||
        frame_size > frame.data.size()) {
      saw_framing_error = true;
      discard_stream_prefix(2U);
      return extract_frame(frame, saw_framing_error);
    }
    if (stream_size < frame_size) {
      return false;
    }
    frame.channel = 0U;
    frame.size = frame_size;
    std::copy_n(stream.begin(), frame_size, frame.data.begin());
    discard_stream_prefix(frame_size);
    return true;
  }

  SerialError receive_one(std::chrono::milliseconds timeout) noexcept {
    bool saw_framing_error{};
    Frame frame;
    if (extract_frame(frame, saw_framing_error)) {
      std::scoped_lock queue_lock(queue_mutex);
      if (!push(rx_queue, rx_head, rx_size, frame)) {
        return SerialError::queue_full;
      }
      return SerialError::none;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const bool nonblocking = timeout.count() == 0;
    bool polled{};
    for (;;) {
      const int wait = remaining_timeout_ms(deadline);
      if (wait == 0 && (!nonblocking || polled)) {
        return saw_framing_error ? SerialError::framing
                                 : SerialError::timeout;
      }
      std::array<struct pollfd, 2> descriptors{{
          {fd, POLLIN, 0},
          {wake_fd.load(std::memory_order_acquire), POLLIN, 0},
      }};
      int ready{};
      for (;;) {
        const auto bounded_wait = nonblocking ? 0 : remaining_timeout_ms(deadline);
        if (!nonblocking && bounded_wait == 0) {
          return saw_framing_error ? SerialError::framing
                                   : SerialError::timeout;
        }
        polled = true;
        ready = ::poll(descriptors.data(), static_cast<nfds_t>(descriptors.size()),
                       bounded_wait);
        if (ready >= 0 || errno != EINTR) {
          break;
        }
      }
      if (ready == 0) {
        return saw_framing_error ? SerialError::framing
                                 : SerialError::timeout;
      }
      if (ready < 0) {
        return SerialError::io;
      }
      if ((descriptors[1].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) !=
          0) {
        static_cast<void>(stop_requested());
        return SerialError::closed;
      }
      if ((descriptors[0].revents & (POLLERR | POLLNVAL)) != 0) {
        return SerialError::io;
      }
      if ((descriptors[0].revents & (POLLIN | POLLHUP)) == 0) {
        continue;
      }
      std::array<std::byte, kMaximumReadBuffer> incoming{};
      const auto capacity = std::min(config.read_buffer_size, incoming.size());
      const auto count = ::read(fd, incoming.data(), capacity);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      if (count <= 0) {
        return SerialError::io;
      }
      const auto amount = static_cast<std::size_t>(count);
      if (amount > stream.size() - stream_size) {
        stream_size = 0U;
        saw_framing_error = true;
        if (amount > stream.size()) {
          continue;
        }
      }
      std::copy_n(incoming.begin(), amount, stream.begin() + stream_size);
      stream_size += amount;
      while (extract_frame(frame, saw_framing_error)) {
        std::scoped_lock queue_lock(queue_mutex);
        if (!push(rx_queue, rx_head, rx_size, frame)) {
          return SerialError::queue_full;
        }
        return SerialError::none;
      }
    }
  }

  void resynchronize_after_timeout() noexcept {
    // ST3215 carries no transaction identifier. After a timed-out request,
    // quarantine one full response window and discard anything received so a
    // late same-device frame cannot satisfy the next queued transaction.
    static_cast<void>(receive_one(config.read_timeout));
    stream_size = 0U;
    static_cast<void>(::tcflush(fd, TCIFLUSH));
    std::scoped_lock queue_lock(queue_mutex);
    rx_head = 0U;
    rx_size = 0U;
  }

  void publish_error(SerialError error) noexcept {
    last_error.store(error, std::memory_order_release);
    if (error == SerialError::none) {
      health.store(TransportHealth::healthy, std::memory_order_release);
      return;
    }
    if (error == SerialError::timeout) {
      timeout_count.fetch_add(1U, std::memory_order_relaxed);
      health.store(TransportHealth::degraded, std::memory_order_release);
      return;
    }
    if (error == SerialError::framing || error == SerialError::queue_full) {
      health.store(TransportHealth::degraded, std::memory_order_release);
      return;
    }
    io_error_count.fetch_add(1U, std::memory_order_relaxed);
    health.store(TransportHealth::failed, std::memory_order_release);
  }

  SerialConfig config;
  mutable std::mutex lifecycle_mutex;
  std::mutex io_mutex;
  std::mutex queue_mutex;
  int fd{-1};
  std::atomic<int> wake_fd{-1};
  struct termios original_termios {};
  bool has_original_termios{};
  PortIdentity port_identity{};
  bool has_port_identity{};
  std::array<Frame, kQueueCapacity> tx_queue{};
  std::size_t tx_head{};
  std::size_t tx_size{};
  std::array<Frame, kQueueCapacity> rx_queue{};
  std::size_t rx_head{};
  std::size_t rx_size{};
  std::array<std::byte, 2U * kMaximumFrameStorage> stream{};
  std::size_t stream_size{};
  std::atomic<bool> opened{};
  std::atomic<TransportHealth> health{TransportHealth::failed};
  std::atomic<SerialError> last_error{SerialError::closed};
  std::atomic<std::uint64_t> timeout_count{};
  std::atomic<std::uint64_t> io_error_count{};
};

SerialTransport::SerialTransport(SerialConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

SerialTransport::~SerialTransport() { close(); }

Result<void> SerialTransport::open() {
  std::scoped_lock lifecycle_lock(impl_->lifecycle_mutex);
  if (impl_->opened.load(std::memory_order_acquire)) {
    return Result<void>::success();
  }
  const auto speed = serial_speed(impl_->config.baud_rate);
  if (impl_->config.path.empty() || !speed.has_value() ||
      impl_->config.read_buffer_size == 0U ||
      impl_->config.read_buffer_size > kMaximumReadBuffer ||
      impl_->config.maximum_frame_size < 6U ||
      impl_->config.maximum_frame_size > kMaximumFrameStorage ||
      impl_->config.read_timeout.count() < 0 ||
      impl_->config.write_timeout.count() < 0 ||
      impl_->config.read_timeout > kMaximumIoTimeout ||
      impl_->config.write_timeout > kMaximumIoTimeout) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid serial configuration"});
  }

  const int descriptor = ::open(impl_->config.path.c_str(),
                                O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) {
    if (errno == EBUSY) {
      return Result<void>::failure(
          {ErrorCode::unavailable, "serial port is already open exclusively"});
    }
    return Result<void>::failure(system_error("open serial port"));
  }
  if (::ioctl(descriptor, TIOCEXCL) != 0) {
    const auto message = std::string("claim serial port exclusively: ") +
                         std::error_code(errno, std::generic_category()).message();
    ::close(descriptor);
    return Result<void>::failure({ErrorCode::unavailable, message});
  }
  struct stat identity {};
  if (::fstat(descriptor, &identity) != 0) {
    const auto error = system_error("fstat serial port");
    ::close(descriptor);
    return Result<void>::failure(error);
  }
  const PortIdentity port_identity{
      static_cast<std::uint64_t>(identity.st_dev),
      static_cast<std::uint64_t>(identity.st_ino),
      static_cast<std::uint64_t>(identity.st_rdev)};
  {
    std::scoped_lock ports_lock(open_ports_mutex);
    if (!open_ports.insert(port_identity).second) {
      ::close(descriptor);
      return Result<void>::failure(
          {ErrorCode::unavailable,
           "serial port is already open in this process"});
    }
  }

  struct termios original {};
  if (::tcgetattr(descriptor, &original) != 0) {
    const auto error = system_error("tcgetattr serial port");
    {
      std::scoped_lock ports_lock(open_ports_mutex);
      open_ports.erase(port_identity);
    }
    ::close(descriptor);
    return Result<void>::failure(error);
  }
  auto configured = original;
  ::cfmakeraw(&configured);
  configured.c_cflag |= CLOCAL | CREAD;
  configured.c_cflag &= ~CSTOPB;
  configured.c_cflag &= ~CRTSCTS;
  configured.c_cflag &= ~CSIZE;
  configured.c_cflag |= CS8;
  configured.c_cc[VMIN] = 0;
  configured.c_cc[VTIME] = 0;
  if (::cfsetispeed(&configured, *speed) != 0 ||
      ::cfsetospeed(&configured, *speed) != 0 ||
      ::tcsetattr(descriptor, TCSANOW, &configured) != 0 ||
      ::tcflush(descriptor, TCIOFLUSH) != 0) {
    const auto error = system_error("configure serial port");
    static_cast<void>(::tcsetattr(descriptor, TCSANOW, &original));
    {
      std::scoped_lock ports_lock(open_ports_mutex);
      open_ports.erase(port_identity);
    }
    ::close(descriptor);
    return Result<void>::failure(error);
  }
  const int wake_descriptor = ::eventfd(0U, EFD_NONBLOCK | EFD_CLOEXEC);
  if (wake_descriptor < 0) {
    const auto error = system_error("create serial stop wakeup");
    static_cast<void>(::tcsetattr(descriptor, TCSANOW, &original));
    {
      std::scoped_lock ports_lock(open_ports_mutex);
      open_ports.erase(port_identity);
    }
    ::close(descriptor);
    return Result<void>::failure(error);
  }

  impl_->fd = descriptor;
  impl_->wake_fd.store(wake_descriptor, std::memory_order_release);
  impl_->original_termios = original;
  impl_->has_original_termios = true;
  impl_->port_identity = port_identity;
  impl_->has_port_identity = true;
  impl_->stream_size = 0U;
  {
    std::scoped_lock queue_lock(impl_->queue_mutex);
    impl_->tx_head = 0U;
    impl_->tx_size = 0U;
    impl_->rx_head = 0U;
    impl_->rx_size = 0U;
  }
  impl_->last_error.store(SerialError::none, std::memory_order_release);
  impl_->health.store(TransportHealth::healthy, std::memory_order_release);
  impl_->opened.store(true, std::memory_order_release);
  return Result<void>::success();
}

void SerialTransport::close() noexcept {
  std::scoped_lock lifecycle_lock(impl_->lifecycle_mutex);
  if (!impl_->opened.exchange(false, std::memory_order_acq_rel) &&
      impl_->fd < 0) {
    return;
  }
  impl_->request_stop();
  std::scoped_lock io_lock(impl_->io_mutex);
  if (impl_->fd >= 0) {
    if (impl_->has_original_termios) {
      static_cast<void>(
          ::tcsetattr(impl_->fd, TCSANOW, &impl_->original_termios));
    }
    // Keep the kernel exclusion claim through termios restoration so another
    // opener cannot race with teardown and observe partially restored state.
    static_cast<void>(::ioctl(impl_->fd, TIOCNXCL));
    static_cast<void>(::close(impl_->fd));
    impl_->fd = -1;
  }
  if (impl_->has_port_identity) {
    std::scoped_lock ports_lock(open_ports_mutex);
    open_ports.erase(impl_->port_identity);
    impl_->has_port_identity = false;
  }
  const int wake_descriptor = impl_->wake_fd.exchange(-1, std::memory_order_acq_rel);
  if (wake_descriptor >= 0) {
    static_cast<void>(::close(wake_descriptor));
  }
  impl_->has_original_termios = false;
  impl_->last_error.store(SerialError::closed, std::memory_order_release);
  impl_->health.store(TransportHealth::failed, std::memory_order_release);
}

void SerialTransport::request_stop() noexcept { impl_->request_stop(); }

TransportHealth SerialTransport::health() const noexcept {
  return impl_->health.load(std::memory_order_acquire);
}

SchedulingClass SerialTransport::scheduling_class() const noexcept {
  return SchedulingClass::blocking_event_driven;
}

void SerialTransport::cycle(const CycleContext&) noexcept {
  try {
    std::scoped_lock io_lock(impl_->io_mutex);
    if (!impl_->opened.load(std::memory_order_acquire) || impl_->fd < 0) {
      impl_->publish_error(SerialError::closed);
      return;
    }
    if (impl_->last_error.load(std::memory_order_acquire) ==
        SerialError::timeout) {
      impl_->resynchronize_after_timeout();
      impl_->last_error.store(SerialError::none, std::memory_order_release);
    }
    Impl::Frame outgoing;
    bool has_outgoing;
    {
      std::scoped_lock queue_lock(impl_->queue_mutex);
      has_outgoing = Impl::pop(impl_->tx_queue, impl_->tx_head,
                               impl_->tx_size, outgoing);
    }
    if (!has_outgoing) {
      const auto receive_error =
          impl_->receive_one(std::chrono::milliseconds{0});
      if (receive_error != SerialError::timeout) {
        impl_->publish_error(receive_error);
      }
      return;
    }
    const auto write_error = impl_->write_all(outgoing);
    if (write_error != SerialError::none) {
      impl_->publish_error(write_error);
      return;
    }
    impl_->publish_error(impl_->receive_one(impl_->config.read_timeout));
  } catch (...) {
    impl_->publish_error(SerialError::internal);
  }
}

Result<void> SerialTransport::write(ChannelId channel,
                                    std::span<const std::byte> data) {
  if (!impl_->opened.load(std::memory_order_acquire)) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "serial port is closed"});
  }
  if (data.empty() || data.size() > impl_->config.maximum_frame_size ||
      data.size() > kMaximumFrameStorage) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid serial frame size"});
  }
  Impl::Frame frame;
  frame.channel = channel;
  frame.size = data.size();
  std::copy(data.begin(), data.end(), frame.data.begin());
  std::scoped_lock queue_lock(impl_->queue_mutex);
  if (!Impl::push(impl_->tx_queue, impl_->tx_head, impl_->tx_size, frame)) {
    impl_->last_error.store(SerialError::queue_full,
                            std::memory_order_release);
    impl_->health.store(TransportHealth::degraded,
                        std::memory_order_release);
    return Result<void>::failure(
        {ErrorCode::unavailable, "serial transmit queue is full"});
  }
  return Result<void>::success();
}

Result<std::size_t> SerialTransport::read(ChannelId channel,
                                          std::span<std::byte> buffer) {
  if (!impl_->opened.load(std::memory_order_acquire)) {
    return Result<std::size_t>::failure(
        {ErrorCode::unavailable, "serial port is closed"});
  }
  std::scoped_lock queue_lock(impl_->queue_mutex);
  if (impl_->rx_size == 0U) {
    const auto last_error = impl_->last_error.load(std::memory_order_acquire);
    const auto code = last_error == SerialError::timeout ? ErrorCode::timeout
                                                         : ErrorCode::unavailable;
    return Result<std::size_t>::failure(
        {code, last_error == SerialError::timeout
                   ? "serial response timed out"
                   : "no serial frame is available"});
  }
  const auto& frame = impl_->rx_queue[impl_->rx_head];
  if (frame.channel != channel) {
    return Result<std::size_t>::failure(
        {ErrorCode::unavailable, "no serial frame for channel"});
  }
  if (buffer.size() < frame.size) {
    return Result<std::size_t>::failure(
        {ErrorCode::invalid_argument, "serial receive buffer is too small"});
  }
  std::copy_n(frame.data.begin(), frame.size, buffer.begin());
  const auto count = frame.size;
  Impl::Frame ignored;
  static_cast<void>(Impl::pop(impl_->rx_queue, impl_->rx_head,
                              impl_->rx_size, ignored));
  return Result<std::size_t>::success(count);
}

SerialError SerialTransport::last_error() const noexcept {
  return impl_->last_error.load(std::memory_order_acquire);
}

std::uint64_t SerialTransport::timeout_count() const noexcept {
  return impl_->timeout_count.load(std::memory_order_acquire);
}

std::uint64_t SerialTransport::io_error_count() const noexcept {
  return impl_->io_error_count.load(std::memory_order_acquire);
}

bool SerialTransport::is_open() const noexcept {
  return impl_->opened.load(std::memory_order_acquire);
}

const SerialConfig& SerialTransport::config() const noexcept {
  return impl_->config;
}

}  // namespace policy_runtime
