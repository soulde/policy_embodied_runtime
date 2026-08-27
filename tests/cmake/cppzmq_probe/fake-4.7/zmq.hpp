#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace zmq {

enum class socket_type { rep };
enum class recv_flags { none };
enum class send_flags { none };

namespace sockopt {
struct linger_t {};
struct rcvtimeo_t {};
struct sndtimeo_t {};
inline constexpr linger_t linger;
inline constexpr rcvtimeo_t rcvtimeo;
inline constexpr sndtimeo_t sndtimeo;
}  // namespace sockopt

class context_t {
 public:
  explicit context_t(int) {}
};

class message_t {
 public:
  void* data() noexcept { return nullptr; }
  std::size_t size() const noexcept { return 0; }
};

struct const_buffer {};
inline const_buffer buffer(const std::string&) { return {}; }

class error_t {
 public:
  int num() const noexcept { return 0; }
};

class socket_t {
 public:
  socket_t(context_t&, socket_type) {}

  template <class Option, class Value>
  void set(Option, Value) {}

  void bind(const std::string&) {}
  std::optional<std::size_t> recv(message_t&, recv_flags) { return 0; }
  std::optional<std::size_t> send(const_buffer, send_flags) { return 0; }
};

}  // namespace zmq
