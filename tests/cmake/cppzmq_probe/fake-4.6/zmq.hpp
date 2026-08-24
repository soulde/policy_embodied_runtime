#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace zmq {

enum class socket_type { rep };
enum class recv_flags { none };
enum class send_flags { none };

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
  void setsockopt(int, const void*, std::size_t) {}
  void bind(const std::string&) {}
  std::optional<std::size_t> recv(message_t&, recv_flags) { return 0; }
  std::optional<std::size_t> send(const_buffer, send_flags) { return 0; }
};

}  // namespace zmq
