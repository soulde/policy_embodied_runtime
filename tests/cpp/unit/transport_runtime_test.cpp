#include <array>
#include <chrono>
#include <cstddef>
#include <future>
#include <fcntl.h>
#include <linux/can.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "policy_runtime/robot_io/daemon/transport_runtime.hpp"

namespace policy_runtime::robot_io {
namespace {

TEST(TransportRuntimeTest, SendsRawFramesWithoutKnowingDeviceProtocol) {
  int sockets[2]{};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);
  ASSERT_GE(::fcntl(sockets[0], F_SETFL, O_NONBLOCK), 0);
  SocketCanTransport transport(sockets[0]);
  std::promise<DeviceFrame> received_promise;
  auto received = received_promise.get_future();
  auto runtime = TransportRuntime::create(
      std::move(transport),
      [&received_promise](const DeviceFrame& frame, std::uint64_t) {
        received_promise.set_value(frame);
      });
  ASSERT_TRUE(runtime.has_value());
  auto instance = std::move(runtime.value());
  ASSERT_TRUE(instance.start().has_value());

  DeviceFrame frame;
  frame.sequence = 1U;
  frame.address = 0x123U;
  frame.size = 3U;
  frame.bytes[0] = std::byte{0x11};
  frame.bytes[1] = std::byte{0x22};
  frame.bytes[2] = std::byte{0x33};
  ASSERT_TRUE(instance.stage(0U, frame).has_value());

  struct pollfd peer{.fd = sockets[1], .events = POLLIN};
  ASSERT_EQ(::poll(&peer, 1, 1000), 1);
  ::can_frame wire{};
  ASSERT_EQ(::recv(sockets[1], &wire, sizeof(wire), 0),
            static_cast<ssize_t>(sizeof(wire)));
  EXPECT_EQ(wire.can_id, 0x123U);
  EXPECT_EQ(wire.can_dlc, 3U);
  EXPECT_EQ(wire.data[0], 0x11U);
  EXPECT_EQ(wire.data[1], 0x22U);
  EXPECT_EQ(wire.data[2], 0x33U);
  instance.stop();
  ::close(sockets[1]);
}

}  // namespace
}  // namespace policy_runtime::robot_io
