#include <array>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"
#include "policy_runtime/transport/serial/virtual_can_transport.hpp"

namespace {

using policy_runtime::CanFrame;
using policy_runtime::SocketCanTransport;
using policy_runtime::VirtualCanTransport;

void set_nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  ASSERT_GE(flags, 0);
  ASSERT_GE(fcntl(fd, F_SETFL, flags | O_NONBLOCK), 0);
}

CanFrame sample_frame() {
  CanFrame frame{};
  frame.id = 0x101U;
  frame.dlc = 8U;
  frame.data = {std::byte{0x0A}, std::byte{0x80}, std::byte{0x00},
                std::byte{0x80}, std::byte{0x08}, std::byte{0x00},
                std::byte{40},   std::byte{41}};
  return frame;
}

void wait_readable(int fd) {
  struct pollfd pfd{fd, POLLIN, 0};
  ASSERT_GE(poll(&pfd, 1, 1000), 1);
}

void read_exact(int fd, std::byte* data, std::size_t length) {
  std::size_t done = 0U;
  while (done < length) {
    wait_readable(fd);
    const auto n = ::read(fd, data + done, length - done);
    ASSERT_GT(n, 0);
    done += static_cast<std::size_t>(n);
  }
}

TEST(SocketCanTransportTest, RoundTripsOneFramePerCall) {
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);
  set_nonblocking(fds[0]);
  set_nonblocking(fds[1]);

  SocketCanTransport transport(fds[0]);
  ASSERT_TRUE(transport.send_frame(sample_frame()).has_value());
  SocketCanTransport peer(fds[1]);
  auto peer_frame = peer.receive_frame();
  ASSERT_TRUE(peer_frame.has_value());
  ASSERT_TRUE(peer_frame.value().has_value());
  EXPECT_EQ(peer_frame.value().value().id, 0x101U);
  EXPECT_EQ(peer_frame.value().value().dlc, 8U);
  EXPECT_EQ(peer_frame.value().value().data, sample_frame().data);

  close(fds[0]);
  close(fds[1]);
}

TEST(SocketCanTransportTest, EmptyReadIsReportedAsIoFault) {
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);
  set_nonblocking(fds[0]);
  set_nonblocking(fds[1]);

  SocketCanTransport transport(fds[0]);
  // EAGAIN on an empty nonblocking descriptor surfaces as a fault result.
  EXPECT_FALSE(transport.receive_frame().has_value());

  close(fds[0]);
  close(fds[1]);
}

TEST(SocketCanTransportTest, RejectsInvalidFrameFields) {
  int fds[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);
  SocketCanTransport transport(fds[0]);

  auto invalid_id = sample_frame();
  invalid_id.id = 0x800U;
  EXPECT_FALSE(transport.send_frame(invalid_id).has_value());

  auto invalid_dlc = sample_frame();
  invalid_dlc.dlc = 9U;
  EXPECT_FALSE(transport.send_frame(invalid_dlc).has_value());

  close(fds[0]);
  close(fds[1]);
}

class PtyFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_EQ(openpty(&master_, &slave_, nullptr, nullptr, nullptr), 0);
    struct termios tio;
    ASSERT_EQ(tcgetattr(slave_, &tio), 0);
    cfmakeraw(&tio);
    ASSERT_EQ(tcsetattr(slave_, TCSANOW, &tio), 0);
    ASSERT_EQ(fcntl(master_, F_SETFL, fcntl(master_, F_GETFL, 0) | O_NONBLOCK),
              0);
    ASSERT_EQ(fcntl(slave_, F_SETFL, fcntl(slave_, F_GETFL, 0) | O_NONBLOCK),
              0);
  }
  void TearDown() override {
    close(master_);
    close(slave_);
  }
  int master_{-1};
  int slave_{-1};
};

TEST_F(PtyFixture, VirtualCanRoundTripsFramedBytes) {
  VirtualCanTransport transport(master_);
  ASSERT_TRUE(transport.send_frame(sample_frame()).has_value());

  VirtualCanTransport peer(slave_);
  wait_readable(slave_);
  auto received = peer.receive_frame();
  ASSERT_TRUE(received.has_value());
  ASSERT_TRUE(received.value().has_value());
  const auto& frame = received.value().value();
  EXPECT_EQ(frame.id, 0x101U);
  EXPECT_EQ(frame.dlc, 8U);
  EXPECT_EQ(frame.data, sample_frame().data);
}

TEST_F(PtyFixture, PartialReadsAccumulateUntilFrameIsComplete) {
  VirtualCanTransport transport(master_);
  ASSERT_TRUE(transport.send_frame(sample_frame()).has_value());

  VirtualCanTransport peer(slave_);
  // PTYs deliver writes in chunks; each receive_frame() call consumes one
  // chunk and partial frames accumulate across calls until complete.
  auto received = peer.receive_frame();
  ASSERT_TRUE(received.has_value());
  while (!received.value().has_value()) {
    wait_readable(slave_);
    received = peer.receive_frame();
    ASSERT_TRUE(received.has_value());
  }
  ASSERT_TRUE(received.value().has_value());
  EXPECT_EQ(peer.pending_bytes(), 0U);
}

TEST_F(PtyFixture, ChecksumCorruptionIsRejected) {
  VirtualCanTransport transport(master_);
  ASSERT_TRUE(transport.send_frame(sample_frame()).has_value());
  // Drain the full 15-byte framed message and flip the trailing checksum.
  std::byte wire[15];
  read_exact(slave_, wire, sizeof(wire));
  wire[sizeof(wire) - 1U] =
      static_cast<std::byte>(std::to_integer<int>(wire[sizeof(wire) - 1U]) ^
                             0xFF);
  ASSERT_EQ(static_cast<std::size_t>(write(master_, wire, sizeof(wire))),
            sizeof(wire));
  VirtualCanTransport peer(slave_);
  bool rejected = false;
  while (peer.pending_bytes() < sizeof(wire) && !rejected) {
    wait_readable(slave_);
    const auto result = peer.receive_frame();
    if (!result.has_value()) {
      rejected = true;
    }
  }
  EXPECT_TRUE(rejected);
}

TEST_F(PtyFixture, MalformedHeaderIsRejected) {
  const std::byte garbage[] = {std::byte{0x55}, std::byte{0x00},
                               std::byte{0x00}, std::byte{0x00},
                               std::byte{0x00}, std::byte{0x08},
                               std::byte{0x00}};
  ASSERT_EQ(static_cast<std::size_t>(write(master_, garbage, sizeof(garbage))),
            sizeof(garbage));
  VirtualCanTransport peer(slave_);
  EXPECT_FALSE(peer.receive_frame().has_value());
}

}  // namespace
