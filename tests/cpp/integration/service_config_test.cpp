#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
#include "policy_runtime/robot_io/service_listener.hpp"
#include "policy_runtime/runtime/robot_io_client.hpp"
#include "policy_runtime/runtime/robot_io_service.hpp"

namespace {

std::filesystem::path source_path(const char* relative_path) {
  return std::filesystem::path(POLICY_RUNTIME_SOURCE_DIR) / relative_path;
}

std::string read_text(const std::filesystem::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::string invoke_tool(const char* name, const char* argument, int& status) {
  const auto output = std::filesystem::temp_directory_path() /
                      (std::string("policy-runtime-") + name + ".out");
  const auto tool = std::filesystem::path(POLICY_RUNTIME_TOOL_DIR) / name;
  const auto command = "\"" + tool.string() + "\" " + argument +
                       " > \"" + output.string() + "\" 2>&1";
  status = std::system(command.c_str());
  std::ifstream input(output);
  const std::string text{std::istreambuf_iterator<char>(input),
                         std::istreambuf_iterator<char>()};
  std::filesystem::remove(output);
  return text;
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    std::array<char, 64> path{};
    const auto pattern =
        (std::filesystem::temp_directory_path() /
         "policy-runtime-service-XXXXXX")
            .string();
    if (pattern.size() >= path.size()) {
      throw std::runtime_error("temporary directory template is too long");
    }
    std::copy(pattern.begin(), pattern.end(), path.begin());
    const auto* created = mkdtemp(path.data());
    if (created == nullptr) {
      throw std::runtime_error("could not create temporary directory");
    }
    path_ = created;
  }

  TemporaryDirectory(const TemporaryDirectory&) = delete;
  TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path& path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

class ChildProcess {
 public:
  explicit ChildProcess(pid_t pid) noexcept : pid_(pid) {}
  ChildProcess(const ChildProcess&) = delete;
  ChildProcess& operator=(const ChildProcess&) = delete;
  ~ChildProcess() {
    if (pid_ > 0) {
      static_cast<void>(kill(pid_, SIGKILL));
      static_cast<void>(waitpid(pid_, nullptr, 0));
    }
  }

  bool exited_within(std::chrono::milliseconds timeout, int& status) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto waited = waitpid(pid_, &status, WNOHANG);
      if (waited == pid_) {
        pid_ = -1;
        return true;
      }
      if (waited < 0 && errno != EINTR) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return false;
  }

  bool poll_exit(int& status) {
    const auto waited = waitpid(pid_, &status, WNOHANG);
    if (waited == pid_) {
      pid_ = -1;
      return true;
    }
    return false;
  }

  pid_t pid() const noexcept { return pid_; }

 private:
  pid_t pid_{-1};
};

std::optional<std::uint32_t> wait_for_generation(
    const std::filesystem::path& generation_path, ChildProcess& process) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{3};
  while (std::chrono::steady_clock::now() < deadline) {
    std::ifstream input(generation_path);
    std::uint64_t generation{};
    if (input >> generation && generation > 0U &&
        generation <= UINT32_MAX) {
      return static_cast<std::uint32_t>(generation);
    }
    int status{};
    if (process.poll_exit(status)) {
      return std::nullopt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return std::nullopt;
}

int connect_seqpacket(const std::filesystem::path& socket_path) {
  const int descriptor = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (descriptor < 0) {
    return -1;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  const auto path = socket_path.string();
  if (path.size() >= sizeof(address.sun_path)) {
    close(descriptor);
    return -1;
  }
  std::copy(path.begin(), path.end(), address.sun_path);
  if (connect(descriptor, reinterpret_cast<const sockaddr*>(&address),
              sizeof(address)) != 0) {
    close(descriptor);
    return -1;
  }
  timeval timeout{2, 0};
  static_cast<void>(setsockopt(descriptor, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                              sizeof(timeout)));
  return descriptor;
}

void write_empty_robot_profile(const std::filesystem::path& profile_path) {
  std::ofstream profile(profile_path);
  profile << R"({"sensors":[],"actuators":[]})";
}

ChildProcess start_service_daemon(
    const std::filesystem::path& profile_path,
    const std::filesystem::path& socket_path,
    const std::filesystem::path& generation_path) {
  const auto socket_option = "--ipc-socket=" + socket_path.string();
  const auto generation_option =
      "--generation-file=" + generation_path.string();
  const auto child = fork();
  if (child == 0) {
    execl(POLICY_RUNTIME_DAEMON_PATH, "robot-io-daemon",
          profile_path.c_str(), socket_option.c_str(), generation_option.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  return ChildProcess(child);
}

ChildProcess start_runtime_host(
    const std::filesystem::path& robot_profile,
    const std::filesystem::path& socket_path,
    const std::filesystem::path& generation_path,
    const std::filesystem::path& endpoint_path) {
  const auto endpoint = "ipc://" + endpoint_path.string();
  const auto child = fork();
  if (child == 0) {
    execl(POLICY_RUNTIME_HOST_PATH, "policy-runtime-host",
          "--policy-profile", POLICY_RUNTIME_FAKE_POLICY_PATH,
          "--robot-profile", robot_profile.c_str(), "--endpoint",
          endpoint.c_str(), "--robot-io-socket", socket_path.c_str(),
          "--robot-io-generation-file", generation_path.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  return ChildProcess(child);
}

#if POLICY_RUNTIME_HAS_ZMQ
bool wait_for_path(const std::filesystem::path& path,
                   std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::exists(path)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return false;
}
#endif

std::uint32_t run_service_incarnation(const std::filesystem::path& directory) {
  const auto profile_path = directory / "empty-profile.json";
  const auto socket_path = directory / "robot-io.sock";
  const auto generation_path = directory / "robot-io.generation";
  {
    std::ofstream profile(profile_path);
    profile << R"({"sensors":[],"actuators":[]})";
  }

  const auto socket_option = "--ipc-socket=" + socket_path.string();
  const auto generation_option =
      "--generation-file=" + generation_path.string();
  const auto child = fork();
  EXPECT_GE(child, 0);
  if (child == 0) {
    execl(POLICY_RUNTIME_DAEMON_PATH, "robot-io-daemon",
          profile_path.c_str(), socket_option.c_str(), generation_option.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  if (child < 0) {
    return 0;
  }
  ChildProcess process(child);
  const auto generation = wait_for_generation(generation_path, process);
  EXPECT_TRUE(generation.has_value());
  if (!generation.has_value()) {
    return 0;
  }

  struct stat socket_status {};
  if (lstat(socket_path.c_str(), &socket_status) != 0) {
    ADD_FAILURE() << "listener socket was not created";
    return 0;
  }
  EXPECT_TRUE(S_ISSOCK(socket_status.st_mode));
  EXPECT_EQ(socket_status.st_mode & 0777, 0600);
  EXPECT_EQ(socket_status.st_uid, geteuid());

  const auto connected = connect_seqpacket(socket_path);
  if (connected < 0) {
    ADD_FAILURE() << "could not connect to listener socket";
    return 0;
  }
  auto client = policy_runtime::RobotIoClient::connect(connected, *generation);
  if (!client.has_value()) {
    ADD_FAILURE() << client.error().message;
    return 0;
  }
  EXPECT_EQ(client.value().axis_count(), 0U);
  EXPECT_EQ(client.value().generation(), *generation);

  EXPECT_EQ(kill(process.pid(), SIGTERM), 0);
  int status{};
  if (!process.exited_within(std::chrono::seconds{3}, status)) {
    ADD_FAILURE() << "daemon did not stop within three seconds";
    return 0;
  }
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_FALSE(std::filesystem::exists(socket_path));
  EXPECT_FALSE(std::filesystem::exists(generation_path));
  return *generation;
}

TEST(ServiceConfigTest, ExampleProfileHasStaticModesAndTimeouts) {
  auto profile = policy_runtime::profiles::load_robot_profile(
      source_path("configs/robot_profiles/elmo_gold_example.json"));

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().axes.size(), 3U);

  std::array<bool, 3> modes{};
  for (const auto& axis : profile.value().axes) {
    EXPECT_GT(axis.command_timeout, std::chrono::milliseconds::zero());
    EXPECT_FALSE(axis.name.empty());
    switch (axis.mode) {
      case policy_runtime::profiles::Cia402Mode::csp:
        modes[0] = true;
        break;
      case policy_runtime::profiles::Cia402Mode::csv:
        modes[1] = true;
        break;
      case policy_runtime::profiles::Cia402Mode::cst:
        modes[2] = true;
        break;
    }
  }
  EXPECT_EQ(modes, (std::array<bool, 3>{true, true, true}));
}

TEST(ServiceConfigTest, DiagnosticToolsAreSafeWhenNoDaemonIsAvailable) {
  for (const char* tool : {"robot-io-health", "ethercat-cycle-stats"}) {
    int help_status{};
    const auto help = invoke_tool(tool, "--help", help_status);
    EXPECT_EQ(help_status, 0) << tool << ": " << help;
    EXPECT_NE(help.find("Usage:"), std::string::npos) << tool << ": " << help;

    int diagnostic_status{};
    const auto diagnostic = invoke_tool(tool, "", diagnostic_status);
    EXPECT_NE(diagnostic_status, 0) << tool;
    EXPECT_NE(diagnostic.find("unavailable"), std::string::npos)
        << tool << ": " << diagnostic;
  }
}

TEST(ServiceConfigTest, PackagedUnitStartsTheDirectServiceListener) {
  const auto unit = read_text(POLICY_RUNTIME_SYSTEMD_UNIT_PATH);

  EXPECT_NE(unit.find("RuntimeDirectory=policy-runtime"), std::string::npos);
  EXPECT_NE(unit.find("--ipc-socket=/run/policy-runtime/robot-io.sock"),
            std::string::npos);
  EXPECT_NE(unit.find(
                "--generation-file=/run/policy-runtime/robot-io.generation"),
            std::string::npos);
  EXPECT_EQ(unit.find("ROBOT_IO_CONNECTED_SOCKET_FD"), std::string::npos);
  EXPECT_EQ(unit.find("ROBOT_IO_GENERATION=1"), std::string::npos);
}

TEST(ServiceConfigTest, DaemonHelpDoesNotTouchHardware) {
  int status{};
  const auto help = invoke_tool("robot-io-daemon", "--help", status);
  EXPECT_EQ(status, 0) << help;
  EXPECT_NE(help.find("Usage:"), std::string::npos) << help;
}

TEST(ServiceConfigTest,
     DirectListenerStartupUsesFreshGenerationAndCleansRuntimeFiles) {
  TemporaryDirectory directory;
  const auto first = run_service_incarnation(directory.path());
  const auto second = run_service_incarnation(directory.path());

  EXPECT_NE(first, 0U);
  EXPECT_NE(second, 0U);
  EXPECT_NE(first, second);
}

TEST(ServiceConfigTest,
     ConcurrentListenerHandoffKeepsSuccessorGenerationPublished) {
  TemporaryDirectory directory;
  const auto socket_path = directory.path() / "robot-io.sock";
  const auto generation_path = directory.path() / "robot-io.generation";

  for (int iteration = 0; iteration < 100; ++iteration) {
    auto created = policy_runtime::RobotIoServiceListener::create(
        socket_path.string(), generation_path.string(), geteuid());
    ASSERT_TRUE(created.has_value()) << created.error().message;
    std::optional<policy_runtime::RobotIoServiceListener> first;
    first.emplace(std::move(created.value()));

    std::atomic_bool start{};
    std::atomic_bool attempted{};
    std::optional<policy_runtime::RobotIoServiceListener> successor;
    std::string unexpected_error;
    std::thread handoff([&] {
      while (!start.load(std::memory_order_acquire)) {
      }
      for (;;) {
        auto next = policy_runtime::RobotIoServiceListener::create(
            socket_path.string(), generation_path.string(), geteuid());
        attempted.store(true, std::memory_order_release);
        if (next.has_value()) {
          successor.emplace(std::move(next.value()));
          return;
        }
        if (next.error().code != policy_runtime::ErrorCode::unavailable) {
          unexpected_error = next.error().message;
          return;
        }
        std::this_thread::yield();
      }
    });
    start.store(true, std::memory_order_release);
    while (!attempted.load(std::memory_order_acquire)) {
    }
    first.reset();
    handoff.join();

    ASSERT_TRUE(unexpected_error.empty()) << unexpected_error;
    ASSERT_TRUE(successor.has_value());
    std::ifstream published(generation_path);
    std::uint64_t generation{};
    ASSERT_TRUE(published >> generation);
    EXPECT_EQ(generation, successor->generation());
    successor.reset();
    EXPECT_FALSE(std::filesystem::exists(socket_path));
    EXPECT_FALSE(std::filesystem::exists(generation_path));
  }
}

TEST(ServiceConfigTest, RuntimeHostRejectsUnsafeGenerationFilesBeforeConnect) {
  TemporaryDirectory directory;
  const auto socket_path = directory.path() / "robot-io.sock";
  const auto generation_path = directory.path() / "robot-io.generation";
  {
    std::ofstream generation(generation_path);
    generation << "not-a-generation\n";
  }
  ASSERT_EQ(chmod(generation_path.c_str(), 0600), 0);

  auto malformed = policy_runtime::connect_robot_io_service(
      socket_path.string(), generation_path.string(), 100);

  ASSERT_FALSE(malformed.has_value());
  EXPECT_EQ(malformed.error().code,
            policy_runtime::ErrorCode::invalid_argument);

  const auto link_path = directory.path() / "generation-link";
  ASSERT_EQ(symlink(generation_path.c_str(), link_path.c_str()), 0);
  auto linked = policy_runtime::connect_robot_io_service(
      socket_path.string(), link_path.string(), 100);

  ASSERT_FALSE(linked.has_value());
  EXPECT_EQ(linked.error().code, policy_runtime::ErrorCode::invalid_argument);
}

TEST(ServiceConfigTest,
     PackagedRuntimeHostRejectsMismatchAndConnectsAfterDaemonRestart) {
#if !POLICY_RUNTIME_HAS_ZMQ
  GTEST_SKIP() << "packaged runtime host requires ZeroMQ for its RPC loop";
#endif
  TemporaryDirectory directory;
  const auto profile_path = directory.path() / "empty-profile.json";
  const auto socket_path = directory.path() / "robot-io.sock";
  const auto generation_path = directory.path() / "robot-io.generation";
  write_empty_robot_profile(profile_path);

  auto first_daemon =
      start_service_daemon(profile_path, socket_path, generation_path);
  ASSERT_GT(first_daemon.pid(), 0);
  const auto first_generation =
      wait_for_generation(generation_path, first_daemon);
  ASSERT_TRUE(first_generation.has_value());
  const auto mismatched_generation =
      *first_generation == std::numeric_limits<std::uint32_t>::max()
          ? *first_generation - 1U
          : *first_generation + 1U;
  {
    std::ofstream generation(generation_path, std::ios::trunc);
    generation << mismatched_generation << '\n';
  }

  auto stale_host = start_runtime_host(
      profile_path, socket_path, generation_path,
      directory.path() / "stale-host.sock");
  ASSERT_GT(stale_host.pid(), 0);
  int stale_host_status{};
  ASSERT_TRUE(stale_host.exited_within(std::chrono::seconds{3},
                                       stale_host_status));
  EXPECT_TRUE(WIFEXITED(stale_host_status));
  EXPECT_EQ(WEXITSTATUS(stale_host_status), 1);
  const auto unexpected_second_client = connect_seqpacket(socket_path);
  EXPECT_LT(unexpected_second_client, 0)
      << "runtime host did not consume the daemon's one-client listener";
  if (unexpected_second_client >= 0) {
    close(unexpected_second_client);
  }

  ASSERT_EQ(kill(first_daemon.pid(), SIGTERM), 0);
  int first_daemon_status{};
  ASSERT_TRUE(first_daemon.exited_within(std::chrono::seconds{3},
                                         first_daemon_status));
  ASSERT_TRUE(WIFEXITED(first_daemon_status));
  ASSERT_EQ(WEXITSTATUS(first_daemon_status), 0);
  ASSERT_FALSE(std::filesystem::exists(generation_path));

  auto second_daemon =
      start_service_daemon(profile_path, socket_path, generation_path);
  ASSERT_GT(second_daemon.pid(), 0);
  const auto second_generation =
      wait_for_generation(generation_path, second_daemon);
  ASSERT_TRUE(second_generation.has_value());
  ASSERT_NE(*second_generation, *first_generation);

  const auto endpoint_path = directory.path() / "fresh-host.sock";
  auto fresh_host = start_runtime_host(profile_path, socket_path,
                                       generation_path, endpoint_path);
  ASSERT_GT(fresh_host.pid(), 0);
  int fresh_host_status{};
#if POLICY_RUNTIME_HAS_ZMQ
  ASSERT_TRUE(wait_for_path(endpoint_path, std::chrono::seconds{3}));
  EXPECT_FALSE(fresh_host.poll_exit(fresh_host_status));
  ASSERT_EQ(kill(fresh_host.pid(), SIGTERM), 0);
  ASSERT_TRUE(fresh_host.exited_within(std::chrono::seconds{3},
                                       fresh_host_status));
  EXPECT_TRUE(WIFEXITED(fresh_host_status));
  EXPECT_EQ(WEXITSTATUS(fresh_host_status), 0);
#else
  ASSERT_TRUE(fresh_host.exited_within(std::chrono::seconds{3},
                                       fresh_host_status));
  EXPECT_TRUE(WIFEXITED(fresh_host_status));
  EXPECT_EQ(WEXITSTATUS(fresh_host_status), 1);
#endif
  const auto second_unexpected_client = connect_seqpacket(socket_path);
  EXPECT_LT(second_unexpected_client, 0)
      << "runtime host did not connect after daemon restart";
  if (second_unexpected_client >= 0) {
    close(second_unexpected_client);
  }
  ASSERT_EQ(kill(second_daemon.pid(), SIGTERM), 0);
  int second_daemon_status{};
  ASSERT_TRUE(second_daemon.exited_within(std::chrono::seconds{3},
                                          second_daemon_status));
  EXPECT_TRUE(WIFEXITED(second_daemon_status));
  EXPECT_EQ(WEXITSTATUS(second_daemon_status), 0);
}

TEST(ServiceConfigTest, ServiceListenerRejectsDifferentEffectiveUser) {
  TemporaryDirectory directory;
  const auto socket_path = directory.path() / "robot-io.sock";
  const auto generation_path = directory.path() / "robot-io.generation";
  const auto current_uid = geteuid();
  const auto different_uid =
      current_uid == std::numeric_limits<uid_t>::max() ? current_uid - 1U
                                                       : current_uid + 1U;
  auto listener = policy_runtime::RobotIoServiceListener::create(
      socket_path.string(), generation_path.string(), different_uid);
  ASSERT_TRUE(listener.has_value()) << listener.error().message;
  policy_runtime::DaemonSignalLatch signals;
  ASSERT_TRUE(signals.install().has_value());
  const auto client = connect_seqpacket(socket_path);
  ASSERT_GE(client, 0);

  auto accepted = listener.value().accept_authenticated(signals);

  EXPECT_FALSE(accepted.has_value());
  EXPECT_EQ(accepted.error().code, policy_runtime::ErrorCode::unavailable);
  close(client);
}

TEST(ServiceConfigTest, DirectListenerCleansFilesWhenStoppedBeforeAccept) {
  TemporaryDirectory directory;
  const auto profile_path = directory.path() / "empty-profile.json";
  const auto socket_path = directory.path() / "robot-io.sock";
  const auto generation_path = directory.path() / "robot-io.generation";
  {
    std::ofstream profile(profile_path);
    profile << R"({"sensors":[],"actuators":[]})";
  }
  const auto socket_option = "--ipc-socket=" + socket_path.string();
  const auto generation_option =
      "--generation-file=" + generation_path.string();
  const auto child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    execl(POLICY_RUNTIME_DAEMON_PATH, "robot-io-daemon",
          profile_path.c_str(), socket_option.c_str(), generation_option.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  ChildProcess process(child);
  ASSERT_TRUE(wait_for_generation(generation_path, process).has_value());

  ASSERT_EQ(kill(process.pid(), SIGTERM), 0);
  int status{};
  ASSERT_TRUE(process.exited_within(std::chrono::seconds{3}, status));
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_FALSE(std::filesystem::exists(socket_path));
  EXPECT_FALSE(std::filesystem::exists(generation_path));
}

}  // namespace
