#include <iostream>
#include <string_view>

namespace {

constexpr std::string_view kUsage =
    "Usage: robot-io-health [--help]\n"
    "\n"
    "Reports whether a daemon-owned health endpoint is available. This tool\n"
    "never opens EtherCAT hardware or starts a daemon.\n";

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string_view(argv[1]) == "--help" ||
                    std::string_view(argv[1]) == "-h")) {
    std::cout << kUsage;
    return 0;
  }
  if (argc != 1) {
    std::cerr << kUsage;
    return 2;
  }

  std::cerr << "robot-io-health: unavailable: no daemon-owned health endpoint "
               "is configured; no hardware was opened\n";
  return 1;
}
