#include <iostream>
#include <string_view>

namespace {

constexpr std::string_view kUsage =
    "Usage: ethercat-cycle-stats [--help]\n"
    "\n"
    "Reports whether daemon-owned EtherCAT cycle telemetry is available. This\n"
    "tool never acquires an EtherCAT master or performs hardware validation.\n";

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

  std::cerr << "ethercat-cycle-stats: unavailable: daemon-owned cycle telemetry "
               "is not configured; no EtherCAT master was acquired\n";
  return 1;
}
