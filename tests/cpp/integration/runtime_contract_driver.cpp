#include <iostream>
#include <string>

#include "policy_runtime/runtime/runtime_host.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: runtime_contract_driver POLICY_PROFILE\n";
    return 2;
  }
  auto host = policy_runtime::RuntimeHost::from_profiles(argv[1]);
  if (!host.has_value()) {
    std::cerr << host.error().message << '\n';
    return 1;
  }
  auto opened = host.value().open();
  if (!opened.has_value()) {
    std::cerr << opened.error().message << '\n';
    return 1;
  }
  std::string request;
  while (std::getline(std::cin, request)) {
    std::cout << host.value().handle_text(request) << '\n';
    std::cout.flush();
  }
  host.value().close();
  return 0;
}
