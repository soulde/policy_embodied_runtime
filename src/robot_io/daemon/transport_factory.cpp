#include "policy_runtime/robot_io/daemon/transport_factory.hpp"

#include <exception>
#include <string>
#include <utility>

#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"

namespace policy_runtime::robot_io {

Result<void> TransportFactory::validate(const PhysicalTransportKey& key) {
  if (key.path.empty()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "physical transport path must not be empty"});
  }
  if (key.kind != PhysicalTransportKind::socketcan &&
      key.kind != PhysicalTransportKind::ethercat) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "physical transport kind is not implemented by the factory"});
  }
  return Result<void>::success();
}

Result<std::unique_ptr<TransportRuntime>> TransportFactory::create_socketcan(
    const PhysicalTransportKey& key, TransportRuntime::ReceiveCallback callback,
    std::size_t actuator_slots) {
  if (auto checked = validate(key); !checked.has_value()) {
    return Result<std::unique_ptr<TransportRuntime>>::failure(checked.error());
  }
  if (key.kind != PhysicalTransportKind::socketcan) {
    return Result<std::unique_ptr<TransportRuntime>>::failure(
        {ErrorCode::invalid_argument,
         "SocketCAN factory requires a socketcan transport key"});
  }
  auto opened = SocketCanTransport::open(key.path);
  if (!opened.has_value()) {
    return Result<std::unique_ptr<TransportRuntime>>::failure(opened.error());
  }
  auto runtime = TransportRuntime::create(std::move(opened.value()),
                                          std::move(callback), actuator_slots);
  if (!runtime.has_value()) {
    return Result<std::unique_ptr<TransportRuntime>>::failure(runtime.error());
  }
  return Result<std::unique_ptr<TransportRuntime>>::success(
      std::make_unique<TransportRuntime>(std::move(runtime.value())));
}

Result<std::unique_ptr<EthercatMaster>> TransportFactory::create_ethercat(
    std::shared_ptr<EthercatBackend> backend,
    std::vector<EthercatAxisConfiguration> configurations) {
  if (!backend) {
    return Result<std::unique_ptr<EthercatMaster>>::failure(
        {ErrorCode::invalid_argument, "EtherCAT factory requires a backend"});
  }
  try {
    return Result<std::unique_ptr<EthercatMaster>>::success(
        std::make_unique<EthercatMaster>(std::move(backend),
                                         std::move(configurations)));
  } catch (const std::exception& error) {
    return Result<std::unique_ptr<EthercatMaster>>::failure(
        {ErrorCode::internal,
         std::string("failed to create EtherCAT transport: ") + error.what()});
  }
}

}  // namespace policy_runtime::robot_io
