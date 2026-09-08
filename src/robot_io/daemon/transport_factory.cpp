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

Result<std::unique_ptr<CanTransportRuntime>> TransportFactory::create_socketcan(
    const PhysicalTransportKey& key, CanTransportRuntime::ReceiveCallback callback,
    std::size_t actuator_slots) {
  if (auto checked = validate(key); !checked.has_value()) {
    return Result<std::unique_ptr<CanTransportRuntime>>::failure(checked.error());
  }
  if (key.kind != PhysicalTransportKind::socketcan) {
    return Result<std::unique_ptr<CanTransportRuntime>>::failure(
        {ErrorCode::invalid_argument,
         "SocketCAN factory requires a socketcan transport key"});
  }
  auto opened = SocketCanTransport::open(key.path);
  if (!opened.has_value()) {
    return Result<std::unique_ptr<CanTransportRuntime>>::failure(opened.error());
  }
  auto runtime = CanTransportRuntime::create(std::move(opened.value()),
                                          std::move(callback), actuator_slots);
  if (!runtime.has_value()) {
    return Result<std::unique_ptr<CanTransportRuntime>>::failure(runtime.error());
  }
  return Result<std::unique_ptr<CanTransportRuntime>>::success(
      std::make_unique<CanTransportRuntime>(std::move(runtime.value())));
}

Result<std::shared_ptr<SerialTransport>> TransportFactory::create_serial(
    SerialConfig config) {
  if (config.path.empty() || config.baud_rate == 0U ||
      config.maximum_frame_size == 0U) {
    return Result<std::shared_ptr<SerialTransport>>::failure(
        {ErrorCode::invalid_argument,
         "serial transport requires path, baud rate, and frame size"});
  }
  return Result<std::shared_ptr<SerialTransport>>::success(
      std::make_shared<SerialTransport>(std::move(config)));
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
