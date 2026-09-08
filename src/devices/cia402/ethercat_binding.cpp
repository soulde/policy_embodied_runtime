#include "policy_runtime/devices/cia402/ethercat_binding.hpp"

#include <utility>

#include "policy_runtime/transport/ethercat/elmo_gold.hpp"

namespace policy_runtime {

Result<Cia402PdoHandles> Cia402EthercatBinding::configure_axis(
    EthercatBackend& backend, const EthercatAxisConfiguration& configuration) {
  const auto slave = EthercatSlaveAddress{configuration.axis.alias,
                                          configuration.axis.position};
  auto startup_array =
      ElmoGoldDeviceDescription::startup_sdos(configuration.axis.mode);
  std::vector<SdoDownloadRequest> startup(startup_array.begin(), startup_array.end());
  startup.insert(startup.end(), configuration.startup_parameters.begin(),
                 configuration.startup_parameters.end());
  const auto slave_configuration = EthercatSlaveConfiguration{
      slave,
      EthercatDeviceIdentity{configuration.axis.vendor_id,
                             configuration.axis.product_code,
                             configuration.axis.revision},
      ElmoGoldDeviceDescription::rx_pdo_definition(configuration.axis.mode),
      ElmoGoldDeviceDescription::feedback_pdos(),
      DistributedClockConfiguration{ElmoGoldDeviceDescription::assign_activate(),
                                    ElmoGoldDeviceDescription::sync0_cycle_ns(), 0},
      startup};
  auto configured = backend.configure_slave(slave_configuration);
  if (!configured.has_value()) {
    return Result<Cia402PdoHandles>::failure(configured.error());
  }

  Cia402PdoHandles handles;
  const auto bind_input = [&]<class T>(ObjectAddress address, std::uint8_t bits,
                                       TypedPdoField<T>& field) {
    auto location = backend.bind_pdo_entry(slave, address, bits);
    if (!location.has_value()) {
      return Result<void>::failure(location.error());
    }
    return field.bind(location.value());
  };
  auto result = bind_input(ObjectAddress{0x6041U, 0U}, 16U, handles.status_word);
  if (result.has_value()) {
    result = bind_input(ObjectAddress{0x6061U, 0U}, 8U, handles.mode_display);
  }
  if (result.has_value()) {
    result = bind_input(ObjectAddress{0x6064U, 0U}, 32U,
                        handles.actual_position);
  }
  if (result.has_value()) {
    result = bind_input(ObjectAddress{0x606CU, 0U}, 32U,
                        handles.actual_velocity);
  }
  if (result.has_value()) {
    result = bind_input(ObjectAddress{0x6077U, 0U}, 16U,
                        handles.actual_torque);
  }
  if (result.has_value()) {
    result = bind_input(ObjectAddress{0x6040U, 0U}, 16U,
                        handles.control_word);
  }
  if (!result.has_value()) {
    return Result<Cia402PdoHandles>::failure(result.error());
  }

  Result<void> target = Result<void>::failure(
      {ErrorCode::internal, "unreachable static CiA 402 mode"});
  switch (configuration.axis.mode) {
    case profiles::Cia402Mode::csp:
      target = bind_input({0x607AU, 0U}, 32U, handles.target_position);
      break;
    case profiles::Cia402Mode::csv:
      target = bind_input({0x60FFU, 0U}, 32U, handles.target_velocity);
      break;
    case profiles::Cia402Mode::cst:
      target = bind_input({0x6071U, 0U}, 16U, handles.target_torque);
      break;
  }
  if (!target.has_value()) {
    return Result<Cia402PdoHandles>::failure(target.error());
  }
  return Result<Cia402PdoHandles>::success(std::move(handles));
}

bool Cia402EthercatBinding::read_inputs(const Cia402PdoHandles& handles,
                                   std::span<const std::byte> image,
                                   Cia402PdoView& pdo) noexcept {
  const auto status_word = handles.status_word.read(image);
  const auto mode_display = handles.mode_display.read(image);
  const auto actual_position = handles.actual_position.read(image);
  const auto actual_velocity = handles.actual_velocity.read(image);
  const auto actual_torque = handles.actual_torque.read(image);
  if (!status_word || !mode_display || !actual_position || !actual_velocity ||
      !actual_torque) {
    return false;
  }
  pdo.status_word = *status_word;
  pdo.mode_display = *mode_display;
  pdo.actual_position = *actual_position;
  pdo.actual_velocity = *actual_velocity;
  pdo.actual_torque = *actual_torque;
  return true;
}

bool Cia402EthercatBinding::write_outputs(const Cia402PdoHandles& handles,
                                     std::span<std::byte> image,
                                     const Cia402PdoView& pdo,
                                     profiles::Cia402Mode mode,
                                     bool enabled) noexcept {
  bool written = handles.control_word.write(image, enabled ? pdo.control_word : 0U);
  switch (mode) {
    case profiles::Cia402Mode::csp:
      written = handles.target_position.write(
                    image, enabled ? pdo.target_position : std::int32_t{0}) &&
                written;
      break;
    case profiles::Cia402Mode::csv:
      written = handles.target_velocity.write(
                    image, enabled ? pdo.target_velocity : std::int32_t{0}) &&
                written;
      break;
    case profiles::Cia402Mode::cst:
      written = handles.target_torque.write(
                    image, enabled ? pdo.target_torque : std::int16_t{0}) &&
                written;
      break;
  }
  return written;
}

}  // namespace policy_runtime
