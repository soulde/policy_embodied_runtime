#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

namespace policy_runtime {
namespace detail {

template <typename Integer>
std::optional<Integer> engineering_to_device_units(double value, double scale) noexcept {
  static_assert(std::is_integral_v<Integer> && std::is_signed_v<Integer>);
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0) {
    return std::nullopt;
  }
  const long double scaled = static_cast<long double>(value) * scale;
  if (!std::isfinite(scaled)) {
    return std::nullopt;
  }
  const long double rounded = std::round(scaled);
  if (rounded < static_cast<long double>(std::numeric_limits<Integer>::min()) ||
      rounded > static_cast<long double>(std::numeric_limits<Integer>::max())) {
    return std::nullopt;
  }
  return static_cast<Integer>(rounded);
}

}  // namespace detail

inline std::optional<std::int32_t> position_to_device_units(double value,
                                                            double scale) noexcept {
  return detail::engineering_to_device_units<std::int32_t>(value, scale);
}

inline std::optional<std::int32_t> velocity_to_device_units(double value,
                                                            double scale) noexcept {
  return detail::engineering_to_device_units<std::int32_t>(value, scale);
}

inline std::optional<std::int16_t> torque_to_device_units(double value,
                                                          double scale) noexcept {
  return detail::engineering_to_device_units<std::int16_t>(value, scale);
}

template <typename Integer>
std::optional<double> device_units_to_engineering(Integer value, double scale) noexcept {
  static_assert(std::is_integral_v<Integer>);
  if (!std::isfinite(scale) || scale <= 0.0) {
    return std::nullopt;
  }
  const long double converted = static_cast<long double>(value) / scale;
  if (!std::isfinite(converted) ||
      converted < -static_cast<long double>(std::numeric_limits<double>::max()) ||
      converted > static_cast<long double>(std::numeric_limits<double>::max())) {
    return std::nullopt;
  }
  const double result = static_cast<double>(converted);
  return std::isfinite(result) ? std::optional<double>{result} : std::nullopt;
}

}  // namespace policy_runtime
