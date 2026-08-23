#pragma once

#include <cstddef>
#include <span>

namespace policy_runtime::test::igh_shim {

void reset();
std::size_t last_download_size();
std::span<const std::byte> last_download_data();

}  // namespace policy_runtime::test::igh_shim
