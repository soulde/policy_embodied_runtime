#include "igh_shim.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "ecrt.h"

struct ec_master {};
struct ec_domain {
  std::array<std::uint8_t, 256U> data{};
};
struct ec_slave_config {};
struct ec_sdo_request {
  std::uint16_t index{};
  std::uint8_t subindex{};
  std::vector<std::uint8_t> data;
  std::size_t data_size{};
  ec_request_state_t state{EC_REQUEST_UNUSED};
};

namespace {

ec_master master;
ec_domain domain;
ec_slave_config slave;
std::vector<std::unique_ptr<ec_sdo_request>> requests;
std::vector<std::byte> last_download;
std::size_t next_pdo_offset{};

}  // namespace

namespace policy_runtime::test::igh_shim {

void reset() {
  requests.clear();
  last_download.clear();
  next_pdo_offset = 0U;
  domain.data.fill(0U);
}

std::size_t last_download_size() { return last_download.size(); }

std::span<const std::byte> last_download_data() { return last_download; }

}  // namespace policy_runtime::test::igh_shim

extern "C" {

ec_master_t* ecrt_request_master(unsigned int) { return &master; }

void ecrt_release_master(ec_master_t*) { requests.clear(); }

ec_domain_t* ecrt_master_create_domain(ec_master_t*) { return &domain; }

int ecrt_master(ec_master_t*, ec_master_info_t* info) {
  *info = ec_master_info_t{1U, 0U};
  return 0;
}

int ecrt_master_get_slave(ec_master_t*, uint16_t position, ec_slave_info_t* info) {
  if (position != 0U) {
    return -1;
  }
  *info = ec_slave_info_t{0U, 0U, 0x0000009AU, 0x00030924U, 0x00010420U};
  return 0;
}

ec_slave_config_t* ecrt_master_slave_config(ec_master_t*, uint16_t alias,
                                            uint16_t position, uint32_t vendor_id,
                                            uint32_t product_code) {
  return alias == 0U && position == 0U && vendor_id == 0x0000009AU &&
                 product_code == 0x00030924U
             ? &slave
             : nullptr;
}

int ecrt_slave_config_pdos(ec_slave_config_t*, unsigned int,
                           const ec_sync_info_t[]) {
  return 0;
}

int ecrt_slave_config_sdo(ec_slave_config_t*, uint16_t, uint8_t,
                          const uint8_t*, size_t) {
  return 0;
}

int ecrt_slave_config_dc(ec_slave_config_t*, uint16_t, uint32_t, int32_t,
                         uint32_t, int32_t) {
  return 0;
}

int ecrt_master_select_reference_clock(ec_master_t*, ec_slave_config_t*) { return 0; }

ec_sdo_request_t* ecrt_slave_config_create_sdo_request(
    ec_slave_config_t*, uint16_t index, uint8_t subindex, size_t size) {
  auto request = std::make_unique<ec_sdo_request>();
  request->index = index;
  request->subindex = subindex;
  request->data.resize(size);
  request->data_size = size;
  auto* result = request.get();
  requests.push_back(std::move(request));
  return result;
}

int ecrt_sdo_request_timeout(ec_sdo_request_t*, uint32_t) { return 0; }

int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t*, uint16_t, uint8_t,
                                    ec_domain_t*, unsigned int* bit_position) {
  *bit_position = 0U;
  const auto offset = static_cast<int>(next_pdo_offset);
  next_pdo_offset += 4U;
  return offset;
}

int ecrt_master_activate(ec_master_t*) { return 0; }
void ecrt_master_deactivate(ec_master_t*) {}
size_t ecrt_domain_size(const ec_domain_t*) { return domain.data.size(); }
uint8_t* ecrt_domain_data(ec_domain_t*) { return domain.data.data(); }
int ecrt_master_receive(ec_master_t*) { return 0; }
void ecrt_domain_process(ec_domain_t*) {}
void ecrt_domain_queue(ec_domain_t*) {}
int ecrt_master_send(ec_master_t*) { return 0; }

void ecrt_domain_state(const ec_domain_t*, ec_domain_state_t* state) {
  *state = ec_domain_state_t{1U, EC_WC_COMPLETE};
}

void ecrt_master_state(const ec_master_t*, ec_master_state_t* state) {
  *state = ec_master_state_t{1U, 8U, 1U};
}

void ecrt_slave_config_state(const ec_slave_config_t*,
                             ec_slave_config_state_t* state) {
  *state = ec_slave_config_state_t{1U, 1U, 8U};
}

int ecrt_sdo_request_index(ec_sdo_request_t* request, uint16_t index,
                           uint8_t subindex) {
  request->index = index;
  request->subindex = subindex;
  return 0;
}

uint8_t* ecrt_sdo_request_data(ec_sdo_request_t* request) {
  return request->data.data();
}

size_t ecrt_sdo_request_data_size(const ec_sdo_request_t* request) {
  return request->data_size;
}

int ecrt_sdo_request_write(ec_sdo_request_t* request) {
  const auto* begin = reinterpret_cast<const std::byte*>(request->data.data());
  last_download.assign(begin, begin + request->data_size);
  request->state = EC_REQUEST_SUCCESS;
  return 0;
}

int ecrt_sdo_request_read(ec_sdo_request_t* request) {
  constexpr std::array<std::uint8_t, 7U> upload{0x10U, 0x20U, 0x30U, 0x40U,
                                               0x50U, 0x60U, 0x70U};
  std::copy(upload.begin(), upload.end(), request->data.begin());
  request->data_size = upload.size();
  request->state = EC_REQUEST_SUCCESS;
  return 0;
}

ec_request_state_t ecrt_sdo_request_state(ec_sdo_request_t* request) {
  return request->state;
}

}  // extern "C"
