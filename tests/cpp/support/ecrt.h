#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ec_master ec_master_t;
typedef struct ec_domain ec_domain_t;
typedef struct ec_slave_config ec_slave_config_t;
typedef struct ec_sdo_request ec_sdo_request_t;

typedef enum {
  EC_DIR_OUTPUT = 0,
  EC_DIR_INPUT = 1,
} ec_direction_t;

typedef enum {
  EC_WD_DEFAULT = 0,
} ec_watchdog_mode_t;

typedef enum {
  EC_REQUEST_UNUSED = 0,
  EC_REQUEST_BUSY = 1,
  EC_REQUEST_SUCCESS = 2,
  EC_REQUEST_ERROR = 3,
} ec_request_state_t;

enum { EC_WC_ZERO = 0, EC_WC_INCOMPLETE = 1, EC_WC_COMPLETE = 2 };

typedef struct {
  uint16_t index;
  uint8_t subindex;
  uint8_t bit_length;
} ec_pdo_entry_info_t;

typedef struct {
  uint16_t index;
  unsigned int n_entries;
  ec_pdo_entry_info_t* entries;
} ec_pdo_info_t;

typedef struct {
  uint8_t index;
  ec_direction_t dir;
  unsigned int n_pdos;
  ec_pdo_info_t* pdos;
  ec_watchdog_mode_t watchdog_mode;
} ec_sync_info_t;

typedef struct {
  unsigned int slave_count;
  unsigned int scan_busy;
} ec_master_info_t;

typedef struct {
  uint16_t position;
  uint16_t alias;
  uint32_t vendor_id;
  uint32_t product_code;
  uint32_t revision_number;
} ec_slave_info_t;

typedef struct {
  unsigned int working_counter;
  unsigned int wc_state;
} ec_domain_state_t;

typedef struct {
  unsigned int slaves_responding;
  unsigned int al_states;
  unsigned int link_up;
} ec_master_state_t;

typedef struct {
  unsigned int online;
  unsigned int operational;
  unsigned int al_state;
} ec_slave_config_state_t;

ec_master_t* ecrt_request_master(unsigned int master_index);
void ecrt_release_master(ec_master_t* master);
ec_domain_t* ecrt_master_create_domain(ec_master_t* master);
int ecrt_master(ec_master_t* master, ec_master_info_t* info);
int ecrt_master_get_slave(ec_master_t* master, uint16_t position,
                          ec_slave_info_t* info);
ec_slave_config_t* ecrt_master_slave_config(ec_master_t* master, uint16_t alias,
                                            uint16_t position, uint32_t vendor_id,
                                            uint32_t product_code);
int ecrt_slave_config_pdos(ec_slave_config_t* config, unsigned int end,
                           const ec_sync_info_t syncs[]);
int ecrt_slave_config_sdo(ec_slave_config_t* config, uint16_t index,
                          uint8_t subindex, const uint8_t* data, size_t size);
int ecrt_slave_config_dc(ec_slave_config_t* config, uint16_t assign_activate,
                         uint32_t sync0_cycle, int32_t sync0_shift,
                         uint32_t sync1_cycle, int32_t sync1_shift);
int ecrt_master_select_reference_clock(ec_master_t* master,
                                       ec_slave_config_t* config);
ec_sdo_request_t* ecrt_slave_config_create_sdo_request(
    ec_slave_config_t* config, uint16_t index, uint8_t subindex, size_t size);
int ecrt_sdo_request_timeout(ec_sdo_request_t* request, uint32_t timeout_ms);
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t* config, uint16_t index,
                                    uint8_t subindex, ec_domain_t* domain,
                                    unsigned int* bit_position);
int ecrt_master_activate(ec_master_t* master);
void ecrt_master_deactivate(ec_master_t* master);
size_t ecrt_domain_size(const ec_domain_t* domain);
uint8_t* ecrt_domain_data(ec_domain_t* domain);
int ecrt_master_receive(ec_master_t* master);
void ecrt_domain_process(ec_domain_t* domain);
void ecrt_domain_queue(ec_domain_t* domain);
int ecrt_master_send(ec_master_t* master);
void ecrt_domain_state(const ec_domain_t* domain, ec_domain_state_t* state);
void ecrt_master_state(const ec_master_t* master, ec_master_state_t* state);
void ecrt_slave_config_state(const ec_slave_config_t* config,
                             ec_slave_config_state_t* state);
int ecrt_sdo_request_index(ec_sdo_request_t* request, uint16_t index,
                           uint8_t subindex);
uint8_t* ecrt_sdo_request_data(ec_sdo_request_t* request);
size_t ecrt_sdo_request_data_size(const ec_sdo_request_t* request);
int ecrt_sdo_request_write(ec_sdo_request_t* request);
int ecrt_sdo_request_read(ec_sdo_request_t* request);
ec_request_state_t ecrt_sdo_request_state(ec_sdo_request_t* request);

#ifdef __cplusplus
}
#endif
