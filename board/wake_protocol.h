#pragma once

#include <stdint.h>

// Host/firmware control requests for the offline-wake contract.
#define PANDA_REQUEST_ENABLE_WAKE_MONITOR 0xB5U
#define PANDA_REQUEST_GET_WAKE_DEBUG 0xD5U
#define PANDA_REQUEST_CLEAR_WAKE_SUCCESS 0xD7U
#define PANDA_REQUEST_GET_WAKE_SUCCESS 0xD9U
#define PANDA_REQUEST_GET_WAKE_CAN_TRACE 0xDAU

#define PANDA_WAKE_MONITOR_ARMED_STAGE 0x30U
#define WAKE_DEBUG_MAGIC 0x57414B48U
#define WAKE_SUCCESS_MAGIC 0x57535543U
#define WAKE_CAN_TRACE_MAGIC 0x57435452U

typedef struct {
  uint32_t magic;
  uint32_t boot_count;
  uint32_t reset_reason;
  uint32_t stage;
  uint32_t enter_count;
  uint32_t wfi_return_count;
  uint32_t pre_wfi_exti_pr1;
  uint32_t post_wfi_exti_pr1;
  uint32_t exti_imr1;
  uint32_t exti_rtsr1;
  uint32_t exti_ftsr1;
  uint32_t hw_type_snapshot;
  uint32_t can_exti_line;
  uint32_t exti_emr1;
  uint8_t harness_status;
  uint8_t ignition_line;
  uint8_t ignition_can_seen;
  uint8_t som_gpio;
  uint8_t bootkick_state;
  uint8_t bootkick_prev_state;
  uint8_t bootkick_waiting_countdown;
  uint8_t bootkick_reset_countdown;
} wake_debug_t;

typedef struct {
  uint32_t magic;
  uint32_t latched;
  uint32_t stage;
  uint32_t boot_count;
  uint32_t reset_reason;
  uint32_t can_exti_line;
  uint32_t harness_status;
  uint32_t ignition_line;
  uint32_t ignition_can_seen;
  uint32_t som_gpio;
} wake_success_t;

// Uses the six RTC backup registers left after wake_debug and wake_success.
// Rates are saturated at UINT16_MAX and describe the largest CAN-rate jump
// observed after the wake monitor finished learning its baseline.
typedef struct {
  uint32_t magic;
  uint32_t state;
  uint16_t peak_rx_bus0;
  uint16_t peak_rx_bus1;
  uint16_t peak_rx_bus2;
  uint16_t baseline_bus0;
  uint16_t baseline_bus1;
  uint16_t baseline_bus2;
  uint16_t peak_delta;
  uint8_t tesla_meta;
  uint8_t tesla_counters;
} wake_can_trace_t;
