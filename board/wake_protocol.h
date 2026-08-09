#pragma once

#include <stdint.h>

// Host/firmware control requests for the offline-wake contract.
#define PANDA_REQUEST_ENABLE_WAKE_MONITOR 0xB5U
#define PANDA_REQUEST_GET_WAKE_DEBUG 0xD5U
#define PANDA_REQUEST_CLEAR_WAKE_SUCCESS 0xD7U
#define PANDA_REQUEST_GET_WAKE_SUCCESS 0xD9U
#define PANDA_REQUEST_GET_WAKE_CAN_TRACE 0xDAU
#define PANDA_REQUEST_GET_WAKE_JOURNAL_INFO 0xE9U
#define PANDA_REQUEST_GET_WAKE_JOURNAL_RECORD 0xEAU

#define PANDA_WAKE_MONITOR_ARMED_STAGE 0x30U
#define WAKE_DEBUG_MAGIC 0x57414B48U
#define WAKE_SUCCESS_MAGIC 0x57535543U
#define WAKE_CAN_TRACE_MAGIC 0x57435452U
#define WAKE_JOURNAL_MAGIC 0x574A524EU
#define WAKE_JOURNAL_VERSION 1U
#define WAKE_JOURNAL_RECORD_SIZE 32U

#define WAKE_JOURNAL_RECORD_EVENT 1U
#define WAKE_JOURNAL_RECORD_RESULT 2U
#define WAKE_JOURNAL_SOURCE_TESLA_DOOR 1U
#define WAKE_JOURNAL_SOURCE_TESLA_POWER 2U
#define WAKE_JOURNAL_SOURCE_CAN_RATE 3U
#define WAKE_JOURNAL_SOURCE_IGNITION 4U
#define WAKE_JOURNAL_SOURCE_HARNESS 5U

#define WAKE_JOURNAL_FLAG_FULL (1U << 0U)
#define WAKE_JOURNAL_FLAG_FOREIGN_DATA (1U << 1U)

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
// This compact trace survives Panda resets and captures the first post-arm
// Tesla events without writing flash while the SoM is powered down.
typedef struct {
  uint32_t magic;
  uint32_t state;
  uint16_t peak_rx_bus0;
  uint16_t peak_rx_bus1;
  uint16_t peak_rx_bus2;
  uint16_t first_event_seconds;
  uint32_t event_sequence;
  uint8_t power_meta;
  uint8_t left_door_meta;
  uint8_t right_door_meta;
  uint8_t ui_door_meta;
} wake_can_trace_t;

typedef struct {
  uint32_t magic;
  uint32_t sequence;
  uint32_t cycle;
  uint32_t meta;
  uint32_t value0;
  uint32_t value1;
  uint32_t value2;
  uint32_t crc32;
} wake_journal_record_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t record_size;
  uint16_t capacity;
  uint16_t used_slots;
  uint16_t valid_records;
  uint16_t flags;
  uint32_t next_sequence;
  uint32_t current_cycle;
  uint32_t reserved0;
  uint32_t reserved1;
} wake_journal_info_t;
