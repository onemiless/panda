#pragma once

#include <stddef.h>

#include "board/wake_protocol.h"
#include "board/drivers/wake_active_can_diag_policy.h"
#include "board/drivers/wake_event_trace_policy.h"

volatile wake_debug_t wake_debug;
volatile wake_success_t wake_success;
volatile wake_can_trace_t wake_can_trace;
static volatile uint16_t wake_can_trace_rx_window[PANDA_CAN_CNT] = {0U, 0U, 0U};
static volatile uint8_t wake_debug_active_can_irq_pending = 0U;

#define WAKE_DEBUG_WORDS (sizeof(wake_debug_t) / sizeof(uint32_t))
#define WAKE_SUCCESS_WORDS (sizeof(wake_success_t) / sizeof(uint32_t))
#define WAKE_CAN_TRACE_WORDS (sizeof(wake_can_trace_t) / sizeof(uint32_t))
#define WAKE_DEBUG_ACTIVE_CAN_TAG_WORD (offsetof(wake_debug_t, enter_count) / sizeof(uint32_t))
#define WAKE_DEBUG_ACTIVE_CAN_FIRST_RX_WORD (offsetof(wake_debug_t, post_wfi_exti_pr1) / sizeof(uint32_t))

#define WAKE_CAN_TRACE_FLAG_MONITOR_ENABLED (1U << 0U)
#define WAKE_CAN_TRACE_FLAG_SOM_OFF_SEEN (1U << 1U)
#define WAKE_CAN_TRACE_FLAG_SOM_OFF_READY (1U << 2U)
#define WAKE_CAN_TRACE_FLAG_CAN_ARMED (1U << 3U)
#define WAKE_CAN_TRACE_FLAG_WAKE_REQUESTED (1U << 4U)
#define WAKE_CAN_TRACE_FLAG_RATE_CANDIDATE (1U << 5U)
#define WAKE_CAN_TRACE_FLAG_IGNITION_CAN (1U << 6U)
#define WAKE_CAN_TRACE_FLAG_IGNITION_LINE (1U << 7U)
#define WAKE_CAN_TRACE_SOURCE_RAW_EDGE 0xFCU
#define WAKE_CAN_TRACE_SOURCE_TESLA_DOOR 0xFDU
#define WAKE_CAN_TRACE_SOURCE_TESLA_POWER 0xFEU

static void wake_debug_enable_backup_domain(void) {
  register_set_bits(&(RCC->APB4ENR), RCC_APB4ENR_RTCAPBEN);
  register_set_bits(&(PWR->CR1), PWR_CR1_DBP);
}

static void wake_debug_save(void) {
  wake_debug_enable_backup_domain();
  const uint32_t *src = (const uint32_t *)(&wake_debug);
  volatile uint32_t *dst = &(RTC->BKP0R);
  for (uint8_t i = 0U; i < WAKE_DEBUG_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_debug_save_word(uint8_t word) {
  if (word >= WAKE_DEBUG_WORDS) {
    return;
  }
  wake_debug_enable_backup_domain();
  const uint32_t *src = (const uint32_t *)(&wake_debug);
  volatile uint32_t *dst = &(RTC->BKP0R);
  dst[word] = src[word];
}

// The active snapshot tag is the commit marker. Invalidate it first, persist
// every payload word, then publish the tag last. An asynchronous Panda reset
// can therefore leave either the prior record or an invalid record, never a
// valid marker paired with a partially written arm snapshot.
static void wake_debug_active_can_save_arm_snapshot(void) {
  wake_debug_enable_backup_domain();
  const uint32_t *src = (const uint32_t *)(&wake_debug);
  volatile uint32_t *dst = &(RTC->BKP0R);
  dst[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD] = 0U;
  for (uint8_t i = 0U; i < WAKE_DEBUG_WORDS; i++) {
    if (i != WAKE_DEBUG_ACTIVE_CAN_TAG_WORD) {
      dst[i] = src[i];
    }
  }
  dst[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD] = src[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD];
}

static void wake_success_save(void) {
  wake_debug_enable_backup_domain();
  const uint32_t *src = (const uint32_t *)(&wake_success);
  volatile uint32_t *dst = &(RTC->BKP0R) + WAKE_DEBUG_WORDS;
  for (uint8_t i = 0U; i < WAKE_SUCCESS_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_can_trace_save(void) {
  wake_debug_enable_backup_domain();
  const uint32_t *src = (const uint32_t *)(&wake_can_trace);
  volatile uint32_t *dst = &(RTC->BKP0R) + WAKE_DEBUG_WORDS + WAKE_SUCCESS_WORDS;
  for (uint8_t i = 0U; i < WAKE_CAN_TRACE_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_debug_load(void) {
  wake_debug_enable_backup_domain();
  uint32_t *dst = (uint32_t *)(&wake_debug);
  volatile uint32_t *src = &(RTC->BKP0R);
  for (uint8_t i = 0U; i < WAKE_DEBUG_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_success_load(void) {
  wake_debug_enable_backup_domain();
  uint32_t *dst = (uint32_t *)(&wake_success);
  volatile uint32_t *src = &(RTC->BKP0R) + WAKE_DEBUG_WORDS;
  for (uint8_t i = 0U; i < WAKE_SUCCESS_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_can_trace_load(void) {
  wake_debug_enable_backup_domain();
  uint32_t *dst = (uint32_t *)(&wake_can_trace);
  volatile uint32_t *src = &(RTC->BKP0R) + WAKE_DEBUG_WORDS + WAKE_SUCCESS_WORDS;
  for (uint8_t i = 0U; i < WAKE_CAN_TRACE_WORDS; i++) {
    dst[i] = src[i];
  }
}

static void wake_can_trace_reset(void) {
  uint32_t *dst = (uint32_t *)(&wake_can_trace);
  for (uint8_t i = 0U; i < WAKE_CAN_TRACE_WORDS; i++) {
    dst[i] = 0U;
  }
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    wake_can_trace_rx_window[i] = 0U;
  }
  wake_can_trace.magic = WAKE_CAN_TRACE_MAGIC;
  wake_can_trace.state = 0xFF000000U;
  wake_can_trace_save();
}

static void wake_debug_init(void) {
  wake_debug_load();
  if (wake_debug.magic != WAKE_DEBUG_MAGIC) {
    uint32_t *dst = (uint32_t *)(&wake_debug);
    for (uint8_t i = 0U; i < WAKE_DEBUG_WORDS; i++) {
      dst[i] = 0U;
    }
    wake_debug.magic = WAKE_DEBUG_MAGIC;
  }
  wake_debug.boot_count += 1U;
  wake_debug.reset_reason = RCC->RSR;
  // Reset flags are sticky across boots. Preserve this boot's snapshot in the
  // backup record, then clear the hardware flags so the next boot reports only
  // its actual reset source instead of an accumulated history.
  RCC->RSR = RCC_RSR_RMVF;
  wake_debug_save();

  wake_success_load();
  if (wake_success.magic != WAKE_SUCCESS_MAGIC) {
    uint32_t *dst = (uint32_t *)(&wake_success);
    for (uint8_t i = 0U; i < WAKE_SUCCESS_WORDS; i++) {
      dst[i] = 0U;
    }
    wake_success.magic = WAKE_SUCCESS_MAGIC;
    wake_success_save();
  }

  wake_can_trace_load();
  if (wake_can_trace.magic != WAKE_CAN_TRACE_MAGIC) {
    wake_can_trace_reset();
  }
}

static void wake_can_trace_clear_peak(void) {
  wake_can_trace.peak_rx_bus0 = 0U;
  wake_can_trace.peak_rx_bus1 = 0U;
  wake_can_trace.peak_rx_bus2 = 0U;
  wake_can_trace.first_event_seconds = 0U;
  wake_can_trace.event_sequence = 0U;
  wake_can_trace.power_meta &= 0xE0U;
  wake_can_trace.left_door_meta = wake_event_trace_prepare_binary_postarm(wake_can_trace.left_door_meta);
  wake_can_trace.right_door_meta = wake_event_trace_prepare_binary_postarm(wake_can_trace.right_door_meta);
  wake_can_trace.ui_door_meta = wake_event_trace_prepare_binary_postarm(wake_can_trace.ui_door_meta);
  wake_can_trace.state = (wake_can_trace.state & 0x00FFFFFFU) | 0xFF000000U;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    wake_can_trace_rx_window[i] = 0U;
  }
  wake_can_trace_save();
}

static uint16_t wake_can_trace_peak_rx(uint8_t physical_bus) {
  uint16_t peak = 0U;
  switch (physical_bus) {
    case 0U: peak = wake_can_trace.peak_rx_bus0; break;
    case 1U: peak = wake_can_trace.peak_rx_bus1; break;
    case 2U: peak = wake_can_trace.peak_rx_bus2; break;
    default: break;
  }
  return peak;
}

static void wake_can_trace_set_peak_rx(uint8_t physical_bus, uint16_t peak) {
  switch (physical_bus) {
    case 0U: wake_can_trace.peak_rx_bus0 = peak; break;
    case 1U: wake_can_trace.peak_rx_bus1 = peak; break;
    case 2U: wake_can_trace.peak_rx_bus2 = peak; break;
    default: break;
  }
}

// RTC backup registers have no flash wear. Persist only the first validated
// RX per physical controller immediately, then save again only when a new
// one-second peak is observed.
static void wake_can_trace_record_rx(uint8_t physical_bus) {
  if (physical_bus >= PANDA_CAN_CNT) {
    return;
  }
  if (wake_can_trace_rx_window[physical_bus] < UINT16_MAX) {
    wake_can_trace_rx_window[physical_bus] += 1U;
  }
  if (wake_can_trace_peak_rx(physical_bus) == 0U) {
    wake_can_trace_set_peak_rx(physical_bus, 1U);
    wake_can_trace_save();
  }
}

static void wake_can_trace_flush_rx_window(void) {
  bool changed = false;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    const uint16_t count = wake_can_trace_rx_window[i];
    const uint16_t peak = wake_active_can_diag_peak(wake_can_trace_peak_rx(i), count);
    if (peak != wake_can_trace_peak_rx(i)) {
      wake_can_trace_set_peak_rx(i, peak);
      changed = true;
    }
    wake_can_trace_rx_window[i] = 0U;
  }
  if (changed) {
    wake_can_trace_save();
  }
}

static void wake_debug_active_can_reset(void) {
  if (wake_active_can_diag_valid(wake_debug.enter_count)) {
    // enter_count is the tagged active-FDCAN snapshot only on Tres. Once the
    // snapshot is invalidated it must not be exposed as a legacy STOP count.
    wake_debug.enter_count = 0U;
    wake_debug.pre_wfi_exti_pr1 = 0U;
    wake_debug.post_wfi_exti_pr1 = 0U;
    wake_debug.exti_imr1 = 0U;
    wake_debug.exti_rtsr1 = 0U;
    wake_debug.exti_ftsr1 = 0U;
    wake_debug.exti_emr1 = 0U;
  }
  wake_debug_active_can_irq_pending = 0U;
}

static void wake_debug_active_can_irq_entry(uint8_t physical_bus) {
  const uint32_t old_snapshot = wake_debug.enter_count;
  const uint32_t new_snapshot = wake_active_can_diag_latch_irq(old_snapshot, physical_bus);
  if (new_snapshot != old_snapshot) {
    wake_debug.enter_count = new_snapshot;
    wake_debug_active_can_irq_pending |= (uint8_t)(1U << physical_bus);
  }
}

static void wake_debug_active_can_irq_flush(uint8_t physical_bus) {
  if (physical_bus >= PANDA_CAN_CNT) {
    return;
  }
  const uint8_t pending = (uint8_t)(1U << physical_bus);
  if ((wake_debug_active_can_irq_pending & pending) != 0U) {
    wake_debug_active_can_irq_pending &= (uint8_t)(~pending);
    // Only the tagged flag word changed. A single RTC write keeps this
    // one-shot diagnostic out of the CAN receive critical path as much as
    // possible and does not touch Flash.
    wake_debug_save_word(WAKE_DEBUG_ACTIVE_CAN_TAG_WORD);
  }
}

static void wake_debug_active_can_first_rx(uint8_t physical_bus, uint32_t address) {
  if (wake_active_can_diag_valid(wake_debug.enter_count) &&
      !wake_active_can_diag_first_rx_valid(wake_debug.post_wfi_exti_pr1)) {
    wake_debug.post_wfi_exti_pr1 = wake_active_can_diag_pack_first_rx(physical_bus, address);
    // The ISR entry bit was latched before can_rx(). Persist it together with
    // the first validated frame, then suppress the handler's fallback save.
    wake_debug_active_can_irq_pending = 0U;
    wake_debug_save_word(WAKE_DEBUG_ACTIVE_CAN_FIRST_RX_WORD);
    wake_debug_save_word(WAKE_DEBUG_ACTIVE_CAN_TAG_WORD);
  }
}

static void wake_can_trace_update_state(uint16_t off_seconds, uint8_t flags) {
  const uint32_t old_state = wake_can_trace.state;
  wake_can_trace.state = (old_state & 0xFF000000U) | ((uint32_t)flags << 16U) | off_seconds;
  const bool state_changed = ((old_state ^ wake_can_trace.state) & 0x00FF0000U) != 0U;
  if (state_changed) {
    wake_can_trace_save();
  }
}

static void wake_can_trace_set_source(uint8_t source) {
  wake_can_trace.state = (wake_can_trace.state & 0x00FFFFFFU) | ((uint32_t)source << 24U);
  wake_can_trace_save();
}

static void wake_can_trace_append_event(uint8_t event) {
  const uint32_t sequence = wake_event_trace_append(wake_can_trace.event_sequence, event);
  if (sequence != wake_can_trace.event_sequence) {
    if (wake_can_trace.first_event_seconds == 0U) {
      wake_can_trace.first_event_seconds = (uint16_t)(wake_can_trace.state & 0xFFFFU);
    }
    wake_can_trace.event_sequence = sequence;
    wake_can_trace_save();
  }
}

static void wake_can_trace_prearm_power(uint8_t power_state) {
  const uint8_t meta = wake_event_trace_power_prearm(power_state);
  if ((wake_can_trace.power_meta & 0xE0U) != meta) {
    wake_can_trace.power_meta = meta;
    wake_can_trace_save();
  }
}

static void wake_can_trace_prearm_binary(uint16_t address, bool state) {
  volatile uint8_t *meta = (address == 0x102U) ? &wake_can_trace.left_door_meta :
                           (address == 0x103U) ? &wake_can_trace.right_door_meta :
                                                &wake_can_trace.ui_door_meta;
  const uint8_t new_meta = wake_event_trace_door_prearm(state);
  if ((*meta & 0xC0U) != new_meta) {
    *meta = new_meta;
    wake_can_trace_save();
  }
}

static void wake_can_trace_postarm_power(uint8_t physical_bus, uint8_t power_state) {
  const uint8_t count = wake_event_trace_count(wake_can_trace.power_meta);
  const uint8_t event = wake_event_trace_power_code(physical_bus, power_state);
  wake_can_trace.power_meta = wake_event_trace_increment_count(wake_can_trace.power_meta);
  if ((count == 0U) || (wake_event_trace_last_power(wake_can_trace.event_sequence) != event)) {
    wake_can_trace_append_event(event);
  } else {
    wake_can_trace_save();
  }
}

static void wake_can_trace_postarm_binary(uint16_t address, bool state, bool force_event) {
  volatile uint8_t *meta = (address == 0x102U) ? &wake_can_trace.left_door_meta :
                           (address == 0x103U) ? &wake_can_trace.right_door_meta :
                                                &wake_can_trace.ui_door_meta;
  const uint8_t event = (address == 0x102U) ? WAKE_EVENT_TRACE_LEFT_DOOR :
                        (address == 0x103U) ? WAKE_EVENT_TRACE_RIGHT_DOOR :
                                             WAKE_EVENT_TRACE_UI_DOOR;
  const uint8_t count = wake_event_trace_count(*meta);
  const bool changed = wake_event_trace_postarm_binary_state(*meta) != state;
  *meta = wake_event_trace_set_postarm_binary_state(wake_event_trace_increment_count(*meta), state);
  if ((count == 0U) || changed || force_event) {
    wake_can_trace_append_event(event);
  } else {
    wake_can_trace_save();
  }
}

static void wake_debug_stage(uint32_t stage) {
  wake_debug.magic = WAKE_DEBUG_MAGIC;
  wake_debug.stage = stage;
  wake_debug.harness_status = harness.status;
  wake_debug.ignition_line = (uint8_t)harness_check_ignition();
  wake_debug.ignition_can_seen = (uint8_t)ignition_can;
  wake_debug.som_gpio = (uint8_t)current_board->read_som_gpio();
  wake_debug_save();
}

static void wake_debug_bootkick_pins(BootState state, bool bootkick_level, bool dc_in_level) {
  const uint8_t shift = ((uint8_t)state) * 2U;
  const uint8_t mask = (uint8_t)(0x3U << shift);
  const uint8_t levels = (uint8_t)(((uint8_t)bootkick_level | ((uint8_t)dc_in_level << 1U)) << shift);
  const uint8_t phase_mask = (uint8_t)((wake_debug.hw_type_snapshot >> 8U) | (1U << (uint8_t)state));
  const uint8_t old_levels = (uint8_t)(wake_debug.hw_type_snapshot >> 16U);
  const uint8_t pin_levels = (old_levels & (uint8_t)(~mask)) | levels;
  wake_debug.hw_type_snapshot = (wake_debug.hw_type_snapshot & 0xFF0000FFU) |
                                ((uint32_t)phase_mask << 8U) |
                                ((uint32_t)pin_levels << 16U);
  wake_debug_save();
}

static void wake_debug_bootkick_wake_state(uint8_t attempts, uint8_t retry_countdown, bool uart_seen, bool reset_attempted) {
  const uint8_t state = (attempts & 0x3U) |
                        ((retry_countdown & 0xFU) << 2U) |
                        ((uint8_t)uart_seen << 6U) |
                        ((uint8_t)reset_attempted << 7U);
  wake_debug.hw_type_snapshot = (wake_debug.hw_type_snapshot & 0x00FFFFFFU) | ((uint32_t)state << 24U);
  wake_debug_save();
}

static void wake_debug_latch_success(uint32_t stage) {
  if (wake_success.latched != 0U) {
    return;
  }

  wake_success.magic = WAKE_SUCCESS_MAGIC;
  wake_success.latched = 1U;
  wake_success.stage = stage;
  wake_success.boot_count = wake_debug.boot_count;
  wake_success.reset_reason = wake_debug.reset_reason;
  wake_success.can_exti_line = wake_debug.can_exti_line;
  wake_success.harness_status = harness.status;
  wake_success.ignition_line = (uint32_t)harness_check_ignition();
  wake_success.ignition_can_seen = (uint32_t)ignition_can;
  wake_success.som_gpio = (uint32_t)current_board->read_som_gpio();
  wake_success_save();
}

static void wake_debug_clear_success(void) {
  wake_success.magic = WAKE_SUCCESS_MAGIC;
  wake_success.latched = 0U;
  wake_success.stage = 0U;
  wake_success.boot_count = 0U;
  wake_success.reset_reason = 0U;
  wake_success.can_exti_line = 0U;
  wake_success.harness_status = 0U;
  wake_success.ignition_line = 0U;
  wake_success.ignition_can_seen = 0U;
  wake_success.som_gpio = 0U;
  wake_success_save();
}

static void wake_debug_can_exti(uint32_t can_exti_line) {
  // A new STOP cycle starts with no active bootkick attempt. Reset the packed
  // phase, pin, and retry snapshots so they cannot be mistaken for this cycle.
  wake_debug.hw_type_snapshot = hw_type;
  wake_debug.can_exti_line = can_exti_line;
  wake_debug_save();
}

static void wake_debug_exti_snapshot(bool post_wfi) {
  if (post_wfi) {
    wake_debug.post_wfi_exti_pr1 = EXTI->PR1;
    wake_debug.wfi_return_count += 1U;
  } else {
    wake_debug.pre_wfi_exti_pr1 = EXTI->PR1;
  }
  wake_debug.exti_imr1 = EXTI->IMR1;
  wake_debug.exti_rtsr1 = EXTI->RTSR1;
  wake_debug.exti_ftsr1 = EXTI->FTSR1;
  wake_debug.exti_emr1 = EXTI->EMR1;
  wake_debug_save();
}

static void wake_debug_bootkick(BootState state, BootState prev_state, uint8_t waiting_countdown, uint8_t reset_countdown) {
  wake_debug.magic = WAKE_DEBUG_MAGIC;
  wake_debug.bootkick_state = (uint8_t)state;
  wake_debug.bootkick_prev_state = (uint8_t)prev_state;
  wake_debug.bootkick_waiting_countdown = waiting_countdown;
  wake_debug.bootkick_reset_countdown = reset_countdown;
  wake_debug.som_gpio = (uint8_t)current_board->read_som_gpio();
  wake_debug_save();
}

static void wake_debug_bootkick_schedule(uint8_t waiting_countdown, uint8_t hold_countdown) {
  const uint32_t schedule = (wake_debug.can_exti_line & 0xFFFFU) |
                            ((uint32_t)waiting_countdown << 16U) |
                            ((uint32_t)hold_countdown << 24U);
  if (wake_debug.can_exti_line != schedule) {
    wake_debug.can_exti_line = schedule;
    wake_debug_save();
  }
}
