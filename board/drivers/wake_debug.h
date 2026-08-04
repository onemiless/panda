#pragma once

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

volatile wake_debug_t wake_debug;
volatile wake_success_t wake_success;
volatile wake_can_trace_t wake_can_trace;

#define WAKE_DEBUG_WORDS (sizeof(wake_debug_t) / sizeof(uint32_t))
#define WAKE_SUCCESS_WORDS (sizeof(wake_success_t) / sizeof(uint32_t))
#define WAKE_CAN_TRACE_WORDS (sizeof(wake_can_trace_t) / sizeof(uint32_t))

#define WAKE_CAN_TRACE_FLAG_MONITOR_ENABLED (1U << 0U)
#define WAKE_CAN_TRACE_FLAG_SOM_OFF_SEEN (1U << 1U)
#define WAKE_CAN_TRACE_FLAG_SOM_OFF_READY (1U << 2U)
#define WAKE_CAN_TRACE_FLAG_CAN_ARMED (1U << 3U)
#define WAKE_CAN_TRACE_FLAG_WAKE_REQUESTED (1U << 4U)
#define WAKE_CAN_TRACE_FLAG_RATE_CANDIDATE (1U << 5U)
#define WAKE_CAN_TRACE_FLAG_IGNITION_CAN (1U << 6U)
#define WAKE_CAN_TRACE_FLAG_IGNITION_LINE (1U << 7U)
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

static uint16_t wake_can_trace_sat_u16(uint32_t value) {
  return (uint16_t)MIN(value, (uint32_t)UINT16_MAX);
}

static void wake_can_trace_clear_peak(void) {
  wake_can_trace.peak_rx_bus0 = 0U;
  wake_can_trace.peak_rx_bus1 = 0U;
  wake_can_trace.peak_rx_bus2 = 0U;
  wake_can_trace.baseline_bus0 = 0U;
  wake_can_trace.baseline_bus1 = 0U;
  wake_can_trace.baseline_bus2 = 0U;
  wake_can_trace.peak_delta = 0U;
  wake_can_trace.tesla_meta = 0U;
  wake_can_trace.tesla_counters = 0U;
  wake_can_trace.state = (wake_can_trace.state & 0x00FFFFFFU) | 0xFF000000U;
  wake_can_trace_save();
}

static void wake_can_trace_update_state(uint16_t off_seconds, uint8_t flags) {
  const uint32_t old_state = wake_can_trace.state;
  wake_can_trace.state = (old_state & 0xFF000000U) | ((uint32_t)flags << 16U) | off_seconds;
  const bool state_changed = ((old_state ^ wake_can_trace.state) & 0x00FF0000U) != 0U;
  if (state_changed || ((off_seconds % 60U) == 0U)) {
    wake_can_trace_save();
  }
}

static void wake_can_trace_set_source(uint8_t source) {
  wake_can_trace.state = (wake_can_trace.state & 0x00FFFFFFU) | ((uint32_t)source << 24U);
  wake_can_trace_save();
}

static void wake_can_trace_capture_rates(const uint32_t *rx_per_bus, const uint32_t *baseline_per_bus) {
  uint32_t peak_delta = 0U;
  uint8_t peak_bus = 0U;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    const uint32_t delta = (rx_per_bus[i] > baseline_per_bus[i]) ? (rx_per_bus[i] - baseline_per_bus[i]) : 0U;
    if (delta > peak_delta) {
      peak_delta = delta;
      peak_bus = i;
    }
  }

  const uint8_t old_peak_bus = (uint8_t)(wake_can_trace.state >> 24U);
  if ((peak_delta > wake_can_trace.peak_delta) || (old_peak_bus == 0xFFU)) {
    wake_can_trace.peak_rx_bus0 = wake_can_trace_sat_u16(rx_per_bus[0]);
    wake_can_trace.peak_rx_bus1 = wake_can_trace_sat_u16(rx_per_bus[1]);
    wake_can_trace.peak_rx_bus2 = wake_can_trace_sat_u16(rx_per_bus[2]);
    wake_can_trace.baseline_bus0 = wake_can_trace_sat_u16(baseline_per_bus[0]);
    wake_can_trace.baseline_bus1 = wake_can_trace_sat_u16(baseline_per_bus[1]);
    wake_can_trace.baseline_bus2 = wake_can_trace_sat_u16(baseline_per_bus[2]);
    wake_can_trace.peak_delta = wake_can_trace_sat_u16(peak_delta);
    wake_can_trace.state = (wake_can_trace.state & 0x00FFFFFFU) | ((uint32_t)peak_bus << 24U);
    wake_can_trace_save();
  }
}

static void wake_can_trace_tesla(uint8_t physical_bus, uint8_t logical_bus, uint8_t power_state,
                                 int8_t previous_counter, uint8_t counter, bool valid_counter) {
  const bool old_seen = (wake_can_trace.tesla_meta & 0x80U) != 0U;
  const bool old_valid = (wake_can_trace.tesla_meta & 0x40U) != 0U;
  const bool old_nonoff = ((wake_can_trace.tesla_meta >> 4U) & 0x3U) != 0U;
  const bool new_nonoff = power_state != 0U;
  const uint8_t old_priority = ((uint8_t)old_nonoff << 1U) | (uint8_t)old_valid;
  const uint8_t new_priority = ((uint8_t)new_nonoff << 1U) | (uint8_t)valid_counter;
  if (!old_seen || (new_priority > old_priority)) {
    wake_can_trace.tesla_meta = 0x80U | ((uint8_t)valid_counter << 6U) |
                                ((power_state & 0x3U) << 4U) | ((logical_bus & 0x3U) << 2U) |
                                (physical_bus & 0x3U);
    const uint8_t previous = (previous_counter >= 0) ? (uint8_t)previous_counter : 0xFU;
    wake_can_trace.tesla_counters = ((previous & 0xFU) << 4U) | (counter & 0xFU);
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
