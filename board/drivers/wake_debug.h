#pragma once

#define WAKE_DEBUG_MAGIC 0x57414B47U
#define WAKE_SUCCESS_MAGIC 0x57535543U

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

volatile wake_debug_t wake_debug;
volatile wake_success_t wake_success;

#define WAKE_DEBUG_WORDS (sizeof(wake_debug_t) / sizeof(uint32_t))
#define WAKE_SUCCESS_WORDS (sizeof(wake_success_t) / sizeof(uint32_t))

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
