#include "board/sys/sys.h"
#include "board/drivers/offline_wake_source_policy.h"
#include "board/drivers/wake_monitor_policy.h"

// WARNING: To stay in compliance with the SIL2 rules laid out in STM UM2331, we should never use any of the available hardware low power modes during safety function execution.
// See rule: CoU_3

// Low power state "stop mode" is only entered from SAFETY_SILENT when no safety function is active and exited via reset which is a safe state.

bool power_save_enabled = false;
volatile bool wake_monitor_enabled = false;
volatile bool wake_monitor_tesla_event_pending = false;
volatile uint8_t wake_monitor_tesla_event_source = TESLA_WAKE_SOURCE_NONE;
volatile bool wake_monitor_can_activity_pending = false;
volatile bool wake_monitor_raw_can_edge_pending = false;
volatile bool wake_monitor_can_wake_requested = false;
volatile bool wake_monitor_can_dispatch_pending = false;
volatile uint32_t wake_monitor_can_dispatch_stage = 0U;
volatile bool wake_monitor_som_off_seen = false;
volatile bool wake_monitor_som_off_ready = false;
volatile uint8_t wake_monitor_som_off_countdown = 0U;
volatile bool wake_monitor_can_armed = false;
volatile bool wake_monitor_strict_stop_pending = false;
volatile bool wake_monitor_reset_requested = false;
volatile bool wake_monitor_harness_requested = false;
volatile bool wake_monitor_committed = false;
volatile uint8_t wake_monitor_failure_cooldown = 0U;
volatile wake_monitor_status_t wake_monitor_status = {
  .magic = WAKE_MONITOR_STATUS_MAGIC,
  .transaction = 0U,
  .host_session = 0U,
  .committed_host_session = 0U,
  .state = WAKE_MONITOR_STATE_IDLE,
  .result = WAKE_MONITOR_RESULT_NONE,
  .trigger_stage = 0U,
  .reserved = 0U,
};
#ifdef ALLOW_DEBUG
volatile bool stop_mode_requested = false;
#endif
static volatile uint32_t wake_monitor_raw_can_exti_lines = 0U;
static volatile uint32_t wake_monitor_primary_can_exti_line = 0U;

static void offline_wake_raw_can_exti_disarm(void) {
  const uint32_t lines = wake_monitor_raw_can_exti_lines;
  if (lines != 0U) {
    register_clear_bits(&(EXTI->IMR1), lines);
    EXTI->PR1 = lines;
    wake_monitor_raw_can_exti_lines = 0U;
  }
  wake_monitor_primary_can_exti_line = 0U;
}

static void offline_wake_raw_can_exti_irq_handler(void) {
  const uint32_t armed_lines = wake_monitor_raw_can_exti_lines;
  const uint32_t primary_line = wake_monitor_primary_can_exti_line;
  const uint32_t pending = EXTI->PR1 & armed_lines;
  if (pending != 0U) {
    EXTI->PR1 = pending;
    if (offline_wake_primary_raw_can_edge_ready(
          wake_monitor_enabled, wake_monitor_som_off_ready, wake_monitor_can_armed,
          wake_monitor_can_wake_requested, pending, primary_line)) {
      // Vehicle captures show physical bus 1 is the first bus to resume. Once
      // the shutdown guard has observed ten quiet seconds, its first RX
      // electrical edge is sufficient: do not depend on FDCAN decoding the
      // first frame while the SoM is off.
      register_clear_bits(&(EXTI->IMR1), armed_lines);
      wake_monitor_can_activity_pending = true;
      wake_can_trace_set_source(WAKE_CAN_TRACE_SOURCE_RAW_EDGE);
      wake_debug_can_exti(pending);
      const uint8_t exti_data[8] = {
        (uint8_t)(primary_line & 0xFFU),
        (uint8_t)((primary_line >> 8U) & 0xFFU),
        (uint8_t)((primary_line >> 16U) & 0xFFU),
        (uint8_t)((primary_line >> 24U) & 0xFFU),
        0U, 0U, 0U, 0U,
      };
      wake_journal_queue_event(WAKE_JOURNAL_SOURCE_CAN_PRIMARY, 0x35U, 1U,
                               CAN_NUM_FROM_BUS_NUM(1U), 4U, 0U, exti_data);
    } else if (offline_wake_raw_can_edge_hint_ready(
          wake_monitor_enabled, wake_monitor_som_off_ready, wake_monitor_can_armed,
          wake_monitor_can_wake_requested, pending, armed_lines)) {
      // Other physical buses remain sampling hints and never wake the SoM by
      // themselves, because a sleeping Tesla can keep background traffic on
      // those buses.
      register_clear_bits(&(EXTI->IMR1), armed_lines);
      wake_monitor_raw_can_edge_pending = true;
      wake_debug_can_exti(pending);
    }
  }
}

static void offline_wake_raw_can_exti_init(void) {
  REGISTER_INTERRUPT(EXTI9_5_IRQn, offline_wake_raw_can_exti_irq_handler, 100U, FAULT_INTERRUPT_RATE_EXTI)
  REGISTER_INTERRUPT(EXTI15_10_IRQn, offline_wake_raw_can_exti_irq_handler, 100U, FAULT_INTERRUPT_RATE_EXTI)
}

static void offline_wake_raw_can_exti_arm(void) {
  offline_wake_raw_can_exti_disarm();
  wake_monitor_raw_can_edge_pending = false;
  if (hw_type != HW_TYPE_TRES) {
    return;
  }

  const bool flipped_harness = harness.status == HARNESS_STATUS_FLIPPED;
  const uint32_t lines = offline_wake_tres_can_exti_lines(flipped_harness);
  const uint32_t primary_line = offline_wake_oriented_fdcan2_exti_line(flipped_harness);

  // Keep bus 0/2 on FDCAN for decoded-rate fallback. Switch only Tesla's
  // physical bus 1 (oriented FDCAN2 RX) to GPIO input after the quiet guard so
  // EXTI can observe the first electrical edge independently of the decoder.
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI8_PB, 0xFU);
  if (flipped_harness) {
    set_gpio_pullup(GPIOB, 12, PULL_NONE);
    set_gpio_mode(GPIOB, 12, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR4_EXTI12_PB, 0xFU);
  } else {
    set_gpio_pullup(GPIOB, 5, PULL_NONE);
    set_gpio_mode(GPIOB, 5, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[1]), SYSCFG_EXTICR2_EXTI5_PB, 0xF0U);
  }
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI9_PG, 0xF0U);

  wake_monitor_primary_can_exti_line = primary_line;
  wake_monitor_raw_can_exti_lines = lines;
  EXTI->PR1 = lines;
  register_set_bits(&(EXTI->RTSR1), lines);
  register_set_bits(&(EXTI->FTSR1), lines);
  register_set_bits(&(EXTI->IMR1), lines);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void enable_can_transceivers(bool enabled) {
  // Leave main CAN always on for CAN-based ignition detection
  uint8_t main_bus = (harness.status == HARNESS_STATUS_FLIPPED) ? 3U : 1U;
  for(uint8_t i=1U; i<=4U; i++){
    current_board->enable_can_transceiver(i, (i == main_bus) || enabled);
  }
}

void set_power_save_state(bool enable) {
  if (enable != power_save_enabled) {
    if (enable) {
      print("enable power savings\n");

      // Disable CAN interrupts
      if (harness.status == HARNESS_STATUS_FLIPPED) {
        llcan_irq_disable(cans[0]);
      } else {
        llcan_irq_disable(cans[2]);
      }
      llcan_irq_disable(cans[1]);
    } else {
      print("disable power savings\n");

      if (harness.status == HARNESS_STATUS_FLIPPED) {
        llcan_irq_enable(cans[0]);
      } else {
        llcan_irq_enable(cans[2]);
      }
      llcan_irq_enable(cans[1]);
    }

    enable_can_transceivers(!enable);

    // Switch off IR when in power saving
    if(enable){
      current_board->set_ir_power(0U);
    }

    power_save_enabled = enable;
  }
}

static void enter_stop_mode(void) {
  wake_debug.stage = 0x10U;
  wake_debug.enter_count += 1U;
  wake_debug_stage(0x11U);

  // set all GPIO to analog mode to reduce power, analog mode also disables pull resistors
  register_set(&(GPIOA->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOB->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOC->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOD->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOE->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOF->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  register_set(&(GPIOG->MODER), 0xFFFFFFFFU, 0xFFFFFFFFU);
  wake_debug_stage(0x12U);

  // init GPIO to lowest power state
  current_board->set_bootkick(BOOT_STANDBY);
  current_board->set_amp_enabled(false);
  // STOP mode cannot decode a CAN identifier before the first RX edge. Keep
  // every connected transceiver awake so activity on any physical vehicle bus
  // can reach an armed RX EXTI input.
  const bool normal_harness = harness.status != HARNESS_STATUS_FLIPPED;
  for (uint8_t i = 1U; i <= 4U; i++) {
    current_board->enable_can_transceiver(i, true);
  }
  wake_debug_stage(0x13U);

  // disable ADCs
  ADC1->CR &= ~(ADC_CR_ADEN);
  ADC1->CR |= ADC_CR_DEEPPWD;
  ADC2->CR &= ~(ADC_CR_ADEN);
  ADC2->CR |= ADC_CR_DEEPPWD;

  // disable HSI48: 48 MHz USB clock
  register_clear_bits(&(RCC->CR), RCC_CR_HSI48ON);
  // disable SRAM retention in stop mode
  register_clear_bits(&(RCC->AHB2LPENR), RCC_AHB2LPENR_SRAM1LPEN | RCC_AHB2LPENR_SRAM2LPEN);
  register_clear_bits(&(RCC->AHB4LPENR), RCC_AHB4LPENR_SRAM4LPEN);
  register_clear_bits(&(RCC->AHB3LPENR), RCC_AHB3LPENR_AXISRAMLPEN);

  // Keep comma's two physical SBU wake inputs as a fallback for vehicle wake
  // events that do not create a usable edge on the selected CAN RX pin.
  set_gpio_mode(current_board->harness_config->GPIO_SBU1,
                current_board->harness_config->pin_SBU1, MODE_INPUT);
  set_gpio_mode(current_board->harness_config->GPIO_SBU2,
                current_board->harness_config->pin_SBU2, MODE_INPUT);
  register_set(&(SYSCFG->EXTICR[0]), SYSCFG_EXTICR1_EXTI1_PA, 0xF0U);
  register_set(&(SYSCFG->EXTICR[1]), SYSCFG_EXTICR2_EXTI4_PC, 0xFU);

  // Arm every physical CAN RX available on Tres: FDCAN1 PB8, oriented FDCAN2
  // PB5/PB12, and FDCAN3 PG9. Cuatro uses PD12 for FDCAN3; in flipped mode it
  // conflicts with FDCAN2 PB12, so the proven FDCAN2 wake source takes priority.
  const bool tres = hw_type == HW_TYPE_TRES;
  const uint32_t can_exti_lines = tres ? offline_wake_tres_can_exti_lines(!normal_harness) :
                                         offline_wake_cuatro_can_exti_lines(!normal_harness);
  const uint32_t wake_exti_lines = OFFLINE_WAKE_SBU_EXTI_LINES | can_exti_lines;
  set_gpio_mode(GPIOB, 8, MODE_INPUT);
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI8_PB, 0xFU);
  if (normal_harness) {
    set_gpio_mode(GPIOB, 5, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[1]), SYSCFG_EXTICR2_EXTI5_PB, 0xF0U);
  } else {
    set_gpio_mode(GPIOB, 12, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR4_EXTI12_PB, 0xFU);
  }
  if (tres) {
    set_gpio_mode(GPIOG, 9, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI9_PG, 0xF0U);
  } else if (normal_harness) {
    set_gpio_mode(GPIOD, 12, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR4_EXTI12_PD, 0xFU);
  }
  wake_debug_can_exti(can_exti_lines);
  register_set_bits(&(EXTI->IMR1), wake_exti_lines);
  register_set_bits(&(EXTI->EMR1), wake_exti_lines);
  register_set_bits(&(EXTI->RTSR1), wake_exti_lines);
  register_set_bits(&(EXTI->FTSR1), wake_exti_lines);
  wake_debug_exti_snapshot(false);
  wake_debug_stage(0x14U);

  // clear pending EXTI
  EXTI->PR1 = wake_exti_lines;

  // reset if ignition just came on before going to sleep
  if (harness_check_ignition()) {
    // The SoM may still be completing shutdown, so an immediate BOOTKICK
    // assertion can be lost. Persist a deferred wake request across the Panda
    // reset; bootkick will wait for SoM power-off before creating a fresh edge.
    wake_debug_stage(BOOTKICK_WAKE_PRESTOP_IGNITION_PENDING_STAGE);
    NVIC_SystemReset();
  }

  // stop mode
  register_clear_bits(&(PWR->CPUCR), PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3);

  // set SVOS5 voltage scaling, flash low-power
  register_set(&(PWR->CR1), PWR_CR1_SVOS_0 | PWR_CR1_FLPS, PWR_CR1_SVOS | PWR_CR1_FLPS);

  // enter stop mode on WFI
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  __disable_irq();

  // disable all NVIC interrupts and clear pending
  for (uint32_t i = 0U; i < 8U; i++) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }
  // enable only wakeup EXTI interrupts
  NVIC_EnableIRQ(EXTI1_IRQn);  // SBU2 (PA1)
  NVIC_EnableIRQ(EXTI4_IRQn);  // SBU1 (PC4)
  NVIC_EnableIRQ(EXTI9_5_IRQn);    // FDCAN1 PB8, FDCAN2 PB5, Tres FDCAN3 PG9
  NVIC_EnableIRQ(EXTI15_10_IRQn);  // FDCAN2 PB12 or Cuatro FDCAN3 PD12

  wake_debug_exti_snapshot(false);
  wake_debug_stage(0x16U);
  __DSB();
  __ISB();
  // cppcheck-suppress misra-c2012-17.3 ; CMSIS __WFI macro expands to inline asm
  __WFI();

  wake_debug_exti_snapshot(true);
  if ((EXTI->PR1 & wake_exti_lines) != 0U) {
    // Persist a bootkick-recognised cause before the reset. The restored
    // state starts exactly one SoM wake pulse after Panda reinitializes.
    wake_debug_stage(0x34U);
  } else {
    wake_debug_stage(0x17U);
  }
  NVIC_SystemReset();
}
