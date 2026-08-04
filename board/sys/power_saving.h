#include "board/sys/sys.h"

// WARNING: To stay in compliance with the SIL2 rules laid out in STM UM2331, we should never use any of the available hardware low power modes during safety function execution.
// See rule: CoU_3

// Low power state "stop mode" is only entered from SAFETY_SILENT when no safety function is active and exited via reset which is a safe state.

bool power_save_enabled = false;
volatile bool wake_monitor_enabled = false;
volatile bool wake_monitor_tesla_event_pending = false;
volatile uint8_t wake_monitor_tesla_event_source = TESLA_WAKE_SOURCE_NONE;
volatile bool wake_monitor_can_wake_requested = false;
volatile bool wake_monitor_can_dispatch_pending = false;
volatile uint32_t wake_monitor_can_dispatch_stage = 0U;
volatile bool wake_monitor_som_off_seen = false;
volatile bool wake_monitor_som_off_ready = false;
volatile uint8_t wake_monitor_som_off_countdown = 0U;
volatile bool wake_monitor_can_armed = false;
// The strict Tesla path has completed its quiet interval and must enter STOP.
// This is deliberately independent of the SoM GPIO: on Tres that GPIO can
// remain high after Linux has powered down.
volatile bool wake_monitor_strict_stop_pending = false;
volatile int8_t wake_monitor_tesla_counter = -1;
volatile int8_t wake_monitor_tesla_door_counter = -1;
volatile uint8_t wake_monitor_tesla_front_door_known_mask = 0U;
volatile uint8_t wake_monitor_tesla_front_door_closed_mask = 0U;
#ifdef ALLOW_DEBUG
volatile bool stop_mode_requested = false;
#endif

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
  // Strict offline wake: the capture evidence identifies logical CAN bus 1
  // (FDCAN2) as the first physical wake source. Do not leave the other buses
  // powered or armed, since STOP mode cannot inspect a CAN identifier before
  // an RX edge wakes the MCU.
  const bool normal_harness = harness.status != HARNESS_STATUS_FLIPPED;
  for (uint8_t i = 1U; i <= 4U; i++) {
    current_board->enable_can_transceiver(i, false);
  }
  current_board->enable_can_transceiver(normal_harness ? 2U : 4U, true);
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

  // Logical bus 1 is FDCAN2. Its active RX is PB5 with a normal harness and
  // PB12 with a flipped harness. Arm exactly that EXTI line.
  const uint32_t can_exti_line = normal_harness ? (1UL << 5) : (1UL << 12);
  if (normal_harness) {
    set_gpio_mode(GPIOB, 5, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[1]), SYSCFG_EXTICR2_EXTI5_PB, 0xF0U);
  } else {
    set_gpio_mode(GPIOB, 12, MODE_INPUT);
    register_set(&(SYSCFG->EXTICR[3]), SYSCFG_EXTICR4_EXTI12_PB, 0xFU);
  }
  wake_debug_can_exti(can_exti_line);
  register_set_bits(&(EXTI->IMR1), can_exti_line);
  register_set_bits(&(EXTI->EMR1), can_exti_line);
  register_set_bits(&(EXTI->RTSR1), can_exti_line);
  register_set_bits(&(EXTI->FTSR1), can_exti_line);
  wake_debug_exti_snapshot(false);
  wake_debug_stage(0x14U);

  // clear pending EXTI
  EXTI->PR1 = can_exti_line;

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
  if (normal_harness) {
    NVIC_EnableIRQ(EXTI9_5_IRQn);   // FDCAN2 RX PB5
  } else {
    NVIC_EnableIRQ(EXTI15_10_IRQn); // FDCAN2 RX PB12
  }

  wake_debug_exti_snapshot(false);
  wake_debug_stage(0x16U);
  __DSB();
  __ISB();
  // cppcheck-suppress misra-c2012-17.3 ; CMSIS __WFI macro expands to inline asm
  __WFI();

  wake_debug_exti_snapshot(true);
  if ((EXTI->PR1 & can_exti_line) != 0U) {
    // Persist a bootkick-recognised cause before the reset. The restored
    // state starts exactly one SoM wake pulse after Panda reinitializes.
    wake_debug_stage(0x34U);
  } else {
    wake_debug_stage(0x17U);
  }
  NVIC_SystemReset();
}
