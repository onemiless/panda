#include "board/drivers/drivers.h"
#include "board/drivers/bootkick_policy.h"
#include "board/drivers/wake_debug.h"

bool bootkick_reset_triggered = false;
volatile uint16_t debug_bootkick_countdown = 0U;
volatile uint8_t debug_bootkick_hold_countdown = 0U;
volatile bool bootkick_reset_pulse_requested = false;
volatile bool bootkick_wake_pulse_active = false;
volatile uint8_t bootkick_wake_release_countdown = 0U;
volatile bool bootkick_wake_confirmation_pending = false;
volatile uint32_t bootkick_wake_trigger_stage = 0U;
volatile uint8_t bootkick_wake_attempts = 0U;
volatile uint8_t bootkick_wake_retry_countdown = 0U;
volatile uint16_t bootkick_wake_uart_ptr = 0U;
volatile bool bootkick_wake_uart_seen = false;
volatile bool bootkick_wake_reset_attempted = false;
volatile uint8_t bootkick_wake_final_countdown = 0U;
volatile uint8_t bootkick_wake_post_reset_countdown = 0U;

// Match the proven scheduled bootkick self-test. A shorter CAN-triggered pulse
// followed by the fast recovery reset can interrupt Tres while it is already
// starting but has not produced UART or heartbeat activity yet.
#define BOOTKICK_WAKE_PULSE_S 30U
#define BOOTKICK_WAKE_RELEASE_S 2U
#define BOOTKICK_WAKE_RETRY_DELAY_S 15U
#define BOOTKICK_WAKE_TRES_RESPONSE_WAIT_S 30U
#define BOOTKICK_WAKE_MAX_ATTEMPTS 3U
#define BOOTKICK_WAKE_FINAL_GRACE_S 60U
#define BOOTKICK_WAKE_POST_RESET_RELEASE_S 2U

bool bootkick_debug_active(void) {
  return (debug_bootkick_countdown > 0U) || bootkick_wake_pulse_active ||
         (bootkick_wake_release_countdown > 0U) || (bootkick_wake_post_reset_countdown > 0U) ||
         bootkick_wake_confirmation_pending;
}

void bootkick_debug_restore(void) {
  const uint8_t persisted_debug_wait = (uint8_t)(wake_debug.can_exti_line >> 16U);
  const uint8_t persisted_debug_hold = (uint8_t)(wake_debug.can_exti_line >> 24U);
  if ((persisted_debug_wait > 0U) || (persisted_debug_hold > 0U)) {
    debug_bootkick_countdown = persisted_debug_wait;
    debug_bootkick_hold_countdown = persisted_debug_hold;
    bootkick_wake_pulse_active = debug_bootkick_hold_countdown > 0U;
  }
  const bool initial_wake_stage = (wake_debug.stage >= 0x32U) && (wake_debug.stage <= 0x37U);
  const bool retry_wake_stage = ((wake_debug.stage >= 0x3BU) && (wake_debug.stage <= 0x3DU)) ||
                                (wake_debug.stage == 0x40U) || (wake_debug.stage == 0x41U);
  if ((wake_success.latched == 0U) && (initial_wake_stage || retry_wake_stage)) {
    const uint8_t persisted_state = (uint8_t)(wake_debug.hw_type_snapshot >> 24U);
    bootkick_wake_confirmation_pending = true;
    bootkick_wake_trigger_stage = wake_debug.stage;
    bootkick_wake_attempts = persisted_state & 0x3U;
    bootkick_wake_retry_countdown = (persisted_state >> 2U) & 0xFU;
    bootkick_wake_uart_seen = (persisted_state & (1U << 6U)) != 0U;
    bootkick_wake_reset_attempted = (persisted_state & (1U << 7U)) != 0U;
    bootkick_wake_final_countdown = (bootkick_wake_uart_seen || (bootkick_wake_attempts >= BOOTKICK_WAKE_MAX_ATTEMPTS)) ?
                                      BOOTKICK_WAKE_FINAL_GRACE_S : 0U;
    bootkick_wake_uart_ptr = uart_ring_som_debug.w_ptr_tx;
    if (wake_debug.stage == 0x3BU) {
      bootkick_wake_attempts = MAX(bootkick_wake_attempts, 2U);
    } else if (wake_debug.stage >= 0x3CU) {
      bootkick_wake_attempts = MAX(bootkick_wake_attempts, BOOTKICK_WAKE_MAX_ATTEMPTS);
    } else {
      bootkick_wake_attempts = MAX(bootkick_wake_attempts, 1U);
      // A bus-1 STOP wake is persisted as 0x34. Resume it immediately after
      // reset instead of waiting through the normal retry delay.
      debug_bootkick_hold_countdown = BOOTKICK_WAKE_PULSE_S;
      bootkick_wake_pulse_active = true;
      bootkick_wake_retry_countdown = 0U;
    }
  }
}

void bootkick_debug_schedule(uint16_t delay_s) {
  debug_bootkick_countdown = (delay_s > UINT8_MAX) ? UINT8_MAX : delay_s;
  debug_bootkick_hold_countdown = 0U;
  bootkick_wake_pulse_active = false;
  bootkick_wake_release_countdown = 0U;
  bootkick_wake_post_reset_countdown = 0U;
  bootkick_wake_confirmation_pending = false;
  bootkick_wake_trigger_stage = 0U;
  current_board->set_bootkick(BOOT_STANDBY);
  wake_debug_bootkick(BOOT_STANDBY, BOOT_STANDBY, 0U, 0U);
  wake_debug_bootkick_schedule((uint8_t)debug_bootkick_countdown, debug_bootkick_hold_countdown);
}

void bootkick_request_reset_pulse(void) {
  bootkick_reset_pulse_requested = true;
}

void bootkick_cancel_wake_pulse(void) {
  debug_bootkick_hold_countdown = 0U;
  bootkick_wake_pulse_active = false;
  bootkick_wake_release_countdown = 0U;
  bootkick_wake_post_reset_countdown = 0U;
}

void bootkick_clear_wake_confirmation(void) {
  bootkick_wake_confirmation_pending = false;
  bootkick_wake_trigger_stage = 0U;
  bootkick_wake_attempts = 0U;
  bootkick_wake_retry_countdown = 0U;
  bootkick_wake_uart_ptr = uart_ring_som_debug.w_ptr_tx;
  bootkick_wake_uart_seen = false;
  bootkick_wake_reset_attempted = false;
  bootkick_wake_final_countdown = 0U;
  bootkick_wake_post_reset_countdown = 0U;
}

static void bootkick_start_wake_pulse(uint32_t stage) {
  debug_bootkick_hold_countdown = BOOTKICK_WAKE_PULSE_S;
  bootkick_wake_pulse_active = true;
  bootkick_wake_release_countdown = 0U;
  wake_debug_stage(stage);
}

bool bootkick_request_wake_pulse(uint32_t stage) {
  if (bootkick_wake_pulse_active || bootkick_wake_confirmation_pending) {
    return false;
  }

  debug_bootkick_countdown = 0U;
  bootkick_wake_confirmation_pending = true;
  bootkick_wake_attempts = 1U;
  bootkick_wake_retry_countdown = (hw_type == HW_TYPE_TRES) ?
                                    BOOTKICK_WAKE_TRES_RESPONSE_WAIT_S : BOOTKICK_WAKE_RETRY_DELAY_S;
  bootkick_wake_uart_ptr = uart_ring_som_debug.w_ptr_tx;
  bootkick_wake_uart_seen = false;
  bootkick_wake_reset_attempted = false;
  bootkick_wake_final_countdown = 0U;
  bootkick_wake_post_reset_countdown = 0U;
  // Keep the first cause immutable. Retry/reset stages describe progress,
  // but must not turn a CAN wake into an apparent harness/reset wake.
  bootkick_wake_trigger_stage = stage;
  bootkick_start_wake_pulse(stage);
  return true;
}

void bootkick_tick(bool ignition, bool recent_heartbeat) {
  static uint16_t bootkick_last_serial_ptr = 0;
  static uint8_t waiting_to_boot_countdown = 0;
  static uint8_t boot_reset_countdown = 0;
  static uint8_t bootkick_harness_status_prev = HARNESS_STATUS_NC;
  static bool bootkick_ign_prev = false;
  static BootState boot_state = BOOT_BOOTKICK;
  BootState boot_state_prev = boot_state;
  const bool harness_inserted = (harness.status != bootkick_harness_status_prev) && (harness.status != HARNESS_STATUS_NC);

  if (recent_heartbeat) {
    if (bootkick_wake_confirmation_pending && current_board->read_som_gpio()) {
      wake_debug_latch_success(bootkick_wake_trigger_stage);
      bootkick_clear_wake_confirmation();
    }
    // Release bootkick as soon as the SoM is confirmed alive.
    bootkick_cancel_wake_pulse();
    boot_state = BOOT_STANDBY;
  } else if ((ignition && !bootkick_ign_prev) || harness_inserted) {
    // bootkick on rising edge of ignition or harness insertion
    boot_state = BOOT_BOOTKICK;
  } else {

  }

  if (debug_bootkick_countdown > 0U) {
    debug_bootkick_countdown -= 1U;
    if (debug_bootkick_countdown == 0U) {
      debug_bootkick_hold_countdown = BOOTKICK_WAKE_PULSE_S;
      bootkick_wake_pulse_active = true;
      wake_debug_stage(0x36U);
    }
  }
  if (bootkick_wake_pulse_active) {
    if (debug_bootkick_hold_countdown > 0U) {
      boot_state = BOOT_BOOTKICK;
      debug_bootkick_hold_countdown -= 1U;
    } else {
      // A wake request must be a pulse, not a permanently asserted level.
      bootkick_wake_pulse_active = false;
      bootkick_wake_release_countdown = BOOTKICK_WAKE_RELEASE_S;
      boot_state = BOOT_WAKE_RELEASE;
    }
  } else if (bootkick_wake_release_countdown > 0U) {
    boot_state = BOOT_WAKE_RELEASE;
    bootkick_wake_release_countdown -= 1U;
  } else if (boot_state == BOOT_WAKE_RELEASE) {
    boot_state = BOOT_STANDBY;
  } else {
  }

  if (bootkick_wake_confirmation_pending && !recent_heartbeat) {
    const bool som_powered = current_board->read_som_gpio();
    const bool tres_early_reset_ready = bootkick_tres_early_reset_ready(
      hw_type == HW_TYPE_TRES, bootkick_wake_attempts, bootkick_wake_retry_countdown,
      bootkick_wake_pulse_active, bootkick_wake_release_countdown, bootkick_wake_uart_seen,
      som_powered, bootkick_wake_reset_attempted);
    if (!bootkick_wake_uart_seen && (uart_ring_som_debug.w_ptr_tx != bootkick_wake_uart_ptr)) {
      // UART activity means the SoM is already booting; another DC_IN edge could interrupt it.
      bootkick_wake_uart_seen = true;
      bootkick_wake_final_countdown = BOOTKICK_WAKE_FINAL_GRACE_S;
    } else if (!tres_early_reset_ready &&
               !bootkick_wake_pulse_active && (bootkick_wake_release_countdown == 0U) && !bootkick_wake_uart_seen &&
               ((hw_type != HW_TYPE_TRES) || !som_powered) &&
               !bootkick_wake_reset_attempted &&
               (bootkick_wake_attempts < BOOTKICK_WAKE_MAX_ATTEMPTS)) {
      if (bootkick_wake_retry_countdown > 0U) {
        bootkick_wake_retry_countdown -= 1U;
      } else {
        bootkick_wake_attempts += 1U;
        bootkick_wake_retry_countdown = BOOTKICK_WAKE_RETRY_DELAY_S;
        bootkick_start_wake_pulse(0x39U + bootkick_wake_attempts);
      }
    } else {
    }
  }

  const bool tres_early_reset_ready = bootkick_tres_early_reset_ready(
    hw_type == HW_TYPE_TRES, bootkick_wake_attempts, bootkick_wake_retry_countdown,
    bootkick_wake_pulse_active, bootkick_wake_release_countdown, bootkick_wake_uart_seen,
    current_board->read_som_gpio(), bootkick_wake_reset_attempted);
  if (bootkick_wake_confirmation_pending && !recent_heartbeat &&
      (tres_early_reset_ready ||
       (!bootkick_wake_uart_seen && !current_board->read_som_gpio() &&
        !bootkick_wake_pulse_active && (bootkick_wake_release_countdown == 0U) &&
        (bootkick_wake_attempts >= BOOTKICK_WAKE_MAX_ATTEMPTS) &&
        !bootkick_wake_reset_attempted && (hw_type == HW_TYPE_TRES)))) {
    // Tres has no DC_IN control. If repeated BOOTKICK pulses produce no SoM
    // UART activity, reset the powered-but-stalled SoM before holding BOOTKICK.
    bootkick_wake_reset_attempted = true;
    boot_reset_countdown = 5U;
    bootkick_reset_triggered = true;
    boot_state = BOOT_RESET;
    wake_debug_stage(0x3DU);
  }

  /*
    Ensure SOM boots in case it goes into QDL mode. Reset behavior:
    * shouldn't trigger on the first boot after power-on
    * only try reset once per bootkick, i.e. don't keep trying until booted
    * only try once per panda boot, since openpilot will reset panda on startup
    * once BOOT_RESET is triggered, it stays until countdown is finished
  */
  if (!bootkick_reset_triggered && (boot_state == BOOT_BOOTKICK) && (boot_state_prev == BOOT_STANDBY)) {
    waiting_to_boot_countdown = 20U;
  }
  if (waiting_to_boot_countdown > 0U) {
    bool serial_activity = uart_ring_som_debug.w_ptr_tx != bootkick_last_serial_ptr;
    if (serial_activity || current_board->read_som_gpio() || (boot_state != BOOT_BOOTKICK)) {
      waiting_to_boot_countdown = 0U;
    } else {
      // try a reset
      if (waiting_to_boot_countdown == 1U) {
        boot_reset_countdown = 5U;
      }
    }
  }

  // handle reset state
  if (boot_reset_countdown > 0U) {
    boot_state = BOOT_RESET;
    bootkick_reset_triggered = true;
  } else {
    if (boot_state == BOOT_RESET) {
      // Release RESET and BOOTKICK before creating the wake edge. Calling
      // BOOT_BOOTKICK here changes PA0 before PC12 is released on Tres, so a
      // deeply sleeping PMIC can miss the edge while RESET is still active.
      if (bootkick_wake_confirmation_pending && bootkick_wake_reset_attempted && !bootkick_wake_uart_seen) {
        boot_state = BOOT_STANDBY;
        bootkick_wake_post_reset_countdown = BOOTKICK_WAKE_POST_RESET_RELEASE_S;
        wake_debug_stage(0x40U);
      } else {
        boot_state = BOOT_BOOTKICK;
      }
    }
  }

  if (bootkick_wake_post_reset_countdown > 0U) {
    boot_state = BOOT_STANDBY;
    bootkick_wake_post_reset_countdown -= 1U;
    if (bootkick_wake_post_reset_countdown == 0U) {
      // Start on the next tick so BOOTKICK remains released for the complete
      // final second before the PMIC sees the new falling edge.
      bootkick_start_wake_pulse(0x41U);
    }
  }

  if (bootkick_reset_pulse_requested && !current_board->read_som_gpio()) {
    boot_state = BOOT_RESET;
    boot_reset_countdown = 5U;
    waiting_to_boot_countdown = 25U;
    bootkick_reset_triggered = true;
    bootkick_reset_pulse_requested = false;
    wake_debug_stage(0x33U);
    wake_debug_latch_success(0x33U);
  }

  const bool wake_attempts_finished = (hw_type == HW_TYPE_TRES) ?
                                        bootkick_wake_reset_attempted :
                                        (bootkick_wake_attempts >= BOOTKICK_WAKE_MAX_ATTEMPTS);
  const bool wake_final_wait = bootkick_wake_uart_seen || wake_attempts_finished;
  const bool wake_output_idle = !bootkick_wake_pulse_active && (bootkick_wake_release_countdown == 0U) &&
                                (bootkick_wake_post_reset_countdown == 0U) && (boot_state != BOOT_RESET);
  if (bootkick_wake_confirmation_pending && !recent_heartbeat && wake_final_wait && wake_output_idle) {
    if (bootkick_wake_final_countdown == 0U) {
      bootkick_wake_final_countdown = BOOTKICK_WAKE_FINAL_GRACE_S;
    } else {
      bootkick_wake_final_countdown -= 1U;
      if (bootkick_wake_final_countdown == 0U) {
        wake_debug_stage(0x3EU);
        bootkick_clear_wake_confirmation();
      }
    }
  }

  // update state
  bootkick_ign_prev = ignition;
  bootkick_harness_status_prev = harness.status;
  bootkick_last_serial_ptr = uart_ring_som_debug.w_ptr_tx;
  if (waiting_to_boot_countdown > 0U) {
    waiting_to_boot_countdown--;
  }
  if (boot_reset_countdown > 0U) {
    boot_reset_countdown--;
  }
  current_board->set_bootkick(boot_state);
  wake_debug_bootkick(boot_state, boot_state_prev, waiting_to_boot_countdown, boot_reset_countdown);
  wake_debug_bootkick_schedule((uint8_t)debug_bootkick_countdown, debug_bootkick_hold_countdown);
  if (hw_type == HW_TYPE_CUATRO) {
    wake_debug_bootkick_pins(boot_state, get_gpio_input(GPIOA, 0) != 0, get_gpio_input(GPIOC, 11) != 0);
  } else if (hw_type == HW_TYPE_TRES) {
    wake_debug_bootkick_pins(boot_state, get_gpio_input(GPIOA, 0) != 0, get_gpio_input(GPIOC, 12) != 0);
  } else {
  }
  wake_debug_bootkick_wake_state(bootkick_wake_attempts, bootkick_wake_retry_countdown,
                                 bootkick_wake_uart_seen, bootkick_wake_reset_attempted);
}
