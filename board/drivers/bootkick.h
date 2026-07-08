#include "board/drivers/drivers.h"
#include "board/drivers/wake_debug.h"

bool bootkick_reset_triggered = false;
volatile uint16_t debug_bootkick_countdown = 0U;
volatile uint8_t debug_bootkick_hold_countdown = 0U;
volatile bool bootkick_reset_pulse_requested = false;

bool bootkick_debug_active(void) {
  return (debug_bootkick_countdown > 0U) || (debug_bootkick_hold_countdown > 0U);
}

void bootkick_debug_restore(void) {
  if ((wake_debug.bootkick_state == (uint8_t)BOOT_STANDBY) && (wake_debug.bootkick_prev_state == (uint8_t)BOOT_RESET)) {
    debug_bootkick_countdown = wake_debug.bootkick_waiting_countdown;
    debug_bootkick_hold_countdown = wake_debug.bootkick_reset_countdown;
  }
}

void bootkick_debug_schedule(uint16_t delay_s) {
  debug_bootkick_countdown = (delay_s > UINT8_MAX) ? UINT8_MAX : delay_s;
  debug_bootkick_hold_countdown = 0U;
  current_board->set_bootkick(BOOT_STANDBY);
  wake_debug_bootkick(BOOT_STANDBY, BOOT_RESET, (uint8_t)debug_bootkick_countdown, debug_bootkick_hold_countdown);
}

void bootkick_request_reset_pulse(void) {
  bootkick_reset_pulse_requested = true;
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

  if ((ignition && !bootkick_ign_prev) || harness_inserted) {
    // bootkick on rising edge of ignition or harness insertion
    boot_state = BOOT_BOOTKICK;
  } else if (recent_heartbeat) {
    // disable bootkick once openpilot is up
    boot_state = BOOT_STANDBY;
  } else {

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
      boot_state = BOOT_BOOTKICK;
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

  if (debug_bootkick_countdown > 0U) {
    debug_bootkick_countdown -= 1U;
    if (debug_bootkick_countdown == 0U) {
      debug_bootkick_hold_countdown = 30U;
      wake_debug_stage(0x36U);
      wake_debug_latch_success(0x36U);
    }
  }
  if (debug_bootkick_hold_countdown > 0U) {
    boot_state = BOOT_BOOTKICK;
    debug_bootkick_hold_countdown -= 1U;
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
  if (bootkick_debug_active()) {
    wake_debug_bootkick(BOOT_STANDBY, BOOT_RESET, (uint8_t)debug_bootkick_countdown, debug_bootkick_hold_countdown);
  } else {
    wake_debug_bootkick(boot_state, boot_state_prev, waiting_to_boot_countdown, boot_reset_countdown);
  }
  current_board->set_bootkick(boot_state);
}
