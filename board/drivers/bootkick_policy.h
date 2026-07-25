#pragma once

#include <stdbool.h>
#include <stdint.h>

static inline bool bootkick_tres_early_reset_ready(bool is_tres, uint8_t wake_attempts,
                                                   uint8_t retry_countdown, bool pulse_active,
                                                   uint8_t release_countdown, bool uart_seen,
                                                   bool som_powered, bool reset_attempted) {
  return is_tres && (wake_attempts >= 1U) && (retry_countdown == 0U) &&
         !pulse_active && (release_countdown == 0U) && !uart_seen &&
         !som_powered && !reset_attempted;
}

static inline bool bootkick_wake_request_needs_dispatch(bool monitor_enabled, bool som_off_ready,
                                                        bool can_wake_requested, bool dispatch_pending,
                                                        bool confirmation_pending, bool pulse_active, uint8_t wake_attempts,
                                                        uint32_t wake_stage) {
  const bool persisted_pending = (wake_stage == 0x3FU) || (wake_stage == 0x42U) || (wake_stage == 0x43U);
  return monitor_enabled && som_off_ready && can_wake_requested &&
         !confirmation_pending && !pulse_active && (wake_attempts == 0U) &&
         (dispatch_pending || persisted_pending);
}
