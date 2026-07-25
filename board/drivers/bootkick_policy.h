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
