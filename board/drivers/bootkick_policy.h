#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BOOTKICK_WAKE_PRESTOP_IGNITION_PENDING_STAGE 0x32U
#define BOOTKICK_WAKE_PRESTOP_IGNITION_PULSE_STAGE 0x37U
#define BOOTKICK_SOM_OFF_CONFIRM_S 2U
#define BOOTKICK_SOM_OFF_FALLBACK_S 30U
#define BOOTKICK_UART_PROGRESS_TIMEOUT_S 5U

typedef enum {
  BOOTKICK_DEFERRED_WAIT,
  BOOTKICK_DEFERRED_START,
  BOOTKICK_DEFERRED_ALREADY_ALIVE,
} bootkick_deferred_wake_action;

static inline bool bootkick_restore_waits_for_som_off(uint32_t wake_stage, uint8_t wake_success_latched) {
  return (wake_success_latched == 0U) && (wake_stage == BOOTKICK_WAKE_PRESTOP_IGNITION_PENDING_STAGE);
}

static inline bootkick_deferred_wake_action bootkick_deferred_wake_step(bool recent_heartbeat, bool som_powered,
                                                                       volatile uint8_t *off_confirm_countdown,
                                                                       volatile uint8_t *heartbeat_absent_countdown) {
  bootkick_deferred_wake_action action = BOOTKICK_DEFERRED_WAIT;
  if (recent_heartbeat) {
    *off_confirm_countdown = BOOTKICK_SOM_OFF_CONFIRM_S;
    *heartbeat_absent_countdown = BOOTKICK_SOM_OFF_FALLBACK_S;
    action = BOOTKICK_DEFERRED_ALREADY_ALIVE;
  } else {
    if (*heartbeat_absent_countdown > 0U) {
      *heartbeat_absent_countdown -= 1U;
    }
    if (som_powered) {
      *off_confirm_countdown = BOOTKICK_SOM_OFF_CONFIRM_S;
    } else if (*off_confirm_countdown > 0U) {
      *off_confirm_countdown -= 1U;
    }
    if ((*off_confirm_countdown == 0U) || (*heartbeat_absent_countdown == 0U)) {
      action = BOOTKICK_DEFERRED_START;
    }
  }
  return action;
}

static inline bool bootkick_tres_early_reset_ready(bool is_tres, uint8_t wake_attempts,
                                                   uint8_t retry_countdown, bool pulse_active,
                                                   uint8_t release_countdown, bool uart_seen,
                                                   bool som_powered, bool reset_attempted) {
  (void)som_powered;
  return is_tres && (wake_attempts >= 1U) && (retry_countdown == 0U) &&
         !pulse_active && (release_countdown == 0U) && !uart_seen &&
         !reset_attempted;
}

static inline bool bootkick_can_activity_ready(bool monitor_enabled, bool som_off_ready, bool can_armed,
                                               bool can_activity_pending, bool wake_requested) {
  return monitor_enabled && som_off_ready && can_armed && can_activity_pending && !wake_requested;
}

static inline bool bootkick_heartbeat_confirms_wake(bool recent_heartbeat, bool confirmation_pending) {
  return recent_heartbeat && confirmation_pending;
}

static inline uint32_t bootkick_success_source_stage(bool confirmation_pending,
                                                     uint32_t trigger_stage,
                                                     uint32_t progress_stage) {
  return confirmation_pending ? trigger_stage : progress_stage;
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
