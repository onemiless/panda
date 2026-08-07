#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BOOTKICK_WAKE_PRESTOP_IGNITION_PENDING_STAGE 0x32U
#define BOOTKICK_WAKE_PRESTOP_IGNITION_PULSE_STAGE 0x37U
#define BOOTKICK_SOM_OFF_CONFIRM_S 2U

typedef enum {
  BOOTKICK_DEFERRED_WAIT,
  BOOTKICK_DEFERRED_START,
  BOOTKICK_DEFERRED_ALREADY_ALIVE,
} bootkick_deferred_wake_action;

static inline bool bootkick_restore_waits_for_som_off(uint32_t wake_stage, uint8_t wake_success_latched) {
  return (wake_success_latched == 0U) && (wake_stage == BOOTKICK_WAKE_PRESTOP_IGNITION_PENDING_STAGE);
}

static inline bootkick_deferred_wake_action bootkick_deferred_wake_step(bool recent_heartbeat, bool som_powered,
                                                                       volatile uint8_t *off_confirm_countdown) {
  bootkick_deferred_wake_action action = BOOTKICK_DEFERRED_WAIT;
  if (recent_heartbeat) {
    action = BOOTKICK_DEFERRED_ALREADY_ALIVE;
  } else if (som_powered) {
    *off_confirm_countdown = BOOTKICK_SOM_OFF_CONFIRM_S;
  } else if (*off_confirm_countdown > 1U) {
    *off_confirm_countdown -= 1U;
  } else {
    *off_confirm_countdown = 0U;
    action = BOOTKICK_DEFERRED_START;
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

static inline bool bootkick_wake_request_needs_dispatch(bool monitor_enabled, bool som_off_ready,
                                                        bool can_wake_requested, bool dispatch_pending,
                                                        bool confirmation_pending, bool pulse_active, uint8_t wake_attempts,
                                                        uint32_t wake_stage) {
  const bool persisted_pending = (wake_stage == 0x3FU) || (wake_stage == 0x42U) || (wake_stage == 0x43U);
  return monitor_enabled && som_off_ready && can_wake_requested &&
         !confirmation_pending && !pulse_active && (wake_attempts == 0U) &&
         (dispatch_pending || persisted_pending);
}

static inline bool bootkick_tesla_event_ready(bool monitor_enabled, bool som_off_ready, bool can_armed,
                                              bool tesla_event_pending, bool can_wake_requested) {
  return monitor_enabled && som_off_ready && can_armed && tesla_event_pending && !can_wake_requested;
}

static inline bool tesla_wake_counter_valid(int8_t previous_counter, int8_t counter) {
  return (previous_counter >= 0) && (counter == ((previous_counter + 1) % 16));
}

static inline int8_t tesla_ui_warning_counter(const uint8_t *data) {
  return (int8_t)(data[1] & 0xFU);
}

static inline bool tesla_ui_warning_door_open(const uint8_t *data) {
  return (data[3] & 0x10U) != 0U;
}

static inline bool tesla_door_wake_ready(uint8_t logical_bus, uint8_t len, int8_t previous_counter,
                                         int8_t counter, bool door_open) {
  return (logical_bus == 0U) && (len == 7U) &&
         tesla_wake_counter_valid(previous_counter, counter) && door_open;
}

static inline bool tesla_front_door_latch_closed(const uint8_t *data) {
  return (data[1] & 0x1U) != 0U;
}

static inline bool tesla_front_door_handle_pulled(const uint8_t *data) {
  return (data[1] & 0x4U) != 0U;
}

static inline bool tesla_door_latch_wake_ready(uint8_t logical_bus, uint8_t len,
                                               bool previous_known, bool previous_closed,
                                               const uint8_t *data) {
  const bool closed_to_open = previous_known && previous_closed && !tesla_front_door_latch_closed(data);
  return (logical_bus == 0U) && (len == 8U) &&
         (tesla_front_door_handle_pulled(data) || closed_to_open);
}
