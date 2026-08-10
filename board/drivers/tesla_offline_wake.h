#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/drivers/bootkick_policy.h"

#define TESLA_WAKE_SOURCE_NONE 0U
#define TESLA_WAKE_SOURCE_POWER 1U
#define TESLA_WAKE_SOURCE_DOOR 2U

typedef struct {
  int8_t power_counter;
  int8_t ui_counter;
  bool ui_door_known;
  bool ui_door_open;
  uint8_t ui_unknown_open_streak;
  uint8_t front_door_known_mask;
  uint8_t front_door_closed_mask;
} tesla_offline_wake_state_t;

#define TESLA_OFFLINE_WAKE_STATE_INITIALIZER { \
  .power_counter = -1, \
  .ui_counter = -1, \
  .ui_door_known = false, \
  .ui_door_open = false, \
  .ui_unknown_open_streak = 0U, \
  .front_door_known_mask = 0U, \
  .front_door_closed_mask = 0U, \
}

typedef struct {
  uint8_t source;
  bool power_frame;
  uint8_t power_state;
  int8_t previous_counter;
  int8_t counter;
  bool counter_valid;
  bool checksum_valid;
} tesla_offline_wake_result_t;

static inline tesla_offline_wake_result_t tesla_offline_wake_step(
    tesla_offline_wake_state_t *state, uint16_t address, uint8_t logical_bus,
    uint8_t len, const uint8_t *data) {
  tesla_offline_wake_result_t result = {
    .source = TESLA_WAKE_SOURCE_NONE,
    .power_frame = false,
    .power_state = 0U,
    .previous_counter = -1,
    .counter = -1,
    .counter_valid = false,
    .checksum_valid = false,
  };

  if ((address == 0x221U) && (len == 8U)) {
    result.power_frame = true;
    result.power_state = (data[0] >> 5U) & 0x3U;
    result.counter = (int8_t)(data[6] >> 4U);
    if (logical_bus == 0U) {
      result.previous_counter = state->power_counter;
      result.counter_valid = tesla_wake_counter_valid(result.previous_counter, result.counter);
      result.checksum_valid = tesla_wake_checksum_valid(address, data, len, 7U);
      if (result.checksum_valid) {
        state->power_counter = result.counter;
        if (tesla_power_state_wake_ready(logical_bus, len, true, result.previous_counter,
                                         result.counter, result.power_state)) {
          result.source = TESLA_WAKE_SOURCE_POWER;
        }
      }
    }
  } else if ((address == 0x311U) && (len == 7U) && (logical_bus == 0U)) {
    result.counter = tesla_ui_warning_counter(data);
    result.previous_counter = state->ui_counter;
    result.counter_valid = tesla_wake_counter_valid(result.previous_counter, result.counter);
    result.checksum_valid = tesla_wake_checksum_valid(address, data, len, 0U);
    if (result.checksum_valid) {
      const bool door_open = tesla_ui_warning_door_open(data);
      const bool previous_known = state->ui_door_known;
      const bool previous_open = state->ui_door_open;
      const uint8_t unknown_open_streak = state->ui_unknown_open_streak;
      if (tesla_door_wake_ready(logical_bus, len, true, result.previous_counter,
                                result.counter, previous_known, previous_open,
                                unknown_open_streak, door_open)) {
        result.source = TESLA_WAKE_SOURCE_DOOR;
      }

      if (result.counter_valid) {
        if (previous_known) {
          state->ui_door_open = door_open;
        } else if (door_open) {
          state->ui_unknown_open_streak = (unknown_open_streak < 2U) ?
                                            (unknown_open_streak + 1U) : 2U;
          if (state->ui_unknown_open_streak >= 2U) {
            state->ui_door_known = true;
            state->ui_door_open = true;
          }
        } else {
          state->ui_door_known = true;
          state->ui_door_open = false;
          state->ui_unknown_open_streak = 0U;
        }
      } else if (result.previous_counter >= 0) {
        // Counter gaps resynchronise but never create an edge.
        state->ui_door_known = true;
        state->ui_door_open = door_open;
        state->ui_unknown_open_streak = 0U;
      } else if (!door_open) {
        state->ui_door_known = true;
        state->ui_door_open = false;
        state->ui_unknown_open_streak = 0U;
      } else {
        state->ui_unknown_open_streak = 1U;
      }
      state->ui_counter = result.counter;
    }
  } else if (((address == 0x102U) || (address == 0x103U)) &&
             (len == 8U) && ((logical_bus == 0U) || (logical_bus == 1U))) {
    const uint8_t door_mask = (address == 0x102U) ? 0x1U : 0x2U;
    const bool previous_known = (state->front_door_known_mask & door_mask) != 0U;
    const bool previous_closed = (state->front_door_closed_mask & door_mask) != 0U;
    if (tesla_door_latch_wake_ready(logical_bus, len, previous_known,
                                    previous_closed, data)) {
      result.source = TESLA_WAKE_SOURCE_DOOR;
    }
    state->front_door_known_mask |= door_mask;
    if (tesla_front_door_latch_closed(data)) {
      state->front_door_closed_mask |= door_mask;
    } else {
      state->front_door_closed_mask &= (uint8_t)(~door_mask);
    }
  } else {
  }

  return result;
}
