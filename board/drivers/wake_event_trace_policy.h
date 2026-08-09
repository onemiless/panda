#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WAKE_EVENT_TRACE_LEFT_DOOR 0xDU
#define WAKE_EVENT_TRACE_RIGHT_DOOR 0xEU
#define WAKE_EVENT_TRACE_UI_DOOR 0xFU

#define WAKE_EVENT_TRACE_SEEN_MASK 0x80U
#define WAKE_EVENT_TRACE_PREARM_STATE_MASK 0x60U
#define WAKE_EVENT_TRACE_POSTARM_STATE_MASK 0x20U
#define WAKE_EVENT_TRACE_COUNT_MASK 0x1FU

static inline uint8_t wake_event_trace_power_code(uint8_t physical_bus, uint8_t power_state) {
  return (physical_bus < 3U) ? (uint8_t)(1U + (physical_bus * 4U) + (power_state & 0x3U)) : 0U;
}

static inline uint8_t wake_event_trace_last(uint32_t sequence) {
  uint8_t last = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    const uint8_t event = (uint8_t)((sequence >> (i * 4U)) & 0xFU);
    if (event == 0U) {
      break;
    }
    last = event;
  }
  return last;
}

static inline uint8_t wake_event_trace_last_power(uint32_t sequence) {
  uint8_t last = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    const uint8_t event = (uint8_t)((sequence >> (i * 4U)) & 0xFU);
    if (event == 0U) {
      break;
    }
    if (event <= 0xCU) {
      last = event;
    }
  }
  return last;
}

static inline uint32_t wake_event_trace_append(uint32_t sequence, uint8_t event) {
  if ((event == 0U) || (wake_event_trace_last(sequence) == event)) {
    return sequence;
  }
  for (uint8_t i = 0U; i < 8U; i++) {
    const uint8_t shift = i * 4U;
    if (((sequence >> shift) & 0xFU) == 0U) {
      return sequence | ((uint32_t)(event & 0xFU) << shift);
    }
  }
  return sequence;
}

static inline uint8_t wake_event_trace_power_prearm(uint8_t power_state) {
  return WAKE_EVENT_TRACE_SEEN_MASK | ((power_state & 0x3U) << 5U);
}

static inline uint8_t wake_event_trace_door_prearm(bool state) {
  return WAKE_EVENT_TRACE_SEEN_MASK | ((uint8_t)state << 6U);
}

static inline bool wake_event_trace_prearm_seen(uint8_t meta) {
  return (meta & WAKE_EVENT_TRACE_SEEN_MASK) != 0U;
}

static inline uint8_t wake_event_trace_prearm_state(uint8_t meta) {
  return (meta >> 5U) & 0x3U;
}

static inline bool wake_event_trace_prearm_binary_state(uint8_t meta) {
  return (meta & 0x40U) != 0U;
}

static inline uint8_t wake_event_trace_prepare_binary_postarm(uint8_t meta) {
  const uint8_t prearm_state = wake_event_trace_prearm_binary_state(meta) ? WAKE_EVENT_TRACE_POSTARM_STATE_MASK : 0U;
  return (meta & (WAKE_EVENT_TRACE_SEEN_MASK | 0x40U)) | prearm_state;
}

static inline bool wake_event_trace_postarm_binary_state(uint8_t meta) {
  return (meta & WAKE_EVENT_TRACE_POSTARM_STATE_MASK) != 0U;
}

static inline uint8_t wake_event_trace_set_postarm_binary_state(uint8_t meta, bool state) {
  return (meta & (uint8_t)(~WAKE_EVENT_TRACE_POSTARM_STATE_MASK)) |
         ((uint8_t)state << 5U);
}

static inline uint8_t wake_event_trace_increment_count(uint8_t meta) {
  const uint8_t count = meta & WAKE_EVENT_TRACE_COUNT_MASK;
  return (meta & (uint8_t)(~WAKE_EVENT_TRACE_COUNT_MASK)) |
         ((count < WAKE_EVENT_TRACE_COUNT_MASK) ? (count + 1U) : count);
}

static inline uint8_t wake_event_trace_count(uint8_t meta) {
  return meta & WAKE_EVENT_TRACE_COUNT_MASK;
}
