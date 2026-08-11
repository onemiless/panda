#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/wake_protocol.h"

static inline bool wake_active_can_diag_valid(uint32_t snapshot) {
  return (snapshot & WAKE_ACTIVE_CAN_DIAG_MAGIC_MASK) == WAKE_ACTIVE_CAN_DIAG_V1_MAGIC;
}

static inline uint32_t wake_active_can_diag_make(uint32_t flags) {
  return WAKE_ACTIVE_CAN_DIAG_V1_MAGIC | (flags & WAKE_ACTIVE_CAN_DIAG_PAYLOAD_MASK);
}

static inline uint32_t wake_active_can_diag_latch_irq(uint32_t snapshot, uint8_t physical_bus) {
  if (!wake_active_can_diag_valid(snapshot) || (physical_bus >= 3U)) {
    return snapshot;
  }
  return snapshot | (1UL << (WAKE_ACTIVE_CAN_DIAG_IRQ_SEEN_SHIFT + physical_bus));
}

static inline bool wake_active_can_diag_irq_seen(uint32_t snapshot, uint8_t physical_bus) {
  return wake_active_can_diag_valid(snapshot) && (physical_bus < 3U) &&
         ((snapshot & (1UL << (WAKE_ACTIVE_CAN_DIAG_IRQ_SEEN_SHIFT + physical_bus))) != 0U);
}

static inline uint32_t wake_active_can_diag_pack_first_rx(uint8_t physical_bus, uint32_t address) {
  return WAKE_ACTIVE_CAN_FIRST_RX_VALID |
         (((uint32_t)physical_bus & 0x3U) << 29U) | (address & 0x1FFFFFFFU);
}

static inline bool wake_active_can_diag_first_rx_valid(uint32_t first_rx) {
  return (first_rx & WAKE_ACTIVE_CAN_FIRST_RX_VALID) != 0U;
}

static inline uint16_t wake_active_can_diag_peak(uint16_t previous_peak, uint16_t window_count) {
  return (window_count > previous_peak) ? window_count : previous_peak;
}
