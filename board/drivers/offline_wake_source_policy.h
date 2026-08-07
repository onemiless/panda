#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OFFLINE_WAKE_SBU_EXTI_LINES ((1UL << 1) | (1UL << 4))
#define OFFLINE_WAKE_FDCAN1_EXTI_LINE (1UL << 8)
#define OFFLINE_WAKE_TRES_FDCAN3_EXTI_LINE (1UL << 9)
#define OFFLINE_WAKE_CUATRO_FDCAN3_EXTI_LINE (1UL << 12)

typedef struct {
  bool keep_can_active;
  bool request_power_save;
  bool request_strict_stop;
} offline_wake_heartbeat_loss_policy;

// Tres STOP/EXTI wake was observed entering WFI without ever returning on
// real vehicle traffic. Keep only Tres in receive-only active monitoring;
// preserve the existing low-power behavior for other hardware.
static inline offline_wake_heartbeat_loss_policy offline_wake_policy_after_heartbeat_loss(bool is_tres) {
  const offline_wake_heartbeat_loss_policy policy = {
    .keep_can_active = is_tres,
    .request_power_save = !is_tres,
    .request_strict_stop = !is_tres,
  };
  return policy;
}

static inline uint32_t offline_wake_oriented_fdcan2_exti_line(bool flipped_harness) {
  return flipped_harness ? (1UL << 12) : (1UL << 5);
}

static inline uint32_t offline_wake_tres_can_exti_lines(bool flipped_harness) {
  return OFFLINE_WAKE_FDCAN1_EXTI_LINE | OFFLINE_WAKE_TRES_FDCAN3_EXTI_LINE |
         offline_wake_oriented_fdcan2_exti_line(flipped_harness);
}

static inline uint32_t offline_wake_tres_exti_lines(bool flipped_harness) {
  return OFFLINE_WAKE_SBU_EXTI_LINES | offline_wake_tres_can_exti_lines(flipped_harness);
}

static inline uint32_t offline_wake_cuatro_can_exti_lines(bool flipped_harness) {
  // Cuatro FDCAN3 (PD12) and flipped FDCAN2 (PB12) share EXTI12 and cannot be
  // routed simultaneously. In the flipped case preserve the already-proven
  // FDCAN2 wake input; FDCAN1 remains independently armed on EXTI8.
  uint32_t lines = OFFLINE_WAKE_FDCAN1_EXTI_LINE | offline_wake_oriented_fdcan2_exti_line(flipped_harness);
  if (!flipped_harness) {
    lines |= OFFLINE_WAKE_CUATRO_FDCAN3_EXTI_LINE;
  }
  return lines;
}

static inline uint32_t offline_wake_cuatro_exti_lines(bool flipped_harness) {
  return OFFLINE_WAKE_SBU_EXTI_LINES | offline_wake_cuatro_can_exti_lines(flipped_harness);
}
