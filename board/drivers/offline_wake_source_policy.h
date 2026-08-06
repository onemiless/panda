#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OFFLINE_WAKE_SBU_EXTI_LINES ((1UL << 1) | (1UL << 4))
#define OFFLINE_WAKE_FDCAN1_EXTI_LINE (1UL << 8)
#define OFFLINE_WAKE_TRES_FDCAN3_EXTI_LINE (1UL << 9)
#define OFFLINE_WAKE_CUATRO_FDCAN3_EXTI_LINE (1UL << 12)

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
