#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OFFLINE_WAKE_SBU_EXTI_LINES ((1UL << 1) | (1UL << 4))

static inline uint32_t offline_wake_can_exti_line(bool flipped_harness) {
  return flipped_harness ? (1UL << 12) : (1UL << 5);
}

static inline uint32_t offline_wake_exti_lines(bool flipped_harness) {
  return OFFLINE_WAKE_SBU_EXTI_LINES | offline_wake_can_exti_line(flipped_harness);
}
