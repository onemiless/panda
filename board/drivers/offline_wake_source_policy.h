#pragma once

#include <stdbool.h>
#include <stdint.h>


#define OFFLINE_WAKE_SBU_EXTI_LINES ((1UL << 1) | (1UL << 4))
#define OFFLINE_WAKE_FDCAN1_EXTI_LINE (1UL << 8)
#define OFFLINE_WAKE_TRES_FDCAN3_EXTI_LINE (1UL << 9)
#define OFFLINE_WAKE_CUATRO_FDCAN3_EXTI_LINE (1UL << 12)
#define WAKE_MONITOR_CAN_RATE_DELTA 50U
#define WAKE_MONITOR_CAN_FAST_TICK_HZ 8U
#define WAKE_MONITOR_CAN_BURST_WINDOW_TICKS 2U
#define OFFLINE_WAKE_CAN_BASELINE_UNSET UINT32_MAX

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

// Blue LED status while Tres keeps FDCAN alive with the SoM powered down:
// off before the monitor is ready, one short flash every 4 seconds when armed,
// 4 Hz after CAN activity,
// and solid after a wake request has been dispatched.
static inline bool offline_wake_blue_led_on(bool monitor_ready, bool can_activity_seen,
                                            bool wake_requested, uint8_t tick_phase) {
  bool led_on = false;
  if (monitor_ready) {
    if (wake_requested) {
      led_on = true;
    } else if (can_activity_seen) {
      led_on = (tick_phase & 1U) == 0U;
    } else {
      led_on = tick_phase == 0U;
    }
  }
  return led_on;
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

static inline bool offline_wake_raw_can_edge_hint_ready(bool monitor_enabled, bool som_off_ready,
                                                        bool can_armed, bool wake_requested,
                                                        uint32_t pending_lines, uint32_t armed_lines) {
  return monitor_enabled && som_off_ready && can_armed && !wake_requested &&
         ((pending_lines & armed_lines) != 0U);
}

static inline uint32_t offline_wake_can_sleep_baseline_step(uint32_t baseline_rate, uint32_t current_rate) {
  return (baseline_rate == OFFLINE_WAKE_CAN_BASELINE_UNSET) || (current_rate < baseline_rate) ?
         current_rate : baseline_rate;
}

static inline bool offline_wake_can_rate_increase(uint32_t current_rate, uint32_t baseline_rate) {
  if (baseline_rate == OFFLINE_WAKE_CAN_BASELINE_UNSET) {
    return false;
  }
  const uint32_t delta = (current_rate > baseline_rate) ? (current_rate - baseline_rate) : 0U;
  // Tesla keeps low-rate background CAN traffic after the vehicle goes to
  // sleep. A real door/vehicle wake is a sustained rise above that sleeping
  // traffic, not necessarily a doubling of the host-alive traffic rate.
  return (delta >= WAKE_MONITOR_CAN_RATE_DELTA) && (delta >= (baseline_rate / 2U));
}

static inline uint32_t offline_wake_can_window_rate(uint32_t window_count) {
  const uint32_t scale = WAKE_MONITOR_CAN_FAST_TICK_HZ / WAKE_MONITOR_CAN_BURST_WINDOW_TICKS;
  return (window_count > (UINT32_MAX / scale)) ? UINT32_MAX : (window_count * scale);
}

static inline bool offline_wake_multibus_burst_ready(const uint32_t *window_counts,
                                                     const uint32_t *baseline_rates,
                                                     uint8_t party_physical_bus) {
  if (party_physical_bus >= 3U) {
    return false;
  }

  const bool party_active = offline_wake_can_rate_increase(
    offline_wake_can_window_rate(window_counts[party_physical_bus]), baseline_rates[party_physical_bus]);
  bool supporting_bus_active = false;
  for (uint8_t i = 0U; i < 3U; i++) {
    if (i != party_physical_bus) {
      supporting_bus_active |= offline_wake_can_rate_increase(
        offline_wake_can_window_rate(window_counts[i]), baseline_rates[i]);
    }
  }
  return party_active && supporting_bus_active;
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
