#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/wake_protocol.h"

typedef enum {
  WAKE_MONITOR_PREPARE_INVALID,
  WAKE_MONITOR_PREPARE_IDEMPOTENT,
  WAKE_MONITOR_PREPARE_CONFLICT,
  WAKE_MONITOR_PREPARE_START,
} wake_monitor_prepare_action_t;

typedef enum {
  WAKE_MONITOR_HEARTBEAT_IGNORE,
  WAKE_MONITOR_HEARTBEAT_UNATTRIBUTED,
  WAKE_MONITOR_HEARTBEAT_CONFIRMED,
} wake_monitor_heartbeat_result_t;

static inline wake_monitor_prepare_action_t wake_monitor_prepare_action(uint8_t state,
                                                                        uint32_t current_transaction,
                                                                        uint32_t requested_transaction) {
  if (requested_transaction == 0U) {
    return WAKE_MONITOR_PREPARE_INVALID;
  }
  if ((state == WAKE_MONITOR_STATE_PREPARED) && (current_transaction == requested_transaction)) {
    return WAKE_MONITOR_PREPARE_IDEMPOTENT;
  }
  if (state != WAKE_MONITOR_STATE_IDLE) {
    return WAKE_MONITOR_PREPARE_CONFLICT;
  }
  return WAKE_MONITOR_PREPARE_START;
}

static inline bool wake_monitor_commit_allowed(uint8_t state, uint32_t current_transaction,
                                               uint32_t requested_transaction,
                                               uint32_t prepared_host_session, uint32_t current_host_session,
                                               bool prepare_dirty,
                                               bool can_healthy) {
  return (state == WAKE_MONITOR_STATE_PREPARED) && (requested_transaction != 0U) &&
         (current_transaction == requested_transaction) && (prepared_host_session != 0U) &&
         (prepared_host_session == current_host_session) && !prepare_dirty && can_healthy;
}

static inline bool wake_monitor_rx_snapshot_clean(const volatile uint32_t *prepared_rx, const volatile uint32_t *current_rx,
                                                  const volatile uint32_t *prepared_lost, const volatile uint32_t *current_lost,
                                                  const volatile uint32_t *prepared_resets, const volatile uint32_t *current_resets,
                                                  uint32_t prepared_overflow, uint32_t current_overflow,
                                                  uint8_t bus_count) {
  bool clean = prepared_overflow == current_overflow;
  for (uint8_t i = 0U; i < bus_count; i++) {
    clean &= (prepared_rx[i] == current_rx[i]) &&
             (prepared_lost[i] == current_lost[i]) &&
             (prepared_resets[i] == current_resets[i]);
  }
  return clean;
}

static inline bool wake_monitor_rx_integrity_clean(const volatile uint32_t *prepared_lost, const volatile uint32_t *current_lost,
                                                   const volatile uint32_t *prepared_resets, const volatile uint32_t *current_resets,
                                                   uint32_t prepared_overflow, uint32_t current_overflow,
                                                   uint8_t bus_count) {
  bool clean = prepared_overflow == current_overflow;
  for (uint8_t i = 0U; i < bus_count; i++) {
    clean &= (prepared_lost[i] == current_lost[i]) &&
             (prepared_resets[i] == current_resets[i]);
  }
  return clean;
}

static inline bool wake_monitor_offline_fault_ready(bool monitor_enabled, bool committed,
                                                    bool global_fault, bool controllers_ready,
                                                    bool rx_snapshot_clean, bool wake_requested,
                                                    bool fault_pending) {
  return monitor_enabled && committed && (global_fault || !controllers_ready || !rx_snapshot_clean) &&
         !wake_requested && !fault_pending;
}

static inline wake_monitor_heartbeat_result_t wake_monitor_heartbeat_result(bool committed,
                                                                             bool host_off_seen,
                                                                             uint32_t host_session,
                                                                             uint32_t committed_host_session,
                                                                             bool wake_attempted) {
  if (!committed || !host_off_seen || (host_session == 0U) || (host_session == committed_host_session)) {
    return WAKE_MONITOR_HEARTBEAT_IGNORE;
  }
  return wake_attempted ? WAKE_MONITOR_HEARTBEAT_CONFIRMED : WAKE_MONITOR_HEARTBEAT_UNATTRIBUTED;
}

static inline bool wake_monitor_failure_cooldown_step(volatile uint8_t *countdown) {
  if (*countdown > 0U) {
    *countdown -= 1U;
  }
  return *countdown == 0U;
}

static inline uint8_t wake_monitor_unattributed_result(uint8_t previous_result) {
  return (previous_result == WAKE_MONITOR_RESULT_FAILED) ? previous_result : WAKE_MONITOR_RESULT_UNATTRIBUTED;
}
