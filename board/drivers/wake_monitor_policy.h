#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/wake_protocol.h"

typedef enum {
  WAKE_MONITOR_PREPARE_INVALID,
  WAKE_MONITOR_PREPARE_IDEMPOTENT,
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
  return WAKE_MONITOR_PREPARE_START;
}

static inline bool wake_monitor_commit_allowed(uint8_t state, uint32_t current_transaction,
                                               uint32_t requested_transaction) {
  return (state == WAKE_MONITOR_STATE_PREPARED) && (requested_transaction != 0U) &&
         (current_transaction == requested_transaction);
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
