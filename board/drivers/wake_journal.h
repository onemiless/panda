#pragma once

#include "board/drivers/wake_journal_policy.h"
#include "board/stm32h7/llflash.h"

#define WAKE_JOURNAL_CAPACITY ((WAKE_JOURNAL_END - WAKE_JOURNAL_START) / WAKE_JOURNAL_RECORD_SIZE)
#define WAKE_JOURNAL_FLASH_WRITES_ENABLED true

_Static_assert(sizeof(wake_journal_record_t) == WAKE_JOURNAL_RECORD_SIZE,
               "wake journal record must match one H7 flashword");
_Static_assert((WAKE_JOURNAL_START % WAKE_JOURNAL_RECORD_SIZE) == 0U,
               "wake journal must be flashword aligned");
_Static_assert(WAKE_JOURNAL_CAPACITY <= UINT16_MAX, "wake journal slot index must fit USB param1");

static wake_journal_info_t wake_journal_info_state;
static wake_journal_record_t wake_journal_pending_commit;
static wake_journal_record_t wake_journal_pending_armed;
static wake_journal_record_t wake_journal_pending_event;
static wake_journal_record_t wake_journal_pending_result;
static volatile bool wake_journal_pending_commit_valid = false;
static volatile bool wake_journal_pending_armed_valid = false;
static volatile bool wake_journal_pending_event_valid = false;
static volatile bool wake_journal_pending_result_valid = false;
static bool wake_journal_cycle_active = false;
static bool wake_journal_event_queued = false;
static bool wake_journal_result_queued = false;
static uint8_t wake_journal_cycle_source = 0U;

static const wake_journal_record_t *wake_journal_records(void) {
  return (const wake_journal_record_t *)WAKE_JOURNAL_START;
}

static void wake_journal_init(void) {
  flash_clear_program_status();
  wake_journal_info_state = wake_journal_scan(wake_journal_records(), WAKE_JOURNAL_CAPACITY);
  if ((FLASH->SR1 & (FLASH_SR_SNECCERR | FLASH_SR_DBECCERR)) != 0U) {
    wake_journal_info_state.flags |= WAKE_JOURNAL_FLAG_FOREIGN_DATA;
  }
  flash_clear_program_status();
}

static void wake_journal_begin_cycle(void) {
  wake_journal_info_state.current_cycle = wake_journal_info_state.next_sequence;
  // Reserve deterministic sequence numbers for committed, armed, event, and
  // result records. A power cut may leave a gap; records are never rewritten.
  wake_journal_info_state.next_sequence += 4U;
  wake_journal_cycle_active = true;
  wake_journal_event_queued = false;
  wake_journal_result_queued = false;
  wake_journal_cycle_source = 0U;
}

static void wake_journal_abort_cycle(void) {
  wake_journal_pending_commit_valid = false;
  wake_journal_pending_armed_valid = false;
  wake_journal_pending_event_valid = false;
  wake_journal_pending_result_valid = false;
  wake_journal_cycle_active = false;
  wake_journal_event_queued = false;
  wake_journal_result_queued = false;
  wake_journal_cycle_source = 0U;
}

static void wake_journal_queue_checkpoint(uint8_t state, uint8_t stage,
                                          uint32_t transaction, uint32_t host_session,
                                          uint32_t off_seconds) {
  if (!wake_journal_cycle_active ||
      ((wake_journal_info_state.flags & WAKE_JOURNAL_FLAG_FULL) != 0U)) {
    return;
  }

  wake_journal_record_t *pending = NULL;
  volatile bool *pending_valid = NULL;
  uint32_t sequence_offset = 0U;
  if (state == WAKE_MONITOR_STATE_COMMITTED) {
    pending = &wake_journal_pending_commit;
    pending_valid = &wake_journal_pending_commit_valid;
  } else if (state == WAKE_MONITOR_STATE_ARMED) {
    pending = &wake_journal_pending_armed;
    pending_valid = &wake_journal_pending_armed_valid;
    sequence_offset = 1U;
  } else {
    return;
  }
  if (*pending_valid) {
    return;
  }
  wake_journal_build_checkpoint(pending,
                                wake_journal_info_state.current_cycle + sequence_offset,
                                wake_journal_info_state.current_cycle,
                                state, stage, transaction, host_session, off_seconds);
  *pending_valid = true;
}

static void wake_journal_queue_event(uint8_t source, uint8_t trigger_stage,
                                     uint8_t logical_bus, uint8_t physical_bus,
                                     uint8_t len, uint32_t can_id, const uint8_t *data) {
  if (!wake_journal_cycle_active || wake_journal_event_queued ||
      ((wake_journal_info_state.flags & WAKE_JOURNAL_FLAG_FULL) != 0U)) {
    return;
  }
  wake_journal_build_event(&wake_journal_pending_event,
                           wake_journal_info_state.current_cycle + 2U,
                           wake_journal_info_state.current_cycle,
                           source, trigger_stage, logical_bus, physical_bus,
                           len, can_id, data);
  wake_journal_cycle_source = source;
  wake_journal_event_queued = true;
  // Publish only after the complete record and CRC are in RAM.
  wake_journal_pending_event_valid = true;
}

static void wake_journal_queue_result(bool success, uint8_t attempts,
                                      bool uart_seen, bool reset_attempted,
                                      bool som_gpio, bool heartbeat_seen,
                                      uint32_t trigger_stage, uint32_t final_stage,
                                      uint32_t reset_reason) {
  if (!wake_journal_cycle_active || wake_journal_result_queued ||
      ((wake_journal_info_state.flags & WAKE_JOURNAL_FLAG_FULL) != 0U)) {
    return;
  }
  wake_journal_build_result(&wake_journal_pending_result,
                            wake_journal_info_state.current_cycle + 3U,
                            wake_journal_info_state.current_cycle,
                            wake_journal_cycle_source, success, attempts,
                            uart_seen, reset_attempted, som_gpio, heartbeat_seen,
                            trigger_stage, final_stage, reset_reason);
  wake_journal_result_queued = true;
  wake_journal_cycle_active = false;
  wake_journal_pending_result_valid = true;
}

static bool wake_journal_write_record(const wake_journal_record_t *record) {
  if (wake_journal_info_state.used_slots >= WAKE_JOURNAL_CAPACITY) {
    wake_journal_info_state.flags |= WAKE_JOURNAL_FLAG_FULL;
    return false;
  }

  const uint16_t slot = wake_journal_info_state.used_slots;
  wake_journal_record_t *destination = (wake_journal_record_t *)(WAKE_JOURNAL_START +
                                       ((uint32_t)slot * WAKE_JOURNAL_RECORD_SIZE));
  if (!wake_journal_record_empty(destination)) {
    // Never fill a hole or retry a partially programmed flashword.
    wake_journal_info_state.flags |= WAKE_JOURNAL_FLAG_FOREIGN_DATA;
    wake_journal_info_state.used_slots += 1U;
    return false;
  }

  const uint32_t *words = (const uint32_t *)record;
  disable_interrupts();
  if (flash_is_locked()) {
    flash_unlock();
  }
  const bool programmed = flash_write_flashword(destination, words);
  flash_lock();
  enable_interrupts();

  const bool valid = programmed && wake_journal_record_valid(destination) &&
                     (destination->sequence == record->sequence);
  wake_journal_info_state.used_slots += 1U;
  if (valid) {
    wake_journal_info_state.valid_records += 1U;
  } else {
    wake_journal_info_state.flags |= WAKE_JOURNAL_FLAG_FOREIGN_DATA;
  }
  if (wake_journal_info_state.used_slots >= WAKE_JOURNAL_CAPACITY) {
    wake_journal_info_state.flags |= WAKE_JOURNAL_FLAG_FULL;
  }
  return valid;
}

static void wake_journal_flush_pending(bool safe_to_write) {
  if (!safe_to_write || !WAKE_JOURNAL_FLASH_WRITES_ENABLED) {
    return;
  }
  if (wake_journal_pending_commit_valid) {
    (void)wake_journal_write_record(&wake_journal_pending_commit);
    wake_journal_pending_commit_valid = false;
  }
  if (wake_journal_pending_armed_valid) {
    (void)wake_journal_write_record(&wake_journal_pending_armed);
    wake_journal_pending_armed_valid = false;
  }
  if (wake_journal_pending_event_valid) {
    (void)wake_journal_write_record(&wake_journal_pending_event);
    wake_journal_pending_event_valid = false;
  }
  if (wake_journal_pending_result_valid) {
    (void)wake_journal_write_record(&wake_journal_pending_result);
    wake_journal_pending_result_valid = false;
  }
}

static wake_journal_info_t wake_journal_get_info(void) {
  return wake_journal_info_state;
}

static bool wake_journal_get_record(uint16_t slot, wake_journal_record_t *record) {
  if (slot >= wake_journal_info_state.used_slots) {
    return false;
  }
  *record = wake_journal_records()[slot];
  return true;
}
