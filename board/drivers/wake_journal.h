#pragma once

#include "board/drivers/wake_journal_policy.h"
#include "board/stm32h7/llflash.h"

#define WAKE_JOURNAL_CAPACITY ((WAKE_JOURNAL_END - WAKE_JOURNAL_START) / WAKE_JOURNAL_RECORD_SIZE)
// A torn STM32H7 256-bit flashword can raise DBECCERR on the next read.
// Keep the protocol/read path available, but do not persist records until the
// power-cut/ECC recovery path has been proven on Tres hardware.
#define WAKE_JOURNAL_FLASH_WRITES_ENABLED false

_Static_assert(sizeof(wake_journal_record_t) == WAKE_JOURNAL_RECORD_SIZE,
               "wake journal record must match one H7 flashword");
_Static_assert((WAKE_JOURNAL_START % WAKE_JOURNAL_RECORD_SIZE) == 0U,
               "wake journal must be flashword aligned");
_Static_assert(WAKE_JOURNAL_CAPACITY <= UINT16_MAX, "wake journal slot index must fit USB param1");

static wake_journal_info_t wake_journal_info_state;
static wake_journal_record_t wake_journal_pending_event;
static wake_journal_record_t wake_journal_pending_result;
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
  wake_journal_info_state = wake_journal_scan(wake_journal_records(), WAKE_JOURNAL_CAPACITY);
}

static void wake_journal_begin_cycle(void) {
  wake_journal_info_state.current_cycle = wake_journal_info_state.next_sequence;
  // Reserve deterministic sequence numbers for the event and result. A power
  // cut may leave a gap, which is valid and preferable to rewriting a slot.
  wake_journal_info_state.next_sequence += 2U;
  wake_journal_cycle_active = true;
  wake_journal_event_queued = false;
  wake_journal_result_queued = false;
  wake_journal_cycle_source = 0U;
}

static void wake_journal_queue_event(uint8_t source, uint8_t trigger_stage,
                                     uint8_t logical_bus, uint8_t physical_bus,
                                     uint8_t len, uint32_t can_id, const uint8_t *data) {
  if (!wake_journal_cycle_active || wake_journal_event_queued ||
      ((wake_journal_info_state.flags & WAKE_JOURNAL_FLAG_FULL) != 0U)) {
    return;
  }
  wake_journal_build_event(&wake_journal_pending_event,
                           wake_journal_info_state.current_cycle,
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
                            wake_journal_info_state.current_cycle + 1U,
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
  for (uint8_t i = 0U; i < (WAKE_JOURNAL_RECORD_SIZE / sizeof(uint32_t)); i++) {
    flash_write_word(&((uint32_t *)destination)[i], words[i]);
  }
  flush_write_buffer();
  register_clear_bits(&(FLASH->CR1), FLASH_CR_PG);
  flash_lock();
  enable_interrupts();

  const bool valid = wake_journal_record_valid(destination) &&
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
