#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/wake_protocol.h"

static inline uint32_t wake_journal_crc32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFU;
  for (uint32_t i = 0U; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++) {
      const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

static inline bool wake_journal_record_empty(const wake_journal_record_t *record) {
  const uint32_t *words = (const uint32_t *)record;
  bool empty = true;
  for (uint8_t i = 0U; i < (WAKE_JOURNAL_RECORD_SIZE / sizeof(uint32_t)); i++) {
    empty &= words[i] == UINT32_MAX;
  }
  return empty;
}

static inline bool wake_journal_record_valid(const wake_journal_record_t *record) {
  const uint8_t version = (uint8_t)(record->meta & 0xFFU);
  return (record->magic == WAKE_JOURNAL_MAGIC) && (version == WAKE_JOURNAL_VERSION) &&
         (record->crc32 == wake_journal_crc32((const uint8_t *)record,
                                              WAKE_JOURNAL_RECORD_SIZE - sizeof(uint32_t)));
}

static inline uint32_t wake_journal_meta(uint8_t type, uint8_t source, uint16_t auxiliary) {
  return WAKE_JOURNAL_VERSION | ((uint32_t)(type & 0xFU) << 8U) |
         ((uint32_t)(source & 0xFU) << 12U) | ((uint32_t)auxiliary << 16U);
}

static inline void wake_journal_finish_record(wake_journal_record_t *record) {
  record->crc32 = wake_journal_crc32((const uint8_t *)record,
                                     WAKE_JOURNAL_RECORD_SIZE - sizeof(uint32_t));
}

static inline void wake_journal_build_event(wake_journal_record_t *record,
                                            uint32_t sequence, uint32_t cycle,
                                            uint8_t source, uint8_t trigger_stage,
                                            uint8_t logical_bus, uint8_t physical_bus,
                                            uint8_t len, uint32_t can_id,
                                            const uint8_t *data) {
  const uint8_t copy_len = (len < 8U) ? len : 8U;
  const uint16_t auxiliary = (uint16_t)((logical_bus & 0x3U) |
                                        ((physical_bus & 0x3U) << 2U) |
                                        ((copy_len & 0xFU) << 4U) |
                                        ((uint16_t)trigger_stage << 8U));
  *record = (wake_journal_record_t){
    .magic = WAKE_JOURNAL_MAGIC,
    .sequence = sequence,
    .cycle = cycle,
    .meta = wake_journal_meta(WAKE_JOURNAL_RECORD_EVENT, source, auxiliary),
    .value0 = can_id,
    .value1 = 0U,
    .value2 = 0U,
    .crc32 = 0U,
  };
  uint8_t *payload = (uint8_t *)&record->value1;
  for (uint8_t i = 0U; i < copy_len; i++) {
    payload[i] = data[i];
  }
  wake_journal_finish_record(record);
}

static inline void wake_journal_build_result(wake_journal_record_t *record,
                                             uint32_t sequence, uint32_t cycle,
                                             uint8_t source, bool success,
                                             uint8_t attempts, bool uart_seen,
                                             bool reset_attempted, bool som_gpio,
                                             bool heartbeat_seen, uint32_t trigger_stage,
                                             uint32_t final_stage, uint32_t reset_reason) {
  const uint16_t auxiliary = (attempts & 0x3U) |
                             ((uint16_t)uart_seen << 2U) |
                             ((uint16_t)reset_attempted << 3U) |
                             ((uint16_t)som_gpio << 4U) |
                             ((uint16_t)heartbeat_seen << 5U) |
                             ((uint16_t)success << 6U);
  *record = (wake_journal_record_t){
    .magic = WAKE_JOURNAL_MAGIC,
    .sequence = sequence,
    .cycle = cycle,
    .meta = wake_journal_meta(WAKE_JOURNAL_RECORD_RESULT, source, auxiliary),
    .value0 = trigger_stage,
    .value1 = final_stage,
    .value2 = reset_reason,
    .crc32 = 0U,
  };
  wake_journal_finish_record(record);
}

static inline void wake_journal_build_checkpoint(wake_journal_record_t *record,
                                                 uint32_t sequence, uint32_t cycle,
                                                 uint8_t state, uint8_t stage,
                                                 uint32_t transaction, uint32_t host_session,
                                                 uint32_t off_seconds) {
  const uint16_t auxiliary = (uint16_t)state | ((uint16_t)stage << 8U);
  *record = (wake_journal_record_t){
    .magic = WAKE_JOURNAL_MAGIC,
    .sequence = sequence,
    .cycle = cycle,
    .meta = wake_journal_meta(WAKE_JOURNAL_RECORD_CHECKPOINT, 0U, auxiliary),
    .value0 = transaction,
    .value1 = host_session,
    .value2 = off_seconds,
    .crc32 = 0U,
  };
  wake_journal_finish_record(record);
}

static inline wake_journal_info_t wake_journal_scan(const wake_journal_record_t *records,
                                                    uint16_t capacity) {
  wake_journal_info_t info = {
    .magic = WAKE_JOURNAL_MAGIC,
    .version = WAKE_JOURNAL_VERSION,
    .record_size = WAKE_JOURNAL_RECORD_SIZE,
    .capacity = capacity,
    .used_slots = 0U,
    .valid_records = 0U,
    .flags = 0U,
    .next_sequence = 0U,
    .current_cycle = 0U,
    .reserved0 = 0U,
    .reserved1 = 0U,
  };
  bool sequence_seen = false;
  uint32_t highest_sequence = 0U;
  for (uint16_t slot = 0U; slot < capacity; slot++) {
    const wake_journal_record_t *record = &records[slot];
    if (!wake_journal_record_empty(record)) {
      info.used_slots = slot + 1U;
      if (wake_journal_record_valid(record)) {
        info.valid_records += 1U;
        if (!sequence_seen || ((int32_t)(record->sequence - highest_sequence) > 0)) {
          highest_sequence = record->sequence;
          sequence_seen = true;
        }
      } else {
        info.flags |= WAKE_JOURNAL_FLAG_FOREIGN_DATA;
      }
    }
  }
  info.next_sequence = sequence_seen ? (highest_sequence + 1U) : 0U;
  if (info.used_slots >= capacity) {
    info.flags |= WAKE_JOURNAL_FLAG_FULL;
  }
  return info;
}
