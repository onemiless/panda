#pragma once

#include <stdbool.h>
#include <stdint.h>

// STM32H7 flash consists of eight 128 KiB sectors. Keep the application,
// persistent wake journal, and provisioning data in disjoint erase domains.
#define APP_START_ADDRESS 0x08020000U
#define APP_END_ADDRESS 0x080C0000U
#define APP_FIRST_SECTOR 1U
#define APP_LAST_SECTOR 5U

#define WAKE_JOURNAL_START 0x080C0000U
#define WAKE_JOURNAL_END 0x080E0000U
#define WAKE_JOURNAL_SECTOR 6U

#define PROVISION_SECTOR_START 0x080E0000U
#define PROVISION_SECTOR_END 0x08100000U
#define PROVISION_SECTOR 7U

#define APP_MAX_SIZE (APP_END_ADDRESS - APP_START_ADDRESS)

static inline bool flash_app_sector_allowed(uint16_t sector) {
  return (sector >= APP_FIRST_SECTOR) && (sector <= APP_LAST_SECTOR);
}

static inline bool flash_sector_erase_allowed(uint16_t sector) {
  // Sector 0 contains the bootstub and sector 7 contains device identity and
  // provisioning. Sector 6 remains available only to the wake journal.
  return (sector > 0U) && (sector < PROVISION_SECTOR);
}

static inline bool flash_app_write_allowed(uint32_t address, uint32_t len) {
  return (address >= APP_START_ADDRESS) && (address <= APP_END_ADDRESS) &&
         (len <= (APP_END_ADDRESS - address));
}

static inline bool flash_app_signed_length_valid(uint32_t signed_length, uint32_t signature_length) {
  return (signed_length >= 8U) && (signature_length <= APP_MAX_SIZE) &&
         (signed_length <= (APP_MAX_SIZE - signature_length));
}
