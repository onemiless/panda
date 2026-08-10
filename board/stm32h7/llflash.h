bool flash_is_locked(void) {
  return (FLASH->CR1 & FLASH_CR_LOCK);
}

void flash_unlock(void) {
  FLASH->KEYR1 = 0x45670123;
  FLASH->KEYR1 = 0xCDEF89AB;
}

void flash_lock(void) {
  FLASH->CR1 |= FLASH_CR_LOCK;
}

bool flash_erase_sector(uint8_t sector, bool unlocked) {
  // Never erase the bootstub (sector 0) or provisioning (sector 7). Sector 6
  // is reserved for the wake journal and requires its own address checks.
  if (flash_sector_erase_allowed(sector) && unlocked) {
    FLASH->CR1 = (sector << 8) | FLASH_CR_SER;
    FLASH->CR1 |= FLASH_CR_START;
    while (FLASH->SR1 & FLASH_SR_QW);
    return true;
  }
  return false;
}

void flash_write_word(void *prog_ptr, uint32_t data) {
  uint32_t *pp = prog_ptr;
  FLASH->CR1 |= FLASH_CR_PG;
  *pp = data;
  while (FLASH->SR1 & FLASH_SR_QW);
}

#define FLASH_PROGRAM_ERROR_MASK (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | \
                                  FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR | \
                                  FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR)
#define FLASH_PROGRAM_CLEAR_MASK (FLASH_CCR_CLR_EOP | FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_PGSERR | \
                                  FLASH_CCR_CLR_STRBERR | FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_OPERR | \
                                  FLASH_CCR_CLR_RDPERR | FLASH_CCR_CLR_RDSERR | \
                                  FLASH_CCR_CLR_SNECCERR | FLASH_CCR_CLR_DBECCERR)

static void flash_clear_program_status(void) {
  FLASH->CCR1 = FLASH_PROGRAM_CLEAR_MASK;
}

// STM32H7 programs flash as one 256-bit flashword. All eight stores must be
// issued under a single PG window; forcing a partially filled write buffer can
// leave ECC that is not valid for the next boot.
bool flash_write_flashword(void *prog_ptr, const uint32_t *data) {
  if (((uintptr_t)prog_ptr % (8U * sizeof(uint32_t))) != 0U) {
    return false;
  }

  while (FLASH->SR1 & FLASH_SR_QW) { }
  flash_clear_program_status();
  FLASH->CR1 |= FLASH_CR_PG;
  volatile uint32_t *destination = (volatile uint32_t *)prog_ptr;
  for (uint8_t i = 0U; i < 8U; i++) {
    destination[i] = data[i];
  }
  __DSB();
  while (FLASH->SR1 & FLASH_SR_QW) { }
  register_clear_bits(&(FLASH->CR1), FLASH_CR_PG);
  const bool success = (FLASH->SR1 & FLASH_PROGRAM_ERROR_MASK) == 0U;
  flash_clear_program_status();
  return success;
}

void flush_write_buffer(void) {
  if (FLASH->SR1 & FLASH_SR_WBNE) {
    FLASH->CR1 |= FLASH_CR_FW;
    while (FLASH->SR1 & FLASH_CR_FW);
  }
}
