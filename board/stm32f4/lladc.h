#include "lladc_declarations.h"

void register_set(volatile uint32_t *addr, uint32_t val, uint32_t mask);

void adc_init(ADC_TypeDef *adc) {
  register_set(&(ADC->CCR), ADC_CCR_TSVREFE | ADC_CCR_VBATE, 0xC30000U);
  register_set(&(adc->CR2), ADC_CR2_ADON, 0xFF7F0F03U);
  register_set(&(adc->SMPR1), ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13, 0x7FFFFFFU);
}

static uint16_t adc_get_raw(const adc_signal_t *signal) {
  // Preserve the proven DOS/F4 conversion sequence. Dynamic SMPR rewrites on
  // this board produced unstable voltage readings after adjacent SBU samples.
  ENTER_CRITICAL();
  register_set(&(signal->adc->JSQR), ((uint32_t) signal->channel << 15U), 0x3FFFFFU);

  // start conversion
  signal->adc->SR &= ~(ADC_SR_JEOC);
  signal->adc->CR2 |= ADC_CR2_JSWSTART;
  while (!(signal->adc->SR & ADC_SR_JEOC));

  uint16_t result = signal->adc->JDR1;
  EXIT_CRITICAL();
  return result;
}

uint16_t adc_get_mV(const adc_signal_t *signal) {
  return (adc_get_raw(signal) * current_board->avdd_mV) / 4095U;
}
