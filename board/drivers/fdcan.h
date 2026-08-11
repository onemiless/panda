#include "board/drivers/drivers.h"
#include "board/drivers/offline_wake_source_policy.h"

FDCAN_GlobalTypeDef *cans[PANDA_CAN_CNT] = {FDCAN1, FDCAN2, FDCAN3};

#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
static bool wake_debug_gpio_is_alternate(GPIO_TypeDef *gpio, uint8_t pin, uint8_t alternate) {
  const uint32_t mode = (gpio->MODER >> (pin * 2U)) & 0x3U;
  const uint32_t af = (gpio->AFR[pin / 8U] >> ((pin % 8U) * 4U)) & 0xFU;
  return (mode == MODE_ALTERNATE) && (af == alternate);
}

static bool wake_debug_gpio_output_is_low(GPIO_TypeDef *gpio, uint8_t pin) {
  const uint32_t mode = (gpio->MODER >> (pin * 2U)) & 0x3U;
  return (mode == MODE_OUTPUT) && ((gpio->ODR & (1UL << pin)) == 0U);
}

static bool wake_monitor_tres_can_io_ready(void) {
  if (hw_type != HW_TYPE_TRES) {
    return true;
  }
  const bool flipped = harness.status == HARNESS_STATUS_FLIPPED;
  const bool oriented_rx = flipped ?
    wake_debug_gpio_is_alternate(GPIOB, 12U, GPIO_AF9_FDCAN2) :
    wake_debug_gpio_is_alternate(GPIOB, 5U, GPIO_AF9_FDCAN2);
  const bool oriented_transceiver = flipped ?
    wake_debug_gpio_output_is_low(GPIOB, 11U) :
    wake_debug_gpio_output_is_low(GPIOB, 10U);
  return wake_debug_gpio_is_alternate(GPIOB, 8U, GPIO_AF9_FDCAN1) &&
         wake_debug_gpio_is_alternate(GPIOG, 9U, GPIO_AF2_FDCAN3) &&
         oriented_rx && oriented_transceiver &&
         wake_debug_gpio_output_is_low(GPIOG, 11U) &&
         wake_debug_gpio_output_is_low(GPIOD, 7U);
}

static void wake_debug_active_can_arm_snapshot(void) {
  wake_debug_active_can_reset();
  // The GPIO/transceiver mapping below is specific to Tres. Other H7 boards
  // enter the existing STOP/EXTI path and must not expose a false snapshot.
  if (hw_type != HW_TYPE_TRES) {
    return;
  }

  wake_debug.pre_wfi_exti_pr1 = (FDCAN1->CCCR & 0xFFFFU) | ((FDCAN2->CCCR & 0xFFFFU) << 16U);
  wake_debug.post_wfi_exti_pr1 = 0U;
  wake_debug.exti_imr1 = FDCAN3->CCCR;
  wake_debug.exti_rtsr1 = FDCAN1->IE;
  wake_debug.exti_ftsr1 = FDCAN2->IE;
  wake_debug.exti_emr1 = FDCAN3->IE;

  uint32_t io = 0U;
  io |= (uint32_t)wake_debug_gpio_is_alternate(GPIOB, 5U, GPIO_AF9_FDCAN2) << 0U;
  io |= (uint32_t)wake_debug_gpio_is_alternate(GPIOB, 12U, GPIO_AF9_FDCAN2) << 1U;
  io |= (uint32_t)wake_debug_gpio_is_alternate(GPIOB, 8U, GPIO_AF9_FDCAN1) << 2U;
  io |= (uint32_t)wake_debug_gpio_is_alternate(GPIOG, 9U, GPIO_AF2_FDCAN3) << 3U;
  io |= (uint32_t)wake_debug_gpio_output_is_low(GPIOB, 10U) << 4U;
  io |= (uint32_t)wake_debug_gpio_output_is_low(GPIOB, 11U) << 5U;
  io |= (uint32_t)wake_debug_gpio_output_is_low(GPIOG, 11U) << 6U;
  io |= (uint32_t)wake_debug_gpio_output_is_low(GPIOD, 7U) << 7U;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    io |= (uint32_t)llcan_rx_ready(cans[i]) << (WAKE_ACTIVE_CAN_DIAG_RX_READY_SHIFT + i);
  }
  const IRQn_Type rx_irqs[PANDA_CAN_CNT] = {
    FDCAN1_IT0_IRQn,
    FDCAN2_IT0_IRQn,
    FDCAN3_IT0_IRQn,
  };
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    io |= (uint32_t)(NVIC_GetEnableIRQ(rx_irqs[i]) != 0U) <<
          (WAKE_ACTIVE_CAN_DIAG_RX_IRQ_ENABLED_SHIFT + i);
    io |= (uint32_t)((cans[i]->ILE & FDCAN_ILE_EINT0) != 0U) <<
          (WAKE_ACTIVE_CAN_DIAG_ILE_ENABLED_SHIFT + i);
    io |= (uint32_t)((cans[i]->ILS & FDCAN_ILS_RF0NL) == 0U) <<
          (WAKE_ACTIVE_CAN_DIAG_RX_FIFO0_IT0_SHIFT + i);
  }
  io |= (uint32_t)can_silent << WAKE_ACTIVE_CAN_DIAG_SAFETY_SILENT_SHIFT;
  wake_debug.enter_count = wake_active_can_diag_make(io);
  wake_debug_active_can_save_arm_snapshot();
}
#endif

static bool can_set_speed(uint8_t can_number) {
  bool ret = true;
  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  ret &= llcan_set_speed(
    FDCANx,
    bus_config[bus_number].can_speed,
    bus_config[bus_number].can_data_speed,
    bus_config[bus_number].canfd_non_iso,
    can_loopback,
    can_silent
  );
  return ret;
}

void can_clear_send(FDCAN_GlobalTypeDef *FDCANx, uint8_t can_number) {
  static uint32_t last_reset = 0U;
  uint32_t time = microsecond_timer_get();

  // Resetting CAN core is a slow blocking operation, limit frequency
  if (get_ts_elapsed(time, last_reset) > 100000U) {  // 10 Hz
    can_health[can_number].can_core_reset_cnt += 1U;
    can_health[can_number].total_tx_lost_cnt += (FDCAN_TX_FIFO_EL_CNT - (FDCANx->TXFQS & FDCAN_TXFQS_TFFL)); // TX FIFO msgs will be lost after reset
    llcan_clear_send(FDCANx);
    last_reset = time;
  }
}

void update_can_health_pkt(uint8_t can_number, uint32_t ir_reg) {
  uint8_t can_irq_number[PANDA_CAN_CNT][2] = {
    { FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn },
    { FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn },
    { FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn },
  };

  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint32_t psr_reg = FDCANx->PSR;
  uint32_t ecr_reg = FDCANx->ECR;

  can_health[can_number].bus_off = ((psr_reg & FDCAN_PSR_BO) >> FDCAN_PSR_BO_Pos);
  can_health[can_number].bus_off_cnt += can_health[can_number].bus_off;
  can_health[can_number].error_warning = ((psr_reg & FDCAN_PSR_EW) >> FDCAN_PSR_EW_Pos);
  can_health[can_number].error_passive = ((psr_reg & FDCAN_PSR_EP) >> FDCAN_PSR_EP_Pos);

  can_health[can_number].last_error = ((psr_reg & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos);
  if ((can_health[can_number].last_error != 0U) && (can_health[can_number].last_error != 7U)) {
    can_health[can_number].last_stored_error = can_health[can_number].last_error;
  }

  can_health[can_number].last_data_error = ((psr_reg & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos);
  if ((can_health[can_number].last_data_error != 0U) && (can_health[can_number].last_data_error != 7U)) {
    can_health[can_number].last_data_stored_error = can_health[can_number].last_data_error;
  }

  can_health[can_number].receive_error_cnt = ((ecr_reg & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
  can_health[can_number].transmit_error_cnt = ((ecr_reg & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);

  can_health[can_number].irq0_call_rate = interrupts[can_irq_number[can_number][0]].call_rate;
  can_health[can_number].irq1_call_rate = interrupts[can_irq_number[can_number][1]].call_rate;

  if (ir_reg != 0U) {
    // Clear error interrupts
    FDCANx->IR |= (FDCAN_IR_PED | FDCAN_IR_PEA | FDCAN_IR_EP | FDCAN_IR_BO | FDCAN_IR_RF0L);
    can_health[can_number].total_error_cnt += 1U;
    // Check for RX FIFO overflow
    if ((ir_reg & (FDCAN_IR_RF0L)) != 0U) {
      can_health[can_number].total_rx_lost_cnt += 1U;
    }
    // Cases:
    // 1. while multiplexing between buses 1 and 3 we are getting ACK errors that overwhelm CAN core, by resetting it recovers faster
    // 2. H7 gets stuck in bus off recovery state indefinitely
    if ((((can_health[can_number].last_error == CAN_ACK_ERROR) || (can_health[can_number].last_data_error == CAN_ACK_ERROR)) && (can_health[can_number].transmit_error_cnt > 127U)) ||
     ((ir_reg & FDCAN_IR_BO) != 0U)) {
      can_clear_send(FDCANx, can_number);
    }
  }
}

// ***************************** CAN *****************************
// FDFDCANx_IT1 IRQ Handler (TX)
void process_can(uint8_t can_number) {
  if (can_number != 0xffU) {
    ENTER_CRITICAL();

    FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    FDCANx->IR |= FDCAN_IR_TFE; // Clear Tx FIFO Empty flag

    if ((FDCANx->TXFQS & FDCAN_TXFQS_TFQF) == 0U) {
      CANPacket_t to_send;
      if (can_pop(can_queues[bus_number], &to_send)) {
        if (can_check_checksum(&to_send)) {
          can_health[can_number].total_tx_cnt += 1U;

          uint32_t TxFIFOSA = FDCAN_START_ADDRESS + (can_number * FDCAN_OFFSET) + (FDCAN_RX_FIFO_0_EL_CNT * FDCAN_RX_FIFO_0_EL_SIZE);
          // get the index of the next TX FIFO element (0 to FDCAN_TX_FIFO_EL_CNT - 1)
          uint32_t tx_index = (FDCANx->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 0x1FU;
          // only send if we have received a packet
          canfd_fifo *fifo;
          fifo = (canfd_fifo *)(TxFIFOSA + (tx_index * FDCAN_TX_FIFO_EL_SIZE));

          fifo->header[0] = (to_send.extended << 30) | ((to_send.extended != 0U) ? (to_send.addr) : (to_send.addr << 18));

          // If canfd_auto is set, outgoing packets will be automatically sent as CAN-FD if an incoming CAN-FD packet was seen
          bool fd = bus_config[can_number].canfd_auto ? bus_config[can_number].canfd_enabled : (bool)(to_send.fd > 0U);
          uint32_t canfd_enabled_header = fd ? (1UL << 21) : 0UL;

          uint32_t brs_enabled_header = bus_config[can_number].brs_enabled ? (1UL << 20) : 0UL;
          fifo->header[1] = (to_send.data_len_code << 16) | canfd_enabled_header | brs_enabled_header;

          uint8_t data_len_w = (dlc_to_len[to_send.data_len_code] / 4U);
          data_len_w += ((dlc_to_len[to_send.data_len_code] % 4U) > 0U) ? 1U : 0U;
          for (unsigned int i = 0; i < data_len_w; i++) {
            BYTE_ARRAY_TO_WORD(fifo->data_word[i], &to_send.data[i*4U]);
          }

          FDCANx->TXBAR = (1UL << tx_index);

          // Send back to USB
          CANPacket_t to_push;

          to_push.fd = fd;
          to_push.returned = 1U;
          to_push.rejected = 0U;
          to_push.extended = to_send.extended;
          to_push.addr = to_send.addr;
          to_push.bus = bus_number;
          to_push.data_len_code = to_send.data_len_code;
          (void)memcpy(to_push.data, to_send.data, dlc_to_len[to_push.data_len_code]);
          can_set_checksum(&to_push);

          rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;
        } else {
          can_health[can_number].total_tx_checksum_error_cnt += 1U;
        }

        refresh_can_tx_slots_available();
      }
    }
    EXIT_CRITICAL();
  }
}

// FDFDCANx_IT0 IRQ Handler (RX and errors)
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  uint32_t ir_reg = FDCANx->IR;

  // Clear all new messages from Rx FIFO 0
  FDCANx->IR |= FDCAN_IR_RF0N;
  while ((FDCANx->RXF0S & FDCAN_RXF0S_F0FL) != 0U) {
    can_health[can_number].total_rx_cnt += 1U;
    // get the index of the next RX FIFO element (0 to FDCAN_RX_FIFO_0_EL_CNT - 1)
    uint32_t rx_fifo_idx = (uint8_t)((FDCANx->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3FU);

    // Recommended to offset get index by at least +1 if RX FIFO is in overwrite mode and full (datasheet)
    if ((FDCANx->RXF0S & FDCAN_RXF0S_F0F) == FDCAN_RXF0S_F0F) {
      rx_fifo_idx = ((rx_fifo_idx + 1U) >= FDCAN_RX_FIFO_0_EL_CNT) ? 0U : (rx_fifo_idx + 1U);
      can_health[can_number].total_rx_lost_cnt += 1U; // At least one message was lost
    }

    uint32_t RxFIFO0SA = FDCAN_START_ADDRESS + (can_number * FDCAN_OFFSET);
    CANPacket_t to_push;
    const canfd_fifo *fifo;

    // getting address
    fifo = (const canfd_fifo *)(RxFIFO0SA + (rx_fifo_idx * FDCAN_RX_FIFO_0_EL_SIZE));

    bool canfd_frame = ((fifo->header[1] >> 21) & 0x1U);
    bool brs_frame = ((fifo->header[1] >> 20) & 0x1U);

    to_push.fd = canfd_frame;
    to_push.returned = 0U;
    to_push.rejected = 0U;
    to_push.extended = (fifo->header[0] >> 30) & 0x1U;
    to_push.addr = ((to_push.extended != 0U) ? (fifo->header[0] & 0x1FFFFFFFU) : ((fifo->header[0] >> 18) & 0x7FFU));
    to_push.bus = bus_number;
    to_push.data_len_code = ((fifo->header[1] >> 16) & 0xFU);

    uint8_t data_len_w = (dlc_to_len[to_push.data_len_code] / 4U);
    data_len_w += ((dlc_to_len[to_push.data_len_code] % 4U) > 0U) ? 1U : 0U;
    for (unsigned int i = 0; i < data_len_w; i++) {
      WORD_TO_BYTE_ARRAY(&to_push.data[i*4U], fifo->data_word[i]);
    }
    can_set_checksum(&to_push);

    // forwarding (panda only)
    int bus_fwd_num = safety_fwd_hook(bus_number, to_push.addr);
    if (bus_fwd_num < 0) {
      bus_fwd_num = bus_config[can_number].forwarding_bus;
    }
    if (bus_fwd_num != -1) {
      CANPacket_t to_send;

      to_send.fd = to_push.fd;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = to_push.extended;
      to_send.addr = to_push.addr;
      to_send.bus = to_push.bus;
      to_send.data_len_code = to_push.data_len_code;
      (void)memcpy(to_send.data, to_push.data, dlc_to_len[to_push.data_len_code]);
      can_set_checksum(&to_send);

      can_send(&to_send, bus_fwd_num, true);
      can_health[can_number].total_fwd_cnt += 1U;
    }

    #ifdef PANDA_BODY
    body_can_rx(&to_push);
    #endif

    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
    ignition_can_hook(&to_push);

    #if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
    if (wake_monitor_enabled && !wake_monitor_committed &&
        (wake_monitor_status.state == WAKE_MONITOR_STATE_PREPARED)) {
      // Any vehicle traffic after PREPARE invalidates the host's quiet
      // snapshot. COMMIT must refuse this transaction instead of silently
      // arming from a stale 300-second gate.
      wake_monitor_prepare_dirty = true;
      wake_monitor_status.reserved |= WAKE_MONITOR_STATUS_FLAG_PREPARE_DIRTY;
    }

    // After a clean COMMIT, any hardware-validated frame from a physical CAN
    // controller is sufficient. Latch the wake event before any diagnostic
    // RTC write so diagnostics cannot delay or suppress the wake path.
    const bool physical_wake_ready = offline_wake_physical_bus_rx_ready(
          wake_monitor_enabled, wake_monitor_committed, wake_monitor_can_armed,
          wake_monitor_can_wake_requested, can_number);
    const bool first_activity = physical_wake_ready && !wake_monitor_can_activity_pending;
    if (physical_wake_ready) {
      wake_monitor_can_activity_pending = true;
    }

    const bool production_wake_diag = wake_monitor_enabled && wake_monitor_committed &&
                                      wake_monitor_can_armed && (can_number < PANDA_CAN_CNT);
    if (production_wake_diag) {
      // Continue sampling after the first wake request so a multi-second wake
      // cluster produces a full one-second peak instead of a partial window.
      wake_can_trace_record_rx(can_number);
    }
    if (production_wake_diag || wake_monitor_observer_enabled) {
      wake_debug_active_can_first_rx(can_number, to_push.addr);
    }
    if (first_activity) {
      wake_journal_queue_event(WAKE_JOURNAL_SOURCE_CAN_PRIMARY, 0x35U, to_push.bus, can_number,
                               GET_LEN(&to_push), to_push.addr, to_push.data);
    }
    #endif

    bool queue_for_host = true;
    #if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
    queue_for_host = !wake_monitor_enabled || !wake_monitor_committed;
    #endif
    if (queue_for_host) {
      led_set(LED_BLUE, true);
      rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;
    }

    // Enable CAN FD and BRS if CAN FD message was received
    if (!(bus_config[can_number].canfd_enabled) && (canfd_frame)) {
      bus_config[can_number].canfd_enabled = true;
    }
    if (!(bus_config[can_number].brs_enabled) && (brs_frame)) {
      bus_config[can_number].brs_enabled = true;
    }

    // update read index
    FDCANx->RXF0A = rx_fifo_idx;
  }

  // Error handling
  if ((ir_reg & (FDCAN_IR_PED | FDCAN_IR_PEA | FDCAN_IR_EP | FDCAN_IR_BO | FDCAN_IR_RF0L)) != 0U) {
    update_can_health_pkt(can_number, ir_reg);
  }
}

static void FDCAN1_IT0_IRQ_Handler(void) {
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  const bool wake_diag_active = (wake_monitor_enabled && wake_monitor_committed && wake_monitor_can_armed) ||
                                wake_monitor_observer_enabled;
  if (wake_diag_active) {
    wake_debug_active_can_irq_entry(0U);
  }
#endif
  can_rx(0U);
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  if (wake_diag_active) {
    wake_debug_active_can_irq_flush(0U);
  }
#endif
}
static void FDCAN1_IT1_IRQ_Handler(void) { process_can(0); }

static void FDCAN2_IT0_IRQ_Handler(void) {
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  const bool wake_diag_active = (wake_monitor_enabled && wake_monitor_committed && wake_monitor_can_armed) ||
                                wake_monitor_observer_enabled;
  if (wake_diag_active) {
    wake_debug_active_can_irq_entry(1U);
  }
#endif
  can_rx(1U);
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  if (wake_diag_active) {
    wake_debug_active_can_irq_flush(1U);
  }
#endif
}
static void FDCAN2_IT1_IRQ_Handler(void) { process_can(1); }

static void FDCAN3_IT0_IRQ_Handler(void) {
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  const bool wake_diag_active = (wake_monitor_enabled && wake_monitor_committed && wake_monitor_can_armed) ||
                                wake_monitor_observer_enabled;
  if (wake_diag_active) {
    wake_debug_active_can_irq_entry(2U);
  }
#endif
  can_rx(2U);
#if !defined(PANDA_BODY) && !defined(PANDA_JUNGLE)
  if (wake_diag_active) {
    wake_debug_active_can_irq_flush(2U);
  }
#endif
}
static void FDCAN3_IT1_IRQ_Handler(void) { process_can(2); }

bool can_init(uint8_t can_number) {
  bool ret = false;

  REGISTER_INTERRUPT(FDCAN1_IT0_IRQn, FDCAN1_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(FDCAN1_IT1_IRQn, FDCAN1_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(FDCAN2_IT0_IRQn, FDCAN2_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(FDCAN2_IT1_IRQn, FDCAN2_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(FDCAN3_IT0_IRQn, FDCAN3_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(FDCAN3_IT1_IRQn, FDCAN3_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  if (can_number != 0xffU) {
    FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
    ret &= can_set_speed(can_number);
    ret &= llcan_init(FDCANx);
    // in case there are queued up messages
    process_can(can_number);
  }
  return ret;
}
