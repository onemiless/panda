#include "board/wake_protocol.h"

extern int _app_start[0xc000]; // Only first 3 sectors of size 0x4000 are used

// Prototypes
void set_safety_mode(uint16_t mode, uint16_t param);
bool is_car_safety_mode(uint16_t mode);

static uint32_t wake_monitor_request_transaction(const ControlPacket_t *req) {
  return ((uint32_t)req->param2 << 16U) | req->param1;
}

static void wake_monitor_reset_runtime(void) {
  wake_monitor_observer_enabled = false;
  offline_wake_raw_can_exti_disarm();
  enable_can_transceivers(true);
  wake_monitor_can_activity_pending = false;
  wake_monitor_can_wake_requested = false;
  wake_monitor_can_dispatch_pending = false;
  wake_monitor_can_dispatch_stage = 0U;
  wake_monitor_som_off_seen = false;
  wake_monitor_som_off_ready = false;
  wake_monitor_som_off_countdown = 0U;
  wake_monitor_som_off_low_seconds = 0U;
  wake_monitor_can_armed = false;
  wake_monitor_strict_stop_pending = false;
  wake_monitor_reset_requested = false;
  wake_monitor_harness_requested = false;
  wake_monitor_failure_cooldown = 0U;
  wake_monitor_prepare_dirty = false;
  wake_monitor_panda_fault_pending = false;
  wake_monitor_prepared_host_session = 0U;
  wake_monitor_prepare_rx_overflow = 0U;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    wake_monitor_prepare_rx[i] = 0U;
    wake_monitor_prepare_rx_lost[i] = 0U;
    wake_monitor_prepare_can_resets[i] = 0U;
  }
  wake_monitor_status.reserved = 0U;
  bootkick_cancel_wake_pulse();
  bootkick_clear_wake_confirmation();
  current_board->set_bootkick(BOOT_STANDBY);
}

static bool wake_monitor_can_health_ready(void) {
  bool ready = (faults == 0U) && !power_save_enabled && wake_monitor_tres_can_io_ready();
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    ready &= (can_health[i].bus_off == 0U) && (can_health[i].error_passive == 0U) &&
             llcan_rx_ready(CANIF_FROM_CAN_NUM(i));
  }
  return ready;
}

static bool wake_monitor_offline_source_ready(void) {
  return wake_monitor_can_health_ready();
}

static void wake_monitor_capture_prepare_snapshot(void) {
  wake_monitor_prepare_rx_overflow = rx_buffer_overflow;
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    wake_monitor_prepare_rx[i] = can_health[i].total_rx_cnt;
    wake_monitor_prepare_rx_lost[i] = can_health[i].total_rx_lost_cnt;
    wake_monitor_prepare_can_resets[i] = can_health[i].can_core_reset_cnt;
  }
}

static bool wake_monitor_prepare_snapshot_clean(void) {
  uint32_t current_rx[PANDA_CAN_CNT];
  uint32_t current_rx_lost[PANDA_CAN_CNT];
  uint32_t current_can_resets[PANDA_CAN_CNT];
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    current_rx[i] = can_health[i].total_rx_cnt;
    current_rx_lost[i] = can_health[i].total_rx_lost_cnt;
    current_can_resets[i] = can_health[i].can_core_reset_cnt;
  }
  return wake_monitor_rx_snapshot_clean(wake_monitor_prepare_rx, current_rx,
                                        wake_monitor_prepare_rx_lost, current_rx_lost,
                                        wake_monitor_prepare_can_resets, current_can_resets,
                                        wake_monitor_prepare_rx_overflow, rx_buffer_overflow,
                                        PANDA_CAN_CNT);
}

static bool wake_monitor_prepare_integrity_clean(void) {
  uint32_t current_rx_lost[PANDA_CAN_CNT];
  uint32_t current_can_resets[PANDA_CAN_CNT];
  for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
    current_rx_lost[i] = can_health[i].total_rx_lost_cnt;
    current_can_resets[i] = can_health[i].can_core_reset_cnt;
  }
  return wake_monitor_rx_integrity_clean(wake_monitor_prepare_rx_lost, current_rx_lost,
                                         wake_monitor_prepare_can_resets, current_can_resets,
                                         wake_monitor_prepare_rx_overflow, rx_buffer_overflow,
                                         PANDA_CAN_CNT);
}

static void wake_monitor_prepare(uint32_t transaction, bool committed) {
  wake_monitor_reset_runtime();
  wake_debug_active_can_reset();
  wake_monitor_enabled = true;
  wake_monitor_committed = committed;
  wake_monitor_status.magic = WAKE_MONITOR_STATUS_MAGIC;
  wake_monitor_status.transaction = transaction;
  wake_monitor_status.committed_host_session = committed ? wake_monitor_status.host_session : 0U;
  wake_monitor_status.state = committed ? WAKE_MONITOR_STATE_COMMITTED : WAKE_MONITOR_STATE_PREPARED;
  wake_monitor_status.result = WAKE_MONITOR_RESULT_NONE;
  wake_monitor_status.trigger_stage = 0U;
  wake_can_trace_reset();
  wake_journal_begin_cycle();
  wake_debug_clear_success();
  // PREPARE only enables and verifies RX. It must not reinitialize FDCAN or
  // change safety mode while the host is still checking the transaction.
  set_power_save_state(false);
  enable_can_transceivers(true);
  wake_monitor_prepared_host_session = wake_monitor_status.host_session;
  wake_monitor_capture_prepare_snapshot();
  wake_monitor_can_armed = committed;
  wake_monitor_status.reserved = wake_monitor_prepare_flags(wake_monitor_can_health_ready(),
                                                            wake_monitor_prepared_host_session);
  if (committed) {
    wake_monitor_committed = true;
    set_safety_mode(SAFETY_SILENT, 0U);
    if (!wake_monitor_can_health_ready() || !wake_monitor_prepare_snapshot_clean()) {
      wake_monitor_committed = false;
      wake_monitor_can_armed = false;
      wake_monitor_prepare_dirty = true;
      wake_monitor_status.state = WAKE_MONITOR_STATE_PREPARED;
      wake_monitor_status.reserved = WAKE_MONITOR_STATUS_FLAG_PREPARE_DIRTY;
    } else {
      offline_wake_active_can_exti_arm();
      offline_wake_active_can_diag_snapshot(false);
    }
  }
  #ifdef ALLOW_DEBUG
  stop_mode_requested = false;
  #endif
  wake_debug_stage(PANDA_WAKE_MONITOR_ARMED_STAGE);
}

static int get_health_pkt(void *dat) {
  COMPILE_TIME_ASSERT(sizeof(struct health_t) <= USBPACKET_MAX_SIZE);
  struct health_t * health = (struct health_t*)dat;

  health->uptime_pkt = uptime_cnt;
  health->voltage_pkt = current_board->read_voltage_mV();
  health->current_pkt = current_board->read_current_mA();

  health->ignition_line_pkt = (uint8_t)(harness_check_ignition());
  health->ignition_can_pkt = ignition_can;

  health->controls_allowed_pkt = controls_allowed;
  health->safety_tx_blocked_pkt = safety_tx_blocked;
  health->safety_rx_invalid_pkt = safety_rx_invalid;
  health->tx_buffer_overflow_pkt = tx_buffer_overflow;
  health->rx_buffer_overflow_pkt = rx_buffer_overflow;
  health->car_harness_status_pkt = harness.status;
  health->safety_mode_pkt = (uint8_t)(current_safety_mode);
  health->safety_param_pkt = current_safety_param;
  health->alternative_experience_pkt = alternative_experience;
  health->power_save_enabled_pkt = power_save_enabled;
  health->heartbeat_lost_pkt = heartbeat_lost;
  health->safety_rx_checks_invalid_pkt = safety_rx_checks_invalid;

  health->spi_error_count_pkt = spi_error_count;

  health->fault_status_pkt = fault_status;
  health->faults_pkt = faults;

  health->interrupt_load_pkt = interrupt_load;

  health->fan_power = fan_state.power;

  health->sbu1_voltage_mV = harness.sbu1_voltage_mV;
  health->sbu2_voltage_mV = harness.sbu2_voltage_mV;

  health->som_reset_triggered = bootkick_reset_triggered;

  health->sound_output_level_pkt = sound_output_level;

  health->controls_allowed_lateral_pkt = controls_allowed || controls_allowed_lateral;
  health->controls_allowed_longitudinal_pkt = controls_allowed;

  return sizeof(*health);
}

// send on serial, first byte to select the ring
void comms_endpoint2_write(const uint8_t *data, uint32_t len) {
  uart_ring *ur = get_ring_by_number(data[0]);
  if ((len != 0U) && (ur != NULL)) {
    if ((data[0] < 2U) || (data[0] >= 4U)) {
      for (uint32_t i = 1; i < len; i++) {
        while (!put_char(ur, data[i])) {
          // wait
        }
      }
    }
  }
}

int comms_control_handler(ControlPacket_t *req, uint8_t *resp) {
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  uint32_t time;

#ifdef DEBUG_COMMS
  print("raw control request: "); hexdump(req, sizeof(ControlPacket_t)); print("\n");
  print("- request "); puth(req->request); print("\n");
  print("- param1 "); puth(req->param1); print("\n");
  print("- param2 "); puth(req->param2); print("\n");
#endif

  switch (req->request) {
    // **** 0xa8: get microsecond timer
    case 0xa8:
      time = microsecond_timer_get();
      resp[0] = (time & 0x000000FFU);
      resp[1] = ((time & 0x0000FF00U) >> 8U);
      resp[2] = ((time & 0x00FF0000U) >> 16U);
      resp[3] = ((time & 0xFF000000U) >> 24U);
      resp_len = 4U;
      break;
    // **** 0xb0: set IR power
    case 0xb0:
      current_board->set_ir_power(req->param1);
      break;
    // **** 0xb1: set fan power
    case 0xb1:
      fan_set_power(req->param1);
      break;
    // **** 0xb2: get fan rpm
    case 0xb2:
      resp[0] = (fan_state.rpm & 0x00FFU);
      resp[1] = ((fan_state.rpm & 0xFF00U) >> 8U);
      resp_len = 2;
      break;
    // **** 0xb5: legacy one-phase wake monitor arm
    case PANDA_REQUEST_ENABLE_WAKE_MONITOR:
      wake_monitor_prepare(UINT32_MAX, true);
      break;
    // **** 0xb7: non-triggering wake monitor preparation
    case PANDA_REQUEST_PREPARE_WAKE_MONITOR: {
      const uint32_t transaction = wake_monitor_request_transaction(req);
      const wake_monitor_prepare_action_t action = wake_monitor_prepare_action(
        wake_monitor_status.state, wake_monitor_status.transaction, transaction);
      if (action == WAKE_MONITOR_PREPARE_START) {
        wake_monitor_prepare(transaction, false);
      }
      break;
    }
    // **** 0xb8: final handoff after manager cleanup
    case PANDA_REQUEST_COMMIT_WAKE_MONITOR: {
      const uint32_t transaction = wake_monitor_request_transaction(req);
      if (wake_monitor_commit_allowed(wake_monitor_status.state, wake_monitor_status.transaction, transaction,
                                      wake_monitor_prepared_host_session, wake_monitor_status.host_session,
                                      wake_monitor_prepare_dirty,
                                      wake_monitor_can_health_ready() && wake_monitor_prepare_snapshot_clean())) {
        wake_monitor_committed = true;
        wake_monitor_can_armed = false;
        set_safety_mode(SAFETY_SILENT, 0U);
        if (wake_monitor_can_health_ready() && wake_monitor_prepare_snapshot_clean()) {
          wake_monitor_can_armed = true;
          wake_monitor_status.committed_host_session = wake_monitor_status.host_session;
          wake_monitor_status.state = WAKE_MONITOR_STATE_COMMITTED;
          wake_monitor_status.result = WAKE_MONITOR_RESULT_NONE;
          wake_monitor_status.reserved = wake_monitor_prepare_flags(true, wake_monitor_status.host_session);
          offline_wake_active_can_exti_arm();
          offline_wake_active_can_diag_snapshot(false);
          current_board->set_bootkick(BOOT_STANDBY);
          wake_debug_stage(PANDA_WAKE_MONITOR_ARMED_STAGE);
        } else {
          wake_monitor_committed = false;
          wake_monitor_prepare_dirty = true;
          wake_monitor_status.reserved = WAKE_MONITOR_STATUS_FLAG_PREPARE_DIRTY |
            (wake_monitor_can_health_ready() ? WAKE_MONITOR_STATUS_FLAG_CAN_HEALTHY : 0U);
        }
      } else {
        wake_monitor_status.reserved =
          (wake_monitor_prepare_dirty ? WAKE_MONITOR_STATUS_FLAG_PREPARE_DIRTY : 0U) |
          (wake_monitor_can_health_ready() ? WAKE_MONITOR_STATUS_FLAG_CAN_HEALTHY : 0U);
      }
      break;
    }
    // **** 0xb9: cancel one matching shutdown transaction
    case PANDA_REQUEST_ABORT_WAKE_MONITOR: {
      const uint32_t transaction = wake_monitor_request_transaction(req);
      if ((transaction != 0U) && (transaction == wake_monitor_status.transaction)) {
        wake_journal_abort_cycle();
        wake_monitor_reset_runtime();
        wake_monitor_enabled = false;
        wake_monitor_committed = false;
        wake_monitor_status.transaction = 0U;
        wake_monitor_status.committed_host_session = 0U;
        wake_monitor_status.state = WAKE_MONITOR_STATE_IDLE;
        wake_monitor_status.result = WAKE_MONITOR_RESULT_NONE;
        wake_monitor_status.trigger_stage = 0U;
      }
      break;
    }
    // **** 0xba: identify the current Linux boot session
    case PANDA_REQUEST_SET_HOST_SESSION:
      wake_monitor_status.host_session = wake_monitor_request_transaction(req);
      break;
    // **** 0xec: arm a receive-only PB12/PB5 observer without BOOTKICK
    case PANDA_REQUEST_ARM_WAKE_OBSERVER:
      wake_monitor_observer_enabled = false;
      offline_wake_raw_can_exti_disarm();
      set_power_save_state(false);
      enable_can_transceivers(true);
      wake_monitor_observer_enabled = true;
      offline_wake_active_can_exti_arm();
      offline_wake_active_can_diag_snapshot(true);
      break;
    // **** 0xed: stop observing without clearing the captured RTC evidence
    case PANDA_REQUEST_DISARM_WAKE_OBSERVER:
      wake_monitor_observer_enabled = false;
      offline_wake_raw_can_exti_disarm();
      break;
    // **** 0xb6: schedule bootkick test after N seconds
    case 0xb6:
      bootkick_debug_schedule(req->param1);
      break;
    // **** 0xc0: reset communications state
    case 0xc0:
      comms_can_reset();
      break;
    // **** 0xc1: get hardware type
    case 0xc1:
      resp[0] = hw_type;
      resp_len = 1;
      break;
    // **** 0xc2: CAN health stats
    case 0xc2:
      COMPILE_TIME_ASSERT(sizeof(can_health_t) <= USBPACKET_MAX_SIZE);
      if (req->param1 < 3U) {
        update_can_health_pkt(req->param1, 0U);
        can_health[req->param1].can_speed = (bus_config[req->param1].can_speed / 10U);
        can_health[req->param1].can_data_speed = (bus_config[req->param1].can_data_speed / 10U);
        can_health[req->param1].canfd_enabled = bus_config[req->param1].canfd_enabled;
        can_health[req->param1].brs_enabled = bus_config[req->param1].brs_enabled;
        can_health[req->param1].canfd_non_iso = bus_config[req->param1].canfd_non_iso;
        resp_len = sizeof(can_health[req->param1]);
        (void)memcpy(resp, (uint8_t*)(&can_health[req->param1]), resp_len);
      }
      break;
    // **** 0xc3: fetch MCU UID
    case 0xc3:
      (void)memcpy(resp, ((uint8_t *)UID_BASE), 12);
      resp_len = 12;
      break;
    // **** 0xc4: get interrupt call rate
    case 0xc4:
      if (req->param1 < NUM_INTERRUPTS) {
        uint32_t load = interrupts[req->param1].call_rate;
        resp[0] = (load & 0x000000FFU);
        resp[1] = ((load & 0x0000FF00U) >> 8U);
        resp[2] = ((load & 0x00FF0000U) >> 16U);
        resp[3] = ((load & 0xFF000000U) >> 24U);
        resp_len = 4U;
      }
      break;
    // **** 0xc5: DEBUG: drive relay
    case 0xc5:
      set_intercept_relay((req->param1 & 0x1U), (req->param1 & 0x2U));
      break;
    // **** 0xc6: DEBUG: read SOM GPIO
    case 0xc6:
      resp[0] = current_board->read_som_gpio();
      resp_len = 1;
      break;
    // **** 0xd0: fetch serial (aka the provisioned dongle ID)
    case 0xd0:
      // addresses are OTP
      if (req->param1 == 1U) {
        (void)memcpy(resp, (uint8_t *)DEVICE_SERIAL_NUMBER_ADDRESS, 0x10);
        resp_len = 0x10;
      } else {
        get_provision_chunk(resp);
        resp_len = PROVISION_CHUNK_LEN;
      }
      break;
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      switch (req->param1) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            print("-> entering bootloader\n");
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          #endif
          break;
        case 1:
          print("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          print("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd2: get health packet
    case 0xd2:
      resp_len = get_health_pkt(resp);
      break;
    // **** 0xd3: get first 64 bytes of signature
    case 0xd3:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len], resp_len);
      }
      break;
    // **** 0xd4: get second 64 bytes of signature
    case 0xd4:
      {
        resp_len = 64;
        char * code = (char*)_app_start;
        int code_len = _app_start[0];
        (void)memcpy(resp, &code[code_len + 64], resp_len);
      }
      break;
    // **** 0xd5: get wake debug packet
    case PANDA_REQUEST_GET_WAKE_DEBUG:
      COMPILE_TIME_ASSERT(sizeof(wake_debug_t) <= USBPACKET_MAX_SIZE);
      resp_len = sizeof(wake_debug);
      (void)memcpy(resp, (uint8_t*)(&wake_debug), resp_len);
      break;
    // **** 0xd7: clear latched offline wake success
    case PANDA_REQUEST_CLEAR_WAKE_SUCCESS:
      wake_debug_clear_success();
      break;
    // **** 0xd9: get latched offline wake success
    case PANDA_REQUEST_GET_WAKE_SUCCESS:
      COMPILE_TIME_ASSERT(sizeof(wake_success_t) <= USBPACKET_MAX_SIZE);
      resp_len = sizeof(wake_success);
      (void)memcpy(resp, (uint8_t*)(&wake_success), resp_len);
      break;
    // **** 0xda: get persistent CAN wake trace
    case PANDA_REQUEST_GET_WAKE_CAN_TRACE:
      COMPILE_TIME_ASSERT(sizeof(wake_can_trace_t) <= USBPACKET_MAX_SIZE);
      COMPILE_TIME_ASSERT((WAKE_DEBUG_WORDS + WAKE_SUCCESS_WORDS + WAKE_CAN_TRACE_WORDS) <= 32U);
      resp_len = sizeof(wake_can_trace);
      (void)memcpy(resp, (uint8_t*)(&wake_can_trace), resp_len);
      break;
    // **** 0xe9: get append-only wake journal state
    case PANDA_REQUEST_GET_WAKE_JOURNAL_INFO: {
      const wake_journal_info_t info = wake_journal_get_info();
      COMPILE_TIME_ASSERT(sizeof(wake_journal_info_t) <= USBPACKET_MAX_SIZE);
      resp_len = sizeof(info);
      (void)memcpy(resp, (const uint8_t *)&info, resp_len);
      break;
    }
    // **** 0xea: read one raw 32-byte wake journal slot
    case PANDA_REQUEST_GET_WAKE_JOURNAL_RECORD: {
      wake_journal_record_t record;
      COMPILE_TIME_ASSERT(sizeof(wake_journal_record_t) <= USBPACKET_MAX_SIZE);
      if (wake_journal_get_record(req->param1, &record)) {
        resp_len = sizeof(record);
        (void)memcpy(resp, (const uint8_t *)&record, resp_len);
      }
      break;
    }
    // **** 0xeb: read transaction/session state
    case PANDA_REQUEST_GET_WAKE_MONITOR_STATUS:
      COMPILE_TIME_ASSERT(sizeof(wake_monitor_status_t) <= USBPACKET_MAX_SIZE);
      resp_len = sizeof(wake_monitor_status);
      (void)memcpy(resp, (const uint8_t *)&wake_monitor_status, resp_len);
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= USBPACKET_MAX_SIZE);
      (void)memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion) - 1U;
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xdb: set OBD CAN multiplexing mode
    case 0xdb:
      current_board->set_can_mode((req->param1 == 1U) ? CAN_MODE_OBD_CAN2 : CAN_MODE_NORMAL);
      break;
    // **** 0xdc: set safety mode
    case 0xdc:
      set_safety_mode(req->param1, (uint16_t)req->param2);
      break;
    // **** 0xdd: get health and CAN packet versions
    case 0xdd: {
      uint32_t versions[2] = {HEALTH_PACKET_VERSION, CAN_PACKET_VERSION_HASH};
      (void)memcpy(resp, (uint8_t *)versions, sizeof(versions));
      resp_len = sizeof(versions);
      break;
    }
    // **** 0xde: set can bitrate
    case 0xde:
      if ((req->param1 < PANDA_CAN_CNT) && is_speed_valid(req->param2, speeds, sizeof(speeds)/sizeof(speeds[0]))) {
        bus_config[req->param1].can_speed = req->param2;
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    // **** 0xdf: set alternative experience
    case 0xdf:
      // you can only set this if you are in a non car safety mode
      if (!is_car_safety_mode(current_safety_mode)) {
        alternative_experience = req->param1;
        current_safety_param_sp = req->param2;
        mads_set_alternative_experience(&alternative_experience);
      }
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(req->param1);
      if (!ur) {
        break;
      }

      // read
      uint16_t req_length = MIN(req->length, USBPACKET_MAX_SIZE);
      while ((resp_len < req_length) &&
                         get_char(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xe5: set CAN loopback (for testing)
    case 0xe5:
      can_loopback = req->param1 > 0U;
      can_init_all();
      break;
    // **** 0xe6: set custom clock source period and pulse length
    case 0xe6:
      clock_source_set_timer_params(req->param1, req->param2);
      break;
    // **** 0xe7: set power save state
    case 0xe7:
      set_power_save_state(req->param1 != 0U);
      break;
    // **** 0xe8: set can-fd auto swithing mode
    case 0xe8:
      bus_config[req->param1].canfd_auto = req->param2 > 0U;
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (req->param1 == 0xFFFFU) {
        print("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (req->param1 < PANDA_CAN_CNT) {
        print("Clearing CAN Tx queue\n");
        can_clear(can_queues[req->param1]);
      } else {
        print("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf3: Heartbeat. Resets heartbeat counter.
    case 0xf3:
      {
        heartbeat_counter = 0U;
        heartbeat_lost = false;
        heartbeat_disabled = false;
        heartbeat_engaged = (req->param1 == 1U);
        heartbeat_engaged_mads = (req->param2 == 1U);
        break;
      }
    // **** 0xf6: set siren enabled
    case 0xf6:
      siren_enabled = (req->param1 != 0U);
      break;
    // **** 0xf8: disable heartbeat checks
    case 0xf8:
      if (!is_car_safety_mode(current_safety_mode)) {
        heartbeat_disabled = true;
      }
      break;
    // **** 0xf9: set CAN FD data bitrate
    case 0xf9:
      if ((req->param1 < PANDA_CAN_CNT) &&
           is_speed_valid(req->param2, data_speeds, sizeof(data_speeds)/sizeof(data_speeds[0]))) {
        bus_config[req->param1].can_data_speed = req->param2;
        bus_config[req->param1].canfd_enabled = (req->param2 >= bus_config[req->param1].can_speed);
        bus_config[req->param1].brs_enabled = (req->param2 > bus_config[req->param1].can_speed);
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    // **** 0xfc: set CAN FD non-ISO mode
    case 0xfc:
      if (req->param1 < PANDA_CAN_CNT) {
        bus_config[req->param1].canfd_non_iso = (req->param2 != 0U);
        bool ret = can_init(CAN_NUM_FROM_BUS_NUM(req->param1));
        UNUSED(ret);
      }
      break;
    default:
      print("NO HANDLER ");
      puth(req->request);
      print("\n");
      break;
  }
  return resp_len;
}
