// ********************* Includes *********************
#include "board/config.h"

#include "board/drivers/led.h"
#include "board/drivers/pwm.h"
#include "board/drivers/usb.h"
#include "board/drivers/simple_watchdog.h"
#include "board/drivers/bootkick.h"

#include "board/early_init.h"
#include "board/provision.h"

#include "opendbc/safety/safety.h"

#include "board/health.h"

#include "board/drivers/can_common.h"

#include "board/drivers/fdcan.h"

#include "board/sys/power_saving.h"

#include "board/obj/gitversion.h"

#include "board/can_comms.h"
#include "board/main_comms.h"


// ********************* Serial debugging *********************

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (get_char(ring, &rcv)) {
    (void)put_char(ring, rcv);  // misra-c2012-17.7: cast to void is ok: debug function
  }
}

// ****************************** safety mode ******************************

// this is the only way to leave silent mode
void set_safety_mode(uint16_t mode, uint16_t param) {
  uint16_t mode_copy = mode;
  int err = set_safety_hooks(mode_copy, param);
  if (err == -1) {
    print("Error: safety set mode failed. Falling back to SILENT\n");
    mode_copy = SAFETY_SILENT;
    err = set_safety_hooks(mode_copy, 0U);
    // TERMINAL ERROR: we can't continue if SILENT safety mode isn't succesfully set
    assert_fatal(err == 0, "Error: Failed setting SILENT mode. Hanging\n");
  }
  safety_tx_blocked = 0;
  safety_rx_invalid = 0;

  switch (mode_copy) {
    case SAFETY_SILENT:
      set_intercept_relay(false, false);
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = true;
      break;
    case SAFETY_NOOUTPUT:
      set_intercept_relay(false, false);
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = false;
      break;
    case SAFETY_ELM327:
      set_intercept_relay(false, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;

      // Clear any pending messages in the can core (i.e. sending while comma power is unplugged)
      // TODO: rewrite using hardware queues rather than fifo to cancel specific messages
      can_clear_send(CANIF_FROM_CAN_NUM(1), 1);
      if (param == 0U) {
        current_board->set_can_mode(CAN_MODE_OBD_CAN2);
      } else {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = false;
      break;
    default:
      set_intercept_relay(true, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = false;
      break;
  }
  can_init_all();
}

bool is_car_safety_mode(uint16_t mode) {
  return (mode != SAFETY_SILENT) &&
         (mode != SAFETY_NOOUTPUT) &&
         (mode != SAFETY_ALLOUTPUT) &&
         (mode != SAFETY_ELM327);
}

// ***************************** main code *****************************

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
// cppcheck-suppress misra-c2012-8.4
void __initialize_hardware_early(void) {
  early_initialization();
}

static void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

// go into SILENT when heartbeat isn't received for this amount of seconds.
#define HEARTBEAT_IGNITION_CNT_ON 5U
#define HEARTBEAT_IGNITION_CNT_OFF 2U
#define WAKE_MONITOR_SOM_OFF_SETTLE_S 10U
#define WAKE_MONITOR_CAN_LED_HOLD_S 5U

// called at 8Hz
static void tick_handler(void) {
  static uint32_t siren_countdown = 0; // siren plays while countdown > 0
  static uint32_t controls_allowed_countdown = 0;
  static uint8_t prev_harness_status = HARNESS_STATUS_NC;
  static uint8_t loop_counter = 0U;
  static bool relay_malfunction_prev = false;
  static bool wake_monitor_reset_requested = false;
  static bool wake_monitor_harness_requested = false;
  static uint32_t wake_monitor_prev_rx[PANDA_CAN_CNT] = {0U, 0U, 0U};
  static uint32_t wake_monitor_can_baseline[PANDA_CAN_CNT] = {0U, 0U, 0U};
  static uint8_t wake_monitor_can_led_countdown = 0U;
  static uint8_t wake_monitor_led_phase = 0U;
  static uint16_t wake_monitor_off_seconds = 0U;

  if (TICK_TIMER->SR != 0U) {

    // siren
    current_board->set_siren((loop_counter & 1U) && (siren_enabled || (siren_countdown > 0U)));

    // tick drivers at 8Hz
    fan_tick();
    harness_tick();
    simple_watchdog_kick();
    sound_tick();

    if (relay_malfunction_prev != relay_malfunction) {
      if (relay_malfunction) {
        fault_occurred(FAULT_RELAY_MALFUNCTION);
      } else {
        fault_recovered(FAULT_RELAY_MALFUNCTION);
      }
    }
    relay_malfunction_prev = relay_malfunction;

    // re-init everything that uses harness status
    if (harness.status != prev_harness_status) {
      const uint8_t old_harness_status = prev_harness_status;
      prev_harness_status = harness.status;
      can_set_orientation(harness.status == HARNESS_STATUS_FLIPPED);

      // re-init everything that uses harness status
      can_init_all();
      set_safety_mode(current_safety_mode, current_safety_param);
      set_power_save_state(power_save_enabled);

      if (wake_monitor_enabled && wake_monitor_som_off_ready &&
          (old_harness_status == HARNESS_STATUS_NC) &&
          (harness.status != HARNESS_STATUS_NC) && !wake_monitor_harness_requested) {
        if (bootkick_request_wake_pulse(0x37U)) {
          wake_monitor_harness_requested = true;
        }
      }
    }

    // decimated to 1Hz
    if (loop_counter == 0U) {
      //puth(usart1_dma); print(" "); puth(DMA2_Stream5->M0AR); print(" "); puth(DMA2_Stream5->NDTR); print("\n");
      #ifdef DEBUG
        print("** blink ");
        print("rx:"); puth4(can_rx_q.r_ptr); print("-"); puth4(can_rx_q.w_ptr); print("  ");
        print("tx1:"); puth4(can_tx1_q.r_ptr); print("-"); puth4(can_tx1_q.w_ptr); print("  ");
        print("tx2:"); puth4(can_tx2_q.r_ptr); print("-"); puth4(can_tx2_q.w_ptr); print("  ");
        print("tx3:"); puth4(can_tx3_q.r_ptr); print("-"); puth4(can_tx3_q.w_ptr); print("\n");
      #endif

      // set green LED to be controls allowed
      led_set(LED_GREEN, controls_allowed);

      // turn off the blue LED, turned on by CAN
      // unless we are in power saving mode
      led_set(LED_BLUE, (uptime_cnt & 1U) && power_save_enabled);

      const bool recent_heartbeat = heartbeat_counter == 0U;

      uint32_t rx_per_bus[PANDA_CAN_CNT] = {0U, 0U, 0U};
      for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
        rx_per_bus[i] = can_health[i].total_rx_cnt - wake_monitor_prev_rx[i];
        wake_monitor_prev_rx[i] = can_health[i].total_rx_cnt;
      }
      if (wake_monitor_enabled && recent_heartbeat && !wake_monitor_som_off_seen) {
        for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
          wake_monitor_can_baseline[i] = rx_per_bus[i];
        }
      }
      if (wake_monitor_enabled) {
        if (!recent_heartbeat) {
          if (!wake_monitor_som_off_seen) {
            wake_monitor_off_seconds = 0U;
            wake_monitor_som_off_seen = true;
            wake_monitor_can_activity_pending = false;
            wake_monitor_can_led_countdown = 0U;
            const offline_wake_heartbeat_loss_policy policy =
              offline_wake_policy_after_heartbeat_loss(hw_type == HW_TYPE_TRES);
            if (policy.keep_can_active) {
              wake_monitor_som_off_ready = false;
              wake_monitor_som_off_countdown = WAKE_MONITOR_SOM_OFF_SETTLE_S;
              wake_monitor_can_armed = false;
              // pandad is gone and can no longer drain this queue. Keep only
              // counters/detectors while offline so the restarted host never
              // fingerprints against stale pre-shutdown traffic.
              can_clear(&can_rx_q);
              // hardwared already proved every physical CAN bus quiet for
              // 300 seconds. Freeze that pre-shutdown baseline so a real wake
              // during the settle window cannot be learned away.
              wake_debug_stage(0x39U);
            } else {
              wake_monitor_som_off_ready = true;
              wake_monitor_som_off_countdown = 0U;
              wake_monitor_can_armed = true;
              wake_can_trace_clear_peak();
              wake_debug_stage(0x3FU);
              wake_monitor_strict_stop_pending = policy.request_strict_stop;
              wake_monitor_enabled = false;
              set_power_save_state(policy.request_power_save);
            }
          } else if (!wake_monitor_som_off_ready) {
            for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
              wake_monitor_can_baseline[i] = MAX(wake_monitor_can_baseline[i], rx_per_bus[i]);
            }
            if (wake_monitor_som_off_countdown > 0U) {
              wake_monitor_som_off_countdown -= 1U;
            }
            if (wake_monitor_som_off_countdown == 0U) {
              wake_monitor_som_off_ready = true;
              wake_monitor_can_armed = true;
              wake_can_trace_clear_peak();
              offline_wake_raw_can_exti_arm();
              wake_debug_stage(0x3FU);
            }
          } else {
          }
        } else if (wake_monitor_som_off_seen && !wake_monitor_som_off_ready) {
          // Ignore a short heartbeat gap while Linux is still alive. A real
          // shutdown must restart the full settle sequence.
          wake_monitor_som_off_seen = false;
          wake_monitor_som_off_countdown = 0U;
          wake_monitor_can_armed = false;
          wake_monitor_can_activity_pending = false;
        } else {
        }
      }

      bool can_rate_candidate = false;
      if (wake_monitor_enabled && wake_monitor_som_off_ready && wake_monitor_can_armed &&
          !wake_monitor_can_wake_requested) {
        for (uint8_t i = 0U; i < PANDA_CAN_CNT; i++) {
          can_rate_candidate |= offline_wake_can_rate_increase(rx_per_bus[i], wake_monitor_can_baseline[i]);
        }
        if (can_rate_candidate) {
          wake_can_trace_capture_rates(rx_per_bus, wake_monitor_can_baseline);
        }
        if (offline_wake_can_rate_confirm_step(can_rate_candidate, &wake_monitor_can_activity_confirm_count)) {
          wake_monitor_can_activity_pending = true;
        }
        if (wake_monitor_raw_can_edge_pending) {
          // Re-arm after the decoded-frame sample has been evaluated. Persistent
          // background edges can keep waking Panda, but cannot directly wake SoM.
          offline_wake_raw_can_exti_arm();
        }
      } else {
        (void)offline_wake_can_rate_confirm_step(false, &wake_monitor_can_activity_confirm_count);
      }

      if (bootkick_tesla_event_ready(
            wake_monitor_enabled, wake_monitor_som_off_ready, wake_monitor_can_armed,
            wake_monitor_tesla_event_pending, wake_monitor_can_wake_requested)) {
        const uint8_t tesla_event_source = wake_monitor_tesla_event_source;
        wake_monitor_can_wake_requested = true;
        wake_monitor_tesla_event_pending = false;
        wake_monitor_tesla_event_source = TESLA_WAKE_SOURCE_NONE;
        wake_monitor_can_activity_pending = false;
        wake_monitor_can_dispatch_pending = true;
        wake_monitor_can_dispatch_stage = 0x34U;
        if (tesla_event_source == TESLA_WAKE_SOURCE_DOOR) {
          wake_can_trace_set_source(WAKE_CAN_TRACE_SOURCE_TESLA_DOOR);
        } else if (tesla_event_source == TESLA_WAKE_SOURCE_POWER) {
          wake_can_trace_set_source(WAKE_CAN_TRACE_SOURCE_TESLA_POWER);
        } else {
        }
        wake_debug_stage(0x42U);
        if (bootkick_request_wake_pulse(wake_monitor_can_dispatch_stage)) {
          wake_monitor_can_dispatch_pending = false;
          wake_monitor_can_dispatch_stage = 0U;
        }
      }

      if (bootkick_can_activity_ready(
            wake_monitor_enabled, wake_monitor_som_off_ready, wake_monitor_can_armed,
            wake_monitor_can_activity_pending, wake_monitor_can_wake_requested)) {
        wake_monitor_can_activity_pending = false;
        wake_monitor_can_led_countdown = WAKE_MONITOR_CAN_LED_HOLD_S;
        wake_monitor_can_wake_requested = true;
        wake_monitor_can_dispatch_pending = true;
        wake_monitor_can_dispatch_stage = 0x35U;
        wake_debug_stage(0x43U);
        if (bootkick_request_wake_pulse(wake_monitor_can_dispatch_stage)) {
          wake_monitor_can_dispatch_pending = false;
          wake_monitor_can_dispatch_stage = 0U;
        }
      }
      if (wake_monitor_can_led_countdown > 0U) {
        wake_monitor_can_led_countdown -= 1U;
      }

      // A Tesla wake frame can arrive on the FDCAN interrupt boundary while
      // the 1 Hz monitor is persisting the just-armed (0x3F) stage. Recover
      // the exact stranded signature instead of leaving wake_requested set
      // without ever dispatching BOOTKICK.
      if (bootkick_wake_request_needs_dispatch(
            wake_monitor_enabled, wake_monitor_som_off_ready, wake_monitor_can_wake_requested,
            wake_monitor_can_dispatch_pending, bootkick_wake_confirmation_pending, bootkick_wake_pulse_active,
            bootkick_wake_attempts, wake_debug.stage)) {
        const uint32_t dispatch_stage = ((wake_monitor_can_dispatch_stage == 0x35U) || (wake_debug.stage == 0x43U)) ?
                                          0x35U : 0x34U;
        if (bootkick_request_wake_pulse(dispatch_stage)) {
          wake_monitor_can_dispatch_pending = false;
          wake_monitor_can_dispatch_stage = 0U;
        }
      }

      // tick drivers at 1Hz
      bool started = harness_check_ignition() || ignition_can;
      if (wake_monitor_enabled && wake_monitor_som_off_ready && started && !wake_monitor_reset_requested) {
        if (bootkick_request_wake_pulse(0x32U)) {
          wake_monitor_reset_requested = true;
        }
      }
      const bool wake_was_requested = wake_monitor_can_wake_requested || wake_monitor_harness_requested || wake_monitor_reset_requested;
      if (recent_heartbeat || !started) {
        wake_monitor_reset_requested = false;
      }

      if (wake_monitor_enabled && !recent_heartbeat) {
        if (wake_monitor_off_seconds < UINT16_MAX) {
          wake_monitor_off_seconds += 1U;
        }
        const uint8_t trace_flags =
          ((uint8_t)wake_monitor_enabled * WAKE_CAN_TRACE_FLAG_MONITOR_ENABLED) |
          ((uint8_t)wake_monitor_som_off_seen * WAKE_CAN_TRACE_FLAG_SOM_OFF_SEEN) |
          ((uint8_t)wake_monitor_som_off_ready * WAKE_CAN_TRACE_FLAG_SOM_OFF_READY) |
          ((uint8_t)wake_monitor_can_armed * WAKE_CAN_TRACE_FLAG_CAN_ARMED) |
          ((uint8_t)wake_monitor_can_wake_requested * WAKE_CAN_TRACE_FLAG_WAKE_REQUESTED) |
          ((uint8_t)(wake_monitor_can_activity_confirm_count > 0U) * WAKE_CAN_TRACE_FLAG_RATE_CANDIDATE) |
          ((uint8_t)ignition_can * WAKE_CAN_TRACE_FLAG_IGNITION_CAN) |
          ((uint8_t)harness_check_ignition() * WAKE_CAN_TRACE_FLAG_IGNITION_LINE);
        wake_can_trace_update_state(wake_monitor_off_seconds, trace_flags);
      }

      if (wake_monitor_enabled && wake_monitor_som_off_seen && recent_heartbeat) {
        offline_wake_raw_can_exti_disarm();
        can_clear(&can_rx_q);
        wake_monitor_enabled = false;
        wake_monitor_tesla_event_pending = false;
        wake_monitor_tesla_event_source = TESLA_WAKE_SOURCE_NONE;
        wake_monitor_som_off_seen = false;
        wake_monitor_som_off_ready = false;
        wake_monitor_som_off_countdown = 0U;
        wake_monitor_can_armed = false;
        wake_monitor_can_activity_pending = false;
        wake_monitor_can_activity_confirm_count = 0U;
        wake_monitor_raw_can_edge_pending = false;
        wake_monitor_can_led_countdown = 0U;
        wake_monitor_can_wake_requested = false;
        wake_monitor_can_dispatch_pending = false;
        wake_monitor_can_dispatch_stage = 0U;
        wake_monitor_harness_requested = false;
        if (wake_was_requested) {
          wake_debug_latch_success(wake_debug.stage);
        }
        wake_debug_stage(0x38U);
      }
      const bool wake_activity = wake_monitor_enabled &&
                                 (wake_monitor_can_wake_requested || wake_monitor_harness_requested ||
                                  wake_monitor_reset_requested);
      bootkick_tick(started || wake_activity, recent_heartbeat);

      // increase heartbeat counter and cap it at the uint32 limit
      if (heartbeat_counter < UINT32_MAX) {
        heartbeat_counter += 1U;
      }

      // disabling heartbeat not allowed while in safety mode
      if (is_car_safety_mode(current_safety_mode)) {
        heartbeat_disabled = false;
      }

      if (siren_countdown > 0U) {
        siren_countdown -= 1U;
      }

      if (controls_allowed || heartbeat_engaged) {
        controls_allowed_countdown = 5U;
      } else if (controls_allowed_countdown > 0U) {
        controls_allowed_countdown -= 1U;
      } else {

      }

      // exit controls allowed if unused by openpilot for a few seconds
      if (controls_allowed && !heartbeat_engaged) {
        heartbeat_engaged_mismatches += 1U;
        if (heartbeat_engaged_mismatches >= 3U) {
          controls_allowed = false;
        }
      } else {
        heartbeat_engaged_mismatches = 0U;
      }

      mads_heartbeat_engaged_check();

      if (!heartbeat_disabled) {
        // if the heartbeat has been gone for a while, go to SILENT safety mode and enter power save
        if (heartbeat_counter >= (started ? HEARTBEAT_IGNITION_CNT_ON : HEARTBEAT_IGNITION_CNT_OFF)) {
          print("device hasn't sent a heartbeat for 0x");
          puth(heartbeat_counter);
          print(" seconds. Safety is set to SILENT mode.\n");

          if (controls_allowed_countdown > 0U) {
            siren_countdown = 3U;
            controls_allowed_countdown = 0U;
          }

          // set flag to indicate the heartbeat was lost
          if (is_car_safety_mode(current_safety_mode)) {
            heartbeat_lost = true;
          }

          // clear heartbeat engaged state
          heartbeat_engaged = false;

          if (current_safety_mode != SAFETY_SILENT) {
            set_safety_mode(SAFETY_SILENT, 0U);
          }

          if (wake_monitor_enabled && power_save_enabled) {
            set_power_save_state(false);
            wake_debug_stage(0x31U);
          } else if (!wake_monitor_enabled && !power_save_enabled) {
            set_power_save_state(true);
          }

          // Also disable IR when the heartbeat goes missing
          current_board->set_ir_power(0U);

          // Run fan when device is up but not talking to us.
          // The bootloader enables the SOM GPIO on boot.
          const bool offline_monitoring = wake_monitor_enabled && wake_monitor_som_off_ready;
          fan_set_power(!offline_monitoring && current_board->read_som_gpio() ? 30U : 0U);
        }
      }

      // check registers
      check_registers();

      // set ignition_can to false after 2s of no CAN seen
      if (ignition_can_cnt > 2U) {
        ignition_can = false;
      }

      // on to the next one
      uptime_cnt += 1U;
      safety_mode_cnt += 1U;
      ignition_can_cnt += 1U;

      // synchronous safety check
      safety_tick(&current_safety_config);
    }
    if (wake_monitor_enabled && wake_monitor_som_off_seen) {
      const bool wake_requested = wake_monitor_can_wake_requested || wake_monitor_harness_requested ||
                                  wake_monitor_reset_requested || bootkick_wake_confirmation_pending;
      const bool can_activity_seen = (wake_monitor_can_led_countdown > 0U) ||
                                     wake_monitor_can_activity_pending || wake_monitor_tesla_event_pending;
      led_set(LED_BLUE, offline_wake_blue_led_on(
        wake_monitor_som_off_ready, can_activity_seen, wake_requested, wake_monitor_led_phase));
      wake_monitor_led_phase++;
      wake_monitor_led_phase %= 32U;
    } else {
      wake_monitor_led_phase = 0U;
    }

    loop_counter++;
    loop_counter %= 8U;
  }
  TICK_TIMER->SR = 0;
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  // shouldn't have interrupts here, but just in case
  disable_interrupts();

  // init early devices
  clock_init();
  peripherals_init();
  detect_board_type();
  led_init();
  // red+green leds enabled until succesful USB/SPI init, as a debug indicator
  led_set(LED_RED, true);
  led_set(LED_GREEN, true);
  adc_init(ADC1);
  wake_debug_init();
  bootkick_debug_restore();

  // print hello
  print("\n\n\n************************ MAIN START ************************\n");

  // check for non-supported board types
  assert_fatal(hw_type != HW_TYPE_UNKNOWN, "Unsupported board type");

  print("Config:\n");
  print("  Board type: 0x"); puth(hw_type); print("\n");

  // init board
  current_board->init();
  if (bootkick_wake_waiting_for_som_off) {
    // A pre-STOP ignition edge can arrive while the SoM is still shutting
    // down. Release BOOTKICK immediately after GPIO initialization so the
    // deferred wake path can create a fresh edge after SoM power is gone.
    current_board->set_bootkick(BOOT_STANDBY);
  }
  current_board->set_can_mode(CAN_MODE_NORMAL);
  harness_init();

  // panda has an FPU, let's use it!
  enable_fpu();

  microsecond_timer_init();

  current_board->set_siren(false);
  if (current_board->has_fan) {
    fan_init();
  }

  // init to SILENT and can silent
  set_safety_mode(SAFETY_SILENT, 0U);

  // enable CAN TXs
  enable_can_transceivers(true);

  // init watchdog for heartbeat loop, fed at 8Hz
  simple_watchdog_init(FAULT_HEARTBEAT_LOOP_WATCHDOG, (3U * 1000000U / 8U));

  // 8Hz timer
  REGISTER_INTERRUPT(TICK_TIMER_IRQ, tick_handler, 10U, FAULT_INTERRUPT_RATE_TICK)
  tick_timer_init();
  offline_wake_raw_can_exti_init();

#ifdef DEBUG
  print("DEBUG ENABLED\n");
#endif
  // enable USB (right before interrupts or enum can fail!)
  usb_init();

  if (current_board->has_spi) {
    gpio_spi_init();
    spi_init();
  }

  led_set(LED_RED, false);
  led_set(LED_GREEN, false);
  led_set(LED_BLUE, false);

  print("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // LED should keep on blinking all the time
  while (true) {
    #ifdef ALLOW_DEBUG
    if (stop_mode_requested) {
      enter_stop_mode();
    }
    #endif
    if (!power_save_enabled) {
      if (wake_monitor_enabled && wake_monitor_som_off_seen) {
        // Keep FDCAN and the normal interrupt controller running, but stop the
        // LED fade busy-loop while the SoM is powered off. Tick/CAN interrupts
        // wake the MCU from this shallow sleep; SAFETY_SILENT still blocks TX.
        led_set(LED_RED, false);
        led_set(LED_GREEN, false);
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
        __DSB();
        __ISB();
        // cppcheck-suppress misra-c2012-17.3 ; CMSIS __WFI macro expands to inline asm
        __WFI();
      } else {
        #ifdef DEBUG_FAULTS
        if (fault_status == FAULT_STATUS_NONE) {
        #endif
          // useful for debugging, fade breaks = panda is overloaded
          for (uint32_t fade = 0U; fade < MAX_LED_FADE; fade += 1U) {
            led_set(LED_RED, true);
            delay(fade >> 4);
            led_set(LED_RED, false);
            delay((MAX_LED_FADE - fade) >> 4);
          }

          for (uint32_t fade = MAX_LED_FADE; fade > 0U; fade -= 1U) {
            led_set(LED_RED, true);
            delay(fade >> 4);
            led_set(LED_RED, false);
            delay((MAX_LED_FADE - fade) >> 4);
          }

        #ifdef DEBUG_FAULTS
        } else {
          led_set(LED_RED, 1);
          delay(512000U);
          led_set(LED_RED, 0);
          delay(512000U);
        }
        #endif
      }
    } else {
      const bool normal_stop_allowed = !current_board->read_som_gpio();
      const bool strict_stop_allowed = wake_monitor_strict_stop_pending;
      if (((hw_type == HW_TYPE_TRES) || (hw_type == HW_TYPE_CUATRO)) &&
          (normal_stop_allowed || strict_stop_allowed) && !wake_monitor_enabled && !bootkick_debug_active()) {
        assert_fatal(current_safety_mode == SAFETY_SILENT, "Error: Entering low power mode while not in SAFETY_SILENT. Hanging\n");
        enter_stop_mode();
        assert_fatal(false, "Error: enter_stop_mode returned after system reset. Hanging\n");
      }
      // cppcheck-suppress misra-c2012-17.3 ; CMSIS __WFI macro expands to inline asm
      __WFI();
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
  }

  return 0;
}
