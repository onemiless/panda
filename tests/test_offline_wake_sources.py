import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_offline_wake_source_policy_uses_oriented_physical_rx_without_rate_gates(tmp_path):
  source = tmp_path / "offline_wake_source_policy_test.c"
  executable = tmp_path / "offline_wake_source_policy_test"
  source.write_text(
    """
    #include <assert.h>
    #include <stdbool.h>
    #include "board/drivers/offline_wake_source_policy.h"

    int main(void) {
      const offline_wake_heartbeat_loss_policy tres_policy = offline_wake_policy_after_heartbeat_loss(true);
      assert(tres_policy.keep_can_active);
      assert(!tres_policy.request_power_save);
      assert(!tres_policy.request_strict_stop);
      const offline_wake_heartbeat_loss_policy other_policy = offline_wake_policy_after_heartbeat_loss(false);
      assert(!other_policy.keep_can_active);
      assert(other_policy.request_power_save);
      assert(other_policy.request_strict_stop);

      for (uint8_t phase = 0U; phase < 32U; phase++) {
        assert(!offline_wake_blue_led_on(false, false, false, phase));
        assert(offline_wake_blue_led_on(true, false, false, phase) == (phase == 0U));
        assert(offline_wake_blue_led_on(true, true, false, phase) == ((phase & 1U) == 0U));
        assert(offline_wake_blue_led_on(true, false, true, phase));
        assert(offline_wake_blue_led_on(true, true, true, phase));
      }

      assert(offline_wake_oriented_fdcan2_exti_line(false) == (1UL << 5));
      assert(offline_wake_oriented_fdcan2_exti_line(true) == (1UL << 12));
      assert(offline_wake_tres_can_exti_lines(false) == ((1UL << 5) | (1UL << 8) | (1UL << 9)));
      assert(offline_wake_tres_can_exti_lines(true) == ((1UL << 8) | (1UL << 9) | (1UL << 12)));
      assert(offline_wake_cuatro_can_exti_lines(false) == ((1UL << 5) | (1UL << 8) | (1UL << 12)));
      assert(offline_wake_cuatro_can_exti_lines(true) == ((1UL << 8) | (1UL << 12)));

      const uint32_t primary_raw_line = offline_wake_oriented_fdcan2_exti_line(false);
      assert(offline_wake_primary_raw_can_edge_ready(
        true, true, true, false, primary_raw_line, primary_raw_line));
      // COMMIT owns event capture immediately. SoM readiness gates only the
      // later BOOTKICK dispatch; it must not create another blind window.
      assert(offline_wake_primary_raw_can_edge_ready(
        true, false, true, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        false, true, true, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, false, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, true, true, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, true, false, 1UL << 8, primary_raw_line));

      // Once the host shutdown gate has armed the monitor, the first decoded
      // physical frame must not wait for a rate or multi-bus threshold.
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 0U));
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 1U));
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 2U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, true, false, 128U));
      assert(!offline_wake_physical_bus_rx_ready(true, false, true, false, 1U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, false, false, 1U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, true, true, 1U));

      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_new_prepare_is_idempotent_metadata_only_and_prearms_rx():
  source = (PANDA_ROOT / "board/main_comms.h").read_text()
  prepare_helper = source.split("static void wake_monitor_prepare(", 1)[1].split("static int get_health_pkt", 1)[0]
  prepare_case = source.split("case PANDA_REQUEST_PREPARE_WAKE_MONITOR:", 1)[1].split(
    "case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1
  )[0]

  assert "wake_debug_clear_success();" in prepare_helper
  assert "set_power_save_state(false);" in prepare_helper
  metadata_prepare = prepare_helper.split("if (committed) {", 1)[0]
  assert "set_safety_mode(" not in metadata_prepare
  assert "can_init_all();" not in prepare_helper
  assert "wake_monitor_prepare_flags(wake_monitor_can_health_ready()," in prepare_helper
  assert "wake_monitor_prepared_host_session" in prepare_helper
  assert "wake_monitor_capture_prepare_snapshot();" in prepare_helper
  assert "if (action == WAKE_MONITOR_PREPARE_START)" in prepare_case
  assert "wake_monitor_prepare(transaction, false);" in prepare_case


def test_tres_active_monitor_keeps_fdcan_af_and_arms_primary_raw_edge_fallback():
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()

  arm = power_source.split("static void offline_wake_active_can_exti_arm", 1)[1].split(
    "static void offline_wake_active_can_diag_snapshot", 1
  )[0]
  assert "offline_wake_oriented_fdcan2_exti_line" in arm
  assert "SYSCFG->EXTICR" in arm
  assert "EXTI->IMR1" in arm
  assert "EXTI->RTSR1" in arm
  assert "EXTI->FTSR1" in arm
  assert "RCC_APB1HLPENR_FDCANLPEN" in arm
  assert "const IRQn_Type primary_irq" in arm
  assert "flipped ? EXTI15_10_IRQn : EXTI9_5_IRQn" in arm
  assert "NVIC_ClearPendingIRQ(primary_irq);" in arm
  assert "NVIC_EnableIRQ(primary_irq);" in arm
  assert arm.index("EXTI->PR1 = primary_line;") < arm.index("NVIC_ClearPendingIRQ(primary_irq);")
  assert arm.index("register_set_bits(&(EXTI->IMR1), primary_line);") < arm.index("NVIC_EnableIRQ(primary_irq);")
  assert "set_gpio_mode" not in arm
  assert "set_gpio_alternate" not in arm
  assert "can_init_all" not in arm
  assert "offline_wake_physical_bus_rx_ready(" in fdcan_source


def test_real_transaction_commit_arms_primary_raw_edge_fallback():
  comms_source = (PANDA_ROOT / "board/main_comms.h").read_text()

  commit = comms_source.split("case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1)[1].split(
    "case PANDA_REQUEST_ABORT_WAKE_MONITOR:", 1
  )[0]
  assert "offline_wake_active_can_exti_arm();" in commit
  assert commit.index("set_safety_mode(SAFETY_SILENT, 0U);") < commit.index("offline_wake_active_can_exti_arm();")
  assert commit.index("wake_monitor_can_armed = true;") < commit.index("offline_wake_active_can_exti_arm();")
  assert commit.index("offline_wake_active_can_exti_arm();") < commit.index("offline_wake_active_can_diag_snapshot(false);")


def test_receive_only_observer_records_raw_exti_without_wake_or_bootkick():
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()
  comms_source = (PANDA_ROOT / "board/main_comms.h").read_text()

  observer = comms_source.split("case PANDA_REQUEST_ARM_WAKE_OBSERVER:", 1)[1].split(
    "case PANDA_REQUEST_DISARM_WAKE_OBSERVER:", 1
  )[0]
  assert "set_power_save_state(false);" in observer
  assert "enable_can_transceivers(true);" in observer
  assert "offline_wake_active_can_exti_arm();" in observer
  assert "offline_wake_active_can_diag_snapshot(true);" in observer
  assert "set_safety_mode" not in observer
  assert "set_bootkick" not in observer

  raw_irq = power_source.split("static void offline_wake_raw_can_exti_irq_handler", 1)[1].split(
    "static void offline_wake_raw_can_exti_init", 1
  )[0]
  observer_irq = raw_irq.split("if (wake_monitor_observer_enabled", 1)[1].split(
    "if (offline_wake_primary_raw_can_edge_ready", 1
  )[0]
  assert "wake_debug_active_can_exti_irq" in observer_irq
  assert "wake_monitor_observer_enabled = false" not in observer_irq
  assert "wake_monitor_can_activity_pending" not in observer_irq
  assert "bootkick_" not in observer_irq.lower()
  assert "wake_journal" not in observer_irq


def test_tres_keeps_proven_fdcan_af_irq_and_exti_state_after_som_off():
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  comms_source = (PANDA_ROOT / "board/main_comms.h").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()

  assert "wake_monitor_fdcan2_irq_quiesced" not in power_source
  assert "offline_wake_active_can_irq_quiesce" not in power_source
  assert "offline_wake_active_can_irq_restore" not in power_source
  assert "offline_wake_active_can_gpio" not in power_source

  som_ready = main_source.split("// CAN has been armed since COMMIT", 1)[1].split("wake_debug_stage(0x3FU);", 1)[0]
  assert "llcan_irq_disable" not in som_ready
  assert "set_gpio_mode" not in som_ready
  # COMMIT already captured the stable AF/IRQ configuration. Re-snapshotting
  # here would erase an IRQ/first frame latched while Linux was shutting down.
  assert "offline_wake_active_can_diag_snapshot(false);" not in som_ready

  heartbeat_cleanup = main_source.split("if (wake_monitor_enabled && (heartbeat_result !=", 1)[1].split(
    "wake_debug_stage(0x38U);", 1
  )[0]
  reset_runtime = comms_source.split("static void wake_monitor_reset_runtime", 1)[1].split(
    "static bool wake_monitor_can_health_ready", 1
  )[0]
  assert "offline_wake_active_can_irq_restore" not in heartbeat_cleanup
  assert "offline_wake_active_can_irq_restore" not in reset_runtime
  assert "can_init_all();" not in heartbeat_cleanup
  assert "can_init_all();" not in reset_runtime

  # FDCAN handlers continue draining hardware FIFO while Linux is absent, but
  # committed traffic is never queued for the returned host to fingerprint.
  can_rx = fdcan_source.split("void can_rx(uint8_t can_number)", 1)[1].split(
    "static void FDCAN1_IT0_IRQ_Handler", 1
  )[0]
  assert "queue_for_host = !wake_monitor_enabled || !wake_monitor_committed;" in can_rx
  assert "if (queue_for_host)" in can_rx


def test_fdcan_low_power_clock_is_scoped_to_tres_active_monitor():
  source = (PANDA_ROOT / "board/stm32h7/peripherals.h").read_text()

  assert "RCC->APB1HENR |= RCC_APB1HENR_FDCANEN" in source
  assert "RCC_APB1HLPENR_FDCANLPEN" not in source


def test_host_return_does_not_reinitialize_healthy_fdcan():
  source = (PANDA_ROOT / "board/main.c").read_text()
  heartbeat_cleanup = source.split("if (wake_monitor_enabled && (heartbeat_result !=", 1)[1].split(
    "wake_debug_stage(0x38U);", 1
  )[0]

  assert "offline_wake_raw_can_exti_disarm();" in heartbeat_cleanup
  assert "current_board->set_can_mode(CAN_MODE_NORMAL);" not in heartbeat_cleanup
  assert "can_init_all();" not in heartbeat_cleanup


def test_abort_resets_monitor_state_without_reinitializing_fdcan():
  source = (PANDA_ROOT / "board/main_comms.h").read_text()
  reset_runtime = source.split("static void wake_monitor_reset_runtime", 1)[1].split(
    "static void wake_monitor_prepare", 1
  )[0]
  abort_case = source.split("case PANDA_REQUEST_ABORT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]

  assert "offline_wake_raw_can_exti_disarm();" in reset_runtime
  assert "current_board->set_can_mode(CAN_MODE_NORMAL);" not in reset_runtime
  assert "can_init_all();" not in reset_runtime
  assert "enable_can_transceivers(true);" in reset_runtime
  assert "wake_monitor_reset_runtime();" in abort_case


def test_wake_monitor_keeps_fdcan_active_after_host_shutdown():
  source = (PANDA_ROOT / "board/main.c").read_text()
  heartbeat_transition = source.split("if (wake_monitor_enabled && wake_monitor_committed) {", 1)[1].split(
    "if (bootkick_can_activity_ready(", 1
  )[0]

  # The host is genuinely powered off, but panda must remain in receive-only
  # mode until SoM-off is confirmed. Entering STOP would make the active FDCAN
  # and EXTI receive paths unreachable.
  assert "offline_wake_policy_after_heartbeat_loss(hw_type == HW_TYPE_TRES)" in heartbeat_transition
  assert "if (policy.keep_can_active)" in heartbeat_transition
  assert "wake_monitor_can_armed = false" not in heartbeat_transition
  assert "offline_wake_primary_bus_guard_ready" not in heartbeat_transition

  shallow_idle = source.rsplit("if (wake_monitor_enabled && wake_monitor_som_off_seen) {", 1)[1].split("} else {", 1)[0]
  assert "SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;" in shallow_idle
  assert "__WFI();" in shallow_idle


def test_offline_monitor_exposes_can_and_wake_state_on_blue_led():
  source = (PANDA_ROOT / "board/main.c").read_text()

  assert "WAKE_MONITOR_CAN_LED_HOLD_S" in source
  assert "wake_monitor_can_led_countdown = WAKE_MONITOR_CAN_LED_HOLD_S;" in source
  assert "offline_wake_blue_led_on(" in source
  assert "wake_monitor_led_phase %= 32U;" in source
  assert "wake_monitor_can_wake_requested || wake_monitor_harness_requested" in source

  main_loop = source.split("while (true) {", 1)[1]
  shallow_idle = main_loop.split("if (wake_monitor_enabled && wake_monitor_som_off_seen) {", 1)[1].split("} else {", 1)[0]
  assert "led_set(LED_BLUE, false);" not in shallow_idle


def test_offline_monitor_does_not_buffer_stale_can_for_restarted_host():
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()

  assert main_source.count("can_clear(&can_rx_q);") >= 2
  assert "queue_for_host = !wake_monitor_enabled || !wake_monitor_committed" in fdcan_source


def test_any_valid_can_event_latches_immediately_after_commit_even_before_som_settle():
  source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  wake_latch = source.split("// After a clean COMMIT", 1)[1].split("#endif", 1)[0]

  assert "offline_wake_physical_bus_rx_ready(" in wake_latch
  assert "wake_monitor_can_activity_pending = true;" in wake_latch
  assert "wake_monitor_can_armed" in wake_latch
  assert "wake_monitor_committed" in wake_latch
  assert "wake_monitor_som_off_ready" not in wake_latch

  # A same-session heartbeat during shutdown cleanup must not erase the event;
  # only a new Linux session resolves the committed transaction.
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  assert "wake_monitor_heartbeat_result(" in main_source
  assert "wake_monitor_status.host_session, wake_monitor_status.committed_host_session" in main_source


def test_any_valid_physical_bus_frame_requests_wake_without_rate_confirmation():
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  policy_source = (PANDA_ROOT / "board/drivers/offline_wake_source_policy.h").read_text()
  protocol_source = (PANDA_ROOT / "board/wake_protocol.h").read_text()

  assert "offline_wake_physical_bus_rx_ready(" in fdcan_source
  assert "wake_monitor_can_activity_pending = true;" in fdcan_source
  assert "WAKE_JOURNAL_SOURCE_CAN_PRIMARY" in fdcan_source
  assert "to_push.addr, to_push.data" in fdcan_source
  assert "WAKE_JOURNAL_SOURCE_CAN_PRIMARY" in protocol_source
  assert "offline_wake_multibus_burst_ready(" not in main_source
  assert "offline_wake_can_rate_confirm_step(" not in main_source
  assert "bootkick_can_activity_ready(" in main_source
  assert "WAKE_MONITOR_CAN_ACTIVITY_CONFIRM_S" not in policy_source
  assert "wake_monitor_can_wake_requested = true;" in main_source
  assert "wake_monitor_fast_rx_window" not in main_source


def test_shutdown_guard_never_clears_a_latched_event():
  source = (PANDA_ROOT / "board/main.c").read_text()
  heartbeat_transition = source.split("if (wake_monitor_enabled && wake_monitor_committed) {", 1)[1].split(
    "if (bootkick_can_activity_ready(", 1
  )[0]

  assert "wake_monitor_can_activity_pending = false" not in heartbeat_transition
  assert "offline_wake_primary_bus_guard_ready(" not in heartbeat_transition


def test_committed_wake_handoff_cannot_be_taken_out_of_silent_safety():
  source = (PANDA_ROOT / "board/main.c").read_text()
  setter = source.split("void set_safety_mode(uint16_t mode, uint16_t param) {", 1)[1].split(
    "bool is_car_safety_mode", 1
  )[0]

  assert "wake_monitor_enabled && wake_monitor_committed" in setter
  assert "SAFETY_SILENT" in setter

  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  commit_case = comms.split("case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  assert "set_safety_mode(SAFETY_SILENT, 0U);" in commit_case
  abort_case = comms.split("case PANDA_REQUEST_ABORT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  assert "wake_journal_abort_cycle();" in abort_case


def test_commit_rechecks_fdcan_after_silent_transition_and_offline_faults_self_rescue():
  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  commit_case = comms.split("case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  main_source = (PANDA_ROOT / "board/main.c").read_text()

  assert "wake_monitor_prepared_host_session" in commit_case
  assert "set_safety_mode(SAFETY_SILENT, 0U);" in commit_case
  assert "wake_monitor_can_health_ready()" in commit_case
  assert "wake_monitor_prepare_snapshot_clean()" in commit_case
  offline_ready = comms.split("static bool wake_monitor_offline_source_ready", 1)[1].split(
    "static void wake_monitor_capture_prepare_snapshot", 1
  )[0]
  assert "return wake_monitor_can_health_ready();" in offline_ready
  assert "offline_wake_active_can_irq_quiesced_ready" not in offline_ready
  assert "if (i != 1U)" not in offline_ready
  health_ready = comms.split("static bool wake_monitor_can_health_ready", 1)[1].split(
    "static bool wake_monitor_offline_source_ready", 1
  )[0]
  assert "llcan_rx_ready(CANIF_FROM_CAN_NUM(i))" in health_ready
  assert "wake_monitor_offline_fault_ready(" in main_source
  assert "WAKE_JOURNAL_SOURCE_PANDA_FAULT" in main_source


def test_wake_journal_writes_only_first_event_and_final_result():
  journal = (PANDA_ROOT / "board/drivers/wake_journal.h").read_text()
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  trace = (PANDA_ROOT / "board/drivers/wake_debug.h").read_text()

  assert "wake_journal_pending_commit" not in journal
  assert "wake_journal_pending_armed" not in journal
  assert "next_sequence += 2U" in journal
  assert "wake_journal_queue_checkpoint(" not in main_source
  assert "wake_journal_queue_checkpoint(" not in comms
  assert "off_seconds % 60U" not in trace


def test_active_fdcan_monitor_persists_one_shot_rtc_diagnostics_without_flash_churn():
  fdcan = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  trace = (PANDA_ROOT / "board/drivers/wake_debug.h").read_text()

  offline_rx = fdcan.split("// After a clean COMMIT", 1)[1].split(
    "#endif", 1
  )[0]
  assert "wake_can_trace_record_rx(can_number);" in offline_rx
  assert "wake_debug_active_can_first_rx(can_number, to_push.addr);" in offline_rx
  assert offline_rx.index("wake_monitor_can_activity_pending = true;") < offline_rx.index(
    "wake_can_trace_record_rx(can_number);"
  )
  raw_irq = power_source.split("static void offline_wake_raw_can_exti_irq_handler", 1)[1].split(
    "static void offline_wake_raw_can_exti_init", 1
  )[0]
  assert raw_irq.index("wake_monitor_can_activity_pending = true;") < raw_irq.index(
    "wake_can_trace_set_source(WAKE_CAN_TRACE_SOURCE_RAW_EDGE);"
  )
  assert raw_irq.count("wake_journal_queue_event(") == 1
  sample_gate = offline_rx.split("if (production_wake_diag)", 1)[1].split("}", 1)[0]
  assert "wake_monitor_can_wake_requested" not in sample_gate

  commit_case = comms.split("case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  assert "offline_wake_active_can_diag_snapshot(false);" in commit_case
  assert "wake_can_trace_flush_rx_window();" in main_source

  assert "static void wake_can_trace_record_rx(uint8_t physical_bus)" in trace
  assert "static void wake_can_trace_flush_rx_window(void)" in trace
  trace_reset = trace.split("static void wake_can_trace_reset(void)", 1)[1].split(
    "static void wake_debug_init(void)", 1
  )[0]
  assert "wake_can_trace_rx_window[i] = 0U;" in trace_reset
  arm_snapshot = fdcan.split("static void wake_debug_active_can_arm_snapshot(void)", 1)[1].split("#endif", 1)[0]
  assert "WAKE_ACTIVE_CAN_DIAG_V1_MAGIC" in (PANDA_ROOT / "board/wake_protocol.h").read_text()
  assert "wake_active_can_diag_make(io)" in arm_snapshot
  assert "hw_type != HW_TYPE_TRES" in arm_snapshot
  assert "can_exti_line" not in arm_snapshot
  assert "wake_debug_gpio_output_is_low" in arm_snapshot
  assert "FDCAN_ILS_RF0NL" in arm_snapshot
  assert "FDCAN_ILS_RF0NL" in (PANDA_ROOT / "board/stm32h7/llfdcan.h").read_text()
  assert "wake_monitor_tres_can_io_ready()" in comms
  assert "wake_debug_active_can_irq_entry(1U);" in fdcan
  assert fdcan.index("wake_debug_active_can_irq_entry(1U);") < fdcan.index("can_rx(1U);")
  assert fdcan.index("can_rx(1U);") < fdcan.index("wake_debug_active_can_irq_flush(1U);")
  arm_save = trace.split("static void wake_debug_active_can_save_arm_snapshot(void)", 1)[1].split(
    "static void wake_success_save(void)", 1
  )[0]
  invalidate = "dst[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD] = 0U;"
  publish = "dst[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD] = src[WAKE_DEBUG_ACTIVE_CAN_TAG_WORD];"
  assert invalidate in arm_save
  assert publish in arm_save
  assert arm_save.index(invalidate) < arm_save.index("for (uint8_t i") < arm_save.index(publish)
  assert "wake_journal" not in trace


def test_obsolete_tesla_semantic_and_rate_wake_paths_are_removed():
  board = PANDA_ROOT / "board"
  sources = "\n".join(path.read_text() for path in board.rglob("*.h"))
  sources += "\n" + "\n".join(path.read_text() for path in board.rglob("*.c"))

  for obsolete in (
    "tesla_offline_wake",
    "wake_monitor_tesla_event",
    "bootkick_tesla_event",
    "offline_wake_raw_can_edge_hint_ready",
    "offline_wake_can_rate_increase",
    "offline_wake_multibus_burst_ready",
    "wake_event_trace_policy",
  ):
    assert obsolete not in sources
