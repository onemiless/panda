import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_offline_wake_source_masks_include_sbu_and_all_tres_can_rx(tmp_path):
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
      assert(offline_wake_tres_exti_lines(false) == ((1UL << 1) | (1UL << 4) | (1UL << 5) | (1UL << 8) | (1UL << 9)));
      assert(offline_wake_tres_exti_lines(true) == ((1UL << 1) | (1UL << 4) | (1UL << 8) | (1UL << 9) | (1UL << 12)));
      assert(offline_wake_cuatro_can_exti_lines(false) == ((1UL << 5) | (1UL << 8) | (1UL << 12)));
      assert(offline_wake_cuatro_can_exti_lines(true) == ((1UL << 8) | (1UL << 12)));
      assert(offline_wake_cuatro_exti_lines(false) == ((1UL << 1) | (1UL << 4) | (1UL << 5) | (1UL << 8) | (1UL << 12)));
      assert(offline_wake_cuatro_exti_lines(true) == ((1UL << 1) | (1UL << 4) | (1UL << 8) | (1UL << 12)));

      const uint32_t raw_lines = offline_wake_tres_can_exti_lines(false);
      assert(offline_wake_raw_can_edge_hint_ready(true, true, true, false, 1UL << 8, raw_lines));
      assert(offline_wake_raw_can_edge_hint_ready(true, true, true, false, 1UL << 5, raw_lines));
      assert(!offline_wake_raw_can_edge_hint_ready(false, true, true, false, 1UL << 8, raw_lines));
      assert(!offline_wake_raw_can_edge_hint_ready(true, false, true, false, 1UL << 8, raw_lines));
      assert(!offline_wake_raw_can_edge_hint_ready(true, true, false, false, 1UL << 8, raw_lines));
      assert(!offline_wake_raw_can_edge_hint_ready(true, true, true, true, 1UL << 8, raw_lines));
      assert(!offline_wake_raw_can_edge_hint_ready(true, true, true, false, 1UL << 12, raw_lines));

      const uint32_t primary_raw_line = offline_wake_oriented_fdcan2_exti_line(false);
      assert(offline_wake_primary_raw_can_edge_ready(
        true, true, true, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        false, true, true, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, false, true, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, false, false, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, true, true, primary_raw_line, primary_raw_line));
      assert(!offline_wake_primary_raw_can_edge_ready(
        true, true, true, false, 1UL << 8, primary_raw_line));

      uint32_t sleep_baseline = OFFLINE_WAKE_CAN_BASELINE_UNSET;
      sleep_baseline = offline_wake_can_sleep_baseline_step(sleep_baseline, 600U);
      sleep_baseline = offline_wake_can_sleep_baseline_step(sleep_baseline, 120U);
      sleep_baseline = offline_wake_can_sleep_baseline_step(sleep_baseline, 100U);
      sleep_baseline = offline_wake_can_sleep_baseline_step(sleep_baseline, 110U);
      assert(sleep_baseline == 100U);

      assert(!offline_wake_can_rate_increase(1U, OFFLINE_WAKE_CAN_BASELINE_UNSET));
      assert(!offline_wake_can_rate_increase(1U, 0U));
      assert(!offline_wake_can_rate_increase(100U, 100U));
      assert(!offline_wake_can_rate_increase(140U, 100U));
      // This is the captured failure shape: host traffic was ~600 frames/s,
      // sleep settled near 100 frames/s, and the door burst reached 190.
      // Comparing against the live-host baseline misses it; the learned sleep
      // baseline must accept it without accepting normal +/-40 jitter.
      assert(!offline_wake_can_rate_increase(190U, 600U));
      assert(offline_wake_can_rate_increase(190U, sleep_baseline));
      assert(offline_wake_can_rate_increase(250U, 100U));
      assert(offline_wake_can_rate_increase(50U, 0U));

      // Two independent vehicle captures show physical bus 1 is the first
      // bus to resume after a five-minute sleep. Once the shutdown guard has
      // armed the monitor, its first decoded frame must not wait for Party or
      // another bus to cross a rate threshold.
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 0U));
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 1U));
      assert(offline_wake_physical_bus_rx_ready(true, true, true, false, 2U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, true, false, 128U));
      assert(!offline_wake_physical_bus_rx_ready(true, false, true, false, 1U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, false, false, 1U));
      assert(!offline_wake_physical_bus_rx_ready(true, true, true, true, 1U));

      uint8_t primary_quiet_seconds = 0U;
      for (uint8_t i = 0U; i < WAKE_MONITOR_PRIMARY_BUS_QUIET_S - 1U; i++) {
        primary_quiet_seconds = offline_wake_primary_bus_quiet_step(primary_quiet_seconds, 0U);
        assert(!offline_wake_primary_bus_guard_ready(0U, primary_quiet_seconds));
      }
      primary_quiet_seconds = offline_wake_primary_bus_quiet_step(primary_quiet_seconds, 0U);
      assert(offline_wake_primary_bus_guard_ready(0U, primary_quiet_seconds));
      assert(!offline_wake_primary_bus_guard_ready(1U, primary_quiet_seconds));
      assert(offline_wake_primary_bus_quiet_step(primary_quiet_seconds, 1U) == 0U);

      const uint32_t quiet_window[3] = {25U, 80U, 25U};
      const uint32_t short_door_burst[3] = {48U, 160U, 48U};
      const uint32_t multimedia_only[3] = {25U, 160U, 25U};
      const uint32_t baseline_per_second[3] = {100U, 320U, 100U};
      assert(!offline_wake_multibus_burst_ready(quiet_window, baseline_per_second, 0U));
      assert(offline_wake_multibus_burst_ready(short_door_burst, baseline_per_second, 0U));
      assert(!offline_wake_multibus_burst_ready(multimedia_only, baseline_per_second, 0U));
      // Flipped harness: logical Party bus 0 is physical CAN index 2.
      assert(offline_wake_multibus_burst_ready(short_door_burst, baseline_per_second, 2U));
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


def test_tres_active_monitor_keeps_primary_fdcan_decoding():
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()
  main_source = (PANDA_ROOT / "board/main.c").read_text()

  assert "offline_wake_raw_can_exti_arm" not in power_source
  assert "offline_wake_raw_can_exti_arm" not in main_source
  assert "offline_wake_physical_bus_rx_ready(" in (PANDA_ROOT / "board/drivers/fdcan.h").read_text()


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
    "if (bootkick_tesla_event_ready(", 1
  )[0]

  # The host is genuinely powered off, but panda must remain in receive-only
  # mode. Entering STOP disables FDCAN parsing and makes the Tesla frame/rate
  # wake detectors below this transition unreachable.
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


def test_can_event_latches_immediately_after_commit_even_before_som_settle():
  source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  wake_latch = source.split("const uint8_t tesla_source =", 1)[1].split("#endif", 1)[0]

  assert "tesla_wake_source(&to_push, can_number)" in wake_latch
  assert "bootkick_tesla_event_should_latch(" in wake_latch
  assert "wake_monitor_can_armed" in wake_latch
  assert "wake_monitor_committed" in wake_latch
  assert "wake_monitor_som_off_ready" not in wake_latch

  # A same-session heartbeat during shutdown cleanup must not erase the event;
  # only a new Linux session resolves the committed transaction.
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  assert "wake_monitor_heartbeat_result(" in main_source
  assert "wake_monitor_status.host_session, wake_monitor_status.committed_host_session" in main_source
  assert "wake_monitor_can_armed ? tesla_wake_source" not in wake_latch


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
    "if (bootkick_tesla_event_ready(", 1
  )[0]

  assert "wake_monitor_tesla_event_pending = false" not in heartbeat_transition
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
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  trace = (PANDA_ROOT / "board/drivers/wake_debug.h").read_text()

  physical_rx = fdcan.split("if (offline_wake_physical_bus_rx_ready(", 1)[1].split(
    "if (bootkick_tesla_event_should_latch(", 1
  )[0]
  assert "wake_can_trace_record_rx(can_number);" in physical_rx
  assert physical_rx.index("wake_can_trace_record_rx(can_number);") < physical_rx.index(
    "wake_monitor_can_activity_pending = true;"
  )
  assert "wake_debug_active_can_first_rx(can_number, to_push.addr);" in physical_rx

  commit_case = comms.split("case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  assert "wake_debug_active_can_arm_snapshot();" in commit_case
  assert "wake_can_trace_flush_rx_window();" in main_source

  assert "static void wake_can_trace_record_rx(uint8_t physical_bus)" in trace
  assert "static void wake_can_trace_flush_rx_window(void)" in trace
  assert "wake_journal" not in trace
