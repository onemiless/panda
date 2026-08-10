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

      uint8_t confirmation = 0U;
      assert(!offline_wake_can_rate_confirm_step(true, &confirmation));
      assert(confirmation == 1U);
      assert(offline_wake_can_rate_confirm_step(true, &confirmation));
      assert(confirmation == WAKE_MONITOR_CAN_ACTIVITY_CONFIRM_S);
      assert(!offline_wake_can_rate_confirm_step(false, &confirmation));
      assert(confirmation == 0U);
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_new_prepare_starts_a_clean_transaction_and_configures_receive_only():
  source = (PANDA_ROOT / "board/main_comms.h").read_text()
  prepare_helper = source.split("static void wake_monitor_prepare(", 1)[1].split("static int get_health_pkt", 1)[0]
  prepare_case = source.split("case PANDA_REQUEST_PREPARE_WAKE_MONITOR:", 1)[1].split(
    "case PANDA_REQUEST_COMMIT_WAKE_MONITOR:", 1
  )[0]

  assert "wake_debug_clear_success();" in prepare_helper
  assert prepare_helper.index("set_safety_mode(SAFETY_SILENT, 0U);") < prepare_helper.index("set_power_save_state(false);")
  assert prepare_helper.index("set_power_save_state(false);") < prepare_helper.index("enable_can_transceivers(true);")
  assert "if (action == WAKE_MONITOR_PREPARE_START)" in prepare_case
  assert "wake_monitor_prepare(transaction, false);" in prepare_case


def test_tres_active_monitor_uses_raw_can_edges_only_as_sampling_hints():
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  power_source = (PANDA_ROOT / "board/sys/power_saving.h").read_text()

  settle_path = main_source.split("wake_monitor_som_off_countdown == 0U", 1)[1].split("} else {", 1)[0]
  assert "offline_wake_raw_can_exti_arm();" in settle_path
  assert "offline_wake_raw_can_edge_hint_ready(" in power_source
  raw_irq = power_source.split("offline_wake_raw_can_exti_irq_handler", 1)[1].split("offline_wake_raw_can_exti_init", 1)[0]
  assert "wake_monitor_raw_can_edge_pending = true;" in raw_irq
  assert "wake_monitor_can_activity_pending = true;" not in raw_irq
  assert "wake_can_trace_set_source" not in raw_irq
  assert "wake_debug_can_exti(pending);" in power_source
  assert "offline_wake_raw_can_exti_disarm();" in power_source
  assert "REGISTER_INTERRUPT(EXTI9_5_IRQn" in power_source
  assert "REGISTER_INTERRUPT(EXTI15_10_IRQn" in power_source


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
  assert "WAKE_MONITOR_SOM_OFF_SETTLE_S" in heartbeat_transition

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
  assert "queue_for_host = !wake_monitor_enabled || !wake_monitor_som_off_seen" in fdcan_source


def test_tesla_wake_event_latches_during_shutdown_settle_for_deferred_dispatch():
  source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  wake_latch = source.split("const uint8_t tesla_source =", 1)[1].split("#endif", 1)[0]

  assert "tesla_wake_source(&to_push, can_number)" in wake_latch
  assert "bootkick_tesla_event_should_latch(" in wake_latch
  assert "wake_monitor_som_off_seen || (wake_monitor_status.state == WAKE_MONITOR_STATE_PREPARED)" in wake_latch
  assert "wake_monitor_som_off_ready &&" not in wake_latch

  # A same-session heartbeat during shutdown cleanup must not erase the event;
  # only a new Linux session resolves the committed transaction.
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  assert "wake_monitor_heartbeat_result(" in main_source
  assert "wake_monitor_status.host_session, wake_monitor_status.committed_host_session" in main_source
  assert "wake_monitor_can_armed ? tesla_wake_source" not in wake_latch


def test_background_can_requires_a_sustained_rate_increase_to_request_wake():
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  policy_source = (PANDA_ROOT / "board/drivers/offline_wake_source_policy.h").read_text()

  assert "wake_monitor_can_activity_pending = true;" not in fdcan_source
  assert "offline_wake_can_rate_increase(" in main_source
  assert "offline_wake_can_rate_confirm_step(" in main_source
  assert "bootkick_can_activity_ready(" in main_source
  assert "WAKE_MONITOR_CAN_ACTIVITY_CONFIRM_S" in policy_source
  assert "wake_monitor_can_wake_requested = true;" in main_source
  rate_detector = main_source.split("bool can_rate_candidate = false;", 1)[1].split(
    "if (bootkick_tesla_event_ready(", 1
  )[0]
  assert "wake_monitor_som_off_ready && wake_monitor_can_armed" in rate_detector


def test_shutdown_guard_learns_sleep_baseline_instead_of_freezing_live_traffic():
  source = (PANDA_ROOT / "board/main.c").read_text()
  heartbeat_transition = source.split("if (wake_monitor_enabled && wake_monitor_committed) {", 1)[1].split(
    "if (bootkick_tesla_event_ready(", 1
  )[0]

  assert "OFFLINE_WAKE_CAN_BASELINE_UNSET" in heartbeat_transition
  assert "offline_wake_can_sleep_baseline_step(" in heartbeat_transition
  assert "wake_monitor_can_baseline[i] = rx_per_bus[i];" not in heartbeat_transition
