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
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_arming_wake_monitor_does_not_clear_latched_success():
  source = (PANDA_ROOT / "board/main_comms.h").read_text()
  arm_case = source.split("case PANDA_REQUEST_ENABLE_WAKE_MONITOR:", 1)[1].split("break;", 1)[0]
  assert "wake_debug_clear_success();" not in arm_case
  assert arm_case.index("set_safety_mode(SAFETY_SILENT, 0U);") < arm_case.index("set_power_save_state(false);")


def test_wake_monitor_keeps_fdcan_active_after_host_shutdown():
  source = (PANDA_ROOT / "board/main.c").read_text()
  heartbeat_transition = source.split("if (wake_monitor_enabled) {", 1)[1].split(
    "if (bootkick_tesla_event_ready(", 1
  )[0]

  # The host is genuinely powered off, but panda must remain in receive-only
  # mode. Entering STOP disables FDCAN parsing and makes the Tesla frame/rate
  # wake detectors below this transition unreachable.
  assert "offline_wake_policy_after_heartbeat_loss(hw_type == HW_TYPE_TRES)" in heartbeat_transition
  assert "if (policy.keep_can_active)" in heartbeat_transition
  assert "WAKE_MONITOR_SOM_OFF_SETTLE_S" in heartbeat_transition

  shallow_idle = source.split("if (wake_monitor_enabled && wake_monitor_som_off_seen) {", 1)[1].split("} else {", 1)[0]
  assert "SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;" in shallow_idle
  assert "__WFI();" in shallow_idle


def test_offline_monitor_does_not_buffer_stale_can_for_restarted_host():
  main_source = (PANDA_ROOT / "board/main.c").read_text()
  fdcan_source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()

  assert main_source.count("can_clear(&can_rx_q);") >= 2
  assert "queue_for_host = !wake_monitor_enabled || !wake_monitor_som_off_seen" in fdcan_source


def test_tesla_wake_event_latches_only_after_shutdown_settle():
  source = (PANDA_ROOT / "board/drivers/fdcan.h").read_text()
  wake_latch = source.split("const uint8_t tesla_source =", 1)[1].split("#endif", 1)[0]

  assert "wake_monitor_enabled ? tesla_wake_source" in wake_latch
  assert "wake_monitor_som_off_ready &&" in wake_latch
  assert "wake_monitor_can_armed ? tesla_wake_source" not in wake_latch
