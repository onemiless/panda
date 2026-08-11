import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_tres_early_reset_requires_a_completed_unanswered_first_pulse(tmp_path):
  source = tmp_path / "bootkick_policy_test.c"
  executable = tmp_path / "bootkick_policy_test"
  source.write_text(
    """
    #include <assert.h>
    #include "board/drivers/bootkick_policy.h"

    int main(void) {
      assert(bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, false, false, false));
      // Tres SOM GPIO can remain high after Linux powers off. It must not
      // block reset recovery after an unanswered full BOOTKICK pulse.
      assert(bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, false, true, false));

      assert(!bootkick_tres_early_reset_ready(false, 1U, 0U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 0U, 0U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 1U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, true, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 1U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, true, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, false, false, true));

      // A latched physical CAN event after the shutdown guard is sufficient.
      assert(bootkick_can_activity_ready(true, true, true, true, false));
      assert(!bootkick_can_activity_ready(false, true, true, true, false));
      assert(!bootkick_can_activity_ready(true, false, true, true, false));
      assert(!bootkick_can_activity_ready(true, true, false, true, false));
      assert(!bootkick_can_activity_ready(true, true, true, false, false));
      assert(!bootkick_can_activity_ready(true, true, true, true, true));

      assert(bootkick_heartbeat_confirms_wake(true, true));
      assert(!bootkick_heartbeat_confirms_wake(false, true));
      assert(!bootkick_heartbeat_confirms_wake(true, false));
      assert(bootkick_success_source_stage(true, 0x34U, 0x41U) == 0x34U);
      assert(bootkick_success_source_stage(true, 0x35U, 0x3DU) == 0x35U);
      assert(bootkick_success_source_stage(false, 0U, 0x38U) == 0x38U);

      // A live pending dispatch is authoritative even if another diagnostic
      // stage was written between the CAN interrupt and the 1 Hz monitor.
      assert(bootkick_wake_request_needs_dispatch(true, true, true, true, false, false, 0U, 0x11U));
      // Recover both the legacy stranded snapshot and a pending snapshot
      // restored after a Panda reset.
      assert(bootkick_wake_request_needs_dispatch(true, true, true, false, false, false, 0U, 0x3FU));
      assert(bootkick_wake_request_needs_dispatch(true, true, true, false, false, false, 0U, 0x42U));
      assert(bootkick_wake_request_needs_dispatch(true, true, true, false, false, false, 0U, 0x43U));
      assert(!bootkick_wake_request_needs_dispatch(false, true, true, true, false, false, 0U, 0x42U));
      assert(!bootkick_wake_request_needs_dispatch(true, false, true, true, false, false, 0U, 0x42U));
      assert(!bootkick_wake_request_needs_dispatch(true, true, false, true, false, false, 0U, 0x42U));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, true, true, false, 0U, 0x42U));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, true, false, true, 0U, 0x42U));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, true, false, false, 1U, 0x42U));
      // A completed failure is terminal and must not be retried forever.
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, false, false, false, 0U, 0x3EU));

      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_prestop_ignition_wake_waits_for_som_power_off(tmp_path):
  source = tmp_path / "prestop_ignition_wake_test.c"
  executable = tmp_path / "prestop_ignition_wake_test"
  source.write_text(
    """
    #include <assert.h>
    #include "board/drivers/bootkick_policy.h"

    int main(void) {
      assert(bootkick_restore_waits_for_som_off(0x32U, 0U));
      assert(!bootkick_restore_waits_for_som_off(0x32U, 1U));
      assert(!bootkick_restore_waits_for_som_off(0x34U, 0U));

      uint8_t off_confirm = BOOTKICK_SOM_OFF_CONFIRM_S;
      uint8_t heartbeat_absent = BOOTKICK_SOM_OFF_FALLBACK_S;
      assert(bootkick_deferred_wake_step(false, true, &off_confirm, &heartbeat_absent) == BOOTKICK_DEFERRED_WAIT);
      assert(off_confirm == BOOTKICK_SOM_OFF_CONFIRM_S);

      assert(bootkick_deferred_wake_step(false, false, &off_confirm, &heartbeat_absent) == BOOTKICK_DEFERRED_WAIT);
      assert(off_confirm == 1U);
      assert(bootkick_deferred_wake_step(false, false, &off_confirm, &heartbeat_absent) == BOOTKICK_DEFERRED_START);
      assert(off_confirm == 0U);

      off_confirm = BOOTKICK_SOM_OFF_CONFIRM_S;
      heartbeat_absent = 1U;
      assert(bootkick_deferred_wake_step(true, true, &off_confirm, &heartbeat_absent) == BOOTKICK_DEFERRED_ALREADY_ALIVE);
      assert(heartbeat_absent == BOOTKICK_SOM_OFF_FALLBACK_S);

      off_confirm = BOOTKICK_SOM_OFF_CONFIRM_S;
      heartbeat_absent = 1U;
      assert(bootkick_deferred_wake_step(false, true, &off_confirm, &heartbeat_absent) == BOOTKICK_DEFERRED_START);
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_tres_unanswered_wake_resets_then_generates_fresh_edge():
  source = (PANDA_ROOT / "board/drivers/bootkick.h").read_text()

  assert "#define BOOTKICK_WAKE_PULSE_S 30U" in source
  assert "(hw_type == HW_TYPE_TRES) ? 0U : BOOTKICK_WAKE_RETRY_DELAY_S" in source

  reset_recovery = source.split("if (bootkick_wake_confirmation_pending && !bootkick_wake_waiting_for_som_off", 2)[2]
  reset_recovery = reset_recovery.split("if (bootkick_reset_pulse_requested", 1)[0]
  assert "boot_reset_countdown = 5U;" in reset_recovery
  assert "boot_state = BOOT_STANDBY;" in reset_recovery
  assert "bootkick_wake_post_reset_countdown = BOOTKICK_WAKE_POST_RESET_RELEASE_S;" in reset_recovery
  assert "bootkick_start_wake_pulse(0x41U);" in reset_recovery

  heartbeat_path = source.split("} else if (recent_heartbeat) {", 1)[1].split("} else if", 1)[0]
  assert "bootkick_heartbeat_confirms_wake" in heartbeat_path
  assert "wake_debug_latch_success(bootkick_wake_trigger_stage);" in heartbeat_path
  assert "bootkick_clear_wake_confirmation();" in heartbeat_path


def test_single_uart_byte_cannot_permanently_block_tres_reset():
  source = (PANDA_ROOT / "board/drivers/bootkick.h").read_text()

  assert "BOOTKICK_UART_PROGRESS_TIMEOUT_S" in source
  assert "bootkick_wake_uart_progress_countdown -= 1U;" in source
  assert "const bool uart_progress_active = bootkick_wake_uart_progress_countdown > 0U;" in source
  reset_recovery = source.split("if (bootkick_wake_confirmation_pending && !bootkick_wake_waiting_for_som_off", 2)[2]
  reset_recovery = reset_recovery.split("if (bootkick_reset_pulse_requested", 1)[0]
  assert "bootkick_wake_uart_progress_countdown == 0U" in reset_recovery
  assert "!bootkick_wake_uart_seen" not in reset_recovery
