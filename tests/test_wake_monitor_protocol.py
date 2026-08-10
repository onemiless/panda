import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_wake_monitor_transaction_and_session_policy(tmp_path):
  source = tmp_path / "wake_monitor_protocol_test.c"
  executable = tmp_path / "wake_monitor_protocol_test"
  source.write_text(
    """
    #include <assert.h>
    #include "board/drivers/wake_monitor_policy.h"

    int main(void) {
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_IDLE, 0U, 0x12345678U) == WAKE_MONITOR_PREPARE_START);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U) == WAKE_MONITOR_PREPARE_IDEMPOTENT);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x87654321U) == WAKE_MONITOR_PREPARE_CONFLICT);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_COMMITTED, 0x12345678U, 0x87654321U) == WAKE_MONITOR_PREPARE_CONFLICT);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_ARMED, 0x12345678U, 0x87654321U) == WAKE_MONITOR_PREPARE_CONFLICT);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_WAKING, 0x12345678U, 0x87654321U) == WAKE_MONITOR_PREPARE_CONFLICT);
      assert(wake_monitor_prepare_action(WAKE_MONITOR_STATE_IDLE, 0U, 0U) == WAKE_MONITOR_PREPARE_INVALID);

      assert(wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_IDLE, 0x12345678U, 0x12345678U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x87654321U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U, true, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U, false, false));

      assert(wake_monitor_heartbeat_result(true, true, 2U, 1U, true) == WAKE_MONITOR_HEARTBEAT_CONFIRMED);
      assert(wake_monitor_heartbeat_result(true, true, 2U, 1U, false) == WAKE_MONITOR_HEARTBEAT_UNATTRIBUTED);
      assert(wake_monitor_heartbeat_result(true, true, 1U, 1U, true) == WAKE_MONITOR_HEARTBEAT_IGNORE);
      assert(wake_monitor_heartbeat_result(true, false, 2U, 1U, true) == WAKE_MONITOR_HEARTBEAT_IGNORE);
      assert(wake_monitor_heartbeat_result(false, true, 2U, 1U, true) == WAKE_MONITOR_HEARTBEAT_IGNORE);
      assert(wake_monitor_heartbeat_result(true, true, 0U, 1U, true) == WAKE_MONITOR_HEARTBEAT_IGNORE);

      uint8_t cooldown = 2U;
      assert(!wake_monitor_failure_cooldown_step(&cooldown));
      assert(cooldown == 1U);
      assert(wake_monitor_failure_cooldown_step(&cooldown));
      assert(cooldown == 0U);
      assert(wake_monitor_unattributed_result(WAKE_MONITOR_RESULT_NONE) == WAKE_MONITOR_RESULT_UNATTRIBUTED);
      assert(wake_monitor_unattributed_result(WAKE_MONITOR_RESULT_FAILED) == WAKE_MONITOR_RESULT_FAILED);
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
