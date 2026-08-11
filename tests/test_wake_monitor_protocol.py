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

      assert(wake_monitor_prepare_flags(true, 0x11111111U) ==
             (WAKE_MONITOR_STATUS_FLAG_RX_ARMED | WAKE_MONITOR_STATUS_FLAG_CAN_HEALTHY));
      assert(wake_monitor_prepare_flags(true, 0U) == WAKE_MONITOR_STATUS_FLAG_CAN_HEALTHY);
      assert(wake_monitor_prepare_flags(false, 0x11111111U) == 0U);

      assert(wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U,
                                         0x11111111U, 0x11111111U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_IDLE, 0x12345678U, 0x12345678U,
                                          0x11111111U, 0x11111111U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x87654321U,
                                          0x11111111U, 0x11111111U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U,
                                          0x11111111U, 0x22222222U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U,
                                          0U, 0U, false, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U,
                                          0x11111111U, 0x11111111U, true, true));
      assert(!wake_monitor_commit_allowed(WAKE_MONITOR_STATE_PREPARED, 0x12345678U, 0x12345678U,
                                          0x11111111U, 0x11111111U, false, false));

      const uint32_t prepared_rx[3] = {10U, 20U, 30U};
      const uint32_t prepared_lost[3] = {0U, 0U, 0U};
      const uint32_t prepared_resets[3] = {1U, 1U, 1U};
      uint32_t current_rx[3] = {10U, 20U, 30U};
      uint32_t current_lost[3] = {0U, 0U, 0U};
      uint32_t current_resets[3] = {1U, 1U, 1U};
      assert(wake_monitor_rx_snapshot_clean(prepared_rx, current_rx, prepared_lost, current_lost,
                                            prepared_resets, current_resets, 4U, 4U, 3U));
      current_rx[1] += 1U;
      assert(!wake_monitor_rx_snapshot_clean(prepared_rx, current_rx, prepared_lost, current_lost,
                                             prepared_resets, current_resets, 4U, 4U, 3U));
      current_rx[1] -= 1U;
      current_lost[2] += 1U;
      assert(!wake_monitor_rx_snapshot_clean(prepared_rx, current_rx, prepared_lost, current_lost,
                                             prepared_resets, current_resets, 4U, 4U, 3U));
      current_lost[2] -= 1U;
      current_resets[0] += 1U;
      assert(!wake_monitor_rx_snapshot_clean(prepared_rx, current_rx, prepared_lost, current_lost,
                                             prepared_resets, current_resets, 4U, 4U, 3U));
      current_resets[0] -= 1U;
      assert(!wake_monitor_rx_snapshot_clean(prepared_rx, current_rx, prepared_lost, current_lost,
                                             prepared_resets, current_resets, 4U, 5U, 3U));

      current_rx[1] += 1U;
      assert(wake_monitor_rx_integrity_clean(prepared_lost, current_lost,
                                             prepared_resets, current_resets, 4U, 4U, 3U));
      current_lost[1] += 1U;
      assert(!wake_monitor_rx_integrity_clean(prepared_lost, current_lost,
                                              prepared_resets, current_resets, 4U, 4U, 3U));
      current_lost[1] -= 1U;
      assert(!wake_monitor_rx_integrity_clean(prepared_lost, current_lost,
                                              prepared_resets, current_resets, 4U, 5U, 3U));

      assert(wake_monitor_offline_fault_ready(true, true, true, true, true, false, false));
      assert(wake_monitor_offline_fault_ready(true, true, false, false, true, false, false));
      assert(wake_monitor_offline_fault_ready(true, true, false, true, false, false, false));
      assert(!wake_monitor_offline_fault_ready(true, true, false, true, true, false, false));
      assert(!wake_monitor_offline_fault_ready(false, true, true, true, true, false, false));
      assert(!wake_monitor_offline_fault_ready(true, false, true, true, true, false, false));
      assert(!wake_monitor_offline_fault_ready(true, true, true, true, true, true, false));
      assert(!wake_monitor_offline_fault_ready(true, true, true, true, true, false, true));

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
