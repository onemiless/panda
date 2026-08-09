import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_wake_event_trace_packing(tmp_path):
  source = tmp_path / "wake_event_trace_policy_test.c"
  executable = tmp_path / "wake_event_trace_policy_test"
  source.write_text(
    r"""
    #include <assert.h>
    #include "board/drivers/wake_event_trace_policy.h"

    int main(void) {
      assert(wake_event_trace_power_code(0U, 0U) == 0x1U);
      assert(wake_event_trace_power_code(2U, 1U) == 0xAU);
      assert(wake_event_trace_power_code(2U, 3U) == 0xCU);

      uint32_t sequence = 0U;
      assert(wake_event_trace_append(sequence, 0xAU) == 0xAU);
      sequence = wake_event_trace_append(sequence, 0xAU);
      sequence = wake_event_trace_append(sequence, WAKE_EVENT_TRACE_LEFT_DOOR);
      assert(sequence == 0xDAU);
      assert(wake_event_trace_last(sequence) == WAKE_EVENT_TRACE_LEFT_DOOR);

      sequence = 0x87654321U;
      assert(wake_event_trace_append(sequence, 0xFU) == sequence);

      uint8_t power_meta = wake_event_trace_power_prearm(1U);
      assert(wake_event_trace_prearm_seen(power_meta));
      assert(wake_event_trace_prearm_state(power_meta) == 1U);
      for (uint8_t i = 0U; i < 40U; i++) {
        power_meta = wake_event_trace_increment_count(power_meta);
      }
      assert(wake_event_trace_count(power_meta) == 31U);

      uint8_t door_meta = wake_event_trace_door_prearm(true);
      assert(wake_event_trace_prearm_seen(door_meta));
      assert(wake_event_trace_prearm_binary_state(door_meta));
      door_meta = wake_event_trace_prepare_binary_postarm(door_meta);
      assert(wake_event_trace_postarm_binary_state(door_meta));
      door_meta = wake_event_trace_set_postarm_binary_state(door_meta, false);
      assert(!wake_event_trace_postarm_binary_state(door_meta));
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
