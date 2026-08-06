import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_offline_wake_source_masks_include_sbu_and_oriented_bus1(tmp_path):
  source = tmp_path / "offline_wake_source_policy_test.c"
  executable = tmp_path / "offline_wake_source_policy_test"
  source.write_text(
    """
    #include <assert.h>
    #include <stdbool.h>
    #include "board/drivers/offline_wake_source_policy.h"

    int main(void) {
      assert(offline_wake_can_exti_line(false) == (1UL << 5));
      assert(offline_wake_can_exti_line(true) == (1UL << 12));
      assert(offline_wake_exti_lines(false) == ((1UL << 1) | (1UL << 4) | (1UL << 5)));
      assert(offline_wake_exti_lines(true) == ((1UL << 1) | (1UL << 4) | (1UL << 12)));
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
