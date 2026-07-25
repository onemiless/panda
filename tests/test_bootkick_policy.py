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

      assert(!bootkick_tres_early_reset_ready(false, 1U, 0U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 0U, 0U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 1U, false, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, true, 0U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 1U, false, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, true, false, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, false, true, false));
      assert(!bootkick_tres_early_reset_ready(true, 1U, 0U, false, 0U, false, false, true));

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

      // A Tesla frame may arrive before CAN arming completes. Keep it pending
      // until the 1 Hz owner has completed arming, then consume it exactly once.
      assert(!bootkick_tesla_event_ready(true, true, false, true, false));
      assert(bootkick_tesla_event_ready(true, true, true, true, false));
      assert(!bootkick_tesla_event_ready(true, true, true, false, false));
      assert(!bootkick_tesla_event_ready(true, true, true, true, true));
      assert(!bootkick_tesla_event_ready(false, true, true, true, false));
      assert(!bootkick_tesla_event_ready(true, false, true, true, false));

      // Tesla UI_warning must show a real sequential counter and an open
      // door on Party bus before it can wake a powered-down SoM.
      assert(tesla_wake_counter_valid(0, 1));
      assert(tesla_wake_counter_valid(15, 0));
      assert(!tesla_wake_counter_valid(-1, 0));
      assert(!tesla_wake_counter_valid(1, 1));
      assert(!tesla_wake_counter_valid(1, 3));
      const uint8_t closed_frame[7] = {0U, 4U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t open_frame[7] = {0U, 5U, 0U, 0x10U, 0U, 0U, 0U};
      assert(tesla_ui_warning_counter(open_frame) == 5);
      assert(!tesla_ui_warning_door_open(closed_frame));
      assert(tesla_ui_warning_door_open(open_frame));
      assert(tesla_door_wake_ready(0U, 7U, 4, 5, true));
      assert(!tesla_door_wake_ready(1U, 7U, 4, 5, true));
      assert(!tesla_door_wake_ready(0U, 8U, 4, 5, true));
      assert(!tesla_door_wake_ready(0U, 7U, 4, 5, false));
      assert(!tesla_door_wake_ready(0U, 7U, 4, 6, true));
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
