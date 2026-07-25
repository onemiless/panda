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

      assert(bootkick_wake_request_needs_dispatch(true, true, true, false, false, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(false, true, true, false, false, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, false, true, false, false, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, true, false, false, false, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, true, false, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, false, true, 0U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, false, false, 1U, 0x3FU));
      assert(!bootkick_wake_request_needs_dispatch(true, true, true, false, false, 0U, 0x3EU));
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
