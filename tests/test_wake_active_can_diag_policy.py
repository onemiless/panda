import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_active_can_snapshot_tag_irq_first_rx_and_peak_policy(tmp_path):
  source = tmp_path / "wake_active_can_diag_policy_test.c"
  executable = tmp_path / "wake_active_can_diag_policy_test"
  source.write_text(
    """
    #include <assert.h>
    #include "board/drivers/wake_active_can_diag_policy.h"

    int main(void) {
      const uint32_t flags = 0x0007FFFFU;
      const uint32_t snapshot = wake_active_can_diag_make(flags);
      assert(wake_active_can_diag_valid(snapshot));
      assert((snapshot & WAKE_ACTIVE_CAN_DIAG_PAYLOAD_MASK) ==
             (flags & WAKE_ACTIVE_CAN_DIAG_PAYLOAD_MASK));
      assert(!wake_active_can_diag_valid(0x00000001U));

      assert(wake_active_can_diag_latch_irq(0x12345678U, 1U) == 0x12345678U);
      assert(wake_active_can_diag_latch_irq(snapshot, 3U) == snapshot);
      const uint32_t bus1_seen = wake_active_can_diag_latch_irq(snapshot, 1U);
      assert(!wake_active_can_diag_irq_seen(bus1_seen, 0U));
      assert(wake_active_can_diag_irq_seen(bus1_seen, 1U));
      assert(!wake_active_can_diag_irq_seen(bus1_seen, 2U));

      const uint32_t first_rx = wake_active_can_diag_pack_first_rx(1U, 0x122U);
      assert(wake_active_can_diag_first_rx_valid(first_rx));
      assert(((first_rx >> 29U) & 0x3U) == 1U);
      assert((first_rx & 0x1FFFFFFFU) == 0x122U);

      assert(wake_active_can_diag_peak(0U, 94U) == 94U);
      assert(wake_active_can_diag_peak(94U, 12U) == 94U);
      assert(wake_active_can_diag_peak(94U, 120U) == 120U);
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
