import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_append_only_wake_journal_policy(tmp_path):
  source = tmp_path / "wake_journal_policy_test.c"
  executable = tmp_path / "wake_journal_policy_test"
  source.write_text(
    r"""
    #include <assert.h>
    #include <string.h>
    #include "board/drivers/wake_journal_policy.h"

    int main(void) {
      _Static_assert(sizeof(wake_journal_record_t) == 32U, "record must be one H7 flashword");
      _Static_assert(sizeof(wake_journal_info_t) == 32U, "info must fit one USB packet");

      wake_journal_record_t slots[4];
      memset(slots, 0xFF, sizeof(slots));
      wake_journal_info_t info = wake_journal_scan(slots, 4U);
      assert(info.used_slots == 0U);
      assert(info.valid_records == 0U);
      assert(info.next_sequence == 0U);
      assert(info.flags == 0U);

      wake_journal_record_t checkpoint;
      wake_journal_build_checkpoint(&checkpoint, 8U, 8U, WAKE_MONITOR_STATE_ARMED,
                                    0x3FU, 0x12345678U, 0x87654321U, 45U);
      assert(wake_journal_record_valid(&checkpoint));
      assert(((checkpoint.meta >> 8U) & 0xFU) == WAKE_JOURNAL_RECORD_CHECKPOINT);
      assert(((checkpoint.meta >> 16U) & 0xFFU) == WAKE_MONITOR_STATE_ARMED);
      assert(((checkpoint.meta >> 24U) & 0xFFU) == 0x3FU);
      assert(checkpoint.value0 == 0x12345678U);
      assert(checkpoint.value1 == 0x87654321U);
      assert(checkpoint.value2 == 45U);

      const uint8_t door[8] = {0U, 0U, 1U, 2U, 3U, 4U, 5U, 6U};
      wake_journal_build_event(&slots[0], 10U, 10U, WAKE_JOURNAL_SOURCE_TESLA_DOOR,
                               0x34U, 1U, 1U, 8U, 0x102U, door);
      assert(wake_journal_record_valid(&slots[0]));
      assert(slots[0].value0 == 0x102U);

      wake_journal_build_result(&slots[1], 11U, 10U, WAKE_JOURNAL_SOURCE_TESLA_DOOR,
                                true, 1U, false, true, true, true,
                                0x34U, 0x41U, 0x12345678U);
      assert(wake_journal_record_valid(&slots[1]));
      info = wake_journal_scan(slots, 4U);
      assert(info.used_slots == 2U);
      assert(info.valid_records == 2U);
      assert(info.next_sequence == 12U);

      // A torn flashword is consumed but invalid. Scanning continues after it.
      memset(&slots[2], 0xFF, sizeof(slots[2]));
      slots[2].magic = WAKE_JOURNAL_MAGIC;
      wake_journal_build_event(&slots[3], 12U, 12U, WAKE_JOURNAL_SOURCE_TESLA_POWER,
                               0x34U, 0U, 2U, 8U, 0x221U, door);
      info = wake_journal_scan(slots, 4U);
      assert(info.used_slots == 4U);
      assert(info.valid_records == 3U);
      assert(info.next_sequence == 13U);
      assert((info.flags & WAKE_JOURNAL_FLAG_FOREIGN_DATA) != 0U);
      assert((info.flags & WAKE_JOURNAL_FLAG_FULL) != 0U);

      // Corruption must be rejected without making an earlier slot reusable.
      slots[0].value1 ^= 1U;
      assert(!wake_journal_record_valid(&slots[0]));
      info = wake_journal_scan(slots, 4U);
      assert(info.used_slots == 4U);
      assert(info.valid_records == 2U);
      return 0;
    }
    """
  )
  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
