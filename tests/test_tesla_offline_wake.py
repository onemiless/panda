import os
from pathlib import Path
import subprocess


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_real_tesla_frame_sequences(tmp_path):
  source = tmp_path / "tesla_offline_wake_test.c"
  executable = tmp_path / "tesla_offline_wake_test"
  source.write_text(
    r"""
    #include <assert.h>
    #include "board/drivers/tesla_offline_wake.h"

    static uint8_t source_for(tesla_offline_wake_state_t *state, uint16_t address,
                              uint8_t bus, uint8_t len, const uint8_t *data) {
      return tesla_offline_wake_step(state, address, bus, len, data).source;
    }

    int main(void) {
      const uint8_t latch_closed[8] = {0U, 0x01U, 0U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t latch_open[8] = {0U, 0x00U, 0U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t handle_pulled[8] = {0U, 0x05U, 0U, 0U, 0U, 0U, 0U, 0U};

      tesla_offline_wake_state_t direct = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&direct, 0x102U, 0U, 8U, latch_closed) == TESLA_WAKE_SOURCE_NONE);
      assert(direct.front_door_known_mask == 0U);
      assert(source_for(&direct, 0x102U, 1U, 8U, latch_open) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&direct, 0x102U, 1U, 8U, latch_closed) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&direct, 0x102U, 1U, 8U, latch_open) == TESLA_WAKE_SOURCE_DOOR);
      assert(source_for(&direct, 0x102U, 1U, 8U, latch_open) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&direct, 0x103U, 1U, 8U, latch_closed) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&direct, 0x103U, 1U, 8U, latch_open) == TESLA_WAKE_SOURCE_DOOR);

      tesla_offline_wake_state_t handle = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&handle, 0x102U, 0U, 8U, handle_pulled) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&handle, 0x102U, 1U, 8U, handle_pulled) == TESLA_WAKE_SOURCE_DOOR);

      const uint8_t ui_closed_4[7] = {0x18U, 0x04U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t ui_open_5[7] = {0x29U, 0x05U, 0U, 0x10U, 0U, 0U, 0U};
      const uint8_t ui_open_6[7] = {0x2AU, 0x06U, 0U, 0x10U, 0U, 0U, 0U};
      const uint8_t ui_open_6_bad[7] = {0x00U, 0x06U, 0U, 0x10U, 0U, 0U, 0U};
      const uint8_t ui_closed_7[7] = {0x1BU, 0x07U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t ui_closed_8[7] = {0x1CU, 0x08U, 0U, 0U, 0U, 0U, 0U};
      const uint8_t ui_open_9[7] = {0x2DU, 0x09U, 0U, 0x10U, 0U, 0U, 0U};

      tesla_offline_wake_state_t ui = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&ui, 0x311U, 0U, 7U, ui_closed_4) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&ui, 0x311U, 0U, 7U, ui_open_5) == TESLA_WAKE_SOURCE_DOOR);
      assert(source_for(&ui, 0x311U, 0U, 7U, ui_open_6) == TESLA_WAKE_SOURCE_NONE);

      tesla_offline_wake_state_t initial_open = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&initial_open, 0x311U, 0U, 7U, ui_open_5) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&initial_open, 0x311U, 0U, 7U, ui_open_6) == TESLA_WAKE_SOURCE_DOOR);
      assert(source_for(&initial_open, 0x311U, 0U, 7U, ui_open_6) == TESLA_WAKE_SOURCE_NONE);

      tesla_offline_wake_state_t invalid = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&invalid, 0x311U, 1U, 7U, ui_closed_4) == TESLA_WAKE_SOURCE_NONE);
      assert(invalid.ui_counter == -1);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_closed_4) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_open_6_bad) == TESLA_WAKE_SOURCE_NONE);
      assert(invalid.ui_counter == 4);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_open_6) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_closed_7) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_closed_8) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&invalid, 0x311U, 0U, 7U, ui_open_9) == TESLA_WAKE_SOURCE_DOOR);

      const uint8_t power_off_0[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0x00U, 0x23U};
      const uint8_t power_conditioning_1[8] = {0x20U, 0U, 0U, 0U, 0U, 0U, 0x10U, 0x53U};
      const uint8_t power_accessory_2[8] = {0x40U, 0U, 0U, 0U, 0U, 0U, 0x20U, 0x83U};
      const uint8_t power_drive_3[8] = {0x60U, 0U, 0U, 0U, 0U, 0U, 0x30U, 0xB3U};
      const uint8_t power_drive_3_bad[8] = {0x60U, 0U, 0U, 0U, 0U, 0U, 0x30U, 0U};

      tesla_offline_wake_state_t power = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&power, 0x221U, 0U, 8U, power_off_0) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&power, 0x221U, 0U, 8U, power_conditioning_1) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&power, 0x221U, 0U, 8U, power_accessory_2) == TESLA_WAKE_SOURCE_NONE);
      assert(source_for(&power, 0x221U, 0U, 8U, power_drive_3) == TESLA_WAKE_SOURCE_POWER);

      tesla_offline_wake_state_t bad_power = TESLA_OFFLINE_WAKE_STATE_INITIALIZER;
      assert(source_for(&bad_power, 0x221U, 1U, 8U, power_off_0) == TESLA_WAKE_SOURCE_NONE);
      assert(bad_power.power_counter == -1);
      assert(source_for(&bad_power, 0x221U, 0U, 8U, power_drive_3_bad) == TESLA_WAKE_SOURCE_NONE);
      assert(bad_power.power_counter == -1);
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)
