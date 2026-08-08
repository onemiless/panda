import os
from pathlib import Path
import subprocess
import sys

import pytest

from panda import Panda, McuType


PANDA_ROOT = Path(__file__).resolve().parents[1]


class FakeFlasherHandle:
  def __init__(self):
    self.control_writes = []
    self.bulk_bytes = 0

  def controlRead(self, request_type, request, value, index, length):
    assert request == 0xB0
    assert length == 0xC
    return b"\x00" * 4 + b"\xde\xad\xd0\x0d" + b"\x00" * 4

  def controlWrite(self, request_type, request, value, index, data, **kwargs):
    self.control_writes.append((request, value, index, data, kwargs))

  def bulkWrite(self, endpoint, data):
    assert endpoint == 2
    self.bulk_bytes += len(data)
    return len(data)


def test_h7_flash_layout_policy_compiles_and_enforces_reserved_sectors(tmp_path):
  source = tmp_path / "flash_layout_test.c"
  executable = tmp_path / "flash_layout_test"
  source.write_text(
    """
    #include <assert.h>
    #include "board/stm32h7/flash_layout.h"

    int main(void) {
      assert(APP_START_ADDRESS == 0x08020000U);
      assert(APP_END_ADDRESS == 0x080C0000U);
      assert(WAKE_JOURNAL_START == 0x080C0000U);
      assert(WAKE_JOURNAL_END == 0x080E0000U);
      assert(PROVISION_SECTOR_START == 0x080E0000U);
      assert(APP_MAX_SIZE == 0xA0000U);

      assert(!flash_app_sector_allowed(0U));
      assert(flash_app_sector_allowed(1U));
      assert(flash_app_sector_allowed(5U));
      assert(!flash_app_sector_allowed(6U));
      assert(!flash_app_sector_allowed(7U));

      assert(!flash_sector_erase_allowed(0U));
      assert(flash_sector_erase_allowed(1U));
      assert(flash_sector_erase_allowed(6U));
      assert(!flash_sector_erase_allowed(7U));

      assert(flash_app_write_allowed(APP_START_ADDRESS, APP_MAX_SIZE));
      assert(flash_app_write_allowed(APP_END_ADDRESS, 0U));
      assert(!flash_app_write_allowed(APP_END_ADDRESS, 4U));
      assert(!flash_app_write_allowed(APP_START_ADDRESS - 4U, 4U));
      assert(!flash_app_write_allowed(0xFFFFFFFCU, 8U));

      assert(flash_app_signed_length_valid(APP_MAX_SIZE - 128U, 128U));
      assert(!flash_app_signed_length_valid(APP_MAX_SIZE - 127U, 128U));
      assert(!flash_app_signed_length_valid(7U, 128U));
      return 0;
    }
    """
  )

  subprocess.run(
    [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror", "-I", str(PANDA_ROOT), str(source), "-o", str(executable)],
    check=True,
  )
  subprocess.run([str(executable)], check=True)


def test_h7_python_flasher_uses_only_application_sectors_at_exact_boundary():
  handle = FakeFlasherHandle()
  app_capacity = McuType.H7.config.app_end_address - McuType.H7.config.app_address

  Panda.flash_static(handle, bytes(app_capacity), McuType.H7)

  erased_sectors = [value for request, value, _, _, _ in handle.control_writes if request == 0xB2]
  assert erased_sectors == [1, 2, 3, 4, 5]
  assert handle.bulk_bytes == app_capacity


def test_h7_python_flasher_rejects_sector6_before_unlocking():
  handle = FakeFlasherHandle()
  app_capacity = McuType.H7.config.app_end_address - McuType.H7.config.app_address

  with pytest.raises(AssertionError, match="reserved flash"):
    Panda.flash_static(handle, bytes(app_capacity + 1), McuType.H7)

  assert handle.control_writes == []
  assert handle.bulk_bytes == 0


def test_signer_rejects_image_after_signature_crosses_limit(tmp_path):
  sign_script = PANDA_ROOT / "board/crypto/sign.py"
  debug_key = PANDA_ROOT / "board/certs/debug"
  max_size = 0x1000
  signing_overhead = 8 + 128

  exact_input = tmp_path / "exact.bin"
  exact_output = tmp_path / "exact.bin.signed"
  exact_input.write_bytes(bytes(max_size - signing_overhead))
  env = {**os.environ, "SETLEN": "1", "MAX_SIZE": hex(max_size)}
  subprocess.run([sys.executable, str(sign_script), str(exact_input), str(exact_output), str(debug_key)], check=True, env=env)
  assert exact_output.stat().st_size == max_size

  oversized_input = tmp_path / "oversized.bin"
  oversized_output = tmp_path / "oversized.bin.signed"
  oversized_input.write_bytes(bytes(max_size - signing_overhead + 1))
  result = subprocess.run(
    [sys.executable, str(sign_script), str(oversized_input), str(oversized_output), str(debug_key)],
    check=False,
    env=env,
    capture_output=True,
    text=True,
  )
  assert result.returncode != 0
  assert "exceeds reserved application space" in result.stderr
  assert not oversized_output.exists()
