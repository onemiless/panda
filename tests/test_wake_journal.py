import binascii
from pathlib import Path

from panda import Panda


PANDA_ROOT = Path(__file__).resolve().parents[1]


class FakeHandle:
  def __init__(self, info: bytes, record: bytes):
    self.info = info
    self.record = record

  def controlRead(self, request_type, request, value, index, length, timeout=None):
    assert timeout == 15000
    assert request_type == Panda.REQUEST_IN
    assert index == 0
    if request == Panda.WAKE_JOURNAL_INFO_REQUEST:
      assert value == 0
      assert length == Panda.WAKE_JOURNAL_INFO_STRUCT.size
      return self.info
    assert request == Panda.WAKE_JOURNAL_RECORD_REQUEST
    assert value == 7
    assert length == Panda.WAKE_JOURNAL_RECORD_STRUCT.size
    return self.record


def test_wake_journal_decodes_info_event_and_crc():
  info_payload = Panda.WAKE_JOURNAL_INFO_STRUCT.pack(
    Panda.WAKE_JOURNAL_MAGIC, Panda.WAKE_JOURNAL_VERSION, 32,
    4096, 8, 7, 0x2, 12, 10, 0, 0,
  )
  auxiliary = 1 | (1 << 2) | (8 << 4) | (0x35 << 8)
  meta = Panda.WAKE_JOURNAL_VERSION | (1 << 8) | (6 << 12) | (auxiliary << 16)
  prefix = Panda.WAKE_JOURNAL_RECORD_STRUCT.pack(
    Panda.WAKE_JOURNAL_MAGIC, 10, 10, meta, 0x122,
    int.from_bytes(bytes.fromhex("00006c93"), "little"),
    int.from_bytes(bytes.fromhex("09005d00"), "little"), 0,
  )[:28]
  record_payload = prefix + (binascii.crc32(prefix) & 0xFFFFFFFF).to_bytes(4, "little")
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(info_payload, record_payload)

  assert panda.wake_journal_info() == {
    "magic": Panda.WAKE_JOURNAL_MAGIC,
    "version": 1,
    "record_size": 32,
    "capacity": 4096,
    "used_slots": 8,
    "valid_records": 7,
    "full": False,
    "foreign_data": True,
    "next_sequence": 12,
    "current_cycle": 10,
  }
  assert panda.wake_journal_record(7) == {
    "valid": True,
    "magic": Panda.WAKE_JOURNAL_MAGIC,
    "version": 1,
    "type": "event",
    "source": "canPrimary",
    "sequence": 10,
    "cycle": 10,
    "trigger_stage": 0x35,
    "logical_bus": 1,
    "physical_bus": 1,
    "length": 8,
    "can_id": 0x122,
    "data": "00006c9309005d00",
  }


def test_wake_journal_decodes_result_and_rejects_bad_crc():
  info_payload = bytes(Panda.WAKE_JOURNAL_INFO_STRUCT.size)
  auxiliary = 1 | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6)
  meta = Panda.WAKE_JOURNAL_VERSION | (2 << 8) | (2 << 12) | (auxiliary << 16)
  record_payload = Panda.WAKE_JOURNAL_RECORD_STRUCT.pack(
    Panda.WAKE_JOURNAL_MAGIC, 11, 10, meta, 0x34, 0x41, 0x1234, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(info_payload, record_payload)

  record = panda.wake_journal_record(7)
  assert not record["valid"]
  assert record["type"] == "result"
  assert record["source"] == "teslaPower"
  assert record["success"]
  assert record["attempts"] == 1
  assert record["reset_attempted"]
  assert record["heartbeat_seen"]
  assert record["trigger_stage"] == 0x34
  assert record["final_stage"] == 0x41
  assert record["reset_reason"] == 0x1234


def test_wake_journal_decodes_shutdown_checkpoint():
  info_payload = bytes(Panda.WAKE_JOURNAL_INFO_STRUCT.size)
  auxiliary = 4 | (0x3F << 8)
  meta = Panda.WAKE_JOURNAL_VERSION | (3 << 8) | (auxiliary << 16)
  prefix = Panda.WAKE_JOURNAL_RECORD_STRUCT.pack(
    Panda.WAKE_JOURNAL_MAGIC, 21, 20, meta, 0x12345678, 0x87654321, 45, 0,
  )[:28]
  record_payload = prefix + (binascii.crc32(prefix) & 0xFFFFFFFF).to_bytes(4, "little")
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(info_payload, record_payload)

  assert panda.wake_journal_record(7) == {
    "valid": True,
    "magic": Panda.WAKE_JOURNAL_MAGIC,
    "version": 1,
    "type": "checkpoint",
    "source": "unknown",
    "sequence": 21,
    "cycle": 20,
    "state": 4,
    "stage": 0x3F,
    "transaction": 0x12345678,
    "host_session": 0x87654321,
    "off_seconds": 45,
  }


def test_wake_journal_uses_atomic_h7_flashword_and_persists_phases():
  journal = (PANDA_ROOT / "board/drivers/wake_journal.h").read_text()
  llflash = (PANDA_ROOT / "board/stm32h7/llflash.h").read_text()
  comms = (PANDA_ROOT / "board/main_comms.h").read_text()
  main = (PANDA_ROOT / "board/main.c").read_text()

  assert "#define WAKE_JOURNAL_FLASH_WRITES_ENABLED true" in journal
  assert "flash_write_flashword(destination, words)" in journal
  assert "wake_journal_queue_checkpoint(" not in comms
  assert "WAKE_MONITOR_STATE_COMMITTED" in comms
  assert "wake_journal_queue_checkpoint(" not in main
  assert "WAKE_MONITOR_STATE_ARMED" in main

  writer = llflash.split("bool flash_write_flashword(", 1)[1].split("void flush_write_buffer", 1)[0]
  assert "FLASH->CR1 |= FLASH_CR_PG;" in writer
  assert "for (uint8_t i = 0U; i < 8U; i++)" in writer
  assert writer.index("FLASH->CR1 |= FLASH_CR_PG;") < writer.index("for (uint8_t i = 0U; i < 8U; i++)")
  assert writer.index("for (uint8_t i = 0U; i < 8U; i++)") < writer.rindex("while (FLASH->SR1 & FLASH_SR_QW)")
  assert "FLASH_CR_FW" not in writer
