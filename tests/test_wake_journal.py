import binascii

from panda import Panda


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
  auxiliary = 1 | (2 << 2) | (8 << 4) | (0x34 << 8)
  meta = Panda.WAKE_JOURNAL_VERSION | (1 << 8) | (1 << 12) | (auxiliary << 16)
  prefix = Panda.WAKE_JOURNAL_RECORD_STRUCT.pack(
    Panda.WAKE_JOURNAL_MAGIC, 10, 10, meta, 0x102,
    int.from_bytes(bytes.fromhex("00010203"), "little"),
    int.from_bytes(bytes.fromhex("04050607"), "little"), 0,
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
    "source": "teslaDoor",
    "sequence": 10,
    "cycle": 10,
    "trigger_stage": 0x34,
    "logical_bus": 1,
    "physical_bus": 2,
    "length": 8,
    "can_id": 0x102,
    "data": "0001020304050607",
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
