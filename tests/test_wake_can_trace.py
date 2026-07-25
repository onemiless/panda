from panda import Panda


class FakeHandle:
  def __init__(self, payload: bytes):
    self.payload = payload

  def controlRead(self, request_type, request, value, index, length):
    assert request_type == Panda.REQUEST_IN
    assert request == 0xDA
    assert value == 0
    assert index == 0
    assert length == Panda.WAKE_CAN_TRACE_STRUCT.size
    return self.payload


def test_wake_can_trace_decodes_persistent_can_snapshot():
  flags = 0x3F
  state = 1234 | (flags << 16) | (2 << 24)
  tesla_meta = 0x80 | 0x40 | (2 << 4) | (1 << 2) | 2
  payload = Panda.WAKE_CAN_TRACE_STRUCT.pack(
    0x57435452, state,
    120, 340, 560,
    100, 200, 300,
    260, tesla_meta, 0xAB,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(payload)

  trace = panda.wake_can_trace()

  assert trace == {
    "magic": 0x57435452,
    "off_seconds": 1234,
    "monitor_enabled": True,
    "som_off_seen": True,
    "som_off_ready": True,
    "can_armed": True,
    "wake_requested": True,
    "rate_candidate": True,
    "ignition_can": False,
    "ignition_line": False,
    "peak_bus": 2,
    "wake_source": None,
    "peak_rx_per_sec": [120, 340, 560],
    "baseline_per_sec": [100, 200, 300],
    "peak_delta": 260,
    "tesla_seen": True,
    "tesla_counter_valid": True,
    "tesla_power_state": 2,
    "tesla_logical_bus": 1,
    "tesla_physical_bus": 2,
    "tesla_previous_counter": 10,
    "tesla_counter": 11,
  }


def test_wake_can_trace_decodes_tesla_door_source():
  state = 77 | (0x1F << 16) | (0xFD << 24)
  payload = Panda.WAKE_CAN_TRACE_STRUCT.pack(
    0x57435452, state,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(payload)

  trace = panda.wake_can_trace()

  assert trace["peak_bus"] is None
  assert trace["wake_source"] == "teslaDoor"


def test_wake_can_trace_decodes_tesla_power_source():
  state = 78 | (0x1F << 16) | (0xFE << 24)
  payload = Panda.WAKE_CAN_TRACE_STRUCT.pack(
    0x57435452, state,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeHandle(payload)

  trace = panda.wake_can_trace()

  assert trace["peak_bus"] is None
  assert trace["wake_source"] == "teslaPower"
