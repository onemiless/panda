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
  event_sequence = 0xFDA  # physical bus 2 conditioning, left door, UI door
  payload = Panda.WAKE_CAN_TRACE_STRUCT.pack(
    0x57435452, state,
    120, 340, 560,
    780, event_sequence,
    0x83, 0xC2, 0xE1, 0xA4,
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
    "rx_irq_seen": [True, True, True],
    "first_event_seconds": 780,
    "event_sequence": ["power:bus2:conditioning", "leftDoor", "uiDoor"],
    "prearm_power_state": "off",
    "power_frame_count": 3,
    "left_door_frame_count": 2,
    "right_door_frame_count": 1,
    "ui_door_frame_count": 4,
    "prearm_left_door_closed": True,
    "postarm_left_door_closed": False,
    "prearm_right_door_closed": True,
    "postarm_right_door_closed": True,
    "prearm_ui_door_open": False,
    "postarm_ui_door_open": True,
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


def test_wake_can_trace_decodes_raw_can_edge_source():
  state = 76 | (0x1F << 16) | (0xFC << 24)
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
  assert trace["wake_source"] == "rawCanEdge"


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
