from pathlib import Path

from panda import Panda


PANDA_ROOT = Path(__file__).resolve().parents[1]


class FakeHandle:
  def __init__(self):
    self.writes = []

  def controlWrite(self, request_type, request, value, index, data, **kwargs):
    self.writes.append((request_type, request, value, index, data, kwargs))


class FakeReadHandle:
  def __init__(self, payload: bytes):
    self.payload = payload

  def controlRead(self, request_type, request, value, index, length):
    assert request_type == Panda.REQUEST_IN
    assert request == Panda.WAKE_DEBUG_REQUEST
    assert value == 0
    assert index == 0
    assert length == Panda.WAKE_DEBUG_STRUCT.size
    return self.payload


def test_enable_deepsleep_uses_shared_firmware_request():
  panda = object.__new__(Panda)
  panda._handle = FakeHandle()

  panda.enable_deepsleep()

  assert panda._handle.writes == [
    (Panda.REQUEST_OUT, Panda.WAKE_MONITOR_REQUEST, 0, 0, b'', {}),
  ]
  assert Panda.WAKE_MONITOR_REQUEST == 0xB5
  assert Panda.WAKE_MONITOR_ARMED_STAGE == 0x30
  assert Panda.WAKE_DEBUG_MAGIC == 0x57414B48


def test_transaction_requests_use_both_usb_words():
  panda = object.__new__(Panda)
  panda._handle = FakeHandle()
  panda.wake_monitor_status = lambda: {"transaction": 0x12345678}

  assert panda.prepare_wake_monitor(0x12345678) == {"transaction": 0x12345678}
  assert panda.commit_wake_monitor(0x12345678) == {"transaction": 0x12345678}
  panda.set_host_session(0x87654321)

  assert panda._handle.writes == [
    (Panda.REQUEST_OUT, Panda.WAKE_MONITOR_PREPARE_REQUEST, 0x5678, 0x1234, b'', {}),
    (Panda.REQUEST_OUT, Panda.WAKE_MONITOR_COMMIT_REQUEST, 0x5678, 0x1234, b'', {}),
    (Panda.REQUEST_OUT, Panda.WAKE_MONITOR_HOST_SESSION_REQUEST, 0x4321, 0x8765, b'', {}),
  ]


def test_wake_packet_layouts_come_from_shared_header():
  assert Panda.WAKE_DEBUG_STRUCT.size == 64
  assert Panda.WAKE_SUCCESS_STRUCT.size == 40
  assert Panda.WAKE_CAN_TRACE_STRUCT.size == 24
  assert Panda.WAKE_MONITOR_STATUS_STRUCT.size == 20
  assert Panda.WAKE_MONITOR_STATUS_MAGIC == 0x574D4F4E


def test_wake_debug_decodes_active_fdcan_arm_and_first_rx_snapshot():
  first_rx = (1 << 31) | (1 << 29) | 0x122
  io = 0
  for bit in (1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21):
    io |= 1 << bit
  payload = Panda.WAKE_DEBUG_STRUCT.pack(
    Panda.WAKE_DEBUG_MAGIC, 8, 0x420000, 0x30, 2, 0,
    0x56781234, first_rx, 0x9ABC,
    0x11111111, 0x22222222, 9, io, 0x33333333,
    2, 0, 0, 1, 0, 0, 0, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeReadHandle(payload)

  debug = panda.wake_debug()

  assert debug["active_can_cccr"] == [0x1234, 0x5678, 0x9ABC]
  assert debug["active_can_ie"] == [0x11111111, 0x22222222, 0x33333333]
  assert debug["active_can_first_rx"] == {"bus": 1, "address": 0x122}
  assert debug["active_can_io"]["fdcan2_pb5_af"] is False
  assert debug["active_can_io"]["fdcan2_pb12_af"] is True
  assert debug["active_can_io"]["rx_ready"] == [True, True, True]
  assert debug["active_can_io"]["rx_irq_enabled"] == [[True, True], [True, True], [True, True]]
  assert debug["active_can_io"]["ile_enabled"] == [True, True, True]
  assert debug["active_can_io"]["safety_silent"] is True
  assert debug["active_can_io"]["harness_flipped"] is True


def test_reset_reason_is_snapshotted_then_hardware_flags_are_cleared():
  source = (PANDA_ROOT / "board/drivers/wake_debug.h").read_text()
  init = source.split("static void wake_debug_init(void) {", 1)[1].split("wake_success_load();", 1)[0]

  snapshot = "wake_debug.reset_reason = RCC->RSR;"
  clear = "RCC->RSR = RCC_RSR_RMVF;"
  assert snapshot in init
  assert clear in init
  assert init.index(snapshot) < init.index(clear) < init.index("wake_debug_save();")
