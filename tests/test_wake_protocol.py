from panda import Panda


class FakeHandle:
  def __init__(self):
    self.writes = []

  def controlWrite(self, request_type, request, value, index, data, **kwargs):
    self.writes.append((request_type, request, value, index, data, kwargs))


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
