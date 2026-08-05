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


def test_wake_packet_layouts_come_from_shared_header():
  assert Panda.WAKE_DEBUG_STRUCT.size == 64
  assert Panda.WAKE_SUCCESS_STRUCT.size == 40
  assert Panda.WAKE_CAN_TRACE_STRUCT.size == 24
