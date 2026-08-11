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


def test_receive_only_wake_observer_requests_do_not_use_bootkick_api():
  panda = object.__new__(Panda)
  panda._handle = FakeHandle()
  panda.wake_debug = lambda: {"active_can_snapshot_valid": True}

  assert panda.arm_wake_observer() == {"active_can_snapshot_valid": True}
  assert panda.disarm_wake_observer() == {"active_can_snapshot_valid": True}
  assert panda._handle.writes == [
    (Panda.REQUEST_OUT, Panda.WAKE_OBSERVER_ARM_REQUEST, 0, 0, b'', {}),
    (Panda.REQUEST_OUT, Panda.WAKE_OBSERVER_DISARM_REQUEST, 0, 0, b'', {}),
  ]
  assert Panda.WAKE_OBSERVER_ARM_REQUEST == 0xEC
  assert Panda.WAKE_OBSERVER_DISARM_REQUEST == 0xED


def test_wake_packet_layouts_come_from_shared_header():
  assert Panda.WAKE_DEBUG_STRUCT.size == 64
  assert Panda.WAKE_SUCCESS_STRUCT.size == 40
  assert Panda.WAKE_CAN_TRACE_STRUCT.size == 24
  assert Panda.WAKE_MONITOR_STATUS_STRUCT.size == 20
  assert Panda.WAKE_MONITOR_STATUS_MAGIC == 0x574D4F4E


def test_wake_debug_decodes_active_fdcan_arm_and_first_rx_snapshot():
  first_rx = (1 << 31) | (1 << 29) | 0x122
  io = 0
  for bit in (1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23):
    io |= 1 << bit
  exti = (Panda.WAKE_ACTIVE_CAN_EXTI_OBSERVER_ARMED | Panda.WAKE_ACTIVE_CAN_EXTI_IMR_ENABLED |
          Panda.WAKE_ACTIVE_CAN_EXTI_RISING_ENABLED | Panda.WAKE_ACTIVE_CAN_EXTI_FALLING_ENABLED |
          Panda.WAKE_ACTIVE_CAN_EXTI_NVIC_ENABLED | Panda.WAKE_ACTIVE_CAN_EXTI_MAPPING_OK |
          Panda.WAKE_ACTIVE_CAN_EXTI_IRQ_SEEN | Panda.WAKE_ACTIVE_CAN_EXTI_PRIMARY_PENDING |
          Panda.WAKE_ACTIVE_CAN_EXTI_ARM_LEVEL_HIGH | Panda.WAKE_ACTIVE_CAN_EXTI_GPIO_MODE)
  payload = Panda.WAKE_DEBUG_STRUCT.pack(
    Panda.WAKE_DEBUG_MAGIC, 8, 0x420000, 0x30, Panda.WAKE_ACTIVE_CAN_DIAG_V1_MAGIC | io, exti,
    0x56781234, first_rx, 0x9ABC,
    0x11111111, 0x22222222, 9, 0, 0x33333333,
    2, 0, 0, 1, 0, 0, 0, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeReadHandle(payload)

  debug = panda.wake_debug()

  assert debug["active_can_snapshot_valid"] is True
  assert debug["active_can_snapshot_version"] == 1
  assert debug["snapshot_kind"] == "activeFdcanV1"
  assert debug["enter_count"] is None
  assert debug["pre_wfi_exti_pr1"] is None
  assert debug["active_can_cccr"] == [0x1234, 0x5678, 0x9ABC]
  assert debug["active_can_ie"] == [0x11111111, 0x22222222, 0x33333333]
  assert debug["active_can_exti"]["gpio_mode"] is True
  assert debug["active_can_first_rx"] == {"bus": 1, "address": 0x122}
  assert debug["active_can_exti"] == {
    "observer_armed": True,
    "imr_enabled": True,
    "rising_enabled": True,
    "falling_enabled": True,
    "nvic_enabled": True,
    "mapping_ok": True,
    "irq_seen": True,
    "primary_pending": True,
    "arm_level_high": True,
    "irq_level_high": False,
    "gpio_mode": True,
  }
  assert debug["active_can_io"]["fdcan2_pb5_af"] is False
  assert debug["active_can_io"]["fdcan2_pb12_af"] is True
  assert debug["active_can_io"]["rx_ready"] == [True, True, True]
  assert debug["active_can_io"]["rx_irq_enabled"] == [True, True, True]
  assert debug["active_can_io"]["ile_enabled"] == [True, True, True]
  assert debug["active_can_io"]["rx_fifo0_to_it0"] == [True, True, True]
  assert debug["active_can_io"]["safety_silent"] is True
  assert debug["active_can_io"]["rx_irq_entry_seen"] == [True, True, True]
  assert debug["bootkick_debug_waiting_countdown"] == 0
  assert debug["bootkick_debug_hold_countdown"] == 0


def test_wake_debug_does_not_misdecode_stop_mode_fields_as_active_can_snapshot():
  payload = Panda.WAKE_DEBUG_STRUCT.pack(
    Panda.WAKE_DEBUG_MAGIC, 8, 0x420000, 0x16, 2, 0,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 9, 0x7FFFFFFF, 0xFFFFFFFF,
    2, 0, 0, 1, 0, 0, 0, 0,
  )
  panda = object.__new__(Panda)
  panda._handle = FakeReadHandle(payload)

  debug = panda.wake_debug()

  assert debug["active_can_snapshot_valid"] is False
  assert debug["active_can_snapshot_version"] is None
  assert debug["snapshot_kind"] == "legacyOrStopExti"
  assert debug["enter_count"] == 2
  assert debug["active_can_cccr"] is None
  assert debug["active_can_ie"] is None
  assert debug["active_can_first_rx"] is None
  assert debug["active_can_exti"] is None
  assert debug["active_can_io"] is None


def test_reset_reason_is_snapshotted_then_hardware_flags_are_cleared():
  source = (PANDA_ROOT / "board/drivers/wake_debug.h").read_text()
  init = source.split("static void wake_debug_init(void) {", 1)[1].split("wake_success_load();", 1)[0]

  snapshot = "wake_debug.reset_reason = RCC->RSR;"
  clear = "RCC->RSR = RCC_RSR_RMVF;"
  assert snapshot in init
  assert clear in init
  assert init.index(snapshot) < init.index(clear) < init.index("wake_debug_save();")
