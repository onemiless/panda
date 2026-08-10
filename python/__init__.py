# python library to interface with panda
import os
import re
import sys
import time
import usb1
import struct
import hashlib
import binascii
import ctypes
from functools import wraps, partial
from itertools import accumulate

import opendbc
from opendbc.car.structs import CarParams

from .base import BaseHandle
from .constants import BASEDIR, FW_PATH, McuType, compute_version_hash
from .dfu import PandaDFU
from .spi import PandaSpiHandle, PandaSpiException, PandaProtocolMismatch
from .usb import PandaUsbHandle
from .utils import logger

# load libusb from pip package
try:
  import libusb_package
  usb1._libusb1.loadLibrary(ctypes.CDLL(str(libusb_package.get_library_path())))
except ImportError:
  # TODO: remove this on next AGNOS update
  pass

__version__ = '0.0.10'

CANPACKET_HEAD_SIZE = 0x6
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
LEN_TO_DLC = {length: dlc for (dlc, length) in enumerate(DLC_TO_LEN)}
PANDA_CAN_CNT = 3


def calculate_checksum(data):
  res = 0
  for b in data:
    res ^= b
  return res

def _parse_c_struct(path, name):
  type_to_format = {"uint8_t": "B", "uint16_t": "H", "uint32_t": "I", "float": "f"}
  with open(path) as f:
    source = f.read()
  packed_start = f"struct __attribute__((packed)) {name} {{"
  if packed_start in source:
    body = source.split(packed_start, 1)[1].split("};", 1)[0]
  else:
    match = re.search(rf"typedef\s+struct\s*\{{([^}}]*)\}}\s*{re.escape(name)}\s*;", source, re.DOTALL)
    if match is None:
      raise ValueError(f"missing struct {name} in {path}")
    body = match[1]
  lines = [l.strip() for l in body.splitlines() if l.strip()]
  fields = [re.fullmatch(rf"({'|'.join(type_to_format)})\s+\w+;", l) for l in lines]
  if not all(fields):
    raise ValueError(f"unsupported {name} layout in {path}")
  return struct.Struct("<" + "".join(type_to_format[m[1]] for m in fields))

def _parse_c_define(path, name):
  with open(path) as f:
    match = re.search(rf"^#define\s+{re.escape(name)}\s+(0x[0-9A-Fa-f]+|[0-9]+)[UuLl]*$", f.read(), re.MULTILINE)
  if match is None:
    raise ValueError(f"missing integer define {name} in {path}")
  return int(match[1], 0)

def pack_can_buffer(arr, chunk=False, fd=False):
  snds = [bytearray(), ]
  for address, dat, bus in arr:
    extended = 1 if address >= 0x800 else 0
    data_len_code = LEN_TO_DLC[len(dat)]
    header = bytearray(CANPACKET_HEAD_SIZE)
    word_4b = (address << 3) | (extended << 2)
    header[0] = (data_len_code << 4) | (bus << 1) | int(fd)
    header[1] = word_4b & 0xFF
    header[2] = (word_4b >> 8) & 0xFF
    header[3] = (word_4b >> 16) & 0xFF
    header[4] = (word_4b >> 24) & 0xFF
    header[5] = calculate_checksum(header[:5] + dat)

    snds[-1].extend(header)
    snds[-1].extend(dat)
    if chunk and len(snds[-1]) > 256:
      snds.append(bytearray())

  return snds

def unpack_can_buffer(dat):
  ret = []

  while len(dat) >= CANPACKET_HEAD_SIZE:
    data_len = DLC_TO_LEN[(dat[0]>>4)]

    header = dat[:CANPACKET_HEAD_SIZE]

    bus = (header[0] >> 1) & 0x7
    address = (header[4] << 24 | header[3] << 16 | header[2] << 8 | header[1]) >> 3

    if (header[1] >> 1) & 0x1:
      # returned
      bus += 128
    if header[1] & 0x1:
      # rejected
      bus += 192

    # we need more from the next transfer
    if data_len > len(dat) - CANPACKET_HEAD_SIZE:
      break

    assert calculate_checksum(dat[:(CANPACKET_HEAD_SIZE+data_len)]) == 0, "CAN packet checksum incorrect"

    data = dat[CANPACKET_HEAD_SIZE:(CANPACKET_HEAD_SIZE+data_len)]
    dat = dat[(CANPACKET_HEAD_SIZE+data_len):]

    ret.append((address, data, bus))

  return (ret, dat)


def ensure_version(desc, lib_field, panda_field, fn):
  @wraps(fn)
  def wrapper(self, *args, **kwargs):
    lib_version = getattr(self, lib_field)
    panda_version = getattr(self, panda_field)
    if lib_version != panda_version:
      raise RuntimeError(f"{desc} packet version mismatch: panda's firmware v{panda_version}, library v{lib_version}. Reflash panda.")
    return fn(self, *args, **kwargs)
  return wrapper
ensure_can_packet_version = partial(ensure_version, "CAN", "CAN_PACKET_VERSION", "can_version")
ensure_health_packet_version = partial(ensure_version, "health", "HEALTH_PACKET_VERSION", "health_version")



class Panda:

  SERIAL_DEBUG = 0
  SERIAL_SOM_DEBUG = 4

  USB_VIDS = (0xbbaa, 0x3801)  # 0x3801 is comma's registered VID
  USB_PIDS = (0xddee, 0xddcc)
  REQUEST_IN = usb1.ENDPOINT_IN | usb1.TYPE_VENDOR | usb1.RECIPIENT_DEVICE
  REQUEST_OUT = usb1.ENDPOINT_OUT | usb1.TYPE_VENDOR | usb1.RECIPIENT_DEVICE

  # from https://github.com/commaai/openpilot/blob/103b4df18cbc38f4129555ab8b15824d1a672bdf/cereal/log.capnp#L648
  HW_TYPE_UNKNOWN = b'\x00'
  HW_TYPE_RED_PANDA = b'\x07'
  HW_TYPE_TRES = b'\x09'
  HW_TYPE_CUATRO = b'\x0a'
  HW_TYPE_BODY = b'\xb1'

  CAN_PACKET_VERSION = compute_version_hash(os.path.join(opendbc.INCLUDE_PATH, "opendbc/safety/can.h"))
  HEALTH_PACKET_VERSION = compute_version_hash(os.path.join(BASEDIR, "board/health.h"))
  HEALTH_STRUCT = _parse_c_struct(os.path.join(BASEDIR, "board/health.h"), "health_t")
  WAKE_PROTOCOL_HEADER = os.path.join(BASEDIR, "board/wake_protocol.h")
  WAKE_MONITOR_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_ENABLE_WAKE_MONITOR")
  WAKE_MONITOR_PREPARE_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_PREPARE_WAKE_MONITOR")
  WAKE_MONITOR_COMMIT_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_COMMIT_WAKE_MONITOR")
  WAKE_MONITOR_ABORT_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_ABORT_WAKE_MONITOR")
  WAKE_MONITOR_HOST_SESSION_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_SET_HOST_SESSION")
  WAKE_MONITOR_STATUS_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_MONITOR_STATUS")
  WAKE_MONITOR_ARMED_STAGE = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_WAKE_MONITOR_ARMED_STAGE")
  WAKE_MONITOR_STATUS_MAGIC = _parse_c_define(WAKE_PROTOCOL_HEADER, "WAKE_MONITOR_STATUS_MAGIC")
  WAKE_DEBUG_MAGIC = _parse_c_define(WAKE_PROTOCOL_HEADER, "WAKE_DEBUG_MAGIC")
  WAKE_DEBUG_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_DEBUG")
  WAKE_SUCCESS_CLEAR_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_CLEAR_WAKE_SUCCESS")
  WAKE_SUCCESS_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_SUCCESS")
  WAKE_CAN_TRACE_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_CAN_TRACE")
  WAKE_JOURNAL_INFO_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_JOURNAL_INFO")
  WAKE_JOURNAL_RECORD_REQUEST = _parse_c_define(WAKE_PROTOCOL_HEADER, "PANDA_REQUEST_GET_WAKE_JOURNAL_RECORD")
  WAKE_JOURNAL_MAGIC = _parse_c_define(WAKE_PROTOCOL_HEADER, "WAKE_JOURNAL_MAGIC")
  WAKE_JOURNAL_VERSION = _parse_c_define(WAKE_PROTOCOL_HEADER, "WAKE_JOURNAL_VERSION")
  WAKE_DEBUG_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_debug_t")
  WAKE_SUCCESS_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_success_t")
  WAKE_CAN_TRACE_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_can_trace_t")
  WAKE_JOURNAL_INFO_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_journal_info_t")
  WAKE_JOURNAL_RECORD_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_journal_record_t")
  WAKE_MONITOR_STATUS_STRUCT = _parse_c_struct(WAKE_PROTOCOL_HEADER, "wake_monitor_status_t")
  CAN_HEALTH_STRUCT = struct.Struct("<BIBBBBBBBBIIIIIIIHHBBBIIII")

  H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]
  SUPPORTED_DEVICES = H7_DEVICES

  INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO)

  HARNESS_STATUS_NC = 0
  HARNESS_STATUS_NORMAL = 1
  HARNESS_STATUS_FLIPPED = 2

  def __init__(self, serial: str | None = None, claim: bool = True, disable_checks: bool = True, can_speed_kbps: int = 500, cli: bool = True):
    self._disable_checks = disable_checks

    self._handle: BaseHandle
    self._handle_open = False
    self.can_rx_overflow_buffer = b''
    self._can_speed_kbps = can_speed_kbps

    if cli and serial is None:
      self._connect_serial = self._cli_select_panda()
    else:
      self._connect_serial = serial

    self.connect(claim)

  def _cli_select_panda(self):
    dfu_pandas = PandaDFU.list()
    if len(dfu_pandas) > 0:
      print("INFO: some attached pandas are in DFU mode.")

    pandas = self.list()
    if len(pandas) == 0:
      print("INFO: panda not available")
      return None
    if len(pandas) == 1:
      print(f"INFO: connecting to panda {pandas[0]}")
      return pandas[0]
    while True:
      print("Multiple pandas available:")
      pandas.sort()
      for idx, serial in enumerate(pandas):
        print(f"{[idx]}: {serial}")
      try:
        choice = int(input("Choose serial [0]:") or "0")
        return pandas[choice]
      except (ValueError, IndexError):
        print("Enter a valid index.")

  def __enter__(self):
    return self

  def __exit__(self, *args):
    self.close()

  def close(self):
    if self._handle_open:
      self._handle.close()
      self._handle_open = False
      if self._context is not None:
        self._context.close()

  def connect(self, claim=True, wait=False):
    self.close()

    self._handle = None
    while self._handle is None:
      # try USB first, then SPI
      self._context, self._handle, serial, self.bootstub = self.usb_connect(self._connect_serial, claim=claim, no_error=wait)
      if self._handle is None:
        self._context, self._handle, serial, self.bootstub = self.spi_connect(self._connect_serial)
      if not wait:
        break

    if self._handle is None:
      raise Exception("failed to connect to panda")

    self._serial = serial
    self._connect_serial = serial
    self._handle_open = True
    self.health_version, self.can_version = self.get_packets_versions()
    logger.debug("connected")

    # disable openpilot's heartbeat checks
    if self._disable_checks:
      self.set_heartbeat_disabled()
      self.set_power_save(0)

    # reset comms
    self.can_reset_communications()

    # disable automatic CAN-FD switching
    for bus in range(PANDA_CAN_CNT):
      self.set_canfd_auto(bus, False)

    # set CAN speed
    for bus in range(PANDA_CAN_CNT):
      self.set_can_speed_kbps(bus, self._can_speed_kbps)

  @property
  def spi(self) -> bool:
    return isinstance(self._handle, PandaSpiHandle)

  @classmethod
  def spi_connect(cls, serial, ignore_version=False):
    try:
      handle = PandaSpiHandle()
      dat = handle.get_protocol_version()
    except PandaSpiException:
      return None, None, None, False

    spi_serial = binascii.hexlify(dat[:12]).decode()
    pid = dat[13]
    if pid not in (0xcc, 0xee):
      raise PandaProtocolMismatch(f"invalid bootstub status ({pid=}). reflash panda")
    bootstub = pid == 0xee
    spi_version = dat[14]

    # did we get the right panda?
    if serial is not None and spi_serial != serial:
      return None, None, None, False

    # ensure our protocol version matches the panda
    if (not ignore_version) and spi_version != handle.PROTOCOL_VERSION:
      raise PandaProtocolMismatch(f"panda protocol mismatch: expected {handle.PROTOCOL_VERSION}, got {spi_version}. reflash panda")

    # got a device and all good
    return None, handle, spi_serial, bootstub

  @classmethod
  def usb_connect(cls, serial, claim=True, no_error=False):
    handle, usb_serial, bootstub = None, None, None
    context = usb1.USBContext()
    context.open()
    try:
      for device in context.getDeviceList(skip_on_error=True):
        if device.getVendorID() in cls.USB_VIDS and device.getProductID() in cls.USB_PIDS:
          try:
            this_serial = device.getSerialNumber()
          except Exception:
            # Allow to ignore errors on reconnect. USB hubs need some time to initialize after panda reset
            if not no_error:
              logger.exception("failed to get serial number of panda")
            continue

          if serial is None or this_serial == serial:
            logger.debug("opening device %s %s", this_serial, hex(device.getProductID()))

            usb_serial = this_serial
            bootstub = (device.getProductID() & 0xF0) == 0xe0
            handle = device.open()
            if sys.platform not in ("win32", "cygwin", "msys", "darwin"):
              handle.setAutoDetachKernelDriver(True)
            if claim or sys.platform == "darwin":
              handle.claimInterface(0)
              # handle.setInterfaceAltSetting(0, 0)  # Issue in USB stack

            break
    except Exception:
      logger.exception("USB connect error")

    usb_handle = None
    if handle is not None:
      usb_handle = PandaUsbHandle(handle)
    else:
      context.close()

    return context, usb_handle, usb_serial, bootstub

  def is_connected_spi(self):
    return isinstance(self._handle, PandaSpiHandle)

  def is_connected_usb(self):
    return isinstance(self._handle, PandaUsbHandle)

  @classmethod
  def list(cls, usb_only: bool = False):
    ret = cls.usb_list()
    if not usb_only:
      ret += cls.spi_list()
    return list(set(ret))

  @classmethod
  def usb_list(cls):
    ret = []
    try:
      with usb1.USBContext() as context:
        for device in context.getDeviceList(skip_on_error=True):
          if device.getVendorID() in cls.USB_VIDS and device.getProductID() in cls.USB_PIDS:
            try:
              serial = device.getSerialNumber()
              if len(serial) == 24:
                ret.append(serial)
              else:
                logger.warning(f"found device with panda descriptors but invalid serial: {serial}", RuntimeWarning)
            except Exception:
              logger.exception("error connecting to panda")
    except Exception:
      logger.exception("exception while listing pandas")
    return ret

  @classmethod
  def spi_list(cls):
    _, _, serial, _ = cls.spi_connect(None, ignore_version=True)
    if serial is not None:
      return [serial, ]
    return []

  def reset(self, enter_bootstub=False, enter_bootloader=False, reconnect=True):
    if enter_bootstub or enter_bootloader:
      assert (hw_type := self.get_type()) in self.SUPPORTED_DEVICES, f"Unknown HW: {hw_type}"

    # no response is expected since it resets right away
    timeout = 5000 if isinstance(self._handle, PandaSpiHandle) else 15000
    try:
      if enter_bootloader:
        self._handle.controlWrite(Panda.REQUEST_IN, 0xd1, 0, 0, b'', timeout=timeout, expect_disconnect=True)
      else:
        if enter_bootstub:
          self._handle.controlWrite(Panda.REQUEST_IN, 0xd1, 1, 0, b'', timeout=timeout, expect_disconnect=True)
        else:
          self._handle.controlWrite(Panda.REQUEST_IN, 0xd8, 0, 0, b'', timeout=timeout, expect_disconnect=True)
    except Exception:
      pass

    self.close()
    if not enter_bootloader and reconnect:
      self.reconnect()

  @property
  def connected(self) -> bool:
    return self._handle_open

  def reconnect(self):
    if self._handle_open:
      self.close()

    success = False
    # wait up to 15 seconds
    for _ in range(15*10):
      try:
        self.connect(claim=False, wait=True)
        success = True
        break
      except Exception:
        pass
      time.sleep(0.1)
    if not success:
      raise Exception("reconnect failed")

  @staticmethod
  def flasher_present(handle: BaseHandle) -> bool:
    fr = handle.controlRead(Panda.REQUEST_IN, 0xb0, 0, 0, 0xc)
    return fr[4:8] == b"\xde\xad\xd0\x0d"

  @staticmethod
  def flash_static(handle, code, mcu_type):
    assert mcu_type is not None, "must set valid mcu_type to flash"

    # confirm flasher is present
    assert Panda.flasher_present(handle)

    # determine sectors to erase
    app_capacity = mcu_type.config.app_end_address - mcu_type.config.app_address
    assert 0 < len(code) <= app_capacity, "Binary too large! Risk of overwriting reserved flash."
    app_sector_sizes = mcu_type.config.sector_sizes[1:mcu_type.config.app_last_sector + 1]
    apps_sectors_cumsum = accumulate(app_sector_sizes)
    last_sector = next((i + 1 for i, v in enumerate(apps_sectors_cumsum) if v >= len(code)), -1)
    assert 1 <= last_sector <= mcu_type.config.app_last_sector, "No writable application sector for binary."

    # unlock flash
    logger.info("flash: unlocking")
    handle.controlWrite(Panda.REQUEST_IN, 0xb1, 0, 0, b'')

    # erase sectors
    logger.info(f"flash: erasing sectors 1 - {last_sector}")
    for i in range(1, last_sector + 1):
      handle.controlWrite(Panda.REQUEST_IN, 0xb2, i, 0, b'')

    # flash over EP2
    STEP = 0x200
    logger.info("flash: flashing")
    for i in range(0, len(code), STEP):
      handle.bulkWrite(2, code[i:i + STEP])

    # reset
    logger.info("flash: resetting")
    try:
      handle.controlWrite(Panda.REQUEST_IN, 0xd8, 0, 0, b'', expect_disconnect=True)
    except Exception:
      pass

  def flash(self, fn=None, code=None, reconnect=True):
    assert (hw_type := self.get_type()) in self.SUPPORTED_DEVICES, f"Unknown HW: {hw_type}"

    if self.up_to_date(fn=fn):
      logger.info("flash: already up to date")
      return

    if not fn:
      fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
    assert os.path.isfile(fn)
    logger.debug("flash: main version is %s", self.get_version())
    if not self.bootstub:
      self.reset(enter_bootstub=True)
    assert(self.bootstub)

    if code is None:
      with open(fn, "rb") as f:
        code = f.read()

    # get version
    logger.debug("flash: bootstub version is %s", self.get_version())

    # do flash
    Panda.flash_static(self._handle, code, mcu_type=McuType.H7)

    # reconnect
    if reconnect:
      self.reconnect()

  def recover(self, timeout: int | None = 60, reset: bool = True) -> bool:
    dfu_serial = self.get_dfu_serial()

    if reset:
      self.reset(enter_bootstub=True)
      self.reset(enter_bootloader=True)

    if not self.wait_for_dfu(dfu_serial, timeout=timeout):
      return False

    dfu = PandaDFU(dfu_serial)
    dfu.recover()

    # reflash after recover
    self.connect(True, True)
    self.flash()
    return True

  @staticmethod
  def wait_for_dfu(dfu_serial: str | None, timeout: int | None = None) -> bool:
    t_start = time.monotonic()
    dfu_list = PandaDFU.list()
    while (dfu_serial is None and len(dfu_list) == 0) or (dfu_serial is not None and dfu_serial not in dfu_list):
      logger.debug("waiting for DFU...")
      time.sleep(0.1)
      if timeout is not None and (time.monotonic() - t_start) > timeout:
        return False
      dfu_list = PandaDFU.list()
    return True

  @classmethod
  def wait_for_panda(cls, serial: str | None, timeout: int) -> bool:
    t_start = time.monotonic()
    serials = cls.list()
    while (serial is None and len(serials) == 0) or (serial is not None and serial not in serials):
      logger.debug("waiting for panda...")
      time.sleep(0.1)
      if timeout is not None and (time.monotonic() - t_start) > timeout:
        return False
      serials = cls.list()
    return True

  def up_to_date(self, fn=None) -> bool:
    current = self.get_signature()
    if fn is None:
      fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
    expected = Panda.get_signature_from_firmware(fn)
    return (current == expected)

  def call_control_api(self, msg):
    self._handle.controlWrite(Panda.REQUEST_OUT, msg, 0, 0, b'')

  def enable_deepsleep(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_REQUEST, 0, 0, b'')

  @staticmethod
  def _wake_transaction_words(value: int) -> tuple[int, int]:
    value &= 0xFFFFFFFF
    return value & 0xFFFF, (value >> 16) & 0xFFFF

  def prepare_wake_monitor(self, transaction: int):
    low, high = self._wake_transaction_words(transaction)
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_PREPARE_REQUEST, low, high, b'')
    return self.wake_monitor_status()

  def commit_wake_monitor(self, transaction: int):
    low, high = self._wake_transaction_words(transaction)
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_COMMIT_REQUEST, low, high, b'')
    return self.wake_monitor_status()

  def abort_wake_monitor(self, transaction: int):
    low, high = self._wake_transaction_words(transaction)
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_ABORT_REQUEST, low, high, b'')
    return self.wake_monitor_status()

  def set_host_session(self, host_session: int):
    low, high = self._wake_transaction_words(host_session)
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_HOST_SESSION_REQUEST, low, high, b'')

  def wake_monitor_status(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_MONITOR_STATUS_REQUEST, 0, 0, self.WAKE_MONITOR_STATUS_STRUCT.size)
    a = self.WAKE_MONITOR_STATUS_STRUCT.unpack(dat)
    return {
      "magic": a[0],
      "transaction": a[1],
      "host_session": a[2],
      "committed_host_session": a[3],
      "state": a[4],
      "result": a[5],
      "trigger_stage": a[6],
    }

  # ******************* health *******************

  @ensure_health_packet_version
  def health(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, self.HEALTH_STRUCT.size)
    a = self.HEALTH_STRUCT.unpack(dat)
    return {
      "uptime": a[0],
      "voltage": a[1],
      "current": a[2],
      "safety_tx_blocked": a[3],
      "safety_rx_invalid": a[4],
      "tx_buffer_overflow": a[5],
      "rx_buffer_overflow": a[6],
      "faults": a[7],
      "ignition_line": a[8],
      "ignition_can": a[9],
      "controls_allowed": a[10],
      "car_harness_status": a[11],
      "safety_mode": a[12],
      "safety_param": a[13],
      "fault_status": a[14],
      "power_save_enabled": a[15],
      "heartbeat_lost": a[16],
      "alternative_experience": a[17],
      "interrupt_load": a[18],
      "fan_power": a[19],
      "safety_rx_checks_invalid": a[20],
      "spi_error_count": a[21],
      "sbu1_voltage_mV": a[22],
      "sbu2_voltage_mV": a[23],
      "som_reset_triggered": a[24],
      "sound_output_level": a[25],
      "controls_allowed_lateral": a[26],
      "controls_allowed_longitudinal": a[27],
    }

  def wake_debug(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_DEBUG_REQUEST, 0, 0, self.WAKE_DEBUG_STRUCT.size)
    a = self.WAKE_DEBUG_STRUCT.unpack(dat)
    return {
      "magic": a[0],
      "boot_count": a[1],
      "reset_reason": a[2],
      "stage": a[3],
      "enter_count": a[4],
      "wfi_return_count": a[5],
      "pre_wfi_exti_pr1": a[6],
      "post_wfi_exti_pr1": a[7],
      "exti_imr1": a[8],
      "exti_rtsr1": a[9],
      "exti_ftsr1": a[10],
      "hw_type_snapshot": a[11] & 0xFF,
      "bootkick_phase_mask": (a[11] >> 8) & 0xFF,
      "bootkick_pin_levels": (a[11] >> 16) & 0xFF,
      "bootkick_wake_attempts": (a[11] >> 24) & 0x3,
      "bootkick_wake_retry_countdown": (a[11] >> 26) & 0xF,
      "bootkick_wake_uart_seen": bool((a[11] >> 30) & 0x1),
      "bootkick_wake_reset_attempted": bool((a[11] >> 31) & 0x1),
      "can_exti_line": a[12] & 0xFFFF,
      "bootkick_debug_waiting_countdown": (a[12] >> 16) & 0xFF,
      "bootkick_debug_hold_countdown": (a[12] >> 24) & 0xFF,
      "exti_emr1": a[13],
      "harness_status": a[14],
      "ignition_line": a[15],
      "ignition_can_seen": a[16],
      "som_gpio": a[17],
      "bootkick_state": a[18],
      "bootkick_prev_state": a[19],
      "bootkick_waiting_countdown": a[20],
      "bootkick_reset_countdown": a[21],
    }

  def wake_success(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_SUCCESS_REQUEST, 0, 0, self.WAKE_SUCCESS_STRUCT.size)
    a = self.WAKE_SUCCESS_STRUCT.unpack(dat)
    return {
      "magic": a[0],
      "latched": a[1],
      "stage": a[2],
      "boot_count": a[3],
      "reset_reason": a[4],
      "can_exti_line": a[5],
      "harness_status": a[6],
      "ignition_line": a[7],
      "ignition_can_seen": a[8],
      "som_gpio": a[9],
    }

  def wake_can_trace(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_CAN_TRACE_REQUEST, 0, 0, self.WAKE_CAN_TRACE_STRUCT.size)
    a = self.WAKE_CAN_TRACE_STRUCT.unpack(dat)
    flags = (a[1] >> 16) & 0xFF
    peak_bus = (a[1] >> 24) & 0xFF
    wake_source = {
      0xFC: "rawCanEdge",
      0xFD: "teslaDoor",
      0xFE: "teslaPower",
    }.get(peak_bus)
    power_states = ("off", "conditioning", "accessory", "drive")
    events = []
    for i in range(8):
      event = (a[6] >> (i * 4)) & 0xF
      if event == 0:
        break
      if event <= 0xC:
        encoded = event - 1
        events.append(f"power:bus{encoded // 4}:{power_states[encoded % 4]}")
      else:
        events.append({0xD: "leftDoor", 0xE: "rightDoor", 0xF: "uiDoor"}[event])
    power_meta, left_meta, right_meta, ui_meta = a[7:11]
    prearm_power_seen = bool(power_meta & 0x80)

    def binary_prearm(meta):
      return bool(meta & 0x40) if meta & 0x80 else None

    def binary_postarm(meta):
      return bool(meta & 0x20) if meta & 0x1F else None

    return {
      "magic": a[0],
      "off_seconds": a[1] & 0xFFFF,
      "monitor_enabled": bool(flags & (1 << 0)),
      "som_off_seen": bool(flags & (1 << 1)),
      "som_off_ready": bool(flags & (1 << 2)),
      "can_armed": bool(flags & (1 << 3)),
      "wake_requested": bool(flags & (1 << 4)),
      "rate_candidate": bool(flags & (1 << 5)),
      "ignition_can": bool(flags & (1 << 6)),
      "ignition_line": bool(flags & (1 << 7)),
      "peak_bus": None if peak_bus >= 0xFC else peak_bus,
      "wake_source": wake_source,
      "peak_rx_per_sec": [a[2], a[3], a[4]],
      "first_event_seconds": a[5] or None,
      "event_sequence": events,
      "prearm_power_state": power_states[(power_meta >> 5) & 0x3] if prearm_power_seen else None,
      "power_frame_count": power_meta & 0x1F,
      "left_door_frame_count": left_meta & 0x1F,
      "right_door_frame_count": right_meta & 0x1F,
      "ui_door_frame_count": ui_meta & 0x1F,
      "prearm_left_door_closed": binary_prearm(left_meta),
      "postarm_left_door_closed": binary_postarm(left_meta),
      "prearm_right_door_closed": binary_prearm(right_meta),
      "postarm_right_door_closed": binary_postarm(right_meta),
      "prearm_ui_door_open": binary_prearm(ui_meta),
      "postarm_ui_door_open": binary_postarm(ui_meta),
    }

  def wake_journal_info(self, timeout: int = 15000):
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_JOURNAL_INFO_REQUEST, 0, 0,
                                   self.WAKE_JOURNAL_INFO_STRUCT.size, timeout=timeout)
    a = self.WAKE_JOURNAL_INFO_STRUCT.unpack(dat)
    return {
      "magic": a[0],
      "version": a[1],
      "record_size": a[2],
      "capacity": a[3],
      "used_slots": a[4],
      "valid_records": a[5],
      "full": bool(a[6] & 0x1),
      "foreign_data": bool(a[6] & 0x2),
      "next_sequence": a[7],
      "current_cycle": a[8],
    }

  def wake_journal_record(self, slot: int, timeout: int = 15000):
    if not 0 <= slot <= 0xFFFF:
      raise ValueError(f"invalid wake journal slot {slot}")
    dat = self._handle.controlRead(Panda.REQUEST_IN, Panda.WAKE_JOURNAL_RECORD_REQUEST,
                                   slot, 0, self.WAKE_JOURNAL_RECORD_STRUCT.size, timeout=timeout)
    a = self.WAKE_JOURNAL_RECORD_STRUCT.unpack(dat)
    meta = a[3]
    version = meta & 0xFF
    record_type = (meta >> 8) & 0xF
    source_id = (meta >> 12) & 0xF
    auxiliary = (meta >> 16) & 0xFFFF
    source = {
      1: "teslaDoor",
      2: "teslaPower",
      3: "canRate",
      4: "ignition",
      5: "harness",
      6: "canPrimary",
    }.get(source_id, "unknown")
    valid = a[0] == self.WAKE_JOURNAL_MAGIC and version == self.WAKE_JOURNAL_VERSION \
      and (binascii.crc32(dat[:28]) & 0xFFFFFFFF) == a[7]
    record = {
      "valid": valid,
      "magic": a[0],
      "version": version,
      "type": {1: "event", 2: "result", 3: "checkpoint"}.get(record_type, "unknown"),
      "source": source,
      "sequence": a[1],
      "cycle": a[2],
    }
    if record_type == 1:
      length = (auxiliary >> 4) & 0xF
      payload = struct.pack("<II", a[5], a[6])[:min(length, 8)]
      record.update({
        "trigger_stage": (auxiliary >> 8) & 0xFF,
        "logical_bus": auxiliary & 0x3,
        "physical_bus": (auxiliary >> 2) & 0x3,
        "length": length,
        "can_id": a[4],
        "data": payload.hex(),
      })
    elif record_type == 2:
      record.update({
        "success": bool(auxiliary & (1 << 6)),
        "attempts": auxiliary & 0x3,
        "uart_seen": bool(auxiliary & (1 << 2)),
        "reset_attempted": bool(auxiliary & (1 << 3)),
        "som_gpio": bool(auxiliary & (1 << 4)),
        "heartbeat_seen": bool(auxiliary & (1 << 5)),
        "trigger_stage": a[4],
        "final_stage": a[5],
        "reset_reason": a[6],
      })
    elif record_type == 3:
      record.update({
        "state": auxiliary & 0xFF,
        "stage": (auxiliary >> 8) & 0xFF,
        "transaction": a[4],
        "host_session": a[5],
        "off_seconds": a[6],
      })
    return record

  def clear_wake_success(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_SUCCESS_CLEAR_REQUEST, 0, 0, b'')

  @ensure_health_packet_version
  def can_health(self, can_number):
    LEC_ERROR_CODE = {
      0: "No error",
      1: "Stuff error",
      2: "Form error",
      3: "AckError",
      4: "Bit1Error",
      5: "Bit0Error",
      6: "CRCError",
      7: "NoChange",
    }
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xc2, int(can_number), 0, self.CAN_HEALTH_STRUCT.size)
    a = self.CAN_HEALTH_STRUCT.unpack(dat)
    return {
      "bus_off": a[0],
      "bus_off_cnt": a[1],
      "error_warning": a[2],
      "error_passive": a[3],
      "last_error": LEC_ERROR_CODE[a[4]],
      "last_stored_error": LEC_ERROR_CODE[a[5]],
      "last_data_error": LEC_ERROR_CODE[a[6]],
      "last_data_stored_error": LEC_ERROR_CODE[a[7]],
      "receive_error_cnt": a[8],
      "transmit_error_cnt": a[9],
      "total_error_cnt": a[10],
      "total_tx_lost_cnt": a[11],
      "total_rx_lost_cnt": a[12],
      "total_tx_cnt": a[13],
      "total_rx_cnt": a[14],
      "total_fwd_cnt": a[15],
      "total_tx_checksum_error_cnt": a[16],
      "can_speed": a[17],
      "can_data_speed": a[18],
      "canfd_enabled": a[19],
      "brs_enabled": a[20],
      "canfd_non_iso": a[21],
      "irq0_call_rate": a[22],
      "irq1_call_rate": a[23],
      "irq2_call_rate": a[24],
      "can_core_reset_count": a[25],
    }

  # ******************* control *******************

  def get_version(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xd6, 0, 0, 0x40).decode('utf8')

  @staticmethod
  def get_signature_from_firmware(fn) -> bytes:
    with open(fn, 'rb') as f:
      f.seek(-128, 2)  # Seek from end of file
      return f.read(128)

  def get_signature(self) -> bytes:
    part_1 = self._handle.controlRead(Panda.REQUEST_IN, 0xd3, 0, 0, 0x40)
    part_2 = self._handle.controlRead(Panda.REQUEST_IN, 0xd4, 0, 0, 0x40)
    return bytes(part_1 + part_2)

  def get_type(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xc1, 0, 0, 0x40)

  def get_packets_versions(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xdd, 0, 0, 8)
    if dat and len(dat) == 8:
      return struct.unpack("<II", dat)
    return (0, 0)

  def is_internal(self):
    return self.get_type() in Panda.INTERNAL_DEVICES

  def get_serial(self):
    """
      Returns the comma-issued dongle ID from our provisioning
    """
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd0, 0, 0, 0x20)
    hashsig, calc_hash = dat[0x1c:], hashlib.sha1(dat[0:0x1c]).digest()[0:4]
    assert(hashsig == calc_hash)
    return [dat[0:0x10].decode("utf8"), dat[0x10:0x10 + 10].decode("utf8")]

  def get_usb_serial(self):
    """
      Returns the serial number reported from the USB descriptor;
      matches the MCU UID
    """
    return self._serial

  def get_dfu_serial(self):
    return PandaDFU.st_serial_to_dfu_serial(self._serial, McuType.H7)

  def get_uid(self):
    """
      Returns the UID from the MCU
    """
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xc3, 0, 0, 12)
    return binascii.hexlify(dat).decode()

  def get_secret(self):
    return self._handle.controlRead(Panda.REQUEST_IN, 0xd0, 1, 0, 0x10)

  def get_interrupt_call_rate(self, irqnum):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xc4, int(irqnum), 0, 4)
    return struct.unpack("I", dat)[0]

  # ******************* configuration *******************

  def set_alternative_experience(self, alternative_experience, safety_param_sp=0):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdf, int(alternative_experience), int(safety_param_sp), b'')

  def set_power_save(self, power_save_enabled=0):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe7, int(power_save_enabled), 0, b'')

  def enter_stop_mode(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, Panda.WAKE_MONITOR_REQUEST, 0, 0, b'', expect_disconnect=True)

  def schedule_bootkick_test(self, delay_s):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb6, int(delay_s), 0, b'')

  def set_safety_mode(self, mode=CarParams.SafetyModel.silent, param=0):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdc, mode, param, b'')

  def set_obd(self, obd):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xdb, int(obd), 0, b'')

  def set_can_loopback(self, enable):
    # set can loopback mode for all buses
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe5, int(enable), 0, b'')

  def set_can_enable(self, bus_num, enable):
    # sets the can transceiver enable pin
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf4, int(bus_num), int(enable), b'')

  def set_can_speed_kbps(self, bus, speed):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xde, bus, int(speed * 10), b'')

  def set_can_data_speed_kbps(self, bus, speed):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf9, bus, int(speed * 10), b'')

  def set_canfd_non_iso(self, bus, non_iso):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xfc, bus, int(non_iso), b'')

  def set_canfd_auto(self, bus, auto):
      self._handle.controlWrite(Panda.REQUEST_OUT, 0xe8, bus, int(auto), b'')

  def set_uart_baud(self, uart, rate):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe4, uart, int(rate / 300), b'')

  def set_uart_parity(self, uart, parity):
    # parity, 0=off, 1=even, 2=odd
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe2, uart, parity, b'')

  def set_uart_callback(self, uart, install):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe3, uart, int(install), b'')

  # ******************* can *******************

  # The panda will NAK CAN writes when there is CAN congestion.
  # libusb will try to send it again, with a max timeout.
  # Timeout is in ms. If set to 0, the timeout is infinite.
  CAN_SEND_TIMEOUT_MS = 10

  def can_reset_communications(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xc0, 0, 0, b'')

  @ensure_can_packet_version
  def can_send_many(self, arr, *, fd=False, timeout=CAN_SEND_TIMEOUT_MS):
    snds = pack_can_buffer(arr, chunk=(not self.spi), fd=fd)
    for tx in snds:
      while len(tx) > 0:
        bs = self._handle.bulkWrite(3, tx, timeout=timeout)
        tx = tx[bs:]

  def can_send(self, addr, dat, bus, *, fd=False, timeout=CAN_SEND_TIMEOUT_MS):
    self.can_send_many([[addr, dat, bus]], fd=fd, timeout=timeout)

  @ensure_can_packet_version
  def can_recv(self):
    dat = bytearray()
    while True:
      try:
        dat = self._handle.bulkRead(1, 16384) # Max receive batch size + 2 extra reserve frames
        break
      except (usb1.USBErrorIO, usb1.USBErrorOverflow):
        logger.error("CAN: BAD RECV, RETRYING")
        time.sleep(0.1)
    msgs, self.can_rx_overflow_buffer = unpack_can_buffer(self.can_rx_overflow_buffer + dat)
    return msgs

  def can_clear(self, bus):
    """Clears all messages from the specified internal CAN ringbuffer as
    though it were drained.

    Args:
      bus (int): can bus number to clear a tx queue, or 0xFFFF to clear the
        global can rx queue.

    """
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf1, bus, 0, b'')

  # ******************* serial *******************

  def serial_read(self, port_number, maxlen=1024):
    ret = b''
    while 1:
      r = bytes(self._handle.controlRead(Panda.REQUEST_IN, 0xe0, port_number, 0, 0x40))
      if len(r) == 0 or len(ret) >= maxlen:
        break
      ret += r
    return ret

  def serial_write(self, port_number, ln):
    ret = 0
    if isinstance(ln, str):
      ln = bytes(ln, 'utf-8')
    for i in range(0, len(ln), 0x20):
      ret += self._handle.bulkWrite(2, struct.pack("B", port_number) + ln[i:i + 0x20])
    return ret

  def send_heartbeat(self, engaged=True, engaged_mads=True):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf3, engaged, engaged_mads, b'')

  # disable heartbeat checks for use outside of openpilot
  # sending a heartbeat will reenable the checks
  def set_heartbeat_disabled(self):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf8, 0, 0, b'')

  # ****************** Timer *****************
  def get_microsecond_timer(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xa8, 0, 0, 4)
    return struct.unpack("I", dat)[0]

  # ******************* IR *******************
  def set_ir_power(self, percentage):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb0, int(percentage), 0, b'')

  # ******************* Fan ******************
  def set_fan_power(self, percentage):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xb1, int(percentage), 0, b'')

  def get_fan_rpm(self):
    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xb2, 0, 0, 2)
    a = struct.unpack("H", dat)
    return a[0]

  # ****************** Siren *****************
  def set_siren(self, enabled):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xf6, int(enabled), 0, b'')

  # ****************** Debug *****************

  # arr: timer period
  # ccrN: channel N pulse length
  def set_clock_source_timer_params(self, arr, ccr1, ccr2, ccr3):
    param1 = ((ccr1 & 0xFF) << 8) | (ccr2 & 0xFF)
    param2 = ((ccr3 & 0xFF) << 8) | (arr & 0xFF)
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xe6, param1, param2, b'')

  def force_relay_drive(self, intercept_relay_drive, ignition_relay_drive):
    self._handle.controlWrite(Panda.REQUEST_OUT, 0xc5, (int(intercept_relay_drive) | int(ignition_relay_drive) << 1), 0, b'')

  def read_som_gpio(self) -> bool:
    r = self._handle.controlRead(Panda.REQUEST_IN, 0xc6, 0, 0, 1)
    return r[0] == 1
