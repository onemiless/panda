from panda.tests.libpanda import libpanda_py


def test_tesla_door_open_wakes_som():
  libpanda_py.libpanda.wake_on_can = False
  libpanda_py.libpanda.wake_on_can_cnt = 10

  first = bytearray(7)
  first[1] = 0
  libpanda_py.libpanda.ignition_can_hook(libpanda_py.make_CANPacket(0x311, 0, first))
  assert not libpanda_py.libpanda.wake_on_can

  door_open = bytearray(7)
  door_open[1] = 1
  door_open[3] = 1 << 4
  libpanda_py.libpanda.ignition_can_hook(libpanda_py.make_CANPacket(0x311, 0, door_open))

  assert libpanda_py.libpanda.wake_on_can
  assert libpanda_py.libpanda.wake_on_can_cnt == 0
