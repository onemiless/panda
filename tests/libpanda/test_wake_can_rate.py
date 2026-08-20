from panda.tests.libpanda import libpanda_py


def _update(total_rx, prev_total_rx, wake_counter, wake_active):
  wake_active = libpanda_py.libpanda.can_wake_rate_update(total_rx, prev_total_rx, wake_counter, wake_active)
  return wake_active


def test_wake_can_rate_threshold_and_timeout():
  prev_total_rx = libpanda_py.ffi.new("uint32_t *", 0)
  wake_counter = libpanda_py.ffi.new("uint32_t *", 0)

  wake_active = _update(199, prev_total_rx, wake_counter, False)
  assert not wake_active

  wake_active = _update(399, prev_total_rx, wake_counter, wake_active)
  assert wake_active

  # A quiet bus must retain wake long enough for the SoM to start, then release it.
  for _ in range(5):
    wake_active = _update(399, prev_total_rx, wake_counter, wake_active)
    assert wake_active
  wake_active = _update(399, prev_total_rx, wake_counter, wake_active)
  assert not wake_active


def test_wake_can_rate_uses_wrapping_counter_delta():
  prev_total_rx = libpanda_py.ffi.new("uint32_t *", 0xFFFFFF9B)
  wake_counter = libpanda_py.ffi.new("uint32_t *", 0)

  # 101 frames before wrap plus 99 after wrap is exactly the 200 fps threshold.
  wake_active = _update(99, prev_total_rx, wake_counter, False)
  assert wake_active
