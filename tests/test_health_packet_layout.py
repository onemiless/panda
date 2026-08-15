import pytest

from panda import Panda


class FakeHandle:
  def __init__(self, packet):
    self.packet = packet

  def controlRead(self, *_args):
    return self.packet


@pytest.mark.parametrize("packed_state,lateral,longitudinal", [(0, 0, 0), (1, 1, 0), (2, 0, 1), (3, 1, 1)])
def test_sp_controls_and_temperature_fit_health_packet(packed_state, lateral, longitudinal):
  assert Panda.HEALTH_STRUCT.size <= 64
  values = [0] * 28
  values[26] = packed_state
  values[27] = 42.5

  panda = Panda.__new__(Panda)
  panda.health_version = Panda.HEALTH_PACKET_VERSION
  panda._handle = FakeHandle(Panda.HEALTH_STRUCT.pack(*values))
  health = panda.health()

  assert health["controls_allowed_lateral"] == lateral
  assert health["controls_allowed_longitudinal"] == longitudinal
  assert health["temperature"] == pytest.approx(42.5)
