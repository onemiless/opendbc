import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.tesla.values import TeslaSafetyFlagsSP


class TestTeslaAmbientSafety(unittest.TestCase):
  TEMPLATE = bytes.fromhex("0cffd5aa00f801")
  LEFT = bytes.fromhex("02ff000064a800")
  RIGHT = bytes.fromhex("02ff0000645001")

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()
    self.safety.set_timer(100)
    self.packer = CANPackerSafety("tesla_model3_party")
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)

  def rx(self, addr, bus, data):
    return self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def tx(self, data, bus=1, addr=0x679):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def ready(self, gear=1, speed=0):
    for name, values in (("DI_state", {"DI_autoparkState": 0}), ("DI_systemStatus", {"DI_gear": gear}),
                         ("DI_speed", {"DI_vehicleSpeed": speed})):
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety(name, 0, values))
    self.rx(0x679, 1, self.TEMPLATE)

  def test_only_left_or_right_red(self):
    for data in (self.LEFT, self.RIGHT):
      self.setUp()
      self.ready()
      self.assertTrue(self.tx(data))
      self.assertFalse(self.tx(data))  # No unbounded burst.

  def test_no_fresh_vehicle_context(self):
    self.assertFalse(self.tx(self.LEFT))
    self.ready()
    self.safety.set_timer(1000101)
    self.assertFalse(self.tx(self.LEFT))

  def test_wrong_bus_length_and_color(self):
    self.ready()
    self.assertFalse(self.tx(self.LEFT, bus=0))
    self.assertFalse(self.tx(self.LEFT, bus=2))
    self.assertFalse(self.tx(self.LEFT + b"\0"))
    for index, mask in ((1, 1), (2, 1), (3, 1), (4, 1), (5, 0x10), (6, 1), (0, 8)):
      data = bytearray(self.LEFT)
      data[index] ^= mask
      self.assertFalse(self.tx(data), (index, mask))
    self.assertTrue(self.tx(self.LEFT))

  def test_park_and_zero_speed_required(self):
    for gear, speed in ((4, 0), (3, 0), (2, 0), (1, 1), (1, -1)):
      self.setUp()
      self.ready(gear, speed)
      self.assertFalse(self.tx(self.LEFT))

  def test_rate_limit_and_rearm(self):
    self.ready()
    self.assertTrue(self.tx(self.LEFT))
    self.rx(0x679, 1, self.TEMPLATE)
    self.assertFalse(self.tx(self.RIGHT))
    self.safety.set_timer(1000100)
    self.ready()
    self.assertTrue(self.tx(self.RIGHT))

  def test_controls_active_or_vehicle_bus_missing(self):
    self.ready()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.tx(self.LEFT))
    self.safety.set_controls_allowed(False)
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.ready()
    self.assertFalse(self.tx(self.LEFT))

  def test_nooutput_mode_stays_blocked(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.assertFalse(self.tx(self.LEFT))

  def test_three_second_session_cap(self):
    for index in range(30):
      self.safety.set_timer(100 + index * 100000)
      self.ready()
      self.assertTrue(self.tx(self.LEFT))
    self.safety.set_timer(3000100)
    self.ready()
    self.assertFalse(self.tx(self.LEFT))
    self.safety.set_timer(3900100)
    self.ready()
    self.assertTrue(self.tx(self.RIGHT))

  def test_corrupt_gear_cannot_arm_test(self):
    self.ready(gear=4)
    self.rx(0x118, 0, bytes.fromhex("0000200000000000"))
    self.assertFalse(self.tx(self.LEFT))

  def test_bad_template_sources_cannot_arm_test(self):
    for bus in (0, 2):
      self.setUp()
      self.ready()
      self.safety.set_timer(2000100)
      self.rx(0x679, bus, self.TEMPLATE)
      self.assertFalse(self.tx(self.LEFT))
