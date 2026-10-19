#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")
class TestNoOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  TX_MSGS = []
class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""
  SAFETY_MODEL = CarParams.SafetyModel.silent
class TestNoOutputAmbient(common.SafetyTest):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput
  SAFETY_PARAM = 1

  TX_MSGS = []
  FWD_BUS_LOOKUP = {}
  TEMPLATE = bytes.fromhex("0bff804064f801")
  LEFT = bytes.fromhex("03ff000064a800")
  RIGHT = bytes.fromhex("03ff0000645001")

  def setUp(self):
    super().setUp()
    self.safety.set_timer(100)

  def ready(self):
    self._rx(common.make_msg(1, 0x679, 7, self.TEMPLATE))

  def test_fixed_red_left_and_right(self):
    for frame in (self.LEFT, self.RIGHT):
      self.setUp()
      self.ready()
      self.assertTrue(self._tx(common.make_msg(1, 0x679, 7, frame)))

  def test_requires_fresh_vehicle_template(self):
    self.assertFalse(self._tx(common.make_msg(1, 0x679, 7, self.LEFT)))
    self.setUp()
    self.ready()
    self.safety.set_timer(1000101)
    self.assertFalse(self._tx(common.make_msg(1, 0x679, 7, self.LEFT)))

  def test_rejects_every_other_shape_and_payload(self):
    self.ready()
    self.assertFalse(self._tx(common.make_msg(0, 0x679, 7, self.LEFT)))
    self.assertFalse(self._tx(common.make_msg(1, 0x679, 8, self.LEFT + b"\x00")))
    self.assertFalse(self._tx(common.make_msg(1, 0x678, 7, self.LEFT)))
    for index in range(7):
      frame = bytearray(self.LEFT)
      frame[index] ^= 1
      self.assertFalse(self._tx(common.make_msg(1, 0x679, 7, frame)), index)
    self.assertTrue(self._tx(common.make_msg(1, 0x679, 7, self.LEFT)))

  def test_three_second_thirty_frame_cap(self):
    for index in range(30):
      self.safety.set_timer(100 + index * 100000)
      self.ready()
      self.assertTrue(self._tx(common.make_msg(1, 0x679, 7, self.LEFT)))
    self.safety.set_timer(3000100)
    self.ready()
    self.assertFalse(self._tx(common.make_msg(1, 0x679, 7, self.LEFT)))


class TestAllOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass
class TestAllOutputPassthrough(TestAllOutput):
  SAFETY_PARAM = 1

  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
class TestSafetyFramework(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  def test_unsupported_safety_mode(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(common.make_msg(0, 0x123)))
if __name__ == "__main__":
  unittest.main()
