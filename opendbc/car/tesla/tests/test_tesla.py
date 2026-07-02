import json
import re
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CANBUS, CAR, FSD_14_FW
from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla import dynamic_acc_debug

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

# Hypothesized FSD 14 profile, in terms of variant_code bookends (given software_major >= 4):
#   M3: variant_code starts with '4H',  ends with '015'
#   MY: variant_code starts with '4',   ends with '003'
# Older series (M3 '014', MY '002') are never FSD 14.
FSD_14_FW_RULE = {
  CAR.TESLA_MODEL_3: (b'4H', b'015'),
  CAR.TESLA_MODEL_Y: (b'4',  b'003'),
}


class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULE:
        continue

      variant_prefix, variant_suffix = FSD_14_FW_RULE[car_model]
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)
        assert m is not None, f"Unparsable FW: {fw}"

        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
        expected = (
          m['variant_code'].startswith(variant_prefix)
          and m['variant_code'].endswith(variant_suffix)
          and int(m['software_major']) >= 4
        )
        assert is_fsd_14 == expected, f"{fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC


class TestTeslaLongitudinalHandoff(unittest.TestCase):
  def test_controller_reads_cruise_state_from_carstate_output(self):
    car_state = SimpleNamespace(out=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True)))
    self.assertTrue(CarController._cruise_enabled(car_state))

  def test_dynamic_acc_debug_writes_json_line(self):
    with tempfile.TemporaryDirectory() as temp_dir:
      log_path = Path(temp_dir) / "dynamic_acc_debug.log"
      with patch.object(dynamic_acc_debug, "DYNAMIC_ACC_DEBUG_PATH", str(log_path)):
        dynamic_acc_debug._append_dynamic_acc_debug({"source": "test", "event": "handoff"})

      self.assertEqual({"event": "handoff", "source": "test"}, json.loads(log_path.read_text()))

  def _stock_ready(self, *, acc_state=4, set_speed=86.0, speed_kph=80.0,
                   accel_min=0.2, accel_max=0.6, a_ego=0.2):
    car_state = CarStateExt.__new__(CarStateExt)
    car_state.das_control = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_accelMin": accel_min,
      "DAS_accelMax": accel_max,
    }
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=a_ego, cruiseState=SimpleNamespace(enabled=True))
    return car_state._stock_longitudinal_ready(ret, speed_kph)

  def _touch_toggle(self, *, initially_stock=False, acc_state=4):
    car_state = CarStateExt.__new__(CarStateExt)
    car_state.tesla_stock_longitudinal_active = initially_stock
    car_state._dyn_cooldown_frames = 0
    car_state._dyn_enter_frames = 10
    car_state._dyn_exit_frames = 10
    car_state._dyn_manual_override = False
    car_state._dyn_manual_saw_sp_off = False
    car_state.das_control = {
      "DAS_setSpeed": 80.0,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_accelMin": 0.0,
      "DAS_accelMax": 0.0,
    }
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True))
    changed = car_state._toggle_stock_longitudinal_from_touch(ret, 80.0)
    return changed, car_state

  def test_stock_handoff_rejects_inactive_oem_acc(self):
    self.assertFalse(self._stock_ready(acc_state=0, set_speed=80.0, accel_min=0.0, accel_max=0.0))
    self.assertFalse(self._stock_ready(acc_state=13, set_speed=80.0, accel_min=0.0, accel_max=0.0))

  def test_four_finger_does_not_enter_inactive_oem_acc(self):
    changed, car_state = self._touch_toggle(acc_state=0)
    self.assertFalse(changed)
    self.assertFalse(car_state.tesla_stock_longitudinal_active)

  def test_four_finger_can_enter_ready_oem_acc_and_always_leave(self):
    changed, car_state = self._touch_toggle(acc_state=4)
    self.assertTrue(changed)
    self.assertTrue(car_state.tesla_stock_longitudinal_active)
    self.assertEqual(200, car_state._dyn_cooldown_frames)
    self.assertEqual(0, car_state._dyn_enter_frames)
    self.assertEqual(0, car_state._dyn_exit_frames)

    changed, car_state = self._touch_toggle(initially_stock=True, acc_state=0)
    self.assertTrue(changed)
    self.assertFalse(car_state.tesla_stock_longitudinal_active)

  def test_manual_switch_pauses_dynamic_until_sp_is_reenabled(self):
    changed, car_state = self._touch_toggle(initially_stock=True, acc_state=4)
    self.assertTrue(changed)
    self.assertTrue(car_state._dyn_manual_override)

    car_state._update_dynamic_manual_override(cruise_enabled=True)
    self.assertTrue(car_state._dyn_manual_override)
    car_state._update_dynamic_manual_override(cruise_enabled=False)
    self.assertTrue(car_state._dyn_manual_override)
    car_state._update_dynamic_manual_override(cruise_enabled=True)
    self.assertFalse(car_state._dyn_manual_override)

  def test_dynamic_stock_handoff_is_reachable_with_matched_demand(self):
    self.assertTrue(self._stock_ready())

  def test_stock_handoff_rejects_unmatched_demand(self):
    self.assertFalse(self._stock_ready(set_speed=90.0))
    self.assertFalse(self._stock_ready(accel_min=0.8, accel_max=1.0))
    self.assertFalse(self._stock_ready(a_ego=0.5))

  def test_counter_resyncs_after_each_stock_period(self):
    controller = CarController.__new__(CarController)
    controller.long_control_counter = None

    self.assertEqual(3, controller._next_long_control_counter(2))
    self.assertEqual(4, controller._next_long_control_counter(7))
    self.assertEqual(7, controller._next_long_control_counter(6, resync=True))
    self.assertEqual(0, controller._next_long_control_counter(3))
    self.assertEqual(2, controller._next_long_control_counter(1, resync=True))

  def test_inactive_sp_takeover_preserves_enabled_cruise_with_zero_accel(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=True, long_active=False, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(4, state)
    self.assertEqual(0.0, accel)

  def test_sp_takeover_cancels_only_when_cruise_is_disabled(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=False, long_active=False, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(13, state)
    self.assertEqual(0.0, accel)

  def test_active_sp_takeover_preserves_control(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=True, long_active=True, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(4, state)
    self.assertEqual(-1.2, accel)

  def test_stock_handoff_uses_blocked_internal_marker(self):
    values = {
      "DAS_setSpeed": 50.0,
      "DAS_accState": 4,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": -1.0,
      "DAS_jerkMax": 0.5,
      "DAS_accelMin": -0.2,
      "DAS_accelMax": 0.3,
      "DAS_controlCounter": 6,
    }
    tesla_can = TeslaCAN(SimpleNamespace(flags=0), CANPacker("tesla_model3_party"))
    _, actual, _ = tesla_can.create_stock_longitudinal_handoff(values)
    aeb_event = actual[2] & 0x03

    self.assertEqual(3, aeb_event)

  def test_stw_action_request(self):
    tesla_can = TeslaCAN(SimpleNamespace(flags=0), CANPacker("tesla_model3_party"), CANPacker("tesla_model3_vehicle"))

    addr, dat, bus = tesla_can.create_stw_action_request(16, 7)

    self.assertEqual(0x238, addr)
    self.assertEqual(CANBUS.vehicle, bus)
    self.assertEqual(16, dat[0] & 0x3F)
    self.assertEqual(7, dat[6] >> 4)
