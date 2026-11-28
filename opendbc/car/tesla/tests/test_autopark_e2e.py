"""Synthetic CAN -> actual Tesla CarInterface (CarState + CarController) -> freshly compiled panda safety.

Autopark is announced by the AP ECU through DAS_control.DAS_accState APC_* states on bus 2. While it runs,
sunnypilot must not transmit 0x488/0x27D/0x2B9 (panda forwards the OEM frames instead), must not report a
cruise engagement or an invalid LKAS setting, and must resume from the measured steering angle afterwards.
"""
import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

SP_ACTUATION = (0x488, 0x27D, 0x2B9)
APC_BACKWARD = 5
STEER_ANGLE = 12.0


def packet(frame):
  return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])


@pytest.mark.parametrize("sp_long", [False, True])
def test_autopark_yields_and_recovers(sp_long):
  fingerprint = gen_empty_fingerprint()
  cp = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], sp_long, False, False)
  sp = CarInterface.get_params_sp(cp, CAR.TESLA_MODEL_Y, fingerprint, [], sp_long, False, False)
  assert cp.openpilotLongitudinalControl == sp_long
  ci = CarInterface(cp, sp)
  ci.update([])
  packers = {bus: CANPacker(parser.dbc_name) for bus, parser in ci.can_parsers.items()}

  safety = libsafety_py.libsafety
  safety.set_current_safety_param_sp(sp.safetyParam)
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.tesla, cp.safetyConfigs[0].safetyParam) == 0
  safety.init_tests()
  now = 1_000_000_000

  def sample(acc_state=0, cruise_state=1, autopilot_state=2, oem_steer=0):
    nonlocal now
    now += 10_000_000
    values = {
      'DI_speed': {'DI_vehicleSpeed': 0},
      'DI_state': {'DI_cruiseState': cruise_state, 'DI_speedUnits': 0, 'DI_digitalSpeed': 0},
      'DI_systemStatus': {'DI_gear': 2},
      'ESP_B': {'ESP_vehicleSpeed': 0, 'ESP_wheelSpeedsQF': 1, 'ESP_vehicleStandstillSts': 1},
      'ESP_status': {'ESP_driverBrakeApply': 1},
      'EPAS3S_sysStatus': {'EPAS3S_internalSAS': -STEER_ANGLE, 'EPAS3S_torsionBarTorque': 0, 'EPAS3S_eacStatus': 1},
      'DAS_control': {'DAS_accState': acc_state, 'DAS_accelMin': 0, 'DAS_accelMax': 0},
      'DAS_steeringControl': {'DAS_steeringControlType': oem_steer, 'DAS_steeringAngleRequest': -STEER_ANGLE},
      'DAS_status': {'DAS_autopilotState': autopilot_state},
      'UI_warning': {'buckleStatus': 1},
    }
    frames = [packers[bus].make_can_msg(name, parser.bus, values.get(name, {}))
              for bus, parser in ci.can_parsers.items() for name in list(parser.vl) if isinstance(name, str)]
    cs, cs_sp = ci.update([(now, frames)])
    safety.set_timer((now // 1000) & 0xffffffff)
    for frame in frames:
      safety.safety_rx_hook(packet(frame))
    return cs, cs_sp

  # MADS lateral-only at standstill (the state that used to block Autopark)
  cc = structs.CarControl()
  cc.latActive = True
  cc.actuators.steeringAngleDeg = 0.0
  cc.cruiseControl.cancel = True  # worst case: a cancel request must not leak while Autopark runs

  def control():
    _, sends = ci.apply(cc.as_reader(), structs.CarControlSP(), now)
    return [s for s in sends if s[0] in SP_ACTUATION]

  for _ in range(20):
    cs, cs_sp = sample()
  assert not ci.CS.autopark
  safety.set_controls_allowed_lateral(True)

  # Autopark starts; OEM cruise and autopilot state both look "active"
  sp_sends = []
  for _ in range(20):
    cs, cs_sp = sample(acc_state=APC_BACKWARD, cruise_state=2, autopilot_state=3, oem_steer=1)
    sp_sends += control()
    assert ci.CS.autopark
    assert not cs.cruiseState.enabled
    assert not cs.invalidLkasSetting
    assert cs_sp.flags & TeslaFlagsSP.STOCK_PARKING_ACTIVE
  assert sp_sends == []
  for addr in SP_ACTUATION:
    assert safety.safety_fwd_hook(2, addr) == 0, hex(addr)
  assert not safety.get_controls_allowed()

  # Autopark exits; SP resumes from the measured wheel angle without fighting it
  cc.cruiseControl.cancel = False
  cc.actuators.steeringAngleDeg = STEER_ANGLE
  resumed = []
  for _ in range(20):
    cs, cs_sp = sample()
    resumed += control()
  assert not ci.CS.autopark
  assert not cs_sp.flags & TeslaFlagsSP.STOCK_PARKING_ACTIVE
  steer = [s for s in resumed if s[0] == 0x488]
  assert steer and any(s[0] == 0x27D for s in resumed)
  assert all(safety.safety_tx_hook(packet(s)) for s in resumed)
  assert safety.safety_fwd_hook(2, 0x488) == -1
  assert safety.safety_fwd_hook(2, 0x27D) == -1
  first = steer[0]
  angle_raw = ((first[1][0] & 0x7F) << 8) | first[1][1]
  assert abs(-(angle_raw * 0.1 - 1638.35) - STEER_ANGLE) < 0.2
