#!/usr/bin/env python3
"""Synthetic CAN -> actual Tesla CI/controller -> freshly compiled safety."""
import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import sys
import time

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, FSD_14_FW, TeslaFlags, TeslaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.tesla.values import TeslaSafetyFlagsSP

ROOT = Path(inspect.getfile(CarInterface)).resolve().parents[3]


def packet(frame):
  return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])


def run():
  results = []
  for mode in ('legacy', 'legacy-fsd14', 'three-bit-489', 'three-bit-054'):
    fingerprint = gen_empty_fingerprint()
    fingerprint[2][0x293] = 4
    if mode == 'three-bit-489':
      fingerprint[2][0x489] = 8
    if mode == 'three-bit-054':
      fingerprint[0][0x054] = 8
    fw = [] if mode != 'legacy-fsd14' else [structs.CarParams.CarFw.new_message(
      ecu=structs.CarParams.Ecu.eps, address=0x730, fwVersion=FSD_14_FW[CAR.TESLA_MODEL_Y][0])]
    cp = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, fw, True, False, False)
    sp = CarInterface.get_params_sp(cp, CAR.TESLA_MODEL_Y, fingerprint, fw, True, False, False)
    sp.safetyParam |= TeslaSafetyFlagsSP.AP_HYBRID_LATERAL_HANDOFF
    three_bit = mode.startswith('three-bit')
    assert bool(cp.flags & TeslaFlags.STEERING_3_BIT) == three_bit
    assert bool(cp.safetyConfigs[0].safetyParam & TeslaSafetyFlags.STEERING_3_BIT) == three_bit
    assert not cp.dashcamOnly
    ci = CarInterface(cp, sp)
    ci.update([])
    packers = {bus: CANPacker(parser.dbc_name) for bus, parser in ci.can_parsers.items()}
    safety = libsafety_py.libsafety
    safety.set_current_safety_param_sp(sp.safetyParam)
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.tesla, cp.safetyConfigs[0].safetyParam) == 0
    safety.init_tests()
    now = time.monotonic_ns()

    def sample(stock_ctrl=0, *, ci=ci, packers=packers, safety=safety):
      nonlocal now
      now += 20_000_000
      values = {
        'DI_speed': {'DI_vehicleSpeed': 0},
        'DI_state': {'DI_cruiseState': 2, 'DI_speedUnits': 0, 'DI_digitalSpeed': 80},
        'DI_systemStatus': {'DI_gear': 4},
        'ESP_B': {'ESP_vehicleSpeed': 0, 'ESP_wheelSpeedsQF': 1, 'ESP_vehicleStandstillSts': 1},
        'ESP_status': {'ESP_driverBrakeApply': 1},
        'EPAS3S_sysStatus': {'EPAS3S_internalSAS': 0, 'EPAS3S_torsionBarTorque': 0, 'EPAS3S_eacStatus': 1},
        'DAS_control': {'DAS_accState': 4, 'DAS_accelMin': 0, 'DAS_accelMax': 0},
        'DAS_steeringControl': {'DAS_steeringControlType': stock_ctrl},
        'UI_warning': {'buckleStatus': 1},
      }
      frames = [packers[bus].make_can_msg(name, parser.bus, values.get(name, {}))
                for bus, parser in ci.can_parsers.items() for name in list(parser.vl) if isinstance(name, str)]
      cs, _ = ci.update([(now, frames)])
      safety.set_timer((now // 1000) & 0xffffffff)
      for frame in frames:
        safety.safety_rx_hook(packet(frame))
      return cs

    sample()
    cc = structs.CarControl()
    cc.enabled = cc.latActive = cc.longActive = True
    cc.actuators.steeringAngleDeg = 0
    cc.actuators.accel = 0
    safety.set_controls_allowed(True)
    safety.set_angle_meas(-1, 1)
    safety.set_desired_angle_last(0)
    _, sends = ci.apply(cc.as_reader(), structs.CarControlSP(), now)
    steer = next(frame for frame in sends if frame[0] == 0x488)
    shift = 5 if three_bit else 6
    expected = 2 if mode == 'legacy-fsd14' else 1
    assert steer[1][2] >> shift == expected, (mode, steer)
    assert safety.safety_tx_hook(packet(steer)), (mode, steer)
    safety.set_controls_allowed(False)
    assert not safety.safety_tx_hook(packet(steer)), mode
    handoff = ci.CC.tesla_can.create_stock_lateral_handoff(0)
    assert not safety.safety_tx_hook(packet(handoff)), mode
    safety.set_safety_hooks(structs.CarParams.SafetyModel.tesla, cp.safetyConfigs[0].safetyParam)
    safety.init_tests()
    lkas = 1 if mode == 'legacy-fsd14' else 2
    stock = packers[Bus.ap_party].make_can_msg('DAS_steeringControl', 2, {'DAS_steeringControlType': lkas})
    assert safety.safety_rx_hook(packet(stock))
    cs = sample(lkas)
    assert cs.stockLkas, mode
    assert safety.safety_fwd_hook(2, 0x488) == 0, mode
    cs = sample(0)
    assert not cs.stockLkas, mode
    for forbidden in range(3, 8 if three_bit else 4):
      msg = packers[Bus.party].make_can_msg('DAS_steeringControl', 0, {
        'DAS_steeringAngleRequest': 0, 'DAS_steeringControlType': forbidden})
      safety.set_controls_allowed(True)
      assert not safety.safety_tx_hook(packet(msg)), (mode, forbidden)
    results.append({'mode': mode, 'flags': cp.flags, 'safetyParam': cp.safetyConfigs[0].safetyParam,
                    'dbc': ci.can_parsers[Bus.party].dbc_name, 'active_steering': steer[1].hex(), 'passed': True})
  return {'source': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
          'interface': inspect.getfile(CarInterface), 'cases': results,
          'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


if __name__ == '__main__':
  result = run()
  Path(sys.argv[1]).write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(result, indent=2))
