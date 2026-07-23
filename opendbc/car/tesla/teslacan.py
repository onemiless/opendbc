from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Returns the flipped signal value for DAS_steeringControlType on FSD 14
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  else:
    return ctrl_type


class TeslaCAN:
  def __init__(self, CP, packer, vehicle_packer=None):
    self.CP = CP
    self.packer = packer
    self.vehicle_packer = vehicle_packer
    self.jerk = 0.0

  def create_steering_control(self, angle, enabled):
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # with openpilot, after overriding w/ ANGLE_CONTROL the wheel snaps back to the original angle abruptly
    # so we now use LANE_KEEP_ASSIST to match stock FSD.
    # see carstate.py for more details
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, 1 if enabled else 0),
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_stock_lateral_handoff(self, steering_angle):
    values = {
      "DAS_steeringAngleRequest": -steering_angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 3,  # Internal handoff marker; panda safety blocks this frame.
    }
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, cruise_override):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    # ramping max jerk fixes jerkiness after gas override when above max speed
    self.jerk = 0 if cruise_override else (self.jerk + CarControllerParams.JERK_RATE_UP * DT_CTRL * 4)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": min(self.jerk, CarControllerParams.JERK_LIMIT_MAX), # ramping max jerk is enough for some reason
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_stock_longitudinal_handoff(self, das_control):
    values = {
      "DAS_setSpeed": das_control["DAS_setSpeed"],
      "DAS_accState": das_control["DAS_accState"],
      "DAS_aebEvent": 3,  # Internal handoff marker; panda safety blocks this frame.
      "DAS_jerkMin": das_control["DAS_jerkMin"],
      "DAS_jerkMax": das_control["DAS_jerkMax"],
      "DAS_accelMin": das_control["DAS_accelMin"],
      "DAS_accelMax": das_control["DAS_accelMax"],
      "DAS_controlCounter": das_control["DAS_controlCounter"],
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_stw_action_request(self, speed_control_state, counter):
    values = {
      "SpdCtrlLvr_Stat": speed_control_state,
      "SpdCtrlLvrStat_Inv": 0,
      "DTR_Dist_Rq": 0,
      "MC_STW_ACTN_RQ": counter,
    }
    return self.vehicle_packer.make_can_msg("STW_ACTN_RQ", CANBUS.vehicle, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
