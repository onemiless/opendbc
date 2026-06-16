import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CANBUS, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    self.coop_steer = CoopSteeringCarController()
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)


    # Avoid echoing stale CANCEL (DAS_accState=13) on first entry into stock longitudinal mode
    self.prev_stock_longitudinal = False
    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Wait until the override condition clears before steering
    # Canceling is done on rising edge of CS.out.steeringDisengage and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and not CS.out.steeringDisengage

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)

      can_sends.append(self.tesla_can.create_steering_control(*self.coop_steer.update(self.apply_angle_last, lat_active, self.CP_SP, CS, self.VM)))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        # Auto-stock toggle: send fake UI_status2 to sync safety model
        if getattr(CS, '_toggle_request', False):
          CS._toggle_request = False
          can_sends.append([0x3DF, 0, b'\x00\x00\x00\x04\x00\x00\x00\x00', CANBUS.vehicle])

        # SP mode: send OP's own DAS_control. Stock mode: echo car's DAS values.
        # FWD always blocks → echo is the only source on bus 0. No double-send.
        if not CS.tesla_stock_longitudinal_active:
          # When leaving stock longitudinal back to OP longitudinal, avoid
          # sending CANCEL even if the state machine is disabled — the car's
          # ACC is already active and we want a seamless takeover.
          leaving_stock = not CS.tesla_stock_longitudinal_active and self.prev_stock_longitudinal
          if leaving_stock and CS.cruiseState.enabled:
            state = 4  # ACC_ON: preserve active cruise during transition
          else:
            state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
          accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
          cntr = (self.frame // 4) % 8
          can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.cruise_override))
        elif CS.das_control is not None:
          # Stock mode: echo car's DAS_control as fallback (FWD may be blocked for auto-stock).
          das = CS.das_control
          accel_min = max(das["DAS_accelMin"], -3.48)
          accel_max = min(max(das["DAS_accelMax"], 0), 2.0)
          values = {
            "DAS_setSpeed": das["DAS_setSpeed"],
            "DAS_accState": das["DAS_accState"],
            "DAS_aebEvent": 0,
            "DAS_jerkMin": das["DAS_jerkMin"],
            "DAS_jerkMax": das["DAS_jerkMax"],
            "DAS_accelMin": accel_min,
            "DAS_accelMax": accel_max,
            "DAS_controlCounter": (self.frame // 4) % 8,
          }
          can_sends.append(self.packer.make_can_msg("DAS_control", CANBUS.party, values))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, True))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.coop_steer.coop_apply_angle_sat_last # debug
    new_actuators.curvature = float(self.coop_steer.debug_angle_desired_limited) # debug
    new_actuators.torque = float(self.coop_steer.angle_override) # debug

    self.prev_stock_longitudinal = CS.tesla_stock_longitudinal_active
    self.frame += 1
    return new_actuators, can_sends
