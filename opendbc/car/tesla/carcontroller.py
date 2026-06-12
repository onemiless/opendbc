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
    self._stock_entry_frames = 0  # Count frames since entering stock mode
    self._cancel_prev = False  # Track cancel state for falling-edge detection
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
        # On disengagement (cancel falling edge) while in stock mode, reset to SP
        # Note: for Tesla (pcmCruise=False), CC.cruiseControl.cancel = CS.cruiseState.enabled
        # so we detect the FALLING edge (True→False) to fire only once on disengagement.
        cancel_now = CC.cruiseControl.cancel
        if self._cancel_prev and not cancel_now and self.prev_stock_longitudinal:
          CS.tesla_stock_longitudinal_active = False
          self.prev_stock_longitudinal = False
          self._stock_entry_frames = 0
        self._cancel_prev = cancel_now

        if CS.tesla_stock_longitudinal_active and CS.das_control is not None:
          entering_stock = CS.tesla_stock_longitudinal_active and not self.prev_stock_longitudinal
          if entering_stock or self._stock_entry_frames > 0:
            if self._stock_entry_frames < 2 or not CS.out.cruiseState.enabled:
              # Always send at least 2 inactive entry frames before echo, so the safety
              # model's get_longitudinal_allowed() has time to stabilize before non-inactive
              # accel values are sent. Also send inactive if cruise disengages mid-stock.
              self._stock_entry_frames += 1
              state = 4
              accel = 0.0  # inactive raw=375, passes safety check
              cntr = (self.frame // 4) % 8
              can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.cruise_override))
            else:
              # Cruise confirmed engaged: start echoing car's DAS_control
              das = CS.das_control
              accel_min = max(das["DAS_accelMin"], -3.48)
              accel_max = min(max(das["DAS_accelMax"], 0), 2.0)
              values = {
                "DAS_setSpeed": das["DAS_setSpeed"],
                "DAS_accState": 4,  # Always send active — don't echo cancel/inactive
                "DAS_aebEvent": 0,
                "DAS_jerkMin": das["DAS_jerkMin"],
                "DAS_jerkMax": das["DAS_jerkMax"],
                "DAS_accelMin": accel_min,
                "DAS_accelMax": accel_max,
                "DAS_controlCounter": (self.frame // 4) % 8,
              }
              can_sends.append(self.packer.make_can_msg("DAS_control", CANBUS.party, values))
        elif not CS.tesla_stock_longitudinal_active:
          # SP longitudinal mode: send OP's own DAS_control
          leaving_stock = not CS.tesla_stock_longitudinal_active and self.prev_stock_longitudinal
          if leaving_stock:
            self._stock_entry_frames = 0
            state = 4
            accel = 0.0
          else:
            state = 13 if CC.cruiseControl.cancel else 4
            accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
          cntr = (self.frame // 4) % 8
          can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.cruise_override))

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
