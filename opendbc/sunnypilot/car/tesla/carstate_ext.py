"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import DBC, CANBUS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

DYNAMIC_STOCK_MAX_SPEED_ERROR_KPH = 8.0
DYNAMIC_STOCK_MAX_ACCEL_ERROR = 0.7
DYNAMIC_STOCK_MAX_EGO_ACCEL = 0.35


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.active_touch_points = 0
    self.tesla_stock_longitudinal_active = False
    self.prev_touch_points_for_long = 0
    self._dyn_enter_frames = 0
    self._dyn_exit_frames = 0
    self._dyn_cooldown_frames = 0
    self._dyn_manual_override = False
    self._dyn_manual_saw_sp_off = False
    self._stock_counter_last = None
    self._read_dyn_params()

  def _read_dyn_params(self):
    """Read dynamic auto-stock params from Params storage."""
    try:
      from openpilot.common.params import Params
      p = Params()
      self._dyn_enabled = p.get_bool("DynamicAutoStock") and bool(self.CP_SP.flags & TeslaFlagsSP.DYNAMIC_AUTO_STOCK)
      self._dyn_high = max(0, min(155, int(p.get("DynamicAutoStockSpeedKph", return_default=True) or 80)))
      self._dyn_low = max(0, min(155, int(p.get("DynamicAutoStockSpeedLowKph", return_default=True) or 70)))
    except Exception:
      self._dyn_enabled = False
      self._dyn_high = 80
      self._dyn_low = 70
    self._dyn_high = (self._dyn_high // 5) * 5
    self._dyn_low = (self._dyn_low // 5) * 5
    if self._dyn_high == 0:
      self._dyn_high = 80
    if self._dyn_low >= self._dyn_high:
      self._dyn_low = max(0, self._dyn_high - 5)

  def _stock_longitudinal_ready(self, ret: structs.CarState, speed_kph: float) -> bool:
    if ret.brakePressed or ret.gasPressed or not ret.cruiseState.enabled or ret.accFaulted:
      return False

    das = getattr(self, "das_control", None)
    if das is None:
      return False

    stock_set_speed = float(das["DAS_setSpeed"])
    stock_accel = (float(das["DAS_accelMin"]) + float(das["DAS_accelMax"])) / 2.0
    stock_acc_active = int(das["DAS_accState"]) in (2, 3, 4, 5)
    return (stock_acc_active and
            int(das["DAS_aebEvent"]) == 0 and
            abs(stock_set_speed - speed_kph) < DYNAMIC_STOCK_MAX_SPEED_ERROR_KPH and
            abs(stock_accel) < DYNAMIC_STOCK_MAX_ACCEL_ERROR and
            abs(ret.aEgo) < DYNAMIC_STOCK_MAX_EGO_ACCEL)

  def _toggle_stock_longitudinal_from_touch(self, ret: structs.CarState, speed_kph: float) -> bool:
    if not self.tesla_stock_longitudinal_active and not self._stock_longitudinal_ready(ret, speed_kph):
      return False

    self.tesla_stock_longitudinal_active = not self.tesla_stock_longitudinal_active
    self._dyn_cooldown_frames = 200
    self._dyn_enter_frames = 0
    self._dyn_exit_frames = 0
    self._dyn_manual_override = True
    self._dyn_manual_saw_sp_off = False
    return True

  def _update_dynamic_manual_override(self, cruise_enabled: bool) -> None:
    if not self._dyn_manual_override:
      return

    if not cruise_enabled:
      self._dyn_manual_saw_sp_off = True
    elif self._dyn_manual_saw_sp_off:
      self._dyn_manual_override = False
      self._dyn_manual_saw_sp_off = False
      self._dyn_enter_frames = 0
      self._dyn_exit_frames = 0

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    # Auto-stock: use startup params only. Safety receives the same thresholds when the safety mode is set.
    cp_party = can_parsers[Bus.party]
    speed_kph = float(cp_party.vl["DI_speed"]["DI_vehicleSpeed"])

    # Process the real 4-finger edge before the dynamic state machine so a
    # manual decision always wins when both happen in the same update.
    if Bus.adas in can_parsers:
      cp_adas = can_parsers[Bus.adas]

      prev_active_touch_points = self.active_touch_points
      self.active_touch_points = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])

      finger_count = None
      if self.CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER:
        finger_count = 3
      elif self.CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER:
        finger_count = 5

      if finger_count is not None:
        ret.buttonEvents = [*create_button_events(self.active_touch_points, prev_active_touch_points,
                                                  {finger_count: ButtonType.lkas})]

      prev_touch_long = self.prev_touch_points_for_long
      self.prev_touch_points_for_long = self.active_touch_points
      if prev_touch_long != 4 and self.active_touch_points == 4:
        self._toggle_stock_longitudinal_from_touch(ret, speed_kph)

    self._update_dynamic_manual_override(ret.cruiseState.enabled)
    if self._dyn_enabled:
      self._dyn_cooldown_frames = max(0, self._dyn_cooldown_frames - 1)
      stock_counter = int(self.das_control["DAS_controlCounter"])
      stock_das_updated = self._stock_counter_last is None or stock_counter != self._stock_counter_last
      self._stock_counter_last = stock_counter
      stock_acc_active = int(self.das_control["DAS_accState"]) in (2, 3, 4, 5)
      stock_ready = self._stock_longitudinal_ready(ret, speed_kph)
      enter_stock = not self._dyn_manual_override and speed_kph > self._dyn_high and stock_ready
      exit_stock = (not self._dyn_manual_override and speed_kph < self._dyn_low and stock_acc_active and
                    ret.cruiseState.enabled and not ret.brakePressed)

      self._dyn_enter_frames = self._dyn_enter_frames + 1 if enter_stock else 0
      self._dyn_exit_frames = self._dyn_exit_frames + 1 if exit_stock else 0

      if (self._dyn_enter_frames >= 100 and not self.tesla_stock_longitudinal_active and
          self._dyn_cooldown_frames == 0 and stock_das_updated):
        self.tesla_stock_longitudinal_active = True
        self._dyn_cooldown_frames = 200
        self._dyn_enter_frames = 0
      elif (self._dyn_exit_frames >= 100 and self.tesla_stock_longitudinal_active and
            self._dyn_cooldown_frames == 0 and stock_das_updated):
        self.tesla_stock_longitudinal_active = False
        self._dyn_cooldown_frames = 200
        self._dyn_exit_frames = 0
    if self.tesla_stock_longitudinal_active:
      ret_sp.flags |= TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE.value
    cp_ap_party = can_parsers[Bus.ap_party]

    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)
    speed_limit = cp_ap_party.vl["DAS_status"]["DAS_fusedSpeedLimit"]
    if self.can_define.dv["DAS_status"]["DAS_fusedSpeedLimit"].get(int(speed_limit), None) in ["NONE", "UNKNOWN_SNA"]:
      ret_sp.speedLimit = 0
    else:
      if speed_units == "KPH":
        ret_sp.speedLimit = speed_limit * CV.KPH_TO_MS
      elif speed_units == "MPH":
        ret_sp.speedLimit = speed_limit * CV.MPH_TO_MS

  @staticmethod
  def get_parser(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> dict[StrEnum, CANParser]:
    messages = {}

    try:
      messages[Bus.adas] = CANParser(DBC[CP.carFingerprint][Bus.adas], [], CANBUS.vehicle)
    except (KeyError, Exception):
      pass  # Model X may not have Bus.adas in DBC

    return messages
