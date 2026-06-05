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


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.active_touch_points = 0
    self.tesla_stock_longitudinal_active = False
    self.prev_touch_points_for_long = 0

    self._dyn_enabled = False
    self._dyn_speed_high = 80
    self._dyn_speed_low = 70
    self._dyn_manual_override = False
    self._frame = 0

  def _read_dyn_params(self):
    from openpilot.common.params import Params
    p = Params()
    self._dyn_enabled = p.get_bool("DynamicAutoStock")
    self._dyn_speed_high = p.get_int("DynamicAutoStockSpeedKph", default=80)
    self._dyn_speed_low = p.get_int("DynamicAutoStockSpeedLowKph", default=70)

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    self._frame += 1
    if self._frame % 100 == 0:
      self._read_dyn_params()

    if self.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
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

      # 4-finger touch toggles stock longitudinal — pure memory toggle, zero I/O
      prev_touch_long = self.prev_touch_points_for_long
      self.prev_touch_points_for_long = self.active_touch_points
      if prev_touch_long != 4 and self.active_touch_points == 4:
        self.tesla_stock_longitudinal_active = not self.tesla_stock_longitudinal_active
        self._dyn_manual_override = True

    # Dynamic auto-stock based on speed
    speed_kph = ret.vEgo * CV.MS_TO_KPH
    if self._dyn_enabled and not self._dyn_manual_override:
      if speed_kph > self._dyn_speed_high:
        self.tesla_stock_longitudinal_active = True
      elif speed_kph < self._dyn_speed_low:
        self.tesla_stock_longitudinal_active = False
    # Reset manual override when speed crosses opposite boundary
    if self._dyn_manual_override:
      if self.tesla_stock_longitudinal_active and speed_kph < self._dyn_speed_low:
        self._dyn_manual_override = False
      elif not self.tesla_stock_longitudinal_active and speed_kph > self._dyn_speed_high:
        self._dyn_manual_override = False

    if self.tesla_stock_longitudinal_active:
      ret_sp.flags |= TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE.value
    cp_party = can_parsers[Bus.party]

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

    if CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
      messages[Bus.adas] = CANParser(DBC[CP.carFingerprint][Bus.adas], [], CANBUS.vehicle)

    return messages
