"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.sunnypilot.car.tesla.carstate_ext import TeslaLongitudinalSource
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: 16,  # STW_ACTN_RQ.SpdCtrlLvr_Stat = UP_1ST
  SendButtonState.decrease: 32,  # STW_ACTN_RQ.SpdCtrlLvr_Stat = DN_1ST
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, tesla_can, frame, last_button_frame) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if not (self.CP_SP.flags & TeslaFlagsSP.SPEED_LIMIT_CRUISE_BUTTONS):
      return can_sends

    # Tesla AP already owns automatic speed-limit changes in hybrid mode. Dynamic
    # and manually selected stock ACC still use ICBM to adjust the OEM set speed.
    longitudinal_source = getattr(CS, "tesla_longitudinal_source", None)
    if longitudinal_source == TeslaLongitudinalSource.apHybridStock:
      return can_sends
    if longitudinal_source is None and getattr(CS, "tesla_stock_longitudinal_active", False):
      return can_sends

    if self.ICBM.sendButton != SendButtonState.none and self.ICBM.sendButton in BUTTONS:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        counter = (getattr(CS, "stw_action_counter", 0) + 1) % 16
        can_sends.append(tesla_can.create_stw_action_request(BUTTONS[self.ICBM.sendButton], counter))
        self.last_button_frame = self.frame

    return can_sends
