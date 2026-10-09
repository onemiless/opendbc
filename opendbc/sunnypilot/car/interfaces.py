"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import NamedTuple
from collections.abc import Callable

from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.sunnypilot.car.tesla.ars408.constants import TeslaRadarBackend
from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType, TeslaFlagsSP, TeslaSafetyFlagsSP
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackTypeTorqueSpace = Callable[[LatControlInputs, structs.CarParams.LateralTorqueTuning, bool], float]


class CarInterfaceBaseSP:
  @staticmethod
  def torque_from_lateral_accel_linear_in_torque_space(latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                                        gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    return latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)

  def torque_from_lateral_accel_in_torque_space(self) -> TorqueFromLateralAccelCallbackTypeTorqueSpace:
    return self.torque_from_lateral_accel_linear_in_torque_space



def setup_interfaces(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                     params_list: list[dict[str, str]] | None = None,
                     can_recv: CanRecvCallable | None = None, can_send: CanSendCallable | None = None) -> None:
  if params_list is None:
    params_list = []

  params_dict = {k: v for param in params_list for k, v in param.items()}

  _initialize_coop_steering(CP, CP_SP, params_dict)
  _initialize_tesla_mads_screen_button(CP, CP_SP, params_dict)
  _initialize_tesla_dynamic_auto_stock(CP, CP_SP, params_dict)
  _initialize_tesla_ap_hybrid(CP, CP_SP, params_dict)
  _initialize_tesla_auto_speed_limit(CP, CP_SP, params_dict)
  _initialize_tesla_radar_backend(CP, CP_SP, params_dict)
  _initialize_toyota(CP, CP_SP, params_dict)


def _initialize_toyota(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'toyota':
    toyota_stock_long = int(params_dict.get("ToyotaEnforceStockLongitudinal", 0)) == 1
    toyota_stop_and_go_hack = int(params_dict.get("ToyotaStopAndGoHack", 0)) == 1

    if toyota_stock_long:
      CP_SP.flags |= ToyotaFlagsSP.STOCK_LONGITUDINAL.value
      CP.alphaLongitudinalAvailable = False
      CP.openpilotLongitudinalControl = False
      CP.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    if toyota_stop_and_go_hack and CP.openpilotLongitudinalControl:
      CP_SP.flags |= ToyotaFlagsSP.STOP_AND_GO_HACK.value


def _initialize_coop_steering(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                              params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla':
    coop_steering = int(params_dict.get("TeslaCoopSteering", 0)) == 1
    if coop_steering:
      CP_SP.flags |= TeslaFlagsSP.COOP_STEERING.value


def _initialize_tesla_mads_screen_button(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                         params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla' and CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
    selection = int(params_dict.get("TeslaMadsScreenButton", MadsScreenButtonType.THREE_FINGER))
    if selection == 3:  # migrate legacy 5-finger value from when 4-finger option existed
      selection = MadsScreenButtonType.FIVE_FINGER
    if selection == MadsScreenButtonType.THREE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_3_FINGER
    elif selection == MadsScreenButtonType.FIVE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_5_FINGER


def _initialize_tesla_dynamic_auto_stock(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                         params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla' and CP.openpilotLongitudinalControl:
    dynamic_auto_stock = int(params_dict.get("DynamicAutoStock", 0)) == 1
    if dynamic_auto_stock:
      CP_SP.flags |= TeslaFlagsSP.DYNAMIC_AUTO_STOCK.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK


def _initialize_tesla_ap_hybrid(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla' and CP.openpilotLongitudinalControl and int(params_dict.get("TeslaApHybrid", 0)) == 1:
    CP_SP.flags |= TeslaFlagsSP.AP_HYBRID.value
    if int(params_dict.get("TeslaDynamicApLongitudinal", 0)) == 1:
      CP_SP.flags |= TeslaFlagsSP.DYNAMIC_AP_LONGITUDINAL.value
    CP_SP.safetyParam |= TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF | TeslaSafetyFlagsSP.AP_HYBRID_LATERAL_HANDOFF


def _initialize_tesla_auto_speed_limit(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                       _params_dict: dict[str, str]) -> None:
  if (CP.brand == 'tesla' and CP.openpilotLongitudinalControl and
      CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS):
    CP_SP.flags |= TeslaFlagsSP.AUTO_SPEED_LIMIT.value
    CP_SP.safetyParam |= TeslaSafetyFlagsSP.AUTO_SPEED_LIMIT


def _initialize_tesla_radar_backend(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                    params_dict: dict[str, str]) -> None:
  if CP.brand != "tesla":
    return
  try:
    backend = TeslaRadarBackend(int(params_dict.get("TeslaARS408Radar", TeslaRadarBackend.OEM)))
  except (TypeError, ValueError):
    backend = TeslaRadarBackend.OFF

  if backend == TeslaRadarBackend.ARS408:
    CP.radarUnavailable = False
    CP.deprecated.radarTimeStep = 1.0 / 14.0
    CP_SP.flags |= TeslaFlagsSP.ARS408_RADAR.value
    CP_SP.safetyParam |= TeslaSafetyFlagsSP.ARS408_RADAR
  elif backend == TeslaRadarBackend.OFF:
    CP.radarUnavailable = True
    CP_SP.flags |= TeslaFlagsSP.RADAR_DISABLED.value
