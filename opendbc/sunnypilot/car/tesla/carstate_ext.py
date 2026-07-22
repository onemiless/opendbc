"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum
import time

from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import DBC, CANBUS
from opendbc.sunnypilot.car.tesla.dynamic_acc_debug import log_dynamic_acc
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

DYNAMIC_STOCK_MAX_SPEED_ERROR_KPH = 8.0
DYNAMIC_STOCK_MAX_SET_SPEED_OVERSHOOT_KPH = 3.0
DYNAMIC_STOCK_MAX_ACCEL_ERROR = 0.7
DYNAMIC_STOCK_MAX_ACCEL_MAX = 1.0
DYNAMIC_STOCK_MAX_EGO_ACCEL = 0.35
TESLA_AP_ACTIVE_STATES = frozenset((3, 4, 5, 6))
CURVE_PLAN_SOURCES = frozenset((1, 2))  # LongitudinalPlanSource.sccVision/sccMap
BLINKER_CONFIRM_S = 0.3
BLINKER_STALE_S = 0.4
PLAN_STALE_S = 0.2
LANE_CHANGE_STALE_S = 0.2
LATERAL_STABLE_S = 1.0


class TeslaLongitudinalSource(StrEnum):
  sp = "sp"
  dynamicStock = "dynamicStock"
  manualStock = "manualStock"
  apHybridStock = "apHybridStock"


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.active_touch_points = 0
    self.tesla_stock_longitudinal_active = False
    self.tesla_ap_hybrid_active = False
    self.prev_touch_points_for_long = 0
    self._dyn_enter_frames = 0
    self._dyn_exit_frames = 0
    self._dyn_cooldown_frames = 0
    self._dyn_manual_override = False
    self._dyn_manual_saw_sp_off = False
    self._dyn_debug_followup_frames = 0
    self._stock_counter_last = None
    self._init_longitudinal_override_state()
    self._read_dyn_params()

  def _init_longitudinal_override_state(self) -> None:
    self.tesla_longitudinal_source = TeslaLongitudinalSource.sp
    self.tesla_stock_longitudinal_active = False
    self.tesla_ap_hybrid_active = False
    self._ap_hybrid_restore_source = TeslaLongitudinalSource.sp

    self._blinker_last_counter = None
    self._blinker_first_active_time = 0.0
    self._blinker_last_sample_time = 0.0
    self._blinker_active_samples = 0
    self._blinker_confirmed = False
    self._blinker_reported_active = False
    self._blinker_seen = False

    self._plan_source = 0
    self._plan_valid = False
    self._plan_recv_time = 0.0
    self._curve_plan_samples = 0
    self._lane_change_active = False
    self._lane_change_valid = False
    self._lane_change_recv_time = 0.0
    self._context_clear_since = None

  def _set_longitudinal_source(self, source: TeslaLongitudinalSource) -> None:
    self.tesla_longitudinal_source = TeslaLongitudinalSource(source)
    self.tesla_stock_longitudinal_active = self.tesla_longitudinal_source != TeslaLongitudinalSource.sp
    self.tesla_ap_hybrid_active = self.tesla_longitudinal_source == TeslaLongitudinalSource.apHybridStock

  def _get_longitudinal_source(self) -> TeslaLongitudinalSource:
    if hasattr(self, "tesla_longitudinal_source"):
      return self.tesla_longitudinal_source
    return TeslaLongitudinalSource.dynamicStock if self.tesla_stock_longitudinal_active else TeslaLongitudinalSource.sp

  def _read_dyn_params(self):
    """Read dynamic auto-stock params from Params storage."""
    try:
      from openpilot.common.params import Params
      p = Params()
      self._dyn_enabled = p.get_bool("DynamicAutoStock") and bool(self.CP_SP.flags & TeslaFlagsSP.DYNAMIC_AUTO_STOCK)
      self._ap_hybrid_enabled = p.get_bool("TeslaApHybrid") and bool(self.CP_SP.flags & TeslaFlagsSP.AP_HYBRID)
      self._dyn_high = max(0, min(155, int(p.get("DynamicAutoStockSpeedKph", return_default=True) or 80)))
      self._dyn_low = max(0, min(155, int(p.get("DynamicAutoStockSpeedLowKph", return_default=True) or 70)))
    except Exception:
      self._dyn_enabled = False
      self._ap_hybrid_enabled = False
      self._dyn_high = 80
      self._dyn_low = 70
    self._dyn_high = (self._dyn_high // 5) * 5
    self._dyn_low = (self._dyn_low // 5) * 5
    if self._dyn_high == 0:
      self._dyn_high = 80
    if self._dyn_low >= self._dyn_high:
      self._dyn_low = max(0, self._dyn_high - 5)

  @staticmethod
  def _is_ap_active_state(autopilot_state: int) -> bool:
    return int(autopilot_state) in TESLA_AP_ACTIVE_STATES

  def _update_ap_hybrid(self, ret: structs.CarState, autopilot_state: int, speed_kph: float) -> bool:
    requested = (self._ap_hybrid_enabled and self._is_ap_active_state(autopilot_state) and
                 ret.cruiseState.enabled and not ret.accFaulted)
    if requested:
      if self._get_longitudinal_source() != TeslaLongitudinalSource.apHybridStock:
        self._ap_hybrid_restore_source = self._get_longitudinal_source()
        self._set_longitudinal_source(TeslaLongitudinalSource.apHybridStock)
        self._dyn_enter_frames = 0
        self._dyn_exit_frames = 0
        self._dyn_cooldown_frames = 200
        self._dyn_debug_followup_frames = 200
        self._log_dynamic_state("ap_hybrid_enter", ret, speed_kph,
                                restore_source=str(self._ap_hybrid_restore_source), autopilot_state=int(autopilot_state))
      return True

    if self._get_longitudinal_source() == TeslaLongitudinalSource.apHybridStock:
      restore_source = self._ap_hybrid_restore_source
      self._set_longitudinal_source(restore_source)
      self._ap_hybrid_restore_source = TeslaLongitudinalSource.sp
      self._dyn_enter_frames = 0
      self._dyn_exit_frames = 0
      self._dyn_cooldown_frames = 200
      self._dyn_debug_followup_frames = 200
      self._log_dynamic_state("ap_hybrid_exit", ret, speed_kph,
                              restore_source=str(restore_source), autopilot_state=int(autopilot_state))
    return False

  def update_longitudinal_context(self, plan_source: int, plan_updated: bool, plan_valid: bool, plan_recv_time: float,
                                  lane_change_active: bool, lane_change_valid: bool, now: float) -> None:
    if plan_updated:
      plan_source = int(plan_source)
      if plan_valid and plan_source in CURVE_PLAN_SOURCES:
        previous_curve_fresh = (self._plan_valid and self._plan_source in CURVE_PLAN_SOURCES and
                                plan_recv_time - self._plan_recv_time <= PLAN_STALE_S)
        self._curve_plan_samples = self._curve_plan_samples + 1 if previous_curve_fresh else 1
      else:
        self._curve_plan_samples = 0
      self._plan_source = plan_source
      self._plan_valid = bool(plan_valid)
      self._plan_recv_time = float(plan_recv_time)

    self._lane_change_active = bool(lane_change_active)
    self._lane_change_valid = bool(lane_change_valid)
    self._lane_change_recv_time = float(now)
    self._refresh_context_clear_since(now)

  def _update_blinker_sample(self, active: bool, counter: int, now: float) -> None:
    counter = int(counter) & 0xF
    if self._blinker_last_counter == counter:
      return

    self._blinker_last_counter = counter
    self._blinker_last_sample_time = float(now)
    self._blinker_seen = True
    self._blinker_reported_active = bool(active)
    if active:
      if self._blinker_active_samples == 0:
        self._blinker_first_active_time = float(now)
      self._blinker_active_samples += 1
      self._blinker_confirmed = (self._blinker_active_samples >= 3 and
                                 now - self._blinker_first_active_time >= BLINKER_CONFIRM_S)
    else:
      self._blinker_first_active_time = 0.0
      self._blinker_active_samples = 0
      self._blinker_confirmed = False
    self._refresh_context_clear_since(now)

  def _consume_blinker_samples(self, cp_party: CANParser, now: float) -> None:
    values = cp_party.vl_all["UI_warning"]
    counters = values["UI_warningCounter"]
    left = values["leftBlinkerBlinking"]
    right = values["rightBlinkerBlinking"]
    for index, counter in enumerate(counters):
      if index >= len(left) or index >= len(right):
        continue
      active = int(left[index]) in (1, 2) or int(right[index]) in (1, 2)
      self._update_blinker_sample(active, int(counter), now)

  def _blinker_fresh(self, now: float) -> bool:
    return self._blinker_seen and now - self._blinker_last_sample_time <= BLINKER_STALE_S

  def _blinker_force_active(self, now: float) -> bool:
    return self._blinker_fresh(now) and self._blinker_confirmed and self._blinker_reported_active

  def _blinker_known_inactive(self, now: float) -> bool:
    return self._blinker_fresh(now) and not self._blinker_reported_active

  def _plan_fresh(self, now: float) -> bool:
    return self._plan_valid and now - self._plan_recv_time <= PLAN_STALE_S

  def _curve_force_active(self, now: float) -> bool:
    return self._plan_fresh(now) and self._plan_source in CURVE_PLAN_SOURCES and self._curve_plan_samples >= 2

  def _lane_change_fresh(self, now: float) -> bool:
    return self._lane_change_valid and now - self._lane_change_recv_time <= LANE_CHANGE_STALE_S

  def _external_context_clear(self, now: float) -> bool:
    plan_clear = self._plan_fresh(now) and self._plan_source not in CURVE_PLAN_SOURCES
    lane_clear = self._lane_change_fresh(now) and not self._lane_change_active
    return self._blinker_known_inactive(now) and plan_clear and lane_clear

  def _refresh_context_clear_since(self, now: float) -> None:
    if self._external_context_clear(now):
      if self._context_clear_since is None:
        self._context_clear_since = float(now)
    else:
      self._context_clear_since = None

  def _stock_return_context_ready(self, now: float) -> bool:
    self._refresh_context_clear_since(now)
    return self._context_clear_since is not None and now - self._context_clear_since >= LATERAL_STABLE_S

  def _force_sp_reason(self, now: float) -> str | None:
    if self._blinker_force_active(now):
      return "blinker"
    if self._curve_force_active(now):
      return "visionCurve" if self._plan_source == 1 else "mapCurve"
    return None

  def _force_dynamic_stock_to_sp(self, reason: str, ret: structs.CarState, speed_kph: float) -> bool:
    if self._get_longitudinal_source() != TeslaLongitudinalSource.dynamicStock:
      return False
    self._set_longitudinal_source(TeslaLongitudinalSource.sp)
    self._dyn_cooldown_frames = 200
    self._dyn_enter_frames = 0
    self._dyn_exit_frames = 0
    self._dyn_debug_followup_frames = 200
    self._log_dynamic_state("dynamic_force_sp", ret, speed_kph, force_reason=reason)
    return True

  def _stock_longitudinal_ready(self, ret: structs.CarState, speed_kph: float) -> bool:
    if ret.brakePressed or ret.gasPressed or not ret.cruiseState.enabled or ret.accFaulted:
      return False

    das = getattr(self, "das_control", None)
    if das is None:
      return False

    stock_set_speed = float(das["DAS_setSpeed"])
    stock_speed_error = stock_set_speed - speed_kph
    stock_accel_max = float(das["DAS_accelMax"])
    stock_accel = (float(das["DAS_accelMin"]) + stock_accel_max) / 2.0
    stock_acc_active = int(das["DAS_accState"]) in (2, 3, 4, 5)
    return (stock_acc_active and
            int(das["DAS_aebEvent"]) == 0 and
            abs(stock_speed_error) < DYNAMIC_STOCK_MAX_SPEED_ERROR_KPH and
            stock_speed_error <= DYNAMIC_STOCK_MAX_SET_SPEED_OVERSHOOT_KPH and
            abs(stock_accel) < DYNAMIC_STOCK_MAX_ACCEL_ERROR and
            stock_accel_max <= DYNAMIC_STOCK_MAX_ACCEL_MAX and
            abs(ret.aEgo) < DYNAMIC_STOCK_MAX_EGO_ACCEL)

  def _toggle_stock_longitudinal_from_touch(self, ret: structs.CarState, speed_kph: float) -> bool:
    if not self.tesla_stock_longitudinal_active and not self._stock_longitudinal_ready(ret, speed_kph):
      self._log_dynamic_state("manual_rejected", ret, speed_kph)
      return False

    previous_source = self._get_longitudinal_source()
    new_source = TeslaLongitudinalSource.sp if self.tesla_stock_longitudinal_active else TeslaLongitudinalSource.manualStock
    self._set_longitudinal_source(new_source)
    self._dyn_cooldown_frames = 200
    self._dyn_enter_frames = 0
    self._dyn_exit_frames = 0
    self._dyn_manual_override = True
    self._dyn_manual_saw_sp_off = False
    self._dyn_debug_followup_frames = 200
    self._log_dynamic_state("manual_toggle", ret, speed_kph, previous_source=str(previous_source))
    return True

  def _log_dynamic_state(self, event: str, ret: structs.CarState, speed_kph: float, **extra) -> None:
    das = getattr(self, "das_control", {})
    log_dynamic_acc(
      "carstate_ext", event,
      stock_active=self.tesla_stock_longitudinal_active,
      longitudinal_source=str(self._get_longitudinal_source()),
      ap_hybrid_active=getattr(self, "tesla_ap_hybrid_active", False),
      dynamic_enabled=getattr(self, "_dyn_enabled", False),
      manual_override=self._dyn_manual_override,
      manual_saw_sp_off=self._dyn_manual_saw_sp_off,
      speed_kph=speed_kph,
      cruise_enabled=ret.cruiseState.enabled,
      cruise_available=getattr(ret.cruiseState, "available", False),
      brake_pressed=ret.brakePressed,
      gas_pressed=ret.gasPressed,
      acc_faulted=ret.accFaulted,
      ego_accel=ret.aEgo,
      das_acc_state=das.get("DAS_accState"),
      das_set_speed=das.get("DAS_setSpeed"),
      das_accel_min=das.get("DAS_accelMin"),
      das_accel_max=das.get("DAS_accelMax"),
      das_aeb_event=das.get("DAS_aebEvent"),
      das_counter=das.get("DAS_controlCounter"),
      dyn_enter_frames=self._dyn_enter_frames,
      dyn_exit_frames=self._dyn_exit_frames,
      dyn_cooldown_frames=self._dyn_cooldown_frames,
      plan_source=getattr(self, "_plan_source", 0),
      plan_age_s=max(0.0, time.monotonic() - getattr(self, "_plan_recv_time", 0.0)) if getattr(self, "_plan_recv_time", 0.0) else None,
      blinker_age_s=max(0.0, time.monotonic() - getattr(self, "_blinker_last_sample_time", 0.0)) if getattr(self, "_blinker_last_sample_time", 0.0) else None,
      blinker_counter=getattr(self, "_blinker_last_counter", None),
      lane_change_active=getattr(self, "_lane_change_active", False),
      **extra,
    )

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
      log_dynamic_acc("carstate_ext", "manual_override_rearmed")

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    # Auto-stock: use startup params only. Safety receives the same thresholds when the safety mode is set.
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    speed_kph = float(cp_party.vl["DI_speed"]["DI_vehicleSpeed"])
    now = time.monotonic()
    self._consume_blinker_samples(cp_party, now)
    autopilot_state = int(cp_ap_party.vl["DAS_status"]["DAS_autopilotState"])
    ap_hybrid_owns_longitudinal = self._update_ap_hybrid(ret, autopilot_state, speed_kph)

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
        ret.buttonEvents = [
          *ret.buttonEvents,
          *create_button_events(self.active_touch_points, prev_active_touch_points, {finger_count: ButtonType.lkas}),
        ]

      prev_touch_long = self.prev_touch_points_for_long
      self.prev_touch_points_for_long = self.active_touch_points
      if not ap_hybrid_owns_longitudinal and prev_touch_long != 4 and self.active_touch_points == 4:
        self._toggle_stock_longitudinal_from_touch(ret, speed_kph)

    self._update_dynamic_manual_override(ret.cruiseState.enabled)
    if self._dyn_enabled and not ap_hybrid_owns_longitudinal:
      self._dyn_cooldown_frames = max(0, self._dyn_cooldown_frames - 1)
      stock_counter = int(self.das_control["DAS_controlCounter"])
      stock_das_updated = self._stock_counter_last is None or stock_counter != self._stock_counter_last
      self._stock_counter_last = stock_counter
      stock_acc_active = int(self.das_control["DAS_accState"]) in (2, 3, 4, 5)
      stock_ready = self._stock_longitudinal_ready(ret, speed_kph)
      force_reason = self._force_sp_reason(now)
      if force_reason is not None:
        self._force_dynamic_stock_to_sp(force_reason, ret, speed_kph)

      enter_stock = (not self._dyn_manual_override and self._stock_return_context_ready(now) and
                     speed_kph > self._dyn_high and stock_ready)
      exit_stock = (not self._dyn_manual_override and speed_kph < self._dyn_low and stock_acc_active and
                    ret.cruiseState.enabled and not ret.brakePressed)

      self._dyn_enter_frames = self._dyn_enter_frames + 1 if enter_stock else 0
      self._dyn_exit_frames = self._dyn_exit_frames + 1 if exit_stock else 0

      if (self._dyn_enter_frames >= 100 and not self.tesla_stock_longitudinal_active and
          self._dyn_cooldown_frames == 0 and stock_das_updated):
        self._set_longitudinal_source(TeslaLongitudinalSource.dynamicStock)
        self._dyn_cooldown_frames = 200
        self._dyn_enter_frames = 0
        self._dyn_debug_followup_frames = 200
        self._log_dynamic_state("dynamic_enter_stock", ret, speed_kph)
      elif (self._dyn_exit_frames >= 100 and self.tesla_stock_longitudinal_active and
            self._dyn_cooldown_frames == 0 and stock_das_updated):
        self._set_longitudinal_source(TeslaLongitudinalSource.sp)
        self._dyn_cooldown_frames = 200
        self._dyn_exit_frames = 0
        self._dyn_debug_followup_frames = 200
        self._log_dynamic_state("dynamic_exit_stock", ret, speed_kph)

    if self._dyn_debug_followup_frames > 0:
      if self._dyn_debug_followup_frames % 25 == 0:
        self._log_dynamic_state("followup", ret, speed_kph, remaining_frames=self._dyn_debug_followup_frames)
      self._dyn_debug_followup_frames -= 1
    if self.tesla_stock_longitudinal_active:
      ret_sp.flags |= TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE.value
    if self.tesla_ap_hybrid_active:
      ret_sp.flags |= TeslaFlagsSP.AP_HYBRID_ACTIVE.value

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
