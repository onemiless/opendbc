from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


SWITCH_STATUS_ADDRESS = 0x3C2
VEHICLE_BUS = 1
TEMPLATE_MAX_AGE_NS = 300_000_000
MIN_TX_INTERVAL_NS = 500_000_000
FEEDBACK_TIMEOUT_NS = 1_200_000_000
TARGET_TOLERANCE_MS = 0.15
FEEDBACK_MIN_DELTA_MS = 0.05


def create_speed_wheel_frame(template: bytes, tick: int) -> bytes:
  if len(template) != 8 or (template[0] & 0x03) != 1 or (template[3] & 0x3F) != 0:
    raise ValueError("Tesla speed-wheel template must be an idle 0x3C2 mux-1 frame")
  if tick not in (-1, 1):
    raise ValueError("Tesla speed-wheel tick must be -1 or +1")

  data = bytearray(template)
  data[3] = (data[3] & 0xC0) | (tick & 0x3F)
  return bytes(data)


class TeslaSpeedLimitController:
  def __init__(self, CP_SP):
    self.configured = bool(CP_SP.flags & TeslaFlagsSP.AUTO_SPEED_LIMIT)
    self.last_tx_nanos = 0
    self.pending_since_nanos = 0
    self.pending_direction = 0
    self.pending_speed = 0.0

  def _reset_pending(self) -> None:
    self.pending_since_nanos = 0
    self.pending_direction = 0

  def update(self, CC, CS, now_nanos: int) -> list[CanData]:
    if (not self.configured or not CC.enabled or CC.cruiseControl.cancel or
        not CS.out.cruiseState.enabled or CS.out.brakePressed or
        bool(getattr(CS, "tesla_stock_longitudinal_active", False)) or
        not getattr(CS, "tesla_speed_limit_target_valid", False)):
      self._reset_pending()
      return []

    current_speed = float(CS.out.cruiseState.speedCluster)
    target_speed = float(CS.tesla_speed_limit_target)

    if self.pending_direction:
      feedback_delta = current_speed - self.pending_speed
      feedback_received = feedback_delta * self.pending_direction >= FEEDBACK_MIN_DELTA_MS
      feedback_timed_out = now_nanos - self.pending_since_nanos >= FEEDBACK_TIMEOUT_NS
      if not feedback_received and not feedback_timed_out:
        return []
      self._reset_pending()
      return []

    error = target_speed - current_speed
    if abs(error) <= TARGET_TOLERANCE_MS:
      return []
    if self.last_tx_nanos and now_nanos - self.last_tx_nanos < MIN_TX_INTERVAL_NS:
      return []

    template = getattr(CS, "tesla_speed_button_template", None)
    template_nanos = int(getattr(CS, "tesla_speed_button_template_nanos", 0))
    if template is None or now_nanos - template_nanos > TEMPLATE_MAX_AGE_NS:
      return []

    direction = 1 if error > 0.0 else -1
    data = create_speed_wheel_frame(template, direction)
    self.last_tx_nanos = now_nanos
    self.pending_since_nanos = now_nanos
    self.pending_direction = direction
    self.pending_speed = current_speed
    return [CanData(SWITCH_STATUS_ADDRESS, data, VEHICLE_BUS)]
