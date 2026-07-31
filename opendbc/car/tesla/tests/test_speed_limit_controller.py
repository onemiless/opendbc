from types import SimpleNamespace

from opendbc.sunnypilot.car.tesla.speed_limit_controller import TeslaSpeedLimitController, create_speed_wheel_frame
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


IDLE_TEMPLATE = bytes.fromhex("2955000000000080")


def fake_state(current_speed=20.0, target_speed=25.0, template_time=1_000_000_000):
  return SimpleNamespace(
    out=SimpleNamespace(
      cruiseState=SimpleNamespace(enabled=True, speedCluster=current_speed),
      brakePressed=False,
    ),
    tesla_speed_limit_target=target_speed,
    tesla_speed_limit_target_valid=True,
    tesla_speed_button_template=IDLE_TEMPLATE,
    tesla_speed_button_template_nanos=template_time,
  )


def fake_control(enabled=True, cancel=False):
  return SimpleNamespace(enabled=enabled, cruiseControl=SimpleNamespace(cancel=cancel))


def test_speed_wheel_frame_changes_only_signed_right_tick():
  assert create_speed_wheel_frame(IDLE_TEMPLATE, 1) == bytes.fromhex("2955000100000080")
  assert create_speed_wheel_frame(IDLE_TEMPLATE, -1) == bytes.fromhex("2955003f00000080")


def test_controller_sends_one_tick_then_waits_for_speed_feedback():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state()

  sends = controller.update(fake_control(), state, 1_050_000_000)
  assert len(sends) == 1
  assert sends[0].address == 0x3C2
  assert sends[0].src == 1
  assert sends[0].dat == bytes.fromhex("2955000100000080")
  assert controller.update(fake_control(), state, 1_400_000_000) == []

  state.out.cruiseState.speedCluster = 20.3
  assert controller.update(fake_control(), state, 1_650_000_000) == []
  state.tesla_speed_button_template_nanos = 1_700_000_000
  assert len(controller.update(fake_control(), state, 1_900_000_000)) == 1


def test_controller_stops_at_target_and_when_controls_are_inactive():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0)
  assert controller.update(fake_control(), state, 1_050_000_000) == []

  state.out.cruiseState.speedCluster = 20.0
  assert controller.update(fake_control(enabled=False), state, 1_060_000_000) == []
  assert controller.update(fake_control(cancel=True), state, 1_070_000_000) == []


def test_controller_rejects_stale_template_or_invalid_limit():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = fake_state(template_time=1_000_000_000)
  assert controller.update(fake_control(), state, 1_400_000_001) == []

  state.tesla_speed_button_template_nanos = 1_500_000_000
  state.tesla_speed_limit_target_valid = False
  assert controller.update(fake_control(), state, 1_600_000_000) == []
