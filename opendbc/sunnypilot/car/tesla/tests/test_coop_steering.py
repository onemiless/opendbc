"""CoopSteering driver override: releasing the wheel must center the override even while the planner turns the same way."""
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.carcontroller import get_safety_CP
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

adjust = CoopSteeringCarController.adjust_slew_for_planner


@pytest.mark.parametrize("slew, planner, torque, override, expected", [
  # Same direction, override starting at 0 or growing away from 0: subtract the planner overlap (unchanged).
  (0.1, 0.05, 1.0, 0.0, 0.05), (0.1, 0.5, 1.0, 0.0, 0.0), (0.1, 0.05, 1.0, 2.0, 0.05), (-0.1, -0.5, 1.0, -2.0, 0.0),
  # Same direction while centering toward 0: the planner never cancels the decay.
  (-0.1, -0.5, 1.0, 2.0, -0.1), (0.1, 0.5, 1.0, -2.0, 0.1), (-0.1, -0.05, 0.0, 2.0, -0.1),
  # Opposing: grow opposite to the planner, scaled by driver effort (unchanged).
  (0.1, -0.05, 1.0, 2.0, 0.1 + 0.5 * 0.05), (-0.1, 0.05, 1.0, 2.0, -0.1 - 0.5 * 0.05),
  # No planner motion or no slew (unchanged).
  (0.1, 0.0, 1.0, 2.0, 0.1), (0.0, 0.5, 1.0, 2.0, 0.0),
])
def test_adjust_slew_for_planner(slew, planner, torque, override, expected):
  assert adjust(slew, planner, torque, override) == pytest.approx(expected)


def test_adjust_slew_for_planner_default_keeps_overlap_subtraction():
  assert adjust(-0.1, -0.5, 1.0) == 0.0


@pytest.mark.parametrize("sign", (1, -1))
def test_release_centers_while_planner_turns_the_same_way(sign):
  VM = VehicleModel(get_safety_CP())
  CP_SP = SimpleNamespace(flags=TeslaFlagsSP.COOP_STEERING.value)
  coop = CoopSteeringCarController()
  cs = SimpleNamespace(out=SimpleNamespace(vEgo=10., vEgoRaw=10., steeringTorque=0., steeringAngleDeg=0.))
  planner = 0.

  def step(torque, planner_step):
    nonlocal planner
    planner += planner_step
    cs.out.steeringTorque = torque
    cs.out.steeringAngleDeg = coop.update(planner, True, CP_SP, cs, VM).steeringAngleDeg  # the EPS follows the command
    return coop.angle_override

  for _ in range(50):  # 1 s of driver torque on a straight plan: the override builds
    built = step(sign * 2.0, 0.)
  assert sign * built > 5.
  # 2 s hands off while the planner turns 10 deg/s toward the side the override decays to (e.g. into a junction).
  overrides = [step(0., -sign * 0.2) for _ in range(100)]
  assert all(sign * (newer - older) <= 0 for older, newer in zip([built, *overrides[:-1]], overrides, strict=True))
  assert abs(overrides[-1]) < 0.05, overrides[-1]  # used to freeze where |centering step| <= |planner step|
