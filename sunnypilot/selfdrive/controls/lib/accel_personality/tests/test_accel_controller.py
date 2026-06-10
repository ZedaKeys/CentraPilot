"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import AccelController
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import \
  ECO, NORMAL, SPORT, PERSONALITY_MIN, PERSONALITY_MAX, A_CRUISE_MAX_BP, RISE_RATE, \
  STOCK_A_CRUISE_MAX_V, STOCK_RISE_RATE, HARD_BRAKE_TARGET_ACCEL, AccelerationPersonality

T_IDXS = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0]
_EPS = 1e-6


class FakeParams:
  def __init__(self, store=None):
    self.store = dict(store or {})

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def get(self, key, return_default=False):
    return int(self.store.get(key, 1))

  def put(self, key, val, block=False):
    self.store[key] = val


def make_sm(v_ego=20.0):
  return {'carState': SimpleNamespace(vEgo=v_ego)}


def make_controller(enabled=True, personality=NORMAL, crash_cnt=0):
  store = {"AccelPersonalityEnabled": enabled, "AccelPersonality": int(personality)}
  ctrl = AccelController(CP=SimpleNamespace(), mpc=SimpleNamespace(crash_cnt=crash_cnt), params=FakeParams(store))
  ctrl.update(make_sm())
  return ctrl


def flat_traj(value):
  return [float(value)] * len(T_IDXS)


def test_enum_source_parity():
  assert (ECO, NORMAL, SPORT) == (AccelerationPersonality.eco, AccelerationPersonality.normal, AccelerationPersonality.sport)
  assert (PERSONALITY_MIN, PERSONALITY_MAX) == (0, 2)


def test_disabled_forces_normal_and_stock_ceiling():
  ctrl = make_controller(enabled=False, personality=SPORT)
  assert ctrl.personality() == NORMAL
  assert not ctrl.enabled()
  for v in (0.0, 10.0, 25.0, 40.0):
    assert ctrl.get_max_accel(v) == pytest.approx(np.interp(v, A_CRUISE_MAX_BP, STOCK_A_CRUISE_MAX_V))
  assert ctrl.get_rise_rate() == STOCK_RISE_RATE


def test_disabled_passes_brake_through():
  ctrl = make_controller(enabled=False)
  for raw in (-1.5, -0.5, 0.0, 1.0):
    out = ctrl.smooth_target_accel(raw, flat_traj(raw), T_IDXS, should_stop=False)
    assert out == pytest.approx(raw, abs=_EPS)


def test_normal_matches_stock():
  ctrl = make_controller(personality=NORMAL)
  for v in (0.0, 5.0, 10.0, 25.0, 40.0):
    assert ctrl.get_max_accel(v) == pytest.approx(np.interp(v, A_CRUISE_MAX_BP, STOCK_A_CRUISE_MAX_V))
  assert ctrl.get_rise_rate() == STOCK_RISE_RATE


def test_ceiling_ordering_eco_lt_normal_lt_sport():
  eco, normal, sport = (make_controller(personality=p) for p in (ECO, NORMAL, SPORT))
  for v in (0.0, 10.0, 25.0, 40.0):
    assert eco.get_max_accel(v) < normal.get_max_accel(v) < sport.get_max_accel(v)


def test_rise_rate_ordering():
  assert RISE_RATE[ECO] < RISE_RATE[NORMAL] < RISE_RATE[SPORT]


def test_early_soft_braking_brakes_before_plan():
  ctrl = make_controller(personality=NORMAL)
  out = ctrl.smooth_target_accel(0.0, flat_traj(-1.0), T_IDXS, should_stop=False)
  assert out < 0.0
  assert ctrl.smooth_active()
  assert ctrl.brake_need() == pytest.approx(1.0)


@pytest.mark.parametrize("personality", [ECO, NORMAL, SPORT])
def test_never_weaker_than_plan_sustained_closing(personality):
  # never command less braking than the plan (route 000003da regression guard)
  ctrl = make_controller(personality=personality)
  for raw in [0.0, -0.2, -0.5, -0.9, -1.2, -1.5] + [-1.5] * 40:
    out = ctrl.smooth_target_accel(raw, flat_traj(raw), T_IDXS, should_stop=False)
    assert out <= raw + _EPS


@pytest.mark.parametrize("personality", [ECO, NORMAL, SPORT])
def test_never_weaker_random_walk(personality):
  rng = np.random.default_rng(0)
  ctrl = make_controller(personality=personality)
  for _ in range(500):
    raw = float(rng.uniform(-1.9, 1.5))
    traj = flat_traj(raw - float(rng.uniform(0.0, 0.6)))
    out = ctrl.smooth_target_accel(raw, traj, T_IDXS, should_stop=False)
    if raw < 0.0:
      assert out <= raw + _EPS


def test_hard_brake_bypass():
  ctrl = make_controller(personality=ECO)
  raw = HARD_BRAKE_TARGET_ACCEL - 0.5
  out = ctrl.smooth_target_accel(raw, flat_traj(raw), T_IDXS, should_stop=False)
  assert out == pytest.approx(raw, abs=_EPS)
  assert ctrl.bypassed()


def test_should_stop_bypass():
  ctrl = make_controller(personality=ECO)
  out = ctrl.smooth_target_accel(-1.0, flat_traj(-1.0), T_IDXS, should_stop=True)
  assert out == pytest.approx(-1.0, abs=_EPS)
  assert ctrl.bypassed()


def test_fcw_crash_cnt_bypass():
  ctrl = make_controller(personality=ECO, crash_cnt=3)
  out = ctrl.smooth_target_accel(-1.0, flat_traj(-1.0), T_IDXS, should_stop=False)
  assert out == pytest.approx(-1.0, abs=_EPS)
  assert ctrl.bypassed()


def test_e2e_brake_passthrough():
  ctrl = make_controller(personality=ECO)
  out = ctrl.smooth_target_accel(-1.0, flat_traj(-1.0), T_IDXS, should_stop=False, stock_brake=True)
  assert out == pytest.approx(-1.0, abs=_EPS)
  assert not ctrl.smooth_active()


def test_out_of_range_personality_clamps():
  ctrl = AccelController(CP=SimpleNamespace(), mpc=SimpleNamespace(crash_cnt=0),
                         params=FakeParams({"AccelPersonalityEnabled": True, "AccelPersonality": 99}))
  ctrl.update(make_sm())
  assert ctrl.personality() == PERSONALITY_MAX


def test_reset_passes_through():
  ctrl = make_controller(personality=ECO)
  out = ctrl.smooth_target_accel(0.0, flat_traj(-1.0), T_IDXS, should_stop=False, reset=True)
  assert out == pytest.approx(0.0, abs=_EPS)
  assert not ctrl.bypassed()
