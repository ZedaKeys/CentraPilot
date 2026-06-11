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
  STOCK_A_CRUISE_MAX_V, STOCK_RISE_RATE, HARD_BRAKE_TARGET_ACCEL, AccelerationPersonality, \
  LOWSPEED_COMFORT_CAP, LOWSPEED_CAP_MAX_V_EGO, LOWSPEED_CAP_SAFETY_BUFFER

T_IDXS = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0]
_EPS = 1e-6


def stop_in_gap(v_ego, dRel):
  return v_ego ** 2 / (2.0 * max(dRel - LOWSPEED_CAP_SAFETY_BUFFER, 0.1))


class FakeParams:
  def __init__(self, store=None):
    self.store = dict(store or {})

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def get(self, key, return_default=False):
    return int(self.store.get(key, 1))

  def put(self, key, val, block=False):
    self.store[key] = val


def make_sm(v_ego=20.0, lead=False, dRel=0.0, vRel=0.0, aLeadK=0.0):
  leadOne = SimpleNamespace(status=lead, dRel=dRel, vRel=vRel, aLeadK=aLeadK)
  return {'carState': SimpleNamespace(vEgo=v_ego),
          'radarState': SimpleNamespace(leadOne=leadOne)}


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


# --- low-speed comfort brake cap ---

def test_lowspeed_cap_softens_gap_restoration():
  # roomy gap, gentle closing, low speed: a firm -1.9 gap-restoration brake softens toward the comfort cap
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=7.0, lead=True, dRel=25.0, vRel=-4.0, aLeadK=-1.0))
  raw = -1.9
  out = ctrl.smooth_target_accel(raw, flat_traj(-0.5), T_IDXS, should_stop=False)
  assert out == pytest.approx(LOWSPEED_COMFORT_CAP, abs=_EPS)  # floor (-stop_in_gap) is gentler here, so comfort cap binds
  assert out > raw  # softened
  assert out <= -stop_in_gap(7.0, 25.0) + _EPS  # never gentler than the stop-in-gap floor


def test_lowspeed_cap_floor_overrides_comfort_when_gap_tight():
  # tight gap: stop-in-gap floor is firmer than the comfort cap -> floor binds, brake not softened to comfort
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=7.5, lead=True, dRel=11.0, vRel=-3.0, aLeadK=-1.0))
  raw = -1.9
  floor = -stop_in_gap(7.5, 11.0)  # ~-2.01, firmer than -1.5
  out = ctrl.smooth_target_accel(raw, flat_traj(-0.5), T_IDXS, should_stop=False)
  assert out <= LOWSPEED_COMFORT_CAP + _EPS  # not softened to the comfort cap
  assert out <= floor + 1e-3 or out == pytest.approx(raw, abs=_EPS)  # never gentler than the floor


def test_lowspeed_cap_floor_invariant_sweep():
  # the safety invariant: capped output is never gentler than max(raw, -stop_in_gap)
  rng = np.random.default_rng(1)
  ctrl = make_controller(personality=ECO)
  for _ in range(2000):
    v = float(rng.uniform(1.0, 8.4))
    d = float(rng.uniform(5.0, 40.0))
    vr = float(rng.uniform(-5.9, 0.0))
    al = float(rng.uniform(-2.4, 0.0))
    ctrl.update(make_sm(v_ego=v, lead=True, dRel=d, vRel=vr, aLeadK=al))
    raw = float(rng.uniform(-1.99, -1.51))  # firm band that can be capped
    out = ctrl.smooth_target_accel(raw, flat_traj(raw + 0.5), T_IDXS, should_stop=False)
    floor = -stop_in_gap(v, d)
    # safety invariant: output is never gentler than the stop-in-gap floor (unless the plan itself is gentler)
    assert out <= max(raw, floor) + 1e-6


def test_lowspeed_cap_gated_off_high_speed():
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=LOWSPEED_CAP_MAX_V_EGO + 1.0, lead=True, dRel=25.0, vRel=-4.0, aLeadK=-1.0))
  out = ctrl.smooth_target_accel(-1.9, flat_traj(-1.9), T_IDXS, should_stop=False)
  assert out == pytest.approx(-1.9, abs=_EPS)  # no cap above the speed gate


def test_lowspeed_cap_gated_off_hard_lead():
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=7.0, lead=True, dRel=20.0, vRel=-4.0, aLeadK=-3.2))  # lead braking hard
  out = ctrl.smooth_target_accel(-1.9, flat_traj(-1.9), T_IDXS, should_stop=False)
  assert out == pytest.approx(-1.9, abs=_EPS)  # hard-braking lead -> no softening, full brake


def test_lowspeed_cap_gated_off_no_lead():
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=7.0, lead=False))  # no radar lead (e.g. vision/cruise decel)
  out = ctrl.smooth_target_accel(-1.9, flat_traj(-1.9), T_IDXS, should_stop=False)
  assert out == pytest.approx(-1.9, abs=_EPS)


def test_lowspeed_cap_does_not_touch_emergency():
  # raw <= -2.0 hits _emergency_bypass before the cap; never softened
  ctrl = make_controller(personality=ECO)
  ctrl.update(make_sm(v_ego=7.0, lead=True, dRel=25.0, vRel=-4.0, aLeadK=-1.0))
  out = ctrl.smooth_target_accel(-2.4, flat_traj(-2.4), T_IDXS, should_stop=False)
  assert out == pytest.approx(-2.4, abs=_EPS)
  assert ctrl.bypassed()


def test_lowspeed_cap_off_when_disabled():
  ctrl = make_controller(enabled=False)
  ctrl.update(make_sm(v_ego=7.0, lead=True, dRel=25.0, vRel=-4.0, aLeadK=-1.0))
  out = ctrl.smooth_target_accel(-1.9, flat_traj(-1.9), T_IDXS, should_stop=False)
  assert out == pytest.approx(-1.9, abs=_EPS)  # off == stock
