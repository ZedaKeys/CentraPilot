"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections.abc import Sequence

import numpy as np

from cereal import messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import \
  NORMAL, PERSONALITY_MIN, PERSONALITY_MAX, A_CRUISE_MAX_BP, A_CRUISE_MAX_V, RISE_RATE, SMOOTH_DECEL_BP, \
  SMOOTH_DECEL_V, BRAKE_DEEPENING_JERK, BRAKE_RELEASE_JERK, ACCEL_RISE_JERK, SMOOTH_DECEL_LOOKAHEAD_T, \
  MIN_SMOOTH_BRAKE_NEED, HARD_BRAKE_TARGET_ACCEL, HARD_BRAKE_NEED, STOCK_A_CRUISE_MAX_V, STOCK_RISE_RATE, \
  LOWSPEED_COMFORT_CAP, LOWSPEED_CAP_MAX_V_EGO, LOWSPEED_CAP_MIN_TTC, LOWSPEED_CAP_MIN_VREL, \
  LOWSPEED_CAP_MIN_ALEAD, LOWSPEED_CAP_SAFETY_BUFFER, \
  LAUNCH_MAX_V_EGO, LAUNCH_VREL_ON, LAUNCH_VREL_FULL, LAUNCH_ALEAD_ON, LAUNCH_SUSTAIN_FRAMES, \
  LAUNCH_CEIL_FRAC, LAUNCH_B_SLEW_UP, LAUNCH_B_SLEW_DN

_ZERO_ACCEL_EPS = 1e-6


class AccelController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._frame = 0
    self._enabled = self._params.get_bool("AccelPersonalityEnabled")
    self._personality = NORMAL
    self._v_ego = 0.0
    self._last_target_accel = 0.0
    self._brake_need = 0.0
    self._decel_target = 0.0
    self._smooth_active = False
    self._bypassed = False
    self._lead_valid = False
    self._lead_drel = 0.0
    self._lead_vrel = 0.0
    self._lead_alead = 0.0
    self._launch_arm_cnt = 0
    self._launch_boost = 0.0
    self._read_params()

  def _read_params(self) -> None:
    self._enabled = self._params.get_bool("AccelPersonalityEnabled")
    if not self._enabled:
      self._personality = NORMAL
      return
    self._personality = get_sanitize_int_param("AccelPersonality", PERSONALITY_MIN, PERSONALITY_MAX, self._params)

  def update(self, sm: messaging.SubMaster) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._read_params()
    self._v_ego = sm['carState'].vEgo
    lead = sm['radarState'].leadOne
    self._lead_valid = bool(lead.status)
    self._lead_drel = float(lead.dRel)
    self._lead_vrel = float(lead.vRel)
    self._lead_alead = float(lead.aLeadK)
    self._update_launch_boost()
    self._frame += 1

  def _update_launch_boost(self) -> None:
    # Arm only on a sustained genuine lead pull-away; recompute the vRel-scaled boost factor every frame.
    if not self._enabled or self._personality == NORMAL:
      self._launch_arm_cnt = 0
      self._launch_boost = 0.0
      return
    gated = (self._lead_valid and self._v_ego < LAUNCH_MAX_V_EGO
             and self._lead_vrel >= LAUNCH_VREL_ON and self._lead_alead >= LAUNCH_ALEAD_ON)
    self._launch_arm_cnt = self._launch_arm_cnt + 1 if gated else 0
    if self._launch_arm_cnt < LAUNCH_SUSTAIN_FRAMES:
      target = 0.0
    else:
      target = float(np.clip((self._lead_vrel - LAUNCH_VREL_ON) / (LAUNCH_VREL_FULL - LAUNCH_VREL_ON), 0.0, 1.0))
    # slew the factor: rise rate-limited, faster fade-out (decay leads the catch-up so ego can't overshoot)
    self._launch_boost = float(np.clip(target, self._launch_boost - LAUNCH_B_SLEW_DN, self._launch_boost + LAUNCH_B_SLEW_UP))

  def _rise_jerk(self) -> float:
    # positive-accel onset jerk, lifted toward NORMAL by the launch boost (capped at NORMAL, never beyond)
    base = ACCEL_RISE_JERK[self._personality]
    return base + self._launch_boost * max(0.0, ACCEL_RISE_JERK[NORMAL] - base)

  def get_max_accel(self, v_ego: float) -> float:
    base = float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_V[self._personality]))
    if self._launch_boost <= 0.0:
      return base
    norm = float(np.interp(v_ego, A_CRUISE_MAX_BP, STOCK_A_CRUISE_MAX_V))
    return base + self._launch_boost * LAUNCH_CEIL_FRAC * max(0.0, norm - base)

  def get_rise_rate(self) -> float:
    base = RISE_RATE[self._personality]
    return base + self._launch_boost * max(0.0, STOCK_RISE_RATE - base)

  def get_decel_target(self, brake_need: float) -> float:
    return float(np.interp(max(0.0, float(brake_need)), SMOOTH_DECEL_BP, SMOOTH_DECEL_V[self._personality]))

  def smooth_target_accel(self, raw_target_accel: float, accel_trajectory: Sequence[float], t_idxs: Sequence[float],
                          should_stop: bool, reset: bool = False, stock_brake: bool = False) -> float:
    raw_target_accel = float(raw_target_accel)
    self._brake_need = self._compute_brake_need(raw_target_accel, accel_trajectory, t_idxs)
    self._decel_target = 0.0

    # disabled, reset, or blended/e2e braking: hand straight to the plan
    if reset or not self._enabled or (stock_brake and (raw_target_accel < 0.0 or self._brake_need >= MIN_SMOOTH_BRAKE_NEED)):
      self._bypassed = False
      return self._passthrough(raw_target_accel)

    self._bypassed = self._emergency_bypass(raw_target_accel, should_stop)
    if self._bypassed:
      return self._passthrough(raw_target_accel)

    if self._brake_need < MIN_SMOOTH_BRAKE_NEED:
      self._smooth_active = False
      slewed = self._slew(raw_target_accel)
      out = min(slewed, raw_target_accel) if raw_target_accel < 0.0 else slewed
      return self._finalize(self._apply_lowspeed_cap(out, raw_target_accel, should_stop))

    # front-load a gentle early brake, never weaker than the plan
    self._smooth_active = True
    self._decel_target = self.get_decel_target(self._brake_need)
    slewed = self._slew(min(raw_target_accel, self._decel_target))
    out = min(slewed, raw_target_accel)
    return self._finalize(self._apply_lowspeed_cap(out, raw_target_accel, should_stop))

  def _apply_lowspeed_cap(self, out: float, raw_target_accel: float, should_stop: bool) -> float:
    # Soften a firm low-speed gap-restoration brake toward the comfort cap, but never gentler than the
    # decel needed to stop within the current gap if the lead dead-stops. Hard on/off gates, recomputed
    # every frame from live lead state -> instant release (no stale soft value). Only ever softens.
    if (not self._enabled or should_stop or not self._lead_valid
        or self._v_ego >= LOWSPEED_CAP_MAX_V_EGO
        or self._lead_vrel <= LOWSPEED_CAP_MIN_VREL
        or self._lead_alead <= LOWSPEED_CAP_MIN_ALEAD):
      return out
    # only the firm-but-not-emergency band (deeper brakes already passed through _emergency_bypass)
    if not (HARD_BRAKE_TARGET_ACCEL < raw_target_accel < LOWSPEED_COMFORT_CAP):
      return out
    closing = -self._lead_vrel
    ttc = self._lead_drel / closing if closing > _ZERO_ACCEL_EPS else float('inf')
    if ttc <= LOWSPEED_CAP_MIN_TTC:
      return out
    # stop-in-gap physics floor: ego decel to halt within (gap - buffer), worst case lead dead-stops
    stop_in_gap = self._v_ego ** 2 / (2.0 * max(self._lead_drel - LOWSPEED_CAP_SAFETY_BUFFER, 0.1))
    capped = max(raw_target_accel, min(LOWSPEED_COMFORT_CAP, -stop_in_gap))
    return max(out, capped)

  def _compute_brake_need(self, raw_target_accel: float, accel_trajectory: Sequence[float], t_idxs: Sequence[float]) -> float:
    min_accel = float(raw_target_accel)
    for accel, t in zip(accel_trajectory, t_idxs, strict=False):
      if float(t) <= SMOOTH_DECEL_LOOKAHEAD_T:
        min_accel = min(min_accel, float(accel))
    return max(0.0, -min_accel)

  def _emergency_bypass(self, raw_target_accel: float, should_stop: bool) -> bool:
    return (self._mpc.crash_cnt > 0 or should_stop or
            raw_target_accel <= HARD_BRAKE_TARGET_ACCEL or self._brake_need >= HARD_BRAKE_NEED)

  def _slew(self, target_accel: float) -> float:
    target_accel = float(target_accel)
    if target_accel > self._last_target_accel:
      return self._slew_up(target_accel)
    step = BRAKE_DEEPENING_JERK[self._personality] * DT_MDL
    return self._clean_accel(max(target_accel, self._last_target_accel - step))

  def _slew_up(self, target_accel: float) -> float:
    rise_jerk = self._rise_jerk()
    if self._last_target_accel < 0.0:
      released = min(target_accel, self._last_target_accel + BRAKE_RELEASE_JERK * DT_MDL)
      if released <= 0.0:
        return self._clean_accel(released)
      return self._clean_accel(min(target_accel, rise_jerk * DT_MDL))
    step = rise_jerk * DT_MDL
    return self._clean_accel(min(target_accel, self._last_target_accel + step))

  def _passthrough(self, target_accel: float) -> float:
    self._smooth_active = False
    return self._finalize(target_accel)

  def _finalize(self, target_accel: float) -> float:
    target_accel = self._clean_accel(target_accel)
    self._last_target_accel = target_accel
    return target_accel

  @staticmethod
  def _clean_accel(accel: float) -> float:
    accel = float(accel)
    return 0.0 if abs(accel) < _ZERO_ACCEL_EPS else accel

  def enabled(self) -> bool:
    return self._enabled

  def personality(self):
    return self._personality

  def max_accel(self) -> float:
    return self.get_max_accel(self._v_ego)

  def brake_need(self) -> float:
    return self._brake_need

  def decel_target(self) -> float:
    return self._decel_target

  def smooth_active(self) -> bool:
    return self._smooth_active

  def bypassed(self) -> bool:
    return self._bypassed
