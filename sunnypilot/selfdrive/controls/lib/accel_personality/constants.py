"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom

AccelerationPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ECO = AccelerationPersonality.eco
NORMAL = AccelerationPersonality.normal
SPORT = AccelerationPersonality.sport

PERSONALITY_MIN = min(AccelerationPersonality.schema.enumerants.values())
PERSONALITY_MAX = max(AccelerationPersonality.schema.enumerants.values())

# Accel ceiling. NORMAL is stock so a disabled controller (forced to NORMAL) is stock.
A_CRUISE_MAX_BP = [0., 10., 25., 40.]
STOCK_A_CRUISE_MAX_V = [1.6, 1.2, 0.8, 0.6]
STOCK_RISE_RATE = 0.05
A_CRUISE_MAX_V = {
  ECO:    [1.20, 0.85, 0.45, 0.30],
  NORMAL: STOCK_A_CRUISE_MAX_V,
  SPORT:  [1.75, 1.30, 0.90, 0.65],
}
RISE_RATE = {ECO: 0.02, NORMAL: STOCK_RISE_RATE, SPORT: 0.06}

# Early soft braking: predicted brake need (m/s^2) -> early decel target (m/s^2).
SMOOTH_DECEL_BP = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4]
SMOOTH_DECEL_V = {
  ECO:    [0.00, -0.10, -0.26, -0.52, -0.78, -1.00, -1.20],
  NORMAL: [0.00, -0.15, -0.36, -0.68, -1.00, -1.25, -1.50],
  SPORT:  [0.00, -0.17, -0.40, -0.72, -1.05, -1.35, -1.65],
}
BRAKE_DEEPENING_JERK = {ECO: 0.5, NORMAL: 0.8, SPORT: 1.0}
BRAKE_RELEASE_JERK = 2.0
ACCEL_RISE_JERK = {ECO: 0.7, NORMAL: 1.2, SPORT: 1.6}

SMOOTH_DECEL_LOOKAHEAD_T = 3.0
MIN_SMOOTH_BRAKE_NEED = 0.3
HARD_BRAKE_TARGET_ACCEL = -2.0
HARD_BRAKE_NEED = 2.6

# Low-speed comfort brake cap. Firm low-speed brakes (-1.5..-2.0) are mostly the MPC restoring
# follow-gap, not collision-avoidance. In a tightly-gated low-speed-following regime, soften the
# brake toward LOWSPEED_COMFORT_CAP -- but never gentler than the decel needed to stop within the
# current gap if the lead dead-stops (the stop-in-gap physics floor). Gated behind enabled (off==stock).
LOWSPEED_COMFORT_CAP = -1.5       # comfort target (m/s^2); the floor overrides it whenever the gap is tight
LOWSPEED_CAP_MAX_V_EGO = 8.5      # only below this speed (firm events sit at 7.9-8.1)
LOWSPEED_CAP_MIN_TTC = 4.0        # only when not genuinely closing
LOWSPEED_CAP_MIN_VREL = -6.0      # release if closing faster than this
LOWSPEED_CAP_MIN_ALEAD = -2.5     # release if the lead is braking harder than this
LOWSPEED_CAP_SAFETY_BUFFER = 4.0  # meters held in reserve for the stop-in-gap floor
