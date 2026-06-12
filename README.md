# CentraPilot

A custom openpilot fork for the **2020 Hyundai Sonata Hybrid**, pre-configured with optimal lane centering defaults.

Based on [SunnyPilot](https://github.com/sunnypilot/sunnypilot) — keeping all its features (model selection, camera offset, live tuning, full Hyundai support) while baking in the tuning that fixes the common left-lane-hug issue.

## What's Different

| Setting | Stock SunnyPilot | CentraPilot |
|---|---|---|
| CameraOffset | 0.0 | **-0.05** (fixes left lane hug) |
| EnforceTorqueControl | OFF | **ON** |
| NNLC | OFF | OFF (already optimal) |

Everything else — DTRv6 model support, camera offset slider, live tuning, full Hyundai CAN fingerprinting, SunnyLink — works exactly the same as SunnyPilot.

## Quick Start

1. On your Comma 3X, go to **Settings → Software → Install custom software**
2. Enter: `github.com/ZedaKeys/CentraPilot`
3. After install and calibration, select **DTRv6** model in SunnyLink → Models
4. Fine-tune Camera Offset in SunnyLink → Models → Adjust Camera Offset if needed

## Recommended Settings

| Setting | Value |
|---|---|
| Driving Model | DTRv6 (best for Hyundai torque-steer) |
| NNLC | OFF |
| Camera Offset | -0.03 to -0.08 |
| Enforce Torque Control | Disabled (torque control active by default) |
| Self-tune | Enabled |
| MADS Steering | Remain active |

## Live Tuning While Driving

Open the Live Tuner panel while engaged. Primary parameters:

| Parameter | Effect | Your Sonata Default |
|---|---|---|
| latAccelOffset | Negative = car moves RIGHT (fix left-hug) | 0.0 (try -0.05) |
| latAccelFactor | Higher = more aggressive centering | 2.899 |
| friction | Higher = less dithering, less responsive | 0.090 |

## Hardware

- **Car:** 2020 Hyundai Sonata Hybrid (DN8)
- **Device:** Comma 3X
- **Harness:** Hyundai A (non-HDA2) or Hyundai R (HDA2)
- **Steering:** Torque-based (MDPS/LFA)
- **Official support:** Full (lateral + longitudinal)

## Branches

- `master` — Main development
- `release-tizi` — Stable branch for Comma 3X (tizi hardware)

## Maintenance

CentraPilot is kept in sync with SunnyPilot upstream. To update:

```bash
git remote add upstream https://github.com/sunnypilot/sunnypilot.git
git fetch upstream
git checkout master
git merge upstream/master
git push origin master
git checkout release-tizi && git merge master && git push origin release-tizi
```

## Custom Files Modified

- `common/params_keys.h` — Default CameraOffset, EnforceTorqueControl
- `system/version.py` — Added CentraPilot to recognized origins
- `sunnypilot/common/version.h` — CentraPilot version tracking
