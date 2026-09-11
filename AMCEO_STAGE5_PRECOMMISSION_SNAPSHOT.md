# AMCEO Stage 5 Precommission Snapshot

Snapshot updated after clean build: `2026-09-10 19:49:55 +08:00`
Repository: `/home/linyi666/PX4-Autopilot-water-air`  
Git revision: `b47ae65c1073eb5e43f7033591b5f2e2af1f80d9` (branch `AMCEO`)
Build target: `matek_h743_dummy`  
Hardware commissioning state: **HOLD — physical actuator isolation has not yet been confirmed**

## Safety Gate

No flight-controller connection, parameter command, flash command, module command, selector change, or hardware probe was performed while preparing this snapshot.

Before any hardware command, the user must explicitly confirm all of the following:

- propulsion cannot produce thrust;
- hydrofoil servos cannot produce effective mechanical motion;
- propulsion/servo power is disconnected, or equivalent physical isolation is in place;
- the flight controller, IMU, SD card, and ULog can still operate;
- the vehicle will remain disarmed throughout Stage 5.

Software disarm alone is not accepted as the physical safety barrier.

## Frozen Stage 4B Artifacts

Current release directory: `release/AMCEO_STAGE4B_20260910_b47ae65c10/`.
The clean rebuild completed `1065/1065`, exit 0, using GCC 9.3.1; all 12 host/source checks passed. Select the `.px4` in this release directory for subsequent commissioning, and verify `SHA256SUMS`. Build evidence is in its `BUILD_REPORT.md`. No control source or parameter definition changed during this rebuild.

| Artifact | Size | Modification time | SHA-256 |
|---|---:|---|---|
| `release/AMCEO_STAGE4B_20260910_b47ae65c10/matek_h743_dummy.px4` | 1,537,681 B | 2026-09-10 19:49:55.646105572 +0800 | `00c5760f1432d48475c9ec02151945de11379ac7adb796367e69be25a5867486` |
| `release/AMCEO_STAGE4B_20260910_b47ae65c10/matek_h743_dummy.elf` | 36,849,684 B | 2026-09-10 19:49:55.509791378 +0800 | `2a6d60d9af23a23cbeb5501556bcd78e283d52c1aece0cd7984956df61abdbab` |

Only the `.px4` file with the exact hash above is the current candidate for the Stage 5 flash step. The previous Stage 4 hashes and 2026-09-07 Stage 4B hashes (`01cdc277...a8174` / `ce4bc850...2645d`) are superseded. The prior snapshot is preserved in the release directory as `PREVIOUS_SNAPSHOT_20260907.md`. An unrelated MPC package found in the build directory was separately preserved and must not be used for these AMCEO tests.

Generated parameter metadata is also frozen:

| File | SHA-256 |
|---|---|
| `build/matek_h743_dummy/parameters.json` | `2e31c2dc4ce11c5d91c9a8687ab7daca49d937662dc10b58211e9a6a8bbd8e64` |
| `build/matek_h743_dummy/parameters.xml` | `2af67c88007e73b1fe68e91d6f39b946d438af8767e1db2193f32a4ee379cd6e` |

## Git Working Tree at Current Freeze

The Stage 2B–4B implementation is now committed in `b47ae65c10`. `git status --short` was empty before rebuilding. Afterward only this snapshot and the new release bundle differ; flight-control source/configuration is unchanged.

### Historical working tree from the 2026-09-07 freeze

The working tree contains the cumulative, intentionally uncommitted Stage 2B–4B implementation and reports:

```text
 M msg/CMakeLists.txt
 M src/modules/hydro_rate_control/CMakeLists.txt
 M src/modules/hydro_rate_control/HydroRateControl.cpp
 M src/modules/hydro_rate_control/HydroRateControl.hpp
 M src/modules/hydro_rate_control/hydro_rate_control_params.c
 M src/modules/logger/logged_topics.cpp
?? AMCEO_BUILD_ENVIRONMENT.md
?? AMCEO_INTEGRATION_BASELINE_AUDIT.md
?? AMCEO_STAGE2A_BUILD_BASELINE.md
?? AMCEO_STAGE2B_INFRASTRUCTURE_REPORT.md
?? AMCEO_STAGE3_LADRC_MCEO_REPORT.md
?? AMCEO_STAGE4_IMPLEMENTATION_REPORT.md
?? AMCEO_STAGE4B_NADRC_IMPLEMENTATION_REPORT.md
?? AMCEO_STAGE5_PRECOMMISSION_SNAPSHOT.md
?? msg/HydroRollControllerStatus.msg
?? src/modules/hydro_rate_control/AMCEO_ADRC.cpp
?? src/modules/hydro_rate_control/AMCEO_ADRC.hpp
?? src/modules/hydro_rate_control/ClassicalNADRC.cpp
?? src/modules/hydro_rate_control/ClassicalNADRC.hpp
?? src/modules/hydro_rate_control/HydroRollController.cpp
?? src/modules/hydro_rate_control/HydroRollController.hpp
?? src/modules/hydro_rate_control/LADRC.cpp
?? src/modules/hydro_rate_control/LADRC.hpp
?? src/modules/hydro_rate_control/MCEO_ADRC.cpp
?? src/modules/hydro_rate_control/MCEO_ADRC.hpp
?? src/modules/hydro_rate_control/RollReferenceGenerator.cpp
?? src/modules/hydro_rate_control/RollReferenceGenerator.hpp
?? src/modules/hydro_rate_control/test/
```

No Stage 5 hardware action or control-source modification has been made.

## Compiled Controller Parameter Defaults

These values come from the frozen firmware metadata. They are **not yet the current values stored on the physical flight controller**.

| Parameter | Compiled default | Status |
|---|---:|---|
| `HY_CADRC_EN` | 1 | legacy compatibility |
| `HY_RLL_CTRL_SEL` | 1 | CADRC default |
| `HY_RLL_U_MAX` | 1.0 | **NOT YET FINAL HARDWARE TUNING** |
| `HY_RLL_DU_MAX` | 1.0 | **NOT YET FINAL HARDWARE TUNING** |
| `HY_RLL_DT_MAX` | 0.010 s | Stage 4 safety bound |
| `HY_RLL_MAN_TMO` | 0.5 s | **NOT YET FINAL HARDWARE TUNING** |
| `HY_RREF_W` | 2.0 rad/s | **NOT YET FINAL HARDWARE TUNING** |
| `HY_RREF_ZETA` | 1.0 | **NOT YET FINAL HARDWARE TUNING** |
| `HY_RLL_B0` | 10.1746 | Stage 3 starting value |
| `HY_RLL_APHI` | -0.5 1/s | Stage 3 starting value |
| `HY_RLL_K1` | 26.0 1/s² | Stage 3 starting value |
| `HY_RLL_K2` | 9.8 1/s | Stage 3 starting value |
| `HY_LADRC_WO` | 25.0 rad/s | Stage 3 starting value |
| `HY_MCEO_WO` | 22.0 rad/s | Stage 3 starting value |
| `HY_AM_WMIN` | 18.0 rad/s | Stage 4 starting value |
| `HY_AM_WMAX` | 46.0 rad/s | Stage 4 starting value |
| `HY_AM_RHO0` | 0.16 | Stage 4 starting value |
| `HY_AM_CE` | 0.70 | Stage 4 starting value |
| `HY_AM_CU` | 0.01 | Stage 4 starting value |
| `HY_AM_TRHO` | 0.055 s | Stage 4 starting value |
| `HY_AM_E1BAR` | 0.0523598776 rad | 3 deg in SI |
| `HY_AM_E2BAR` | 0.698131701 rad/s | 40 deg/s in SI |
| `HY_NA_WO` | 20.0 rad/s | Stage 4B starting value |
| `HY_NA_TDR` | 20.0 | Stage 4B Han TD algorithmic starting value |
| `HY_NA_TDN0` | 2.0 | Stage 4B TD step multiplier |
| `HY_NA_OA1` / `HY_NA_OA2` | 0.5 / 0.25 | Stage 4B NESO `fal` exponents |
| `HY_NA_ODEL` | 0.01 rad | Stage 4B NESO threshold |
| `HY_NA_CA1` / `HY_NA_CA2` | 0.8 / 1.2 | Stage 4B NLSEF `fal` exponents |
| `HY_NA_CDEL` | 0.01 | shared algorithmic NLSEF threshold |
| `HY_CADRC_J0` | 10.1746 | immutable legacy path |
| `HY_CADRC_KP1` | 500.0 | immutable legacy path |
| `HY_CADRC_KP2` | 1000.0 | immutable legacy path |
| `HY_CADRC_O1` | 15.0 | immutable legacy path |
| `HY_CADRC_O2` | 15.0 | immutable legacy path |
| `HY_CADRC_ALP` | 0.0 | immutable legacy path |
| `HY_CADRC_TAU` | 0.0025 s | immutable legacy path |
| `HY_CADRC_U_SCALE` | 0.001 | immutable legacy path |
| `HY_ROLL_MAX` | 30 deg | manual roll mapping |
| `HY_RR_P` / `HY_RR_I` / `HY_RR_D` | 0.05 / 0.1 / 0.0 | legacy rate branch |
| `HY_RR_IMAX` / `HY_RR_FF` | 0.2 / 0.5 | legacy rate branch |
| `HY_RLL_TO_YAW_FF` | 0.0 | downstream coupling |

The frozen `parameters.json` contains the complete compiled `HY_*` catalog, including pitch/yaw, depth, airspeed, estimator, and allocator parameters. A live whole-vehicle parameter export remains mandatory because compiled defaults do not establish the values currently stored on the vehicle.

All NADRC values above are initial bench/pool tuning values only and are not hardware validated. `HY_RREF_W` and `HY_RREF_ZETA` apply to IDs 2–4; ID 5 uses its private Han TD. The final selector map is `0 Legacy, 1 CADRC, 2 LADRC, 3 MCEO, 4 AMCEO, 5 NADRC`.

## Allocator and PWM Static Evidence

Compiled allocator defaults relevant to the hydrofoil path are:

| Parameter | Compiled default |
|---|---:|
| `HY_SV0_IDX` | 1 |
| `HY_SV1_IDX` | 1 |
| `HY_ALLOCATE_MODE` | 0 (Normal) |
| `HY_FOIL_AUX` | 0 |
| `HY_FOIL_THR` | 0.0 |
| `HY_FOIL_AUXGAIN` | 1.0 |
| `HY_WING_KL` | 5.0 |
| `HY_WING_MAX_A` | 0.8 rad |
| `HY_ALCT_SV_MAN` | 1.0 |
| `HY_ALCT_ROL_MAN` | 1.0 |
| `HY_SV_L_FM` / `HY_SV_R_FM` | 0.0 / 0.0 |

Both compiled servo indexes default to 1. These defaults must not be treated as a valid physical mapping.

Two repository historical parameter snapshots, `release/1.0/hydro3-250110.params` and `ulog_sync/25011601滑行测试/111.params`, agree on the following old configuration:

```text
HY_SV0_IDX = 5 -> hydro_servos.control[4] -> Servo function 205 -> PWM MAIN12
HY_SV1_IDX = 4 -> hydro_servos.control[3] -> Servo function 204 -> PWM MAIN11
PWM MAIN11/MIN/MAX = 800/2200 us
PWM MAIN12/MIN/MAX = 800/2200 us
PWM_MAIN_REV = 6273 (historical bitmask; MAIN12 reported reversed in the Stage 1 audit)
```

This is historical evidence only. It is not accepted as the current vehicle configuration.

Source-level software routing remains:

```text
logical side 0 -> hydro_servos.control[HY_SV0_IDX - 1]
logical side 1 -> hydro_servos.control[HY_SV1_IDX - 1]
-> ControlAllocator addition to the corresponding actuator servo index
-> configured Servo function
-> configured PWM output
```

**Software mapping is confirmed; physical left/right is not yet confirmed.** Live parameters, wiring observation, and physical labeling are required in Stage 5.

## Hardware Identity and Live Data Pending

The following cannot be truthfully populated without a physically isolated, connected vehicle:

- board ID / hardware version / serial identity;
- running firmware version and exposed git/build identity;
- boot console and module status;
- SD-card and logger health;
- original whole-vehicle parameter backup;
- current `HY_*`, allocator, Servo function, PWM reversal/min/max values;
- selector, IMU sign, reference, observer, scheduler, timing, and ULog evidence.

## Mandatory First Hardware Sequence After Safety Confirmation

1. Connect read-only while keeping the vehicle disarmed and actuator power physically isolated.
2. Export the complete original vehicle parameters to a new timestamped file; do not overwrite any earlier backup.
3. Hash and preserve that backup before flashing or changing any parameter.
4. Record board/firmware identity, boot console, module status, SD status, and logger status.
5. Re-check the `.px4` SHA-256 immediately before flashing.
6. Flash only `release/AMCEO_STAGE4B_20260910_b47ae65c10/matek_h743_dummy.px4` with SHA-256 `00c5760f1432d48475c9ec02151945de11379ac7adb796367e69be25a5867486`.
7. Export a second post-flash/pre-test parameter snapshot and compare it to the original.

No selector or temporary `HY_AM_CE` change is allowed until the original backup exists and is verified.
