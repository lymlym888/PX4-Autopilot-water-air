# AMCEO precommission build — 2026-09-10

Decision: source/regression/clean target build PASS. Hardware commissioning remains pending.

## Firmware to select

Use `matek_h743_dummy.px4` in this directory for the subsequent physically isolated, disarmed commissioning. No flashing or hardware command was performed while producing this release.

PX4 SHA-256: `00c5760f1432d48475c9ec02151945de11379ac7adb796367e69be25a5867486`.

ELF SHA-256: `2a6d60d9af23a23cbeb5501556bcd78e283d52c1aece0cd7984956df61abdbab`.

Run `sha256sum -c SHA256SUMS` from this directory to verify the firmware and parameter metadata.

## Source and build identity

- Repository: `/home/linyi666/PX4-Autopilot-water-air`
- Revision: `b47ae65c1073eb5e43f7033591b5f2e2af1f80d9`
- Branch: `AMCEO`
- Firmware package identity: `v1.15.0-beta1-346-gb47ae65c10`
- Package board ID: `1013` (target metadata, not a live hardware identification)
- Target: `matek_h743_dummy`
- Compiler: GNU Arm Embedded Toolchain 9-2020-q2-update, GCC 9.3.1 20200408
- Configure Python: 3.12.3
- Build command: `make matek_h743_dummy`
- Environment: `PATH=/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH`, `CCACHE_DIR=/tmp/amceo-stage2a-ccache`
- Start condition: target build directory absent; previous generated build preserved under `build/amceo_superseded_mpc_20260910.8zdXTm/matek_h743_dummy`
- Result: `1065/1065`, exit 0
- Elapsed: real 58.63 s; user 368.79 s; sys 126.24 s
- FLASH: 1,598,072 B / 1,920 KiB (81.28%)
- AXI SRAM: 44,652 B / 512 KiB (8.52%); excludes unmeasured runtime heap/stack use
- PX4 package: 1,537,681 B; mtime `2026-09-10 19:49:55.646105572 +0800`
- ELF: 36,849,684 B; mtime `2026-09-10 19:49:55.509791378 +0800`

Known nonfatal CMake/Python warnings, including the previously observed SymForce circular import, remain in `clean_build.log`. No compile error, warning-policy change, or source workaround was used.

## Regression evidence

All eight host tests and all four source contracts passed before and after the build:

```text
CadrcGoldenTest
HydroRollInfrastructureTest
LadrcGoldenTest
MceoGoldenTest
AmceoGoldenTest
AmceoMatlabConsistencyTest
NadrcMathTest
NadrcGoldenTest
Stage2BSourceContractTest
Stage3SourceContractTest
Stage4SourceContractTest
Stage4BNadrcSourceContractTest
```

Host tests compile the actual controller sources with `-std=c++14 -Wall -Wextra -Werror`. The exact repeat command is `bash run_regression.sh`; the helper uses the repository's generated target headers for the CADRC host test. Results are in `regression.log`.

The source contracts enforce the CADRC/LADRC/MCEO/AMCEO frozen hashes. The protected allocator, standard control allocator, board and output paths also have no changes relative to the original `6abec104d4680a2c2c36499d665aec7ab18a3063` baseline.

The working tree was clean at build start. No flight-control source, algorithm, parameter definition, CMake, board or PWM configuration was edited for this build. Only this release bundle and the root precommission snapshot were added/updated afterward. `git diff --check` passes.

## Supersession and hardware boundary

The build directory previously contained an MPC-branch package (`b4caa456ec`, PX4 hash `ee7e93309afd40a1a66eff13f80def4a62ec5fcee907c52adb051cb016060078`). It is preserved for recovery but is not the AMCEO commissioning artifact.

This release supersedes the 2026-09-07 Stage 4B build hash `01cdc277...a8174` as the current commissioning candidate. That previous snapshot is retained as `PREVIOUS_SNAPSHOT_20260907.md`. The parameter JSON/XML hashes are identical to the previous Stage 4B metadata. Compiled defaults are not a backup of the physical vehicle's stored parameters.

Selector map: `0 Legacy, 1 CADRC, 2 LADRC, 3 MCEO, 4 AMCEO, 5 NADRC`; compiled default remains 1. Armed switching remains blocked. Existing vehicle parameter values may persist across flashing.

Before flashing, physically isolate propulsion and servo power, preserve a full original parameter export, and keep the vehicle disarmed. After flashing, first verify firmware identity, parameters, controller status and channel mapping with actuators inhibited. Servo movement requires a subsequent explicit check of the mechanical setup and the live mapping. Disarm alone does not prevent prearmed servo motion. No actuator movement or hydrodynamic performance has been verified by this build.
