# AMCEO Stage 3 — LADRC and Fixed-Bandwidth MCEO-ADRC Report

## 1. Executive Summary

Stage 3 已完成 ID 2 LADRC 和 ID 3 fixed-bandwidth MCEO-ADRC 的源码实现、确定性/golden 测试、既有 CADRC 回归检查以及 `matek_h743_dummy` GCC 9 clean build。本阶段没有实现 ID 4 AMCEO-ADRC scheduler/equations，也没有实现 ID 5 ADRC variant。

两种新控制器共用 Stage 2B 的 `HydroRollControllerInput`、二阶 `RollReferenceGenerator`、公共 `b0/k_phi1/k_phi2`、normalized `u_requested`、公共 rate/magnitude limiter、reset/finite safety 和 `HydroRollControllerStatus`。两者没有独立 output scale，没有使用 `HY_CADRC_U_SCALE`，也没有在控制器内部加入 clamp、filter、integrator、nonlinear `fal` 或 anti-windup。

最终状态：

- mandatory pre-change Stage 2B tests：全部 PASS；
- LADRC/MCEO golden and structural tests：全部 PASS；
- post-clean 全部六项 host/static/golden tests：全部 PASS；
- clean PX4 build：`1064/1064`，exit code 0；
- CADRC、HydroAllocator、standard ControlAllocator、boards/PWM mapping：未修改；
- AMCEO `.cpp/.hpp`：保持 Stage 2B placeholder；
- 未烧录、未连接 actuator 做 active control、未进行水池实验。

## 2. Exact LADRC Equations Implemented

Implementation: `src/modules/hydro_rate_control/LADRC.cpp` and `.hpp`.

State meanings:

```text
z1 = estimated roll angle [rad]
z2 = estimated roll rate [rad/s]
z3 = estimated total disturbance [rad/s^2]
```

At sample `k`, innovation and fixed-bandwidth gains are:

```text
e_o = phi[k] - z1[k]
l1 = 3 omega_o
l2 = 3 omega_o^2
l3 = omega_o^3
omega_o = HY_LADRC_WO
```

The implemented continuous derivatives are exactly:

```text
z1_dot = z2 + l1 e_o
z2_dot = b0 u_applied[k-1] + z3 + l2 e_o
z3_dot = l3 e_o
```

After Euler propagation, feedback uses the updated observer states, not measured `p`:

```text
e1 = z1[k+1] - phi_ref[k]
e2 = z2[k+1] - phi_ref_dot[k]

v_phi = phi_ref_ddot[k] - k_phi1 e1 - k_phi2 e2
u_requested[k] = (v_phi - z3[k+1]) / b0
```

`u_requested` is a normalized controller request before the common limiter. Measured `input.p` appears only in validation and reset initialization; it is not used as normal LADRC feedback.

No nominal `a_phi z2` term is used in the LADRC ESO or compensation equation. Consequently its `z3` is the total-disturbance estimate under the specified LADRC decomposition.

## 3. Exact MCEO Equations Implemented

Implementation: `src/modules/hydro_rate_control/MCEO_ADRC.cpp` and `.hpp`.

State meanings:

```text
z1 = estimated roll angle [rad]
z2 = estimated roll rate [rad/s]
z3 = estimated residual disturbance f_phi [rad/s^2]
z4 = estimated residual-disturbance derivative [rad/s^3]
```

Innovation and fixed-bandwidth gains:

```text
e_o = phi[k] - z1[k]
l1 = 4 omega_o
l2 = 6 omega_o^2
l3 = 4 omega_o^3
l4 = omega_o^4
omega_o = HY_MCEO_WO
```

Implemented derivatives:

```text
z1_dot = z2 + l1 e_o
z2_dot = a_phi z2 + b0 u_applied[k-1] + z3 + l2 e_o
z3_dot = z4 + l3 e_o
z4_dot = l4 e_o
```

Control from updated states:

```text
e1 = z1[k+1] - phi_ref[k]
e2 = z2[k+1] - phi_ref_dot[k]

v_phi = phi_ref_ddot[k] - k_phi1 e1 - k_phi2 e2
u_requested[k] = (v_phi - a_phi z2[k+1] - z3[k+1]) / b0
```

MCEO retains `a_phi z2` in both observer and control compensation, and includes the independent fourth state `z4`. Its `z3` estimates residual disturbance after separating nominal damping; it is not physically interchangeable with LADRC total-disturbance `z3`.

No `rho`, `sigma_e`, `sigma_u`, `tau_rho`, time-varying bandwidth or adaptive gain exists in MCEO. `omega_o` is fixed at the applied `HY_MCEO_WO` value until a safe disarmed parameter update/reset.

## 4. Discrete-Time Execution Order

Both controllers use explicit forward Euler with `input.dt_used`. All derivatives in one propagation are evaluated from pre-step states, then all states are assigned their Euler increments.

The causal sample order is:

1. HydroRateControl receives `phi[k]`, `p[k]`, reference quantities and `HydroRollLimiter::appliedPrevious()`.
2. The selected observer evaluates innovation from `phi[k]` and propagates using `u_applied_prev = u_applied[k-1]`.
3. The controller computes `e1/e2`, `v_phi` and `u_requested[k]` from the newly propagated states.
4. The controller returns a normalized, not internally limited request.
5. The Stage 2B common limiter adds trim, rate-limits, magnitude-limits and produces `u_applied[k]`.
6. `u_applied[k]` is assigned to `hydro_torque_setpoint.xyz[0]` and retained by the limiter for the next sample.

For LADRC:

```text
z_i[k+1] = z_i[k] + dt_used * z_i_dot[k], i=1..3
```

For MCEO:

```text
z_i[k+1] = z_i[k] + dt_used * z_i_dot[k], i=1..4
```

`dt_used` for new controllers is the existing HydroRateControl gyro/control `dt`, already constrained by that module to `[0.002, 0.04] s`. The CADRC-specific `HY_CADRC_TAU` override remains confined to CADRC and is not reused.

## 5. Reset/Initialization

LADRC reset:

```text
z1 = input.phi
z2 = input.p
z3 = 0
```

MCEO reset:

```text
z1 = input.phi
z2 = input.p
z3 = 0
z4 = 0
```

Both reset implementations require finite `phi`, finite `p`, and a valid applied parameter set. Otherwise the controller becomes invalid and its states/errors are set to NaN. Measured `p` is intentionally used only to reduce activation transient at initialization.

On entry into the STAB/new-controller path or an accepted disarmed selector change:

- manual latch initializes at measured roll;
- reference generator initializes `phi_ref` at measured roll with zero derivative;
- common limiter initializes from the previous actual roll command;
- only the selected new controller is reset;
- the first valid request goes through the common rate and magnitude limits.

Leaving STAB resets the new-controller subsystem. A safely applied critical-parameter change also forces the active new controller to reset before its next control update. Inactive controllers are not updated.

## 6. Parameter Definitions and Defaults

Definitions are in `src/modules/hydro_rate_control/hydro_rate_control_params.c`:

| Parameter | Default | Validity/purpose |
|---|---:|---|
| `HY_RLL_B0` | 10.1746 | `>0`; nominal normalized-command-to-roll-acceleration effectiveness, `(rad/s²)/normalized-command` |
| `HY_RLL_APHI` | -0.5 | finite; nominal roll damping coefficient `[1/s]` |
| `HY_RLL_K1` | 26.0 | `>0`; roll-angle feedback coefficient `[1/s²]` |
| `HY_RLL_K2` | 9.8 | `>0`; roll-rate feedback coefficient `[1/s]` |
| `HY_LADRC_WO` | 25.0 rad/s | `>0`; LADRC observer bandwidth |
| `HY_MCEO_WO` | 22.0 rad/s | `>0`; fixed MCEO observer bandwidth |

These are engineering/simulation starting values and are not final hardware-validated parameters. They may be adjusted through PX4 parameter tuning before later bench/pool work, subject to the deferred-application policy below.

`HY_RLL_APHI` and `HY_RLL_B0` are nominal controller-model parameters. MATLAB simulation true-plant parameters `a_t` and `b_t` were not added to firmware and are not used by either controller.

Existing common Stage 2B parameters continue to supply reference and actuator-path policy:

- `HY_RREF_W`, `HY_RREF_ZETA`;
- `HY_RLL_U_MAX`, `HY_RLL_DU_MAX`;
- `HY_RLL_MAN_TMO`.

`HY_CADRC_U_SCALE` remains exclusive to legacy CADRC.

## 7. Applied-Command Observer Feedback Evidence

The common input assignment is:

```cpp
controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();
```

Both observer implementations use exactly that field:

```text
LADRC z2_dot = b0 * input.u_applied_prev + z3 + l2 * innovation
MCEO  z2_dot = a_phi*z2 + b0 * input.u_applied_prev + z3 + l2 * innovation
```

There is no controller input field for current `u_requested`, and neither observer references a current/raw request, CADRC raw `Uc`, output scale or downstream actuator servo.

Release-blocking golden tests intentionally choose a previous applied command of `0.3` while the newly calculated request differs. They verify:

```text
LADRC z2[k+1] = 0.2 + dt * b0 * 0.3
MCEO  z2[k+1] = 0.2 + dt * (a_phi * 0.2 + b0 * 0.3)
```

They also verify the difference between these propagations is exactly `dt*a_phi*z2`, demonstrating both the applied-command source and the required structural difference.

The feedback is the previous final common-limited roll torque contribution, not measured fin position and not a post-allocation saturation measurement. That limitation remains explicit for future system-identification work.

## 8. Diagnostic Mapping

The existing `HydroRollControllerStatus` schema is reused without breaking its Stage 2B layout.

LADRC mapping:

| Diagnostic | Value |
|---|---|
| `state_1` | `z1`, estimated roll angle |
| `state_2` | `z2`, estimated roll rate |
| `state_3` | `z3`, total-disturbance estimate |
| `state_4` | NaN |
| `disturbance_estimate` | `z3`, total disturbance |
| `observer_bandwidth` | applied `HY_LADRC_WO` |
| `rho`, `sigma_e`, `sigma_u` | NaN |

MCEO mapping:

| Diagnostic | Value |
|---|---|
| `state_1` | `z1`, estimated roll angle |
| `state_2` | `z2`, estimated roll rate |
| `state_3` | `z3`, residual-disturbance estimate |
| `state_4` | `z4`, residual-disturbance derivative estimate |
| `disturbance_estimate` | `z3`, residual disturbance |
| `observer_bandwidth` | applied fixed `HY_MCEO_WO` |
| `rho`, `sigma_e`, `sigma_u` | NaN |

Offline analysis must use `controller_id` before interpreting `state_3`/`disturbance_estimate`: LADRC and MCEO `z3` have intentionally different disturbance decompositions.

Diagnostics continue at up to 200 Hz with gyro `timestamp_sample`. `u_requested` is pre-limiter normalized output and `u_applied` is the actual final roll contribution published downstream.

## 9. Controller Selector Activation

The selector behavior after Stage 3 is:

| ID | Status | STAB behavior |
|---:|---|---|
| 0 | Legacy | existing legacy rate controller |
| 1 | CADRC | existing immutable CADRC, gated by `HY_CADRC_EN` |
| 2 | LADRC | implemented; can return `valid=true` after valid reset/input |
| 3 | fixed MCEO-ADRC | implemented; can return `valid=true` after valid reset/input |
| 4 | AMCEO-ADRC | unchanged placeholder, `valid=false`, legacy fallback |
| 5 | ADRC variant | no implementation, `valid=false`, legacy fallback |

`HY_RLL_CTRL_SEL` default remains 1, so existing CADRC remains the startup behavior. Armed selector changes continue to be blocked: requested ID is recorded, active ID/output remain unchanged, and warning/fault status is emitted. A blocked request is accepted only after disarm.

For ID 2/3, a valid new output replaces only the legacy roll contribution after the existing branch has supplied the other axes. Invalid input/output prevents the override and uses explicit legacy fallback. ID 4/5 invalid output is expected fallback and does not masquerade as a controller fault.

The first ID 2/3 frame cannot bypass the limiter: limiter state is initialized from the previous applied roll command, and `new_roll_applied` is assigned only after a valid limiter result.

## 10. Parameter Update Policy

The PX4 parameter subscription interval is 1 s. Runtime `parameter_update` still calls the existing `updateParams()` and `parameters_update()` functions, preserving CADRC's historical parameter behavior.

The six critical new-controller values use a separate applied/pending snapshot:

1. A changed value is read into a pending snapshot.
2. All six values are validated together: finite `a_phi`, and positive finite `b0`, `k1`, `k2`, LADRC bandwidth and MCEO bandwidth.
3. If armed, the pending set is not applied. The controller continues with its previous applied snapshot; one warning and fault-counter event records the deferral.
4. On disarm, a valid pending snapshot is atomically applied to both controller configurations.
5. If a new controller is active, its observer reset is required and performed before the next applicable controller update.
6. An invalid pending set is rejected; the last valid applied configuration remains in use and a warning/fault event is recorded.

Parameter float snapshots are compared bit-for-bit, avoiding `-Wfloat-equal` and ensuring a real PX4 parameter-storage change is detected. No critical gain or bandwidth changes silently during armed operation.

Stage 2B reference/limiter parameters are outside this six-value deferred set and retain their existing update behavior. Changing those while armed remains a documented risk and is not claimed safe by this stage.

## 11. Golden/Deterministic Test Results

Mandatory tests were run both before implementation and after the final clean build.

Final results:

```text
CadrcGoldenTest: PASS
Stage2BSourceContractTest: PASS
HydroRollInfrastructureTest: PASS
LadrcGoldenTest: PASS
MceoGoldenTest: PASS
Stage3SourceContractTest: PASS
git diff --check: PASS
```

New files:

- `src/modules/hydro_rate_control/test/LadrcGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/MceoGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/Stage3SourceContractTest.py`

Coverage:

- exact reset state for LADRC and MCEO;
- 50-sample zero equilibrium with finite states and approximately zero output;
- saved eight-sample float32 golden vectors containing every observer state, `e1`, `e2` and `u_requested`;
- nontrivial measurement trajectory/disturbance response;
- release-blocking previous-applied-command propagation;
- explicit LADRC/MCEO `a_phi` and fourth-state structural difference;
- NaN/Inf in `phi`, `p`, reference and `dt`;
- invalid/nonpositive `b0` rejection;
- controller request intentionally exceeding one, proving there is no internal clamp;
- common limiter magnitude and per-sample slew inequalities;
- source-level exact equation/order contracts;
- ID 4 AMCEO placeholder hash preservation and absence of scheduler terms;
- protected path diff verification.

Host test commands use `g++ -std=c++14 -Wall -Wextra -Werror` and compile the actual controller sources. These tests are deterministic but do not model real-time PX4 scheduling or actuator hardware.

## 12. Legacy CADRC Regression

The CADRC files retain their Stage 2B hashes:

```text
a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201  CADRC.cpp
ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54  CADRC.h
```

`CadrcGoldenTest` passed before Stage 3 and after the final clean build. `Stage2BSourceContractTest` also passed and verifies the unchanged CADRC call/scale/clamp/trim order and original manual-reference behavior.

CADRC still uses:

```text
ADRCController::update
raw previous Uc observer feedback
HY_CADRC_U_SCALE
existing first magnitude clamp
TRIM_ROLL
existing second magnitude clamp
hydro_torque_setpoint.xyz[0]
```

It does not use the new controller config, new observer equations or common limiter. A CADRC baseline drift would have failed the release-blocking hash/golden checks; none occurred.

Protected diff checks are also clean for:

- `src/modules/hydro_allocator/*`
- `src/modules/control_allocator/*`
- `boards/*`
- PWM/output mapping files

## 13. Build Result

Environment:

```bash
cd /home/linyi666/PX4-Autopilot-water-air
export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"
make matek_h743_dummy
```

Final clean build used an absent `build/matek_h743_dummy` directory and produced:

| Item | Result |
|---|---|
| Revision | `6abec104d4680a2c2c36499d665aec7ab18a3063` |
| Target | `matek_h743_dummy` |
| Compiler | GNU Arm Embedded 9-2020-q2-update, GCC 9.3.1 20200408 |
| Python | 3.12.3 |
| Build count | `1064/1064` |
| Exit code | 0 |
| Elapsed time | 26.74 s (`user 24.35`, `sys 9.55`, warm ccache) |

Bounded builds were run after controller/parameter definitions and after integration/tests. The only source compile fatal encountered was:

```text
HydroRateControl.cpp:162: error: comparing floating point with == or != is unsafe [-Werror=float-equal]
```

It originated in the newly added parameter snapshot comparison. It was corrected locally with a fixed-size six-float `memcmp` plus a compile-time size assertion. No warning was suppressed and no unrelated PX4 file was changed.

One initial Stage 3 static test failed because a naive string count treated `input.phi*` as occurrences of the token `input.p`; it was corrected to an exact regex-token check. LADRC/MCEO numeric golden tests were already passing at that point.

The known Python 3.12/SymForce circular-import traceback remains visible during configuration but is nonfatal for this target. CMake generation, all compilation, linking and firmware packaging completed.

## 14. Firmware Hashes and Resource Usage

Final artifacts:

| Artifact | Size | Timestamp (Asia/Shanghai) | SHA-256 |
|---|---:|---|---|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,530,921 B | 2026-09-07 19:40:38.592446961 +0800 | `1be0c9451cda8fcf6c1eaaeb1ed1728c6a0eaeaa0cda682a63a3cbb213432d32` |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 36,743,848 B | 2026-09-07 19:40:38.448614975 +0800 | `6edcc995ac988582cca455b54936de431a56b323963f845ecddb9ec7f5cfd218` |

Linker resource report:

| Region | Used | Capacity | Percent |
|---|---:|---:|---:|
| FLASH | 1,590,880 B | 1,920 KiB | 80.92% |
| AXI SRAM | 44,652 B | 512 KiB | 8.52% |
| ITCM/DTCM/SRAM1-4/BKPRAM | 0 reported | respective regions | 0% reported |

Compared with the final Stage 2B link report, FLASH increased by approximately 3.3 KiB; reported AXI SRAM is unchanged.

## 15. Git Diff Summary

The repository entered Stage 3 with uncommitted Stage 2B changes. Therefore normal `git diff` is cumulative relative to revision `6abec104...`, not a pure Stage 3 delta.

Cumulative tracked diff before adding this report:

```text
6 tracked files changed, 737 insertions(+), 3 deletions(-)
```

Stage 3-specific modified files:

- `src/modules/hydro_rate_control/HydroRollController.hpp`
- `src/modules/hydro_rate_control/LADRC.hpp`
- `src/modules/hydro_rate_control/LADRC.cpp`
- `src/modules/hydro_rate_control/MCEO_ADRC.hpp`
- `src/modules/hydro_rate_control/MCEO_ADRC.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`
- `src/modules/hydro_rate_control/test/HydroRollInfrastructureTest.cpp`

Stage 3-specific added files:

- `src/modules/hydro_rate_control/test/LadrcGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/MceoGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/Stage3SourceContractTest.py`
- `AMCEO_STAGE3_LADRC_MCEO_REPORT.md`

No `msg` schema or logger rate change was required in Stage 3 because Stage 2B already reserved all diagnostics. `AMCEO_ADRC.cpp/.hpp` were not modified during Stage 3; their placeholder hashes are enforced by the Stage 3 source-contract test.

Current changes remain intentionally uncommitted. Existing Stage 1/2A/2B reports and all new Stage 2B/3 source files appear as untracked until deliberately added. `git diff --check` passes.

## 16. Remaining Risks

- All new gains and bandwidths are starting values only. No plant identification, bench, HIL, fin-disabled hardware or pool validation has occurred.
- Explicit Euler stability margin depends strongly on actual `dt` and observer bandwidth. The existing `dt` upper bound of 40 ms is very large relative to 22–25 rad/s observer bandwidth; scheduling stalls can produce poor transient behavior even while values remain finite.
- `u_applied_prev` is pre-allocation roll torque contribution. It cannot reveal allocator saturation, servo saturation, linkage limits, fin hydrodynamics or actuator lag.
- The shared `b0` sign and magnitude have not been experimentally confirmed for the physical roll-fin direction. A wrong sign is a critical positive-feedback risk.
- `a_phi=-0.5` is not hardware identified. MCEO explicitly uses it twice; sign/model error affects both observer propagation and compensation.
- Stage 2B `HY_RLL_DU_MAX=1.0 normalized/s` can make engagement deliberately slow. It must not be loosened before actuator-disabled/bench evidence.
- The six critical new-controller parameters are deferred while armed, but Stage 2B reference, timeout and limiter parameters retain immediate PX4 update behavior.
- Parameter deferral is source/static tested but has not been exercised through QGC on a running target.
- Manual latch/data-source timeout behavior requires RC-loss and source-handover tests.
- Observer overflow is caught only after a propagation becomes nonfinite; there is no separate state magnitude guard because none was specified.
- ID 2/3 invalid states fall back to legacy output, which is safer than random output but may produce a control-law discontinuity. The common limiter only constrains valid new-controller output, not the separate legacy fallback trajectory.
- At 200 Hz, diagnostic payload remains about 24 kB/s raw before ULog framing; total logging load needs bench verification.
- LADRC total disturbance and MCEO residual disturbance must never be compared as identical physical signals in offline plots.
- No automated PX4 scheduler-level test currently proves selector/gain edge timing under concurrent uORB publications; host and source-contract tests cover deterministic logic/equations.
- The Python 3.12/SymForce traceback remains a reproducibility warning even though the target build succeeds.

Stage 3 is source + deterministic tests + target-build ready only. It is not approved for actuator-active or pool testing by this report.

## 17. Exact Stage 4 Plan for AMCEO

1. Freeze the Stage 3 sources, controller golden vectors, protected hashes and final firmware hashes as regression evidence.
2. Obtain and audit the exact AMCEO mathematical definition for `rho`, `sigma_e`, `sigma_u`, bandwidth scheduling, scheduler time constant, bounds, reset and finite behavior. Do not infer or invent these formulas.
3. Decide explicitly whether AMCEO extends the fixed MCEO four-state decomposition. Preserve MCEO as an immutable fixed-bandwidth comparison controller.
4. Implement AMCEO only in `AMCEO_ADRC.hpp/.cpp` unless a small shared diagnostic/config extension is strictly required. Continue using `HydroRollControllerInput`, shared `b0/a_phi/k1/k2`, normalized `u_requested`, and `u_applied_prev`.
5. Add only approved AMCEO scheduler parameters with physical units, bounds, starting-value warnings and the same disarmed atomic-application/reset policy.
6. Map `state_1..4`, residual disturbance, scheduled observer bandwidth, `rho`, `sigma_e` and `sigma_u` to the existing uORB message. Do not fabricate unused values.
7. Keep selector ID 4 invalid/fallback until the implementation, reset, finite, scheduler-bound and output tests all pass. Keep ID 5 reserved.
8. Add deterministic AMCEO golden vectors for fixed trajectories, applied-command feedback, scheduler limiting/rate behavior, zero equilibrium, reset and NaN/Inf handling.
9. Add a limiting-case regression showing AMCEO reduces to the approved fixed-bandwidth MCEO behavior when the scheduler is held at the same fixed bandwidth, if and only if the approved mathematics requires that equivalence.
10. Re-run every CADRC, Stage 2B, LADRC and MCEO regression before enabling ID 4; any drift is release-blocking.
11. Perform a clean GCC 9 `matek_h743_dummy` build and resource/log-bandwidth audit.
12. Stop at source/test/build readiness. Plan actuator-disabled bench/HIL and pool gates as a subsequent separately authorized stage.
