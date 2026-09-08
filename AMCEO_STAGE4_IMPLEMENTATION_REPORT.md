# AMCEO Stage 4 Implementation Report

Repository: `/home/linyi666/PX4-Autopilot-water-air`  
Revision audited: `6abec104d4680a2c2c36499d665aec7ab18a3063`  
Target: `matek_h743_dummy`  
Stage result: **PASS — source, deterministic tests, and PX4 target build ready; no hardware test performed**

## 1. Executive Summary

Controller ID 4 now implements the approved AMCEO-ADRC four-state model-compensated expanded observer, actuator-aware bandwidth scheduler, and normalized control law. Controller ID 5 remains reserved and returns no controller output.

The implementation preserves the Stage 3 conventions:

- the observer receives the previous final common-limited command, `input.u_applied_prev`;
- the observer is propagated first with the current measurement and previous applied command;
- the current controller request is calculated from the updated observer states;
- the controller does not contain an internal magnitude or rate limiter;
- the common Stage 2B limiter alone produces `u_applied`;
- CADRC retains its separate legacy raw-`Uc` path.

A new-controller-only timing gate applies to IDs 2, 3, and 4. An invalid interval freezes the selected controller and reference path, records a timing fault, and uses the existing legacy fallback. Recovery requires an explicit controller/reference/limiter reset before another new-controller output can be accepted.

All release-blocking tests passed: CADRC, LADRC, fixed MCEO, applied-command feedback, AMCEO/MCEO fixed-bandwidth equivalence, timing boundaries, MATLAB structural consistency, and protected-source contracts. The clean target build completed `1064/1064`, exit code 0.

No firmware was flashed. No actuator, pool, or outdoor test was performed.

## 2. MATLAB-to-Firmware Mapping

MATLAB source audited:

`/home/linyi666/.codex/attachments/920fc316-f2ac-4d1e-8ad9-5db4693289cc/pasted-text.txt`

The supplied script is the content of `sim_AMCEO_ADRC_roll_paper_impact_tuned.m`. Its final AMCEO structure does not materially conflict with the approved Stage 4 definition. In particular, it uses only tracking-error scheduling and applied-command utilization; it has no `sigma_o`, `c_o`, or `eo_bar` term.

| MATLAB variable | Paper symbol/meaning | PX4 parameter or C++ variable | Unit / treatment |
|---|---|---|---|
| `z(1)` | \(z_1\), estimated roll angle | `AMCEOADRC::_z1`, diagnostic `state_1` | rad |
| `z(2)` | \(z_2\), estimated roll rate | `AMCEOADRC::_z2`, diagnostic `state_2` | rad/s |
| `z(3)` | \(z_3\), residual disturbance estimate | `AMCEOADRC::_z3`, `state_3`, `disturbance_estimate` | rad/s² |
| `z(4)` | \(z_4\), residual-disturbance derivative estimate | `AMCEOADRC::_z4`, `state_4` | rad/s³ |
| `e_obs` | \(e_o=y_\phi-z_1\) | `innovation` | rad |
| `e1` | \(z_1-\phi_r\) | `_e1`, `e1_sched` | rad |
| `e2` | \(z_2-\dot\phi_r\) | `_e2`, `e2_sched` | rad/s |
| `ctrl.k_phi1` = 26.0 | \(k_{\phi1}\) | `HY_RLL_K1`, `_config.common.k_phi1` | 1/s²; shared Stage 3 parameter |
| `ctrl.k_phi2` = 9.8 | \(k_{\phi2}\) | `HY_RLL_K2`, `_config.common.k_phi2` | 1/s; shared Stage 3 parameter |
| `plant.a_nom` = -1.05 | nominal \(a_\phi\) | `HY_RLL_APHI`, `_config.common.a_phi` | 1/s; firmware keeps Stage 3 default **-0.5**, not copied |
| `plant.b0` = 9.4 | nominal \(b_0\) | `HY_RLL_B0`, `_config.common.b0` | (rad/s²)/normalized command; firmware keeps Stage 3 default **10.1746**, not copied |
| `ctrl.amceo.wo_min` = 18 | \(\omega_{o,min}\) | `HY_AM_WMIN`, `_config.omega_min` | rad/s |
| `ctrl.amceo.wo_max` = 46 | \(\omega_{o,max}\) | `HY_AM_WMAX`, `_config.omega_max` | rad/s |
| `ctrl.amceo.rho0` = 0.16 | \(\rho_0\) | `HY_AM_RHO0`, `_config.rho0` | dimensionless |
| `ctrl.amceo.ce` = 0.70 | \(c_e\) | `HY_AM_CE`, `_config.ce` | dimensionless |
| `ctrl.amceo.cu` = 0.01 | \(c_u\) | `HY_AM_CU`, `_config.cu` | dimensionless |
| `ctrl.amceo.tau_rho` = 0.055 | \(\tau_\rho\) | `HY_AM_TRHO`, `_config.tau_rho` | s |
| `ctrl.amceo.e1_bar=deg2rad(3)` | \(\bar e_1\) | `HY_AM_E1BAR`, `_config.e1_bar` | stored as 0.0523598776 rad |
| `ctrl.amceo.e2_bar=deg2rad(40)` | \(\bar e_2\) | `HY_AM_E2BAR`, `_config.e2_bar` | stored as 0.698131701 rad/s |
| `sigma_e` | normalized tracking-error activity | `_sigma_e`, diagnostic `sigma_e` | [0, 1] |
| `sigma_u` | applied roll-command utilization | `_sigma_u`, diagnostic `sigma_u` | [0, 1] |
| `rho_star` | desired scheduling factor | `_rho_star` | [0, 1] |
| `rho` | filtered scheduling factor | `_rho`, diagnostic `rho` | [0, 1] |
| `wo` | current observer bandwidth | `_observer_bandwidth` | rad/s |
| `u_prev` | previous limited command | `input.u_applied_prev` | normalized controller/allocation-interface command |
| `act.u_max` = 0.34 | simulation actuator magnitude | `HY_RLL_U_MAX`, `input.u_max` | MATLAB value not copied; firmware default remains 1.0 |
| `act.du_max` = 2.60 | simulation slew limit | `HY_RLL_DU_MAX` | MATLAB value not copied; firmware default remains 1.0 normalized/s |
| `cfg.Ts` = 0.001 | simulation integration step | real PX4 loop `dt`; safety bound `HY_RLL_DT_MAX` | MATLAB value not copied |
| `plant.a_true` = -1.35 | simulated true plant | none | simulation-only; prohibited from firmware |
| `plant.b_true` = 10.8 | simulated true plant | none | simulation-only; prohibited from firmware |
| disturbance/noise profiles | simulated environment | none | not copied |

The MATLAB actuator variable and firmware `u_applied_prev` both represent controller-level normalized command usage in the scheduler. Neither is a measurement of physical hydrofoil angle.

## 3. Exact AMCEO Equations

States:

\[
z_1=\hat\phi,\quad z_2=\hat p,\quad z_3=\hat f_\phi,\quad z_4=\widehat{\dot f_\phi}
\]

Innovation:

\[
e_o=\phi-z_1
\]

For the scheduled bandwidth \(\omega_o\):

\[
l_1=4\omega_o,\quad l_2=6\omega_o^2,\quad l_3=4\omega_o^3,\quad l_4=\omega_o^4
\]

Observer:

\[
\dot z_1=z_2+l_1e_o
\]

\[
\dot z_2=a_\phi z_2+b_0u_{applied}[k-1]+z_3+l_2e_o
\]

\[
\dot z_3=z_4+l_3e_o,\qquad \dot z_4=l_4e_o
\]

Updated-state tracking errors and virtual acceleration:

\[
e_1=z_1-\phi_r,\qquad e_2=z_2-\dot\phi_r
\]

\[
v_\phi=\ddot\phi_r-k_{\phi1}e_1-k_{\phi2}e_2
\]

Normalized request before the common limiter:

\[
u_{requested}=\frac{v_\phi-a_\phi z_2-z_3}{b_0}
\]

`z4` is not directly compensated in the control expression. There is no integral term, `fal()`, sliding-mode term, additional feedforward, internal clamp, or anti-windup.

## 4. Exact Scheduler Equations

Scheduler tracking errors are calculated from the pre-step observer state:

\[
e_{1,s}=z_1[k]-\phi_r[k],\qquad e_{2,s}=z_2[k]-\dot\phi_r[k]
\]

\[
\sigma_e=\operatorname{sat}_{[0,1]}\left(\frac{|e_{1,s}|}{\bar e_1}+\frac{|e_{2,s}|}{\bar e_2}\right)
\]

\[
\sigma_u=\operatorname{sat}_{[0,1]}\left(\frac{|u_{applied}[k-1]|}{u_{max}}\right)
\]

Here `u_max` is passed from the active `HY_RLL_U_MAX` value used by the common limiter in the same HydroRateControl iteration. It is not hard-coded to MATLAB's 0.34.

\[
\rho^*=\operatorname{sat}_{[0,1]}(\rho_0+c_e\sigma_e-c_u\sigma_u)
\]

\[
\rho[k+1]=\operatorname{sat}_{[0,1]}\left(\rho[k]+\frac{dt}{\tau_\rho}(-\rho[k]+\rho^*[k])\right)
\]

\[
\omega_o[k]=\omega_{o,min}+(\omega_{o,max}-\omega_{o,min})\rho[k+1]
\]

There is deliberately no observer-innovation scheduling term.

## 5. Final Starting Parameters

| Parameter | Default | Unit | Validation |
|---|---:|---|---|
| `HY_RLL_B0` | 10.1746 | (rad/s²)/normalized command | finite, > 0 |
| `HY_RLL_APHI` | -0.5 | 1/s | finite |
| `HY_RLL_K1` | 26.0 | 1/s² | finite, > 0 |
| `HY_RLL_K2` | 9.8 | 1/s | finite, > 0 |
| `HY_AM_WMIN` | 18.0 | rad/s | finite, > 0 |
| `HY_AM_WMAX` | 46.0 | rad/s | finite, > `HY_AM_WMIN` |
| `HY_AM_RHO0` | 0.16 | dimensionless | finite, [0, 1] |
| `HY_AM_CE` | 0.70 | dimensionless | finite, >= 0 |
| `HY_AM_CU` | 0.01 | dimensionless | finite, >= 0 |
| `HY_AM_TRHO` | 0.055 | s | finite, > 0 |
| `HY_AM_E1BAR` | 0.0523598776 | rad | finite, > 0 |
| `HY_AM_E2BAR` | 0.698131701 | rad/s | finite, > 0 |
| `HY_RLL_DT_MAX` | 0.010 | s | finite, > 0 |

The AMCEO thresholds are stored directly in SI units, so there is no runtime degree conversion. All values are numerical-study or engineering starting values and are not hardware-validated gains.

## 6. Discrete Execution Order

At sample `k`, the implementation performs:

1. Read `phi[k]`, reference states, measured PX4 `dt`, and `u_applied[k-1]`.
2. From pre-step `z1[k]`, `z2[k]`, calculate `e1_sched`, `e2_sched`, `sigma_e`, and `sigma_u`.
3. Calculate and saturate `rho_star`.
4. Forward-Euler propagate and saturate `rho[k+1]`.
5. Map the new `rho[k+1]` to `omega_o[k]` and calculate `l1..l4`.
6. Calculate innovation from `phi[k]` and pre-step `z1[k]`.
7. Calculate all four observer derivatives from pre-step states and `u_applied[k-1]`.
8. Perform simultaneous forward-Euler state assignment.
9. Calculate control errors and `u_requested[k]` from the updated observer states.
10. Outside AMCEO, the common limiter produces `u_applied[k]` for the next sample.

The four derivatives are stored before any state assignment, preserving simultaneous Euler semantics.

## 7. Reset Behavior

On a valid reset:

```text
z1 = input.phi
z2 = input.p
z3 = 0
z4 = 0
rho = rho0
omega_o = omega_min + (omega_max - omega_min) * rho0
sigma_e = NaN
sigma_u = NaN
rho_star = NaN
```

Nonfinite reset `phi` or `p`, or an invalid applied parameter set, leaves AMCEO invalid. The first successful update replaces the intentionally unavailable scheduler diagnostics with finite values.

Activation through ID 4 uses the existing Stage 2B/3 state machine: latch and reference initialization, common limiter initialization from the previous downstream roll command, selected-controller reset, and incremented reset counter. Armed selector changes remain blocked. The first accepted AMCEO output still passes through the common rate and magnitude limiter.

## 8. Timing Safety Gate

`HY_RLL_DT_MAX=0.010 s` is a safety engineering bound for IDs 2, 3, and 4 only. It does not alter CADRC's existing `dt` policy and does not apply to reserved ID 5.

Acceptance rule:

```text
finite(dt_used) && finite(dt_max) && dt_used > 0 && dt_max > 0 && dt_used <= dt_max
```

State machine:

| State/event | Action |
|---|---|
| valid timing, no fault | normal reference/controller/common-limiter path |
| first invalid interval | do not update latch, reference generator, or observer; set timing fault; increment fault counter; force diagnostic publication; use legacy fallback |
| repeated invalid interval | continue frozen/fallback state without repeatedly incrementing the edge counter |
| first later valid interval | reset latch to measured roll, reset reference generator, initialize limiter from previous downstream roll command, reset selected controller, clear timing fault, increment reset counter, then permit normal processing |
| leave STAB or accept a disarmed selector edge | reset/clear new-controller timing state through the existing mode/selector reset path |

The gate never replaces a 40 ms interval with 10 ms. Boundary test results:

```text
9.9 ms  accepted
10.0 ms accepted
10.1 ms rejected
40.0 ms rejected
NaN     rejected
```

The host test also verifies that an AMCEO object's update counter does not change when the timing guard rejects the sample.

The diagnostic schema now includes `dt_invalid` for the current sample and latched `timing_fault` for recovery state.

## 9. Parameter Application Policy

The Stage 3 disarmed atomic policy was expanded from six to fifteen floats:

- common `b0`, `a_phi`, `k1`, `k2`;
- LADRC and fixed-MCEO bandwidths;
- new-controller `dt_max`;
- all eight AMCEO scheduler parameters.

On a parameter update, HydroRateControl reads a complete candidate snapshot and validates the complete set. If armed, the candidate remains pending, the currently applied snapshot remains active, and a warning/fault edge is recorded. On disarm, the complete set is applied to LADRC, MCEO, and AMCEO configurations, and any active new controller must reset before another update. No partial AMCEO scheduler configuration is applied.

`HY_RLL_U_MAX` remains the active common limiter parameter rather than an AMCEO-owned parameter. AMCEO receives that same live limiter value through `input.u_max`, ensuring scheduler normalization and actual common magnitude limiting agree.

CADRC parameter behavior is unchanged.

## 10. Applied-Command Feedback Evidence

The controller input chain is:

```text
previous HydroRollLimiter::_u_applied_prev
-> HydroRollLimiter::appliedPrevious()
-> controller_input.u_applied_prev
-> AMCEOADRC::update(input)
-> b0 * input.u_applied_prev in z2_dot
```

The AMCEO scheduler also uses the same input:

```text
sigma_u = sat(abs(input.u_applied_prev) / input.u_max)
```

The current `u_requested` is produced only after observer propagation and cannot feed that same sample's observer. `AmceoGoldenTest` independently checks the expected `z2` propagation with `u_requested != u_applied_prev`. The fixed-bandwidth equivalence test repeats this causal path for 40 deterministic samples.

## 11. Diagnostic Mapping

For controller ID 4:

| Diagnostic | Source / meaning |
|---|---|
| `state_1` | `z1`, estimated roll angle [rad] |
| `state_2` | `z2`, estimated roll rate [rad/s] |
| `state_3` | `z3`, estimated residual disturbance [rad/s²] |
| `state_4` | `z4`, estimated residual-disturbance derivative [rad/s³] |
| `disturbance_estimate` | `z3`; residual disturbance, not LADRC total disturbance |
| `observer_bandwidth` | current scheduled `omega_o` [rad/s] |
| `rho` | current filtered/saturated scheduling factor |
| `sigma_e` | current saturated tracking-error activity |
| `sigma_u` | current saturated normalized applied-command utilization |
| `u_requested` | normalized controller request before common limiter |
| `u_applied` | final common-limited roll command sent downstream |
| `dt_invalid` | current interval rejected by new-controller timing guard |
| `timing_fault` | reset/recovery is required |

The uORB serialized payload increased from 120 to 122 bytes. Publication and logger subscription remain limited to 5 ms intervals, or at most 200 Hz. Raw payload is therefore at most 24.4 kB/s, a 0.4 kB/s increase before ULog framing. Selector, reset, and timing-fault edges force publication even between periodic deadlines.

## 12. AMCEO Golden Tests

`AmceoGoldenTest.cpp` covers:

- exact reset state, initialized `rho`, initialized bandwidth, and reset-only NaN scheduler diagnostics;
- 50-sample finite zero equilibrium;
- eight saved float32 golden samples for `z1..z4`, `rho`, `rho_star`, `sigma_e`, `sigma_u`, bandwidth, `e1`, `e2`, and `u_requested`;
- scheduler [0,1] bounds and bandwidth bounds;
- `sigma_e` and `sigma_u` saturation;
- monotonic tracking-error scheduling and actuator penalty;
- exact scheduler forward-Euler result;
- previous applied-command observer feedback;
- exact updated-state control expression;
- source-contract proof that `z4` does not directly enter control;
- NaN/Inf input and parameter rejection for every input/config category;
- 9.9/10.0/10.1/40 ms and NaN timing cases;
- no observer update after rejected timing;
- an intentionally over-range controller request proving no internal clamp;
- external common magnitude and slew inequalities.

Result: `AmceoGoldenTest: PASS`.

## 13. Fixed-Bandwidth MCEO Equivalence Test

The release-blocking configuration used:

```text
omega_fixed = 22 rad/s
omega_min = 18 rad/s
omega_max = 46 rad/s
rho0 = (22 - 18) / (46 - 18)
ce = 0
cu = 0
```

Thus `rho_star=rho0`, `rho` remains constant, and AMCEO bandwidth remains 22 rad/s. AMCEO and the unchanged MCEO controller receive identical reset values and 40 samples of identical measurement, reference, `dt`, and `u_applied_prev` data.

Every `z1..z4` value and `u_requested` matched within float32 tolerance. Result: **PASS**.

## 14. MATLAB Consistency Audit

`AmceoMatlabConsistencyTest.cpp` extracts one deterministic case using the MATLAB nominal model, 1 ms simulation step, and 0.34 simulation limit strictly as test inputs. Those values are not firmware defaults.

| Item | MATLAB | Firmware | Match / intentional difference |
|---|---|---|---|
| continuous observer derivatives | four-state MCEO equations with scheduled gains | same equations | Match |
| `sigma_e` | `sat(abs(e1)/e1_bar + abs(e2)/e2_bar)` | same | Match |
| `sigma_u` | `sat(abs(u_prev)/act.u_max)` | `sat(abs(input.u_applied_prev)/input.u_max)` | Match after configurable `u_max` substitution |
| `rho_star` | `sat(rho0 + ce*sigma_e - cu*sigma_u)` | same | Match |
| scheduler dynamics | forward Euler `rho + Ts/tau*(-rho+rho_star)` | forward Euler with real `dt_used` | Match |
| bandwidth map | affine min/max map from saturated `rho` | same | Match |
| gains | `[4wo, 6wo², 4wo³, wo⁴]` | same | Match |
| continuous control law | `(v-a*z2-z3)/b0` | same | Match |
| controller/observer timestep order | MATLAB computes control from pre-step observer and propagates with current limited `u(k)` | firmware propagates with `u_applied[k-1]`, then controls from updated observer | **Intentional firmware current-observer convention inherited from Stage 3** |
| actuator values | hard-coded simulation values | active PX4 common limiter parameters | Intentional environment difference |
| plant/noise/disturbance integration | simulated true plant and RK4/noise profiles | real vehicle measurements | Intentionally not copied |

The independent deterministic case matched `sigma_e`, `sigma_u`, `rho_star`, `rho`, bandwidth, all four inferred continuous observer derivatives, and the firmware-order control result. Result: `AmceoMatlabConsistencyTest: PASS`.

## 15. LADRC/MCEO/CADRC Regression

Final post-build results:

```text
CadrcGoldenTest: PASS
Stage2BSourceContractTest: PASS
HydroRollInfrastructureTest: PASS
LadrcGoldenTest: PASS
MceoGoldenTest: PASS
Stage3SourceContractTest: PASS
AmceoGoldenTest: PASS
AmceoMatlabConsistencyTest: PASS
Stage4SourceContractTest: PASS
git diff --check: PASS
```

Protected file SHA-256 values remain:

```text
a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201  CADRC.cpp
ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54  CADRC.h
2bd7ff16ba6614b6ebe5b5dc4369b96b07211e43a7f78f0e79e19d3b70711039  LADRC.cpp
465d43da0678ccfb0018b3af96011f1a6217f9ff39808ef4c33e649cb1f1c569  LADRC.hpp
763b7faa250249d4c007a8fedb84cb9073c2e88f357b74a2b88bb300f374a128  MCEO_ADRC.cpp
ec3a4e960abf53145c42b672d07a01d03540d977e3336dfe1816c4c5ea77d87d  MCEO_ADRC.hpp
```

The source contract also reports no diff under `src/modules/hydro_allocator`, `src/modules/control_allocator`, or `boards`.

## 16. Build Result

Environment:

```bash
export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"
make matek_h743_dummy
```

Compiler: `arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)`  
Configure Python: `Python 3.12.3`

For the final clean build, only `build/matek_h743_dummy` was deleted and regenerated. Result:

```text
1064/1064
exit code: 0
elapsed: 25.34 s (ccache enabled)
FLASH: 1,594,528 B / 1,920 KiB = 81.10%
AXI_SRAM: 44,652 B / 512 KiB = 8.52%
```

An earlier `cmake --build ... --target clean` attempt exposed a generated-file dependency race: step 1 attempted to build `jlink-nuttx.so` before step 13 regenerated NuttX `.config`. This was classified as a generated-file failure, not source or hydro-module failure. Removing only the target build directory and allowing normal PX4 configure/build ordering produced the successful full build above; no source workaround was made.

The configure stage still printed the pre-existing Python 3.12 SymForce circular-import traceback. CMake continued using repository-generated/fallback material, and all 1064 build steps completed. It remains an environment risk rather than a fatal error in this verified build.

## 17. Firmware Hashes and Resource Usage

| Artifact | Path | Size | Timestamp (Asia/Shanghai) | SHA-256 |
|---|---|---:|---|---|
| PX4 package | `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,533,661 B | 2026-09-07 20:20:06.737112568 +0800 | `0c391a061f2ed29cbdbe33914ae0df2ad3d9308f9fa531e6d1eb42c0c49f0664` |
| ELF | `build/matek_h743_dummy/matek_h743_dummy.elf` | 36,794,792 B | 2026-09-07 20:20:06.589112076 +0800 | `71a6c11e883350d1460e6aa424c33faff055887d01813f825d32ba6d9871a171` |

The Stage 4 additions increased FLASH relative to the Stage 3 report, as expected for the implemented AMCEO observer/scheduler and two diagnostic flags. SRAM reported by the linker remains 44,652 B AXI SRAM.

## 18. Git Diff Summary

Stage 4 implementation files added or materially modified:

- `src/modules/hydro_rate_control/AMCEO_ADRC.hpp`
- `src/modules/hydro_rate_control/AMCEO_ADRC.cpp`
- `src/modules/hydro_rate_control/HydroRollController.hpp`
- `src/modules/hydro_rate_control/HydroRollController.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`
- `msg/HydroRollControllerStatus.msg`
- `src/modules/hydro_rate_control/test/AmceoGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/AmceoMatlabConsistencyTest.cpp`
- `src/modules/hydro_rate_control/test/Stage4SourceContractTest.py`

Narrow test maintenance:

- `LadrcGoldenTest.cpp` and `MceoGoldenTest.cpp`: initialize the newly appended AMCEO-only `u_max` input field; their sequences and expected vectors are unchanged.
- `Stage3SourceContractTest.py`: replace the obsolete AMCEO-placeholder hash with immutable LADRC/MCEO hashes so the Stage 3 contract remains meaningful after Stage 4.
- `HydroRollInfrastructureTest.cpp`: clarify that an unconfigured AMCEO object is safely invalid; implemented AMCEO behavior is tested separately.

Because Stages 2B and 3 remain uncommitted in this working tree, `git diff --stat` is cumulative and reports six tracked files with 953 insertions and 3 deletions; new infrastructure files appear as untracked and are not counted by that statistic. The final report itself is also a new file.

No protected controller math, allocator, board, or PWM mapping file changed.

## 19. Remaining Risks

1. The AMCEO and common gains are numerical-study starting points, not tank- or vehicle-identified parameters.
2. Explicit Euler at scheduled bandwidth up to 46 rad/s is sensitive to timing, measurement noise, and plant/model error. The 10 ms gate is a safety bound, not a stability proof.
3. PX4's pre-existing loop code constrains its working `dt` to 2–40 ms before this new gate. A physical stall above 40 ms is diagnosed as a rejected 40 ms interval rather than preserving its full raw duration.
4. `HY_RLL_U_MAX` and other Stage 2B limiter/reference parameters are not part of the new fifteen-value deferred snapshot. An armed live change can alter both limiter behavior and AMCEO utilization normalization.
5. `u_applied_prev` is a normalized controller/allocation-interface command, not measured fin deflection. Allocation saturation, servo dynamics, backlash, or mechanical blockage are not observed.
6. Timing/input faults use the existing legacy fallback. Transition behavior between a frozen new controller, legacy fallback, and reset-limited recovery has only deterministic source-level coverage, not hardware scheduling coverage.
7. The common limiter includes roll trim before rate and magnitude limiting. Consequently, AMCEO sees the previous final command including trim; offline analysis must use the same definition.
8. High observer gains can overflow during severe sensor corruption before finite checks invalidate the result. The invalid output is contained, but the controller then requires reset.
9. The 122-byte, 200 Hz topic adds about 24.4 kB/s raw ULog payload plus framing. Storage and logger dropout margins need hardware confirmation.
10. Python 3.12 still produces a nonfatal SymForce circular-import traceback during fresh configure. The GCC 9 target build is reproducible, but code-generation environment hygiene remains imperfect.
11. No allocator, PWM, physical fin direction, or actuator response was exercised in Stage 4.
12. No armed switching is implemented; selector changes while armed remain blocked by design.

## 20. Exact Stage 5 Hardware-Commissioning Plan

Stage 5 should remain actuator-disabled and disarmed except for any separately approved, physically inhibited arming-state check:

1. Archive the Stage 4 `.px4`, `.elf`, hashes, complete parameter set, and current vehicle parameter backup.
2. Flash only the verified Stage 4 artifact under explicit Stage 5 authorization; keep propulsion power and hydrofoil actuation physically inhibited.
3. Confirm boot health, revision, parameter metadata, and presence/defaults of all `HY_AM_*` and `HY_RLL_DT_MAX` parameters.
4. With the vehicle disarmed, exercise selector IDs 0–5 and verify active/requested/previous IDs, reset counter, ID 5 fallback, and no output actuation.
5. Select ID 4 while disarmed and use `listener hydro_roll_controller_status` plus ULog to verify nonzero timestamps, 200 Hz maximum publication, finite reset states, initialized `rho/bandwidth`, and reset-only NaN `sigma` fields.
6. Feed valid manual roll publications while outputs remain inhibited; verify raw, latched, reference, derivative, requested, and common-limited signals remain synchronized and publication gaps preserve the latch until timeout.
7. Observe real `dt_measured/dt_used` distribution under normal CPU/logger load. Any `dt_invalid` must produce fallback, one fault edge, frozen controller state, and a reset edge before recovery.
8. Validate scheduler monotonicity offline from logs: increased normalized tracking error raises or preserves `rho_star`; increased applied utilization lowers or preserves it; `rho` and bandwidth remain bounded.
9. Change the full AMCEO parameter set while disarmed and verify atomic application/reset. Test armed deferral only if every physical actuator output is independently inhibited and the arming action is separately authorized.
10. Verify ULog sustained bandwidth, dropouts, timestamps, NaN conventions, and the 122-byte schema on the actual flight controller/storage medium.
11. Confirm no PWM movement throughout Stage 5. Do not proceed to a pool test until the actuator-disabled evidence is reviewed and model/gain signs, units, allocator direction, and safe output bounds are independently approved.

Stage 5 must not include water operation or active hydrodynamic control. A later separately authorized stage is required for actuator-enabled bench work and then controlled pool testing.
