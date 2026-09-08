# AMCEO Stage 4B — Classical NADRC Implementation Report

Report time: `2026-09-07T21:49:13+08:00`  
Repository: `/home/linyi666/PX4-Autopilot-water-air`  
Git revision: `6abec104d4680a2c2c36499d665aec7ab18a3063`  
Target: `matek_h743_dummy`  
Decision: **PASS — source, deterministic tests, regression contracts, and clean target build are ready; hardware remains unconnected and untested.**

## 1. Executive Summary

Selector ID 5 is now implemented by the new `ClassicalNADRC` class as a classical Han-style nonlinear ADRC baseline containing:

- an independent two-state canonical `fhan` tracking differentiator;
- a three-state nonlinear ESO using canonical `fal` feedback;
- nonlinear state-error feedback;
- total-disturbance compensation;
- normalized pre-limiter `u_requested` output.

ID 5 uses the Stage 2B manual-command latch directly, not the common second-order `RollReferenceGenerator`. It uses the existing common new-controller timing gate, parameter snapshot policy, rate/magnitude limiter, selector safety state machine, fallback path, and uORB diagnostics. Its ESO receives only the previous common-limited normalized roll command, `input.u_applied_prev`.

The mathematical source files for CADRC, LADRC, fixed MCEO-ADRC, and AMCEO-ADRC retain their frozen hashes. Allocator, board, and PWM/output-mapping sources were not changed. No flight controller was connected, flashed, armed, or commanded.

The final selector map is:

| ID | Controller | Stage 4B state |
|---:|---|---|
| 0 | Legacy | unchanged |
| 1 | CADRC | immutable legacy path |
| 2 | LADRC | unchanged |
| 3 | fixed-bandwidth MCEO-ADRC | unchanged |
| 4 | AMCEO-ADRC | unchanged |
| 5 | classical nonlinear ADRC | implemented by `ClassicalNADRC` |

`HY_RLL_CTRL_SEL` still defaults to 1, preserving the established CADRC default. Armed selector switching remains prohibited.

## 2. Files Added or Modified for Stage 4B

Added:

- `src/modules/hydro_rate_control/ClassicalNADRC.hpp`
- `src/modules/hydro_rate_control/ClassicalNADRC.cpp`
- `src/modules/hydro_rate_control/test/NadrcMathTest.cpp`
- `src/modules/hydro_rate_control/test/NadrcGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/Stage4BNadrcSourceContractTest.py`
- `AMCEO_STAGE4B_NADRC_IMPLEMENTATION_REPORT.md`

Narrow integration changes:

- `src/modules/hydro_rate_control/HydroRollController.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`
- `src/modules/hydro_rate_control/CMakeLists.txt`
- `msg/HydroRollControllerStatus.msg`
- `src/modules/hydro_rate_control/test/Stage4SourceContractTest.py`
- `AMCEO_STAGE5_PRECOMMISSION_SNAPSHOT.md`

The uORB message change only renames the ID 5 compile-time constant from the former reserved label to `CONTROLLER_ID_NADRC`; no serialized field was added or removed. The generated serialized payload remains 122 bytes and the configured logger interval remains 5 ms, at most 200 Hz.

## 3. Exact Classical NADRC Equations

### 3.1 Tracking differentiator

The private TD states are `v1` and `v2`, and the input is the latched roll command:

```text
x1 = v1 - command_roll
x2 = v2
h0 = HY_NA_TDN0 * dt_used

v1_dot = v2
v2_dot = fhan(x1, x2, HY_NA_TDR, h0)

v1_next = v1 + dt_used * v1_dot
v2_next = v2 + dt_used * v2_dot
```

ID 5 does not use `RollReferenceGenerator::update()` as a control input.

### 3.2 Canonical Han `fhan`

The implemented function is:

```text
d  = r_td * h0 * h0
a0 = h0 * x2
y  = x1 + a0
a1 = sqrt(d * (d + 8*abs(y)))
a2 = a0 + sign(y)*(a1-d)/2
sy = (sign(y+d)-sign(y-d))/2
a  = (a0+y-a2)*sy + a2
sa = (sign(a+d)-sign(a-d))/2
fh = -r_td*(a/d-sign(a))*sa - r_td*sign(a)
```

`sign(0)=0`. Inputs must be finite, `r_td>0`, and `h0>0`. Nonfinite/intermediate overflow and an underflowed `d<=FLT_MIN` return NaN, which causes the controller output to become invalid and enter the existing common fallback.

The user-provided Markdown rendered the line following `(a0+y-a2)*sy` as a list item. The code uses the canonical Han trailing term **`+a2`**, not `-a2`. This interpretation was explicitly announced before implementation and is locked by the Stage 4B source-contract and deterministic math tests.

### 3.3 Canonical `fal`

```text
fal(e, alpha, delta) =
    e / delta^(1-alpha),                  abs(e) <= delta
    abs(e)^alpha * sign(e),               abs(e) > delta
```

The equality case uses the linear branch. `delta>0`, `alpha>0`, and all inputs/intermediates must be finite. No smoothing beyond the requested piecewise definition is present.

### 3.4 Nonlinear ESO

States and meaning:

| State | Meaning | Unit |
|---|---|---|
| `z1` | estimated roll angle | rad |
| `z2` | estimated roll rate | rad/s |
| `z3` | estimated total disturbance | rad/s² |

The innovation sign is fixed as estimated minus measured:

```text
e_o = z1 - input.phi
beta01 = 3*omega_o
beta02 = 3*omega_o^2
beta03 = omega_o^3
omega_o = HY_NA_WO

z1_dot = z2 - beta01*e_o
z2_dot = z3 - beta02*fal(e_o, HY_NA_OA1, HY_NA_ODEL)
         + b0*input.u_applied_prev
z3_dot = -beta03*fal(e_o, HY_NA_OA2, HY_NA_ODEL)
```

All three derivatives are evaluated from the pre-step states and assigned simultaneously with forward Euler.

### 3.5 Nonlinear state-error feedback and disturbance compensation

Control is computed after the updated TD and ESO states:

```text
e1 = v1 - z1
e2 = v2 - z2

u0 = k_phi1*fal(e1, HY_NA_CA1, HY_NA_CDEL)
   + k_phi2*fal(e2, HY_NA_CA2, HY_NA_CDEL)

u_requested = (u0 - z3)/b0
```

`b0`, `k_phi1`, and `k_phi2` are the shared `HY_RLL_B0`, `HY_RLL_K1`, and `HY_RLL_K2` applied snapshot. `HY_RLL_APHI` is not referenced by `ClassicalNADRC`.

There is no integral term, sliding mode, adaptive bandwidth, model term `a_phi*z2`, extra feedforward, internal magnitude/rate clamp, CADRC scaling, or AMCEO scheduler term.

## 4. Discrete Execution and Applied-Command Causality

At sample `k`:

1. `HydroRateControl` validates the common new-controller timing gate.
2. The ID 5 TD propagates from `command_roll_latched`.
3. The NESO propagates from `phi[k]` and `u_applied[k-1]`.
4. `e1/e2`, nonlinear `u0`, and normalized `u_requested[k]` are computed from updated states.
5. Outside `ClassicalNADRC`, the existing common limiter applies trim, rate limiting, then magnitude limiting to produce `u_applied[k]`.
6. On sample `k+1`, `_new_roll_limiter.appliedPrevious()` becomes `input.u_applied_prev`.

The implementation line is:

```cpp
const float z2_dot = _z3 - beta02 * fal_o1 + _config.b0 * input.u_applied_prev;
```

No current `u_requested`, CADRC raw/scaled value, or servo angle appears in the ESO. `NadrcGoldenTest` isolates this term with zero innovation and verifies the resulting `z2` exactly while `u_requested != u_applied_prev`.

## 5. Reset and Validity Behavior

On valid reset:

```text
v1 = finite(command_roll) ? command_roll : phi
v2 = 0
TD acceleration = NaN until the first scheduler/TD evaluation
z1 = phi
z2 = p
z3 = 0
e1 = 0
e2 = 0
u0/u_requested = unavailable until first valid update
```

Measured `p` is used only for initialization. An invalid `phi` or `p` makes the controller invalid. All controller inputs, parameters, derivatives, states, nonlinear feedback values, and output are checked for finiteness.

Manual-command timeout/invalidity resets the ID 5 private TD/NESO and common limiter. A later valid request must therefore restart through the common rate limiter. Inactive controllers are not updated.

## 6. Parameters and Starting Values

| PX4 parameter | Default | Meaning |
|---|---:|---|
| `HY_NA_WO` | 20.0 rad/s | fixed nonlinear ESO bandwidth |
| `HY_NA_TDR` | 20.0 | Han `fhan` tracking-speed factor; algorithmic role only, no false physical unit asserted |
| `HY_NA_TDN0` | 2.0 | TD auxiliary-step multiplier, `h0=n0*dt` |
| `HY_NA_OA1` | 0.5 | first NESO `fal` exponent |
| `HY_NA_OA2` | 0.25 | second NESO `fal` exponent |
| `HY_NA_ODEL` | 0.01 rad | NESO `fal` threshold |
| `HY_NA_CA1` | 0.8 | roll-angle NLSEF `fal` exponent |
| `HY_NA_CA2` | 1.2 | roll-rate NLSEF `fal` exponent |
| `HY_NA_CDEL` | 0.01 | shared numerical NLSEF threshold |

These values are initial bench/pool tuning values only. They are neither optimal nor hardware validated.

`HY_NA_CDEL` is shared by the angle and rate channels as required, whose inputs have different dimensions. The parameter metadata therefore does not claim one misleading physical unit. This shared-threshold convention is a tuning/interpretation risk to evaluate after safe commissioning.

## 7. Parameter Application Policy

The nine NADRC parameters are part of the same 24-float complete new-controller snapshot as the Stage 3/4 critical parameters.

- Candidate values are read when PX4 publishes `parameter_update`.
- The entire candidate snapshot must pass finite/range validation.
- While armed, changes are retained as pending and not applied; one warning/fault edge is recorded.
- Once disarmed, the complete snapshot is applied together and the active new controller is reset before reuse.
- CADRC retains its historical parameter path and was not moved into this policy.

No partial NADRC scheduler/observer/control parameter application is intentionally exposed.

## 8. Timing Safety and Fallback

ID 5 is included in the existing new-controller-only `HY_RLL_DT_MAX` gate. With the default 0.010 s:

- 9.9 ms: accepted;
- 10.0 ms: accepted;
- 10.1 ms: rejected;
- 40 ms: rejected;
- NaN: rejected.

Rejected timing does not call `ClassicalNADRC::update()`, so neither TD nor NESO propagates. The existing timing fault and legacy fallback are asserted, and the controller/reference/limiter state must reset before normal new-controller operation resumes. A long interval is not silently clamped to 10 ms. CADRC's `dt` policy is unchanged.

## 9. Reference and Diagnostic Mapping

Reference source distinction:

| Controller IDs | Actual reference path | Generic diagnostic fields |
|---|---|---|
| 2–4 | manual latch → common second-order `RollReferenceGenerator` | common `phi_ref`, `phi_ref_dot`, `phi_ref_ddot` |
| 5 | manual latch → private Han TD | `phi_ref=v1`, `phi_ref_dot=v2`, `phi_ref_ddot=current fhan acceleration` |

ID 5 controller diagnostics are:

| Field | Value |
|---|---|
| `controller_id` | 5 |
| `state_1` | `z1` |
| `state_2` | `z2` |
| `state_3` | `z3` |
| `state_4` | NaN |
| `disturbance_estimate` | `z3`, total disturbance estimate |
| `observer_bandwidth` | applied `HY_NA_WO` |
| `rho`, `sigma_e`, `sigma_u` | NaN |
| `u_requested` | normalized pre-common-limiter request |
| `u_applied` | final common-limited normalized roll contribution |

On reset, TD acceleration is intentionally NaN until the first real TD evaluation; `reference_valid` reflects that distinction.

## 10. Deterministic and Golden Tests

Host tests compile the actual controller source with `g++ -std=c++14 -Wall -Wextra -Werror`.

`NadrcMathTest` covers:

- `fal` at zero, inside/equal/outside the threshold, odd symmetry, bad alpha/delta, NaN, and Inf;
- `fhan` zero equilibrium, positive/negative steps, symmetry, nonlinear inner branch, finite 2,000-sample positive/negative convergence;
- legal small `d`, underflowed `d`, bad arguments, and the 10 ms timing boundary.

`NadrcGoldenTest` covers:

- exact reset and nonfinite reset-command fallback;
- zero equilibrium;
- a saved eight-sample float32 vector containing `v1/v2`, TD acceleration, `z1..z3`, `e1/e2`, `u0`, and `u_requested`;
- NESO propagation and both nonlinear branches;
- release-blocking previous-applied-command feedback;
- disturbance-compensation sign;
- deliberately over-range `u_requested`, proving no internal clamp;
- external common magnitude/rate limiting;
- 9.9/10.0/10.1/40 ms and NaN timing behavior, including state freeze on rejected intervals;
- NaN/Inf inputs and every invalid configuration category;
- ID 5 valid output and inactivity of LADRC/MCEO/AMCEO objects while NADRC runs.

Final results:

```text
CadrcGoldenTest: PASS
HydroRollInfrastructureTest: PASS
LadrcGoldenTest: PASS
MceoGoldenTest: PASS
AmceoGoldenTest: PASS
AmceoMatlabConsistencyTest: PASS
NadrcMathTest: PASS
NadrcGoldenTest: PASS
Stage2BSourceContractTest: PASS
Stage3SourceContractTest: PASS
Stage4SourceContractTest: PASS
Stage4BNadrcSourceContractTest: PASS
git diff --check: PASS
```

## 11. Regression and Protected-Source Evidence

Frozen hashes after Stage 4B:

```text
a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201  CADRC.cpp
ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54  CADRC.h
2bd7ff16ba6614b6ebe5b5dc4369b96b07211e43a7f78f0e79e19d3b70711039  LADRC.cpp
465d43da0678ccfb0018b3af96011f1a6217f9ff39808ef4c33e649cb1f1c569  LADRC.hpp
763b7faa250249d4c007a8fedb84cb9073c2e88f357b74a2b88bb300f374a128  MCEO_ADRC.cpp
ec3a4e960abf53145c42b672d07a01d03540d977e3336dfe1816c4c5ea77d87d  MCEO_ADRC.hpp
c0ea3e0b7dc2af981251ed94253d1e3205343b1f005c7a6e30e694cfbb995ea2  AMCEO_ADRC.cpp
050e3c29a6cf104cafc06dae53a095f4edc148a4d02ab61d5bcb153270076235  AMCEO_ADRC.hpp
```

`Stage4BNadrcSourceContractTest.py` enforces the exact NADRC structure/order, canonical signs, `u_applied_prev`, absence of `a_phi`, absence of AMCEO scheduler terms, absence of internal output limiting, ID 5 activation, TD diagnostic mapping, protected controller hashes, and no diff in allocator/control-allocator/boards/PWM paths.

## 12. Clean PX4 Build Result

Environment:

```bash
export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"
make matek_h743_dummy
```

The final build started with `build/matek_h743_dummy` absent.

| Item | Result |
|---|---|
| Compiler | GNU Arm Embedded Toolchain 9-2020-q2-update, GCC 9.3.1 20200408 |
| Configure Python | 3.12.3 |
| Build count | `1065/1065` |
| Exit code | 0 |
| Elapsed | 24.61 s real, 25.31 s user, 10.34 s sys |
| FLASH | 1,598,064 B / 1,920 KiB = 81.28% |
| AXI SRAM | 44,652 B / 512 KiB = 8.52% |

The configuration still prints the previously documented Python 3.12/SymForce circular-import traceback. It is nonfatal in this verified environment: configure completed and all 1,065 build steps passed. No source, warning flag, or unrelated PX4 workaround was used.

## 13. Frozen Firmware Artifacts

| Artifact | Size | Timestamp (Asia/Shanghai) | SHA-256 |
|---|---:|---|---|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,537,765 B | 2026-09-07 21:48:10.050324703 +0800 | `01cdc277606f080fcc95ad2d79a6585eb68e07e18ab3dfa75c00dee5ae9a8174` |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 36,850,156 B | 2026-09-07 21:48:09.914372241 +0800 | `ce4bc8504e4a3944bbbd1fcbdfe11d8572535ce734ec13ccb1527cf6bed2645d` |
| `build/matek_h743_dummy/parameters.json` | 706,966 B | 2026-09-07 21:48:04.245864992 +0800 | `2e31c2dc4ce11c5d91c9a8687ab7daca49d937662dc10b58211e9a6a8bbd8e64` |
| `build/matek_h743_dummy/parameters.xml` | 739,055 B | 2026-09-07 21:48:04.233794336 +0800 | `2af67c88007e73b1fe68e91d6f39b946d438af8767e1db2193f32a4ee379cd6e` |

This `.px4` is the only firmware candidate produced by this stage for the future Stage 5 commissioning. The former Stage 4 PX4 hash `0c391a...f0664` and ELF hash `71a6c1...171` are superseded and must not be flashed as the final six-controller firmware.

## 14. Git Diff Summary

Before adding this report, the cumulative tracked diff for Stages 2B–4B was:

```text
6 tracked files changed, 1140 insertions(+), 3 deletions(-)
```

Because all staged controller infrastructure remains intentionally uncommitted, controller/test/report additions appear as untracked files and are not included in `git diff --stat`. The exact status is captured in the updated Stage 5 precommission snapshot.

No tracked or untracked modification was found under:

- `src/modules/hydro_allocator/*`
- `src/modules/control_allocator/*`
- `boards/*`
- PWM/output mapping files

## 15. Remaining Risks and Hardware Hold

- All NADRC tuning values are numerical starting points only; none has been validated on the physical robot.
- The explicit-Euler nonlinear ESO/TD can still diverge under real sensor noise, sign errors, timing jitter, inappropriate gains, or actuator/model mismatch.
- `HY_NA_CDEL` is intentionally shared across unlike angle/rate error dimensions and needs disciplined experimental interpretation.
- `HY_NA_TDR` is an algorithmic Han tuning factor; no unsupported physical unit is claimed.
- Bench data without actual hydrodynamic actuation cannot validate disturbance-estimate accuracy or closed-loop stabilization.
- The 10 ms timing gate, manual timeout, reference values, and common output/slew limits still require actuator-inhibited real-hardware validation.
- Physical hydrofoil left/right wiring and output reversal remain unconfirmed.
- Python 3.12/SymForce remains a reproducibility warning even though the GCC 9 clean build passes.
- Armed controller switching remains blocked and was not tested on hardware.

Stage 5 remains **HOLD** until the flight controller is connected with propulsion and hydrofoil actuation physically isolated. No hardware action was performed in Stage 4B.

## 16. Exact Next Step

Resume Stage 5 only after physical isolation is explicitly confirmed. Then, while disarmed, verify the new artifact hash before flashing, export the original vehicle parameters, flash only the Stage 4B `.px4`, confirm all six selector IDs, and perform sensor/reference/observer/logger checks without permitting actuator motion. Do not begin Stage 6 or tune performance gains automatically.
