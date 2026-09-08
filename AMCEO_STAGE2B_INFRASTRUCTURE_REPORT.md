# AMCEO Stage 2B — Multi-Controller Infrastructure and Observability Report

## 1. Executive Summary

Stage 2B 已完成，并在 revision `6abec104d4680a2c2c36499d665aec7ab18a3063`、target `matek_h743_dummy` 上通过最终 clean build（`1064/1064`）。本阶段建立了 selector、未来控制器公共输入/输出、独立 placeholder state、manual roll latch、二阶 reference generator、公共 rate/magnitude limiter、finite safety gate 和 controller-agnostic uORB diagnostics；没有实现 LADRC、MCEO-ADRC、AMCEO-ADRC 或 reserved ADRC 的控制公式。

现有 CADRC 仍是 behavioral baseline：`CADRC.cpp`、`CADRC.h` 未修改，CADRC 调用、raw `Uc` feedback、reference、`dt` policy、scale、两次 clamp 和 trim 顺序均未改变。`HydroAllocator`、标准 `ControlAllocator`、boards、PWM/output mapping 也未修改。

新 controller ID 2–5 当前不会产生控制输出。ID 2–4 的 placeholder 明确返回 `valid=false`、`u_requested=NaN`；ID 5 没有数学类。STAB 中选择这些 ID 时运行现有 legacy roll branch，并通过 `fallback_active` 报告。armed 状态下 selector edge 被阻止，active controller 不变。

新增参数的默认值为：

| Parameter | Default | Meaning |
|---|---:|---|
| `HY_RLL_CTRL_SEL` | 1 | existing CADRC |
| `HY_RREF_W` | 2.0 rad/s | new-controller reference natural frequency；placeholder，未实机整定 |
| `HY_RREF_ZETA` | 1.0 | reference damping ratio；placeholder，未实机整定 |
| `HY_RLL_U_MAX` | 1.0 normalized | final new-controller roll magnitude bound |
| `HY_RLL_DU_MAX` | 1.0 normalized/s | new-controller slew bound；placeholder，未实机整定 |
| `HY_RLL_MAN_TMO` | 0.5 s | manual latch stale timeout |

## 2. Files Added/Modified

Modified tracked files:

- `msg/CMakeLists.txt`
- `src/modules/hydro_rate_control/CMakeLists.txt`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`
- `src/modules/logger/logged_topics.cpp`

Added Stage 2B files:

- `msg/HydroRollControllerStatus.msg`
- `src/modules/hydro_rate_control/HydroRollController.hpp`
- `src/modules/hydro_rate_control/HydroRollController.cpp`
- `src/modules/hydro_rate_control/RollReferenceGenerator.hpp`
- `src/modules/hydro_rate_control/RollReferenceGenerator.cpp`
- `src/modules/hydro_rate_control/LADRC.hpp`
- `src/modules/hydro_rate_control/LADRC.cpp`
- `src/modules/hydro_rate_control/MCEO_ADRC.hpp`
- `src/modules/hydro_rate_control/MCEO_ADRC.cpp`
- `src/modules/hydro_rate_control/AMCEO_ADRC.hpp`
- `src/modules/hydro_rate_control/AMCEO_ADRC.cpp`
- `src/modules/hydro_rate_control/test/HydroRollInfrastructureTest.cpp`
- `src/modules/hydro_rate_control/test/CadrcGoldenTest.cpp`
- `src/modules/hydro_rate_control/test/Stage2BSourceContractTest.py`
- `AMCEO_STAGE2B_INFRASTRUCTURE_REPORT.md`

No `ADRC_VARIANT` source file was created; selector ID 5 is reserved only.

## 3. Controller Selector Design

`HY_RLL_CTRL_SEL` is an integer enum:

| ID | Controller | Stage 2B behavior |
|---:|---|---|
| 0 | Legacy | existing non-CADRC rate-control branch |
| 1 | CADRC | existing `ADRCController` branch, conditional on `HY_CADRC_EN` |
| 2 | LADRC | unimplemented, legacy fallback |
| 3 | MCEO-ADRC | unimplemented, legacy fallback |
| 4 | AMCEO-ADRC | unimplemented, legacy fallback |
| 5 | ADRC variant reserved | unimplemented, legacy fallback |

Priority is explicit: selector chooses the active controller identity; `HY_CADRC_EN` remains the existing CADRC enable/kill switch. Thus `SEL=1, HY_CADRC_EN=1` runs CADRC and is the default. `SEL=1, HY_CADRC_EN=0` runs the existing legacy branch and reports fallback. `HY_CADRC_EN` does not enable CADRC when another selector ID is active. The parameter was not deleted or renamed.

State machine:

| Current event/state | Action | Output policy |
|---|---|---|
| startup/default | `active=previous=requested=CADRC` | existing CADRC behavior |
| valid selector edge while disarmed | save `previous`, accept new `active`, record edge/reset counter, reset only the selected controller state | selected implemented path, or legacy fallback if unimplemented |
| selector edge while armed | keep `active` unchanged, set `switch_blocked`, increment fault counter, emit warning | continue the already-active path; no armed switching |
| disarm after blocked request | accept the pending requested controller and reset its state | selected/fallback policy above |
| enter STAB with a new-controller ID | initialize latch/reference/limiter from measured roll and previous applied command; reset selected controller | placeholder remains invalid, so legacy fallback |
| leave STAB | reset new-controller latch/reference/limiter and selected new controller | normal behavior for the new mode |
| invalid selector value | sanitize request to Legacy, record fault/warning | armed: block edge; disarmed: accept Legacy |

Only `updateNewController(active_controller, input)` is called. Inactive LADRC/MCEO/AMCEO instances are not updated, and each class owns separate reset/update state.

## 4. Legacy CADRC Preservation Evidence

Protected hashes before and after Stage 2B are identical:

```text
a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201  CADRC.cpp
ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54  CADRC.h
```

The executed CADRC roll sequence remains:

```text
_cadrc_roll.update(roll, roll_sp, p, dt_used)
raw previous Uc remains internal observer feedback in ADRCController
getUc() * HY_CADRC_U_SCALE
constrain(..., -1, +1)
+ TRIM_ROLL
constrain(..., -1, +1)
hydro_torque_setpoint.xyz[0]
```

`HY_CADRC_U_SCALE` remains `0.001`. The original manual publication-edge behavior is retained for CADRC: when no valid new `manual_control_setpoint` is published, the stored legacy setpoint is cleared and CADRC obtains the same zero reference as before. CADRC does not use the new latch, reference generator, common limiter, finite gate, or `u_applied_prev` convention.

Preservation checks:

- source-contract test enforces protected hashes and the order of the CADRC statements;
- 12-sample golden vector checks raw `Uc`, generated rate setpoint and observer states;
- the same golden test verifies `Uc * 0.001 -> clamp -> trim 0.1 -> clamp`, producing `-0.9` for its terminal vector;
- `git diff --exit-code` is clean for `CADRC.cpp/.h`, `hydro_allocator`, `control_allocator`, and `boards`.

The diagnostic reads of CADRC states do not write controller state or change execution order.

## 5. New Controller Interface

`HydroRollControllerInput` is used only by future controllers:

| Field | Unit/meaning |
|---|---|
| `phi` | measured roll angle, rad |
| `p` | measured body roll rate, rad/s |
| `command_roll` | raw/latched roll-angle command, rad |
| `phi_ref` | processed reference, rad |
| `phi_ref_dot` | processed reference rate, rad/s |
| `phi_ref_ddot` | processed reference acceleration, rad/s² |
| `dt_measured` | measured/clamped gyro-control interval, s |
| `dt_used` | validated interval passed to a new controller, s |
| `u_applied_prev` | previous final common-limited roll command, normalized |
| `timestamp_sample` | gyro/control sample timestamp, µs |

`HydroRollControllerOutput` contains normalized pre-limiter `u_requested`, `valid`, and `reset_event`. Unlike legacy CADRC raw `Uc`, no scale such as `0.001` is permitted in this contract.

LADRC, MCEO-ADRC and AMCEO-ADRC placeholders each implement `reset(input)`, `update(input)` and `isValid()`. Their state objects and counters are independent. No observer or control equation is present. Each `update()` returns `{NaN, false, reset_event}` and therefore cannot reach the actuator override. The reserved ID has no class.

## 6. Manual Reference Latch

`ManualRollCommandLatch` is isolated from the legacy/CADRC reference path.

- A newly published, valid, finite, nonzero-timestamp, non-future and fresh sample updates `manual_roll_raw`, source, timestamp and `command_roll = manual_roll * radians(HY_ROLL_MAX)`.
- A publication gap retains the last command until `HY_RLL_MAN_TMO` expires.
- An explicit invalid/nonfinite/zero-timestamp/future/stale sample invalidates the latch and holds the current measured roll as the safe command.
- Unknown/invalid age is reported as NaN, not as a fabricated zero.
- Entering a new-controller STAB branch initializes the latch at current measured roll; leaving STAB resets it.
- Raw manual roll, latched radian command, validity, `data_source`, and age are published in diagnostics.

The timeout default is 0.5 s. It is a conservative infrastructure default, not a validated RC failsafe specification. PX4's upstream/manual-control validity and arming safeguards remain authoritative outside this local latch.

## 7. Reference Generator Design

`RollReferenceGenerator` is a resettable, new-controller-only second-order model:

```text
phi_ref_dot  = v_ref
phi_ref_ddot = omega_r^2 (command_roll - phi_ref)
               - 2 zeta_r omega_r v_ref
```

It uses explicit forward Euler with explicit `dt`:

```text
phi_ref[k+1] = phi_ref[k] + v_ref[k] dt
v_ref[k+1]   = v_ref[k] + phi_ref_ddot[k] dt
```

The reported acceleration is recomputed from the post-step state. Reset selects finite current roll first, then finite command, else zero; rate and acceleration initialize to zero. Invalid/nonfinite command, parameters, state, or nonpositive `dt` makes the generator invalid.

`HY_RREF_W=2.0 rad/s` and `HY_RREF_ZETA=1.0` are explicitly placeholder defaults and have not been flight tuned. No Han TD or nonlinear function was introduced. CADRC does not call this class.

## 8. Common Limiter

The new-controller-only sequence is:

```text
normalized u_requested
+ normalized TRIM_ROLL
rate limiter relative to previous final u_applied
magnitude limiter
u_applied
hydro_torque_setpoint.xyz[0]
```

For `dt=dt_used`:

```text
du_max_sample = HY_RLL_DU_MAX * dt
du = constrain((u_requested + trim) - u_applied_prev,
               -du_max_sample, +du_max_sample)
u_applied = constrain(u_applied_prev + du,
                      -HY_RLL_U_MAX, +HY_RLL_U_MAX)
```

Trim is deliberately before both limits, so the final total roll contribution—not only the controller component—obeys both constraints. With the defaults, the final new-controller roll contribution is in `[-1, +1]` normalized. This policy applies only to future controllers and does not alter CADRC's historic clamp/trim/clamp sequence.

The limiter state is initialized from the previous `_hydro_torque_setpoint.xyz[0]`. A future observer receives `HydroRollLimiter::appliedPrevious()`, which is the previous actually common-limited output. Nonfinite request/trim/dt/limits or invalid bounds reject the result and reset the limiter to zero. No Stage 2B placeholder can supply a valid request.

## 9. Diagnostic uORB Schema

`HydroRollControllerStatus.msg` includes:

- publication/sample timestamps;
- active, requested and previous controller IDs;
- reset/fault counters;
- output/reference/manual validity;
- magnitude/rate-limit, selector-edge, switch-blocked and fallback flags;
- synchronized `phi`, `p`, raw/latched manual command, age/source;
- `phi_ref`, first/second derivatives;
- `u_requested`, actual `u_applied`, measured/used `dt`;
- four generic states, disturbance estimate, observer bandwidth, `rho`, `sigma_e`, `sigma_u`.

Unused or unavailable floating-point values are NaN. For CADRC only, the available observer components are mapped to `state_1..state_4`, and `_Z2(1,0)` is exposed as disturbance estimate. CADRC diagnostic `u_requested` is the historic scaled raw `Uc` before clamps; new-controller `u_requested` is normalized by interface definition. `u_applied` always records the roll contribution actually sent in `hydro_torque_setpoint.xyz[0]`, while `fallback_active` identifies a non-selected implementation supplying it.

`timestamp=hrt_absolute_time()` and `timestamp_sample=vehicle_angular_velocity.timestamp_sample`. Normal publication is limited to one message per 5 ms (at most 200 Hz); selector/reset edges force an immediate publication. `controller_id` is the active controller and `requested_controller_id` is separate. During an intentionally blocked armed edge these IDs differ by design.

The existing `hydro_cadrc_control_message` schema and logger registration were not removed or changed.

## 10. Logging Rate/Bandwidth Analysis

Logger registration uses:

```text
add_topic("hydro_roll_controller_status", 5)
```

The generated uORB payload is 120 bytes. Raw payload rate is therefore approximately:

- 200 Hz: 24,000 B/s (23.4 KiB/s);
- 400 Hz: 48,000 B/s (46.9 KiB/s).

Actual ULog storage is higher due to framing, topic metadata and concurrent topics. A 400 Hz full record would double this topic's sustained bandwidth without adding information because the new infrastructure itself publishes at 200 Hz. The selected 200 Hz preserves 5 ms synchronization among state/reference/request/applied fields and is adequate to resolve transients substantially faster than the placeholder reference bandwidth. Storage/SD-card performance must still be verified before experiments; edge events are not lost merely because of the periodic throttle.

## 11. Safety/Finite Handling

The common new-controller path checks `phi`, `p`, command, all three reference quantities, `dt_measured`, `dt_used`, and requires positive `dt_used` before calling a selected controller. It separately checks a controller claiming `valid=true` has finite `u_requested`, then validates limiter inputs/output.

On invalid new-controller input/output:

- the invalid controller output is never assigned to roll torque;
- selected controller/reference/limiter state is reset as appropriate;
- fault/reset counters increment on a new fault edge;
- the fault is latched to avoid repeated reset/counter increments every gyro iteration;
- existing legacy control supplies the explicit fallback output;
- diagnostics report invalid output/reference and fallback/fault state.

The fault latch is cleared only after an accepted selector/mode reset or a genuinely valid, finite and successfully limited new-controller output. These checks are outside the CADRC branch and do not change its historical behavior.

Remaining safety limitation: this is compile- and host-test-level infrastructure. No HIL, actuator-disabled hardware validation, flight, flash, or pool test was performed in Stage 2B.

## 12. Build Result

Final clean build environment and command:

```bash
cd /home/linyi666/PX4-Autopilot-water-air
export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"
make matek_h743_dummy
```

Result:

| Item | Result |
|---|---|
| Compiler | GNU Arm Embedded Toolchain 9-2020-q2-update, GCC 9.3.1 20200408 |
| Python | 3.12.3 |
| Clean-build progress | `1064/1064` |
| Exit code | 0 |
| Elapsed | 24.69 s (`user 24.38`, `sys 9.39`, warm Stage 2A ccache) |
| FLASH | 1,587,512 B / 1,920 KiB, 80.75% |
| AXI SRAM | 44,652 B / 512 KiB, 8.52% |

Bounded builds were performed after foundation, message/logging, integration/state-machine, test, and final-fault-latch changes. The last bounded incremental build was `19/19`, exit 0, before the final clean build.

Two integration errors were resolved only inside the new Stage 2B code/metadata:

1. First fatal parameter-generation error: PX4 rejected `@unit norm/s` for `HY_RLL_DU_MAX`. Unsupported metadata was removed while the description and C++ comments retain the exact normalized/s unit.
2. First fatal C++ error: the target's `-nostdinc++` environment does not provide `<algorithm>`, followed by the same issue for `<limits>`. The new helper code was rewritten using available `<cmath>`, `fminf/fmaxf`, and `NAN`.

No warning flag was disabled and no unrelated PX4 source was modified. The known Python 3.12/SymForce circular-import traceback still appears during configuration, as in Stage 2A, but is nonfatal for this target: generation, compilation, linking and `.px4` creation all completed. A CMake developer warning and one Python `SyntaxWarning` also remain environment/upstream warnings, not promoted compile failures.

## 13. Deterministic/Golden Tests

All tests passed after the final clean build:

```text
HydroRollInfrastructureTest: PASS
CadrcGoldenTest: PASS
Stage2BSourceContractTest: PASS
git diff --check: PASS
```

Repeat commands:

```bash
g++ -std=c++14 -Wall -Wextra -Werror \
  -Isrc/modules/hydro_rate_control \
  src/modules/hydro_rate_control/test/HydroRollInfrastructureTest.cpp \
  src/modules/hydro_rate_control/HydroRollController.cpp \
  src/modules/hydro_rate_control/RollReferenceGenerator.cpp \
  src/modules/hydro_rate_control/LADRC.cpp \
  src/modules/hydro_rate_control/MCEO_ADRC.cpp \
  src/modules/hydro_rate_control/AMCEO_ADRC.cpp \
  -o /tmp/HydroRollInfrastructureTest
/tmp/HydroRollInfrastructureTest

g++ -std=c++14 -Wall -Wextra -Werror \
  -Ibuild/matek_h743_dummy -Isrc -Isrc/lib -Isrc/lib/matrix \
  -Iplatforms/common -Iplatforms/common/include \
  src/modules/hydro_rate_control/test/CadrcGoldenTest.cpp \
  src/modules/hydro_rate_control/CADRC.cpp \
  -o /tmp/CadrcGoldenTest
/tmp/CadrcGoldenTest

python3 src/modules/hydro_rate_control/test/Stage2BSourceContractTest.py
```

Acceptance coverage:

| Requirement | Evidence |
|---|---|
| default path remains CADRC | parameter/default and initial-state source contract |
| CADRC unchanged for fixed sequence | raw/scaled golden vectors plus protected file hashes/order |
| unimplemented selection cannot output random value | placeholders return NaN/invalid; source-contract fallback gate |
| inactive controller does not update | independent placeholder update-counter test |
| latch survives publication gaps | deterministic latch test |
| legacy CADRC reference unchanged | source contract requires original clear-on-gap statements |
| reference reset finite | deterministic reset/update test |
| slew bound | 100-step assertion `abs(u[k]-u[k-1]) <= DU_MAX*dt` |
| magnitude bound | 100-step assertion `abs(u)<=U_MAX` |
| NaN/Inf safety | reference and limiter NaN rejection plus source finite gate |
| timestamp nonzero/valid assignment | source contract checks `hrt_absolute_time` and gyro sample assignment |
| controller ID consistency | active/requested fields and selector source contract; blocked-edge distinction documented |

These are deterministic host/golden/static checks, not real-time PX4 SITL/HIL tests.

## 14. Git Diff Summary

Before Stage 2B, tracked source/configuration was clean; the three Stage 1/2A reports were already untracked. After implementation, tracked diff summary before adding this report was:

```text
6 tracked files changed, 513 insertions(+), 2 deletions(-)
```

Tracked changes are exactly the six modified files in Section 2. New Stage 2B implementation/test/message files and this report are untracked until deliberately added to version control. Existing untracked non-Stage-2B documents are:

```text
AMCEO_BUILD_ENVIRONMENT.md
AMCEO_INTEGRATION_BASELINE_AUDIT.md
AMCEO_STAGE2A_BUILD_BASELINE.md
```

Protected-scope verification is clean:

```text
CADRC.cpp/.h: unchanged
src/modules/hydro_allocator/*: unchanged
src/modules/control_allocator/*: unchanged
boards/*: unchanged
```

No PWM/output mapping file was touched. `git diff --check` passes.

## 15. Remaining Risks

Firmware artifacts:

| Artifact | Size | Timestamp (Asia/Shanghai) | SHA-256 |
|---|---:|---|---|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,528,413 B | 2026-09-07 17:56:23.664631497 +0800 | `e57d6fb1c9a8546397a7db80bb5bdde7e0dffd3aae6d90d39338f62c4614e66d` |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 36,702,724 B | 2026-09-07 17:56:23.525964884 +0800 | `de73186c71d3c69350ea4669b9baeffa73eff825e1f6a899c81fecee19d2bb5f` |

Remaining risks:

- ID 2–5 intentionally run legacy fallback; they are not suitable for controller-performance experiments yet.
- `HY_RREF_W`, `HY_RREF_ZETA`, `HY_RLL_DU_MAX`, `HY_RLL_U_MAX`, and timeout defaults are infrastructure placeholders, not identified/tuned flight values.
- The new trim-before-limits policy must be included in Stage 3 plant/controller definition; CADRC deliberately retains its different legacy sequence.
- `u_applied_prev` is the final roll torque contribution before downstream allocation, not measured fin deflection. Allocator saturation or actuator dynamics are not fed back by the current interface.
- Manual timestamp timeout semantics were host-tested but not validated against all real RC/data-source handover cases.
- Armed selector changes are blocked in software, but parameter tooling/operator workflow should still prevent in-test edits.
- The 200 Hz topic adds approximately 24 kB/s raw logging traffic. SD/flash throughput and total ULog topic load require bench validation.
- Diagnostic CADRC `u_requested` is scaled legacy `Uc`; new algorithms use a normalized request by contract. Analysis scripts must branch on `controller_id`.
- Generic `state_1..4` mappings require a per-controller schema document in Stage 3; unused fields currently remain NaN as intended.
- `dt` is the existing HydroRateControl clamped gyro interval; Stage 3 must not silently substitute a different observer time base.
- No runtime test has yet exercised disarm acceptance after a blocked selector edge, logger dropouts, reboot persistence, NaN injection in PX4 scheduling context, or downstream actuator response.
- The nonfatal Python 3.12/SymForce traceback remains an environment reproducibility risk even though GCC 9 baseline and Stage 2B builds succeed.

## 16. Exact Plan for Stage 3 LADRC/MCEO Implementation

1. Freeze this Stage 2B commit/artifact and preserve the CADRC hashes/golden vectors as mandatory regression gates.
2. Specify LADRC and MCEO equations, state meanings, parameters, units, signs, reset initialization and saturation assumptions from the approved design/paper before coding; do not infer them from names.
3. Implement LADRC only inside `LADRC.hpp/.cpp`. Consume `HydroRollControllerInput`, return normalized `u_requested`, use the supplied `dt_used`, and expose typed diagnostics. Do not edit CADRC or the common limiter.
4. Add controller-level golden vectors for observer propagation, disturbance compensation, reset, finite rejection and exact normalized output. Run host tests and GCC 9 target build before selector activation is considered.
5. Implement MCEO-ADRC only inside `MCEO_ADRC.hpp/.cpp` with completely separate observer/history state. Its observer must consume `u_applied_prev`, never the current/raw requested output.
6. Define and document the controller-specific mapping to `state_1..4`, disturbance, bandwidth, `rho`, `sigma_e`, and `sigma_u`; keep unavailable quantities NaN.
7. Keep armed switching blocked. First test selection only while disarmed, with props/fins mechanically safe or actuator outputs disabled; verify reset counters, active/requested IDs, reference initialization and first-output slew bound.
8. Run a no-actuator SITL/HIL or replay sequence covering mode entry/exit, RC publication gaps, timeout, selector edges, NaN/Inf injection, scheduler `dt` limits and ULog synchronization.
9. Compare selector 1 against the Stage 2B CADRC golden/artifact behavior again. Any CADRC drift is a release blocker.
10. Only after LADRC/MCEO pass code review, deterministic tests, target build and bench observability should AMCEO scheduling/equations be introduced in its own files. ID 5 remains reserved.

Stage 3 should not modify allocator/PWM mapping. If an algorithm requires actual post-allocation actuator feedback rather than the current final roll torque contribution, that is a separate interface/design decision and must be audited before implementation.
