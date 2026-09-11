# Dry-bench response investigation — HOLD, not a verified fix

## Scope and evidence

The user reports high-frequency small foil oscillations and approximately two
seconds of apparent reference-response delay for IDs 2–5. No corresponding ULog
or live new-controller parameter snapshot has yet been supplied. Earlier CADRC
measurements and a waiting-mode gyro sample do not characterize these events.
The previously authorized manual `copy()` correction remains untouched.

This investigation adds an offline test only. No controller equations, gains,
parameter definitions, allocator, board or output mapping are changed. Do not
interpret the test results as authorization for powered actuator testing.

## Confirmed software findings

1. With the compiled placeholder values `HY_RREF_W=2`, `HY_RREF_ZETA=1`, the
   actual `RollReferenceGenerator` takes **1.9425 s to reach 90%** and **2.3675 s
   to reach 95%** of a ten-degree reference step at a test interval of 2.5 ms.
   This fixture has no observer and no z3. It demonstrates reference shaping,
   not a two-second transport dead time. IDs 2–4 use this generator; ID 5 does not.
2. With compiled `HY_RLL_DU_MAX=1`, `HY_RLL_U_MAX=1`, the actual common limiter
   takes **2.0000 s from -1 to +1**, starting to move on the first sample.
   This fixture also has no observer. In general the slew-only travel time is
   at least `abs(delta_u)/DU_MAX`; a small requested change does not necessarily
   take two seconds. IDs 2–5 share this limiter. These limits apply to hydro roll
   contribution, not physical servo angular velocity.
3. All four actual controller implementations, with compiled starting gains,
   zero measured angle/rate/reference/trim, previous limited command feedback,
   and 2.5 ms test intervals, produce zero request, applied command and z3 for
   60 seconds (24,000 samples each). This does NOT exclude noise sensitivity,
   nonzero attitude/trim, timing faults, fallback transitions or actuator effects.
4. There is no z3 clamp in the approved new-controller implementations. They
   have finite checks and external output limiting. LADRC/NADRC z3 is total
   disturbance; MCEO/AMCEO z3 is residual disturbance. Adding an arbitrary
   clamp would change the approved equations and would not address findings 1–2.
5. Selecting a new roll controller does not reproduce the entire ID 1 path:
   ID 1 runs CADRC roll AND pitch and uses manual throttle, while IDs 2–5 enter
   the legacy rate-control branch for the other contributions, use its thrust
   setpoint, and override roll only. The nonlinear hydro allocator consumes
   multiple torque/thrust components. Standard servo allocation subsequently
   adds the hydro servo contribution. Foil motion alone cannot locate a fault
   in roll z3, nor prove the common roll limiter is bypassed.
6. A manual-reference loss can select legacy fallback; recovery for IDs 2–4
   currently does not uniformly reset observer/limiter on latch revalidation.
   This is a separate source-level recovery risk, not a proven cause of the
   reported continuous-run vibration. Runtime reset/fallback evidence is needed
   before attributing these symptoms to it or redesigning the recovery policy.

## Required evidence before choosing a firmware fix

- Existing event ULog, ideally with the stationary period and reference change.
  Do not reproduce hazardous motion just to obtain a log.
- Actual `HY_RREF_W`, `HY_RREF_ZETA`, `HY_RLL_U_MAX`, `HY_RLL_DU_MAX`,
  `HY_RLL_MAN_TMO`, `HY_RLL_DT_MAX`, common gains, controller-specific gains and
  trims. Compiled defaults are not a substitute for the onboard snapshot.
- Time-aligned `hydro_roll_controller_status`: raw/latched/reference signals,
  states, request/applied, rate/magnitude flags, reset/fault/fallback and timing.
- `hydro_torque_setpoint`, `hydro_thrust_setpoint`, `hydro_allocate_message`,
  `hydro_servos`, `actuator_servos`, manual setpoint and angular velocity if logged.

Locate the delay in the earliest affected signal: latch, reference, request,
common-limited roll, hydro servo contribution or final servo command. Distinguish
initial dead time from slow rise time. Locate oscillation in the same way.
Downsampled diagnostics do not prove a per-controller-sample slew violation.

On a stationary dry bench, foil motion cannot produce the intended waterborne
roll acceleration. Observer estimates therefore cannot be interpreted as a
validated hydrodynamic disturbance measurement or used alone for gain tuning.

## Reproduction

Run from the repository root (host test, no hardware access):

```sh
g++ -std=c++14 -Wall -Wextra -Werror \
  src/modules/hydro_rate_control/test/BenchSignalPathTest.cpp \
  src/modules/hydro_rate_control/HydroRollController.cpp \
  src/modules/hydro_rate_control/RollReferenceGenerator.cpp \
  src/modules/hydro_rate_control/LADRC.cpp \
  src/modules/hydro_rate_control/MCEO_ADRC.cpp \
  src/modules/hydro_rate_control/AMCEO_ADRC.cpp \
  src/modules/hydro_rate_control/ClassicalNADRC.cpp \
  -o /tmp/amceo-bench-signal-path-test
/tmp/amceo-bench-signal-path-test
```

`BenchSignalPathTest: PASS`. It deliberately tests components without inventing
a physical robot model or pretending to replay an unavailable hardware log.

## Decision

**HOLD**: slow reference/limiter behavior is reproduced with compiled defaults;
the physical oscillation root cause and actual onboard delay remain unverified.
No z3 cap, performance retuning, branch redesign, or claimed hardware fix is
justified by the available evidence. Keep propulsion/servos physically isolated.
