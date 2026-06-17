# STAGE4I_ONBOARD_IMPLEMENTATION_EVIDENCE_SUMMARY

Actual output path:

```text
F:\Article\MPC\log\stage4i_onboard_implementation_evidence_consolidation\results
```

## Scope

Stage 4i consolidates evidence only. It does not modify source code, compile firmware, flash hardware, or enable active output.

Consolidated stages:

- Stage 3c fixed-linear MPC offline validation
- Stage 4f runtime diagnostics / compile-ready onboard shadow integration
- Stage 4g sensor input, shadow diagnostic chain, and atomic diagnostics
- Stage 4h read-only CADRC/allocator base-command shadow interface
- Stage 4h-r3b hydro zero-output cause audit

Requested Stage 4h-r3c artifact:

```text
MISSING
```

No `F:\Article\MPC\log\stage4h_r3c_*` directory was found in the specified local result set during this consolidation.

## Consolidated evidence

### 1. Offline controller selection

Stage 3c identifies `FIXED_LINEAR_MPC` as the best active offline deployment candidate among the compared controllers. It outperformed `SIMPLE_PD`, `HISTORY_ARX_MPC_TUNED`, and `LOGGED_INPUT_REPLAY` in the unified offline comparison, while remaining more interpretable and deployment-oriented than the history-ARX alternative.

Key evidence:

- mean roll RMSE: `1.57282 deg`
- recommendation: `PROMISING_FOR_HIL_AFTER_SIGN_CONFIRMATION`
- embedded feasibility for H743-class target: `LIKELY`

This remains an offline claim only, not a closed-loop hardware-validation claim.

### 2. Onboard deployment evidence

The PX4 shadow module was compiled repeatedly into the `matek_h743_dummy` firmware target. File-backed evidence from Stage 4f, Stage 4g, and Stage 4h-r2 shows:

- `fixed_mpc_shadow` was linked into the PX4 image
- the final `matek_h743_dummy.px4` artifact was created
- the module exposes explicit `help/start/status/stop` commands
- the module is scheduled on work queue `lp_default`
- the nominal interval is `20 ms`

The current thread state further states that the image was flashed successfully to a real H743/PX4 target and that runtime bench execution was stable at `50 Hz`, but these specific items were not found as standalone archived logs within the listed Stage 4f–4h directories. They are therefore treated separately as `THREAD_CONTEXT_ONLY`.

### 3. Real sensor input evidence

Stage 4g-r1 establishes that the onboard module reads:

- `vehicle_attitude`
- `vehicle_angular_velocity`

and exposes status fields:

- `attitude_valid`
- `angular_velocity_valid`
- `roll_rad`, `roll_deg`
- `roll_rate_rad_s`, `roll_rate_deg_s`
- `pitch_deg`, `yaw_deg`

The documented roll-rate source is:

```text
vehicle_angular_velocity.xyz[0] (PX4 body-frame roll rate)
```

This is sufficient to claim that the shadow module used real onboard attitude and angular-rate inputs.

### 4. Shadow diagnostic-chain evidence

By Stage 4g-r4, the shadow module implements a paper-defensible atomic diagnostic chain:

```text
raw -> amplitude_limited -> rate_limited -> pre_gate -> shadow_correction -> post_gate
```

The semantics were stabilized across Stages 4g-r2 through 4g-r4, with:

- explicit amplitude-limit and rate-limit layers
- compatibility mapping for legacy fields
- atomic snapshot publication
- strict consistency checks
- host-side consistency tests

The retained limitation is also clear:

```text
Current standalone fixed-MPC pre-gate output saturates early in bench small-angle tests.
```

This means the shadow module is useful as an implementation and diagnostic result, but not as evidence of active-ready standalone control.

### 5. Read-only CADRC/allocator base-command interface evidence

Stage 4h-r1 audited the authoritative CADRC roll command source:

```text
src/modules/hydro_rate_control/HydroRateControl.cpp
HydroRateControl::Run()
hydro_torque_setpoint.xyz[0] = u_roll
```

Stage 4h-r2 then added read-only onboard subscriptions to:

- `hydro_servos` as allocator-domain base command
- `hydro_torque_setpoint` as CADRC provenance/debug

and defined dual-sign base-command diagnostics:

```text
delta_base_A = left_servo_base - right_servo_base
delta_base_B = right_servo_base - left_servo_base
```

as well as diagnostic-only candidate shadow combinations:

```text
delta_candidate_shadow_A/B
delta_candidate_post_gate_A/B = 0
```

These additions remained read-only and did not publish any control topic.

### 6. Safety-gate evidence

Across Stages 4f–4h, the safety posture remains consistent:

```text
SHADOW_ONLY
ACTIVE_OUTPUT_BLOCKED
SIGN_UNCONFIRMED
CADRC_AUTHORITATIVE
NO_ACTUATOR_PUBLICATION
```

Operationally, the evidence supports:

- `active_output_allowed = false`
- `actuator_sign_confirmed = false`
- `safety_gate_pass = false`
- `fallback_reason = SIGN_NOT_CONFIRMED` for valid shadow operation
- `delta_mpc_post_gate = 0`
- `delta_candidate_post_gate_A = 0`
- `delta_candidate_post_gate_B = 0`

Static forbidden-publication scans in the archived results remain `PASS`.

### 7. Zero-base bench explanation

The current bench state reported in the thread shows:

```text
hydro_torque_setpoint = 0
hydro_thrust_setpoint = 0
hydro_servos = 0
delta_base_A/B = 0
```

Stage 4h-r3b explains this conservatively as a hydro control-chain activation issue, not a `fixed_mpc_shadow` failure. The strongest source-code explanation is PX4 runtime gating:

- hydro outputs are explicitly forced to zero when `vehicle_status.nav_state` is not one of:
  - `STAB`
  - `ACRO`
  - `MANUAL`

Additional likely contributors are:

- `hydro_rates_setpoint` not being published
- `manual_control_setpoint` being invalid, stale, or centered
- the bench operating in a branch where static body tilt does not create nonzero hydro torque

Therefore, the present zero hydro base command should not be misinterpreted as a failed read-only base-command interface.

## What can be claimed

The following claims are supported:

1. The fixed-linear MPC shadow module was implemented onboard on an H743/PX4 target.
2. It was compiled into the PX4 firmware image and scheduled at a 50 Hz target rate.
3. It read real attitude and angular-rate signals from the onboard PX4 state topics.
4. It computed bounded shadow corrections and maintained an atomic diagnostic chain.
5. It subscribed to the CADRC/allocator base-command chain in read-only mode.
6. All post-gate outputs remained zero under safety blocking.
7. CADRC remained authoritative and the shadow module did not publish actuator or hydro commands.

## What cannot be claimed

The following claims are not supported and must not be made:

1. The fixed MPC controlled the robot.
2. The fixed MPC controlled the fins/servos.
3. Active output was enabled.
4. Actuator sign was confirmed.
5. Closed-loop MPC experiments were completed.
6. Water-tank active validation was completed.
7. Outdoor active MPC validation was completed.
8. CADRC was replaced.

## Bottom line

The consolidated evidence is strong enough for a paper-ready onboard implementation claim in **shadow mode**, with clear safety boundaries and explicit non-claims. It is not yet evidence for active closed-loop MPC control.
