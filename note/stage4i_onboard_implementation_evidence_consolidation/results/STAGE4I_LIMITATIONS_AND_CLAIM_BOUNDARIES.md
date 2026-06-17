# STAGE4I_LIMITATIONS_AND_CLAIM_BOUNDARIES

## Scope boundary

This consolidation covers:

- Stage 3c offline controller selection evidence
- Stage 4f compile-time and runtime-diagnostics implementation evidence
- Stage 4g sensor-input and shadow-diagnostic-chain evidence
- Stage 4h read-only CADRC/allocator base-command interface evidence
- Stage 4h-r3b zero-output cause audit

No new engineering work was performed in Stage 4i.

## Missing or incomplete local artifacts

The following items appear in the current thread state but were not found as dedicated archived artifacts inside the specified Stage 3c / 4f / 4g / 4h result directories:

- explicit flash transcript for the final onboard image
- explicit 5-minute bench runtime log
- explicit onboard solve-time log showing the `13–35 us` range
- explicit CPU-idle log showing `about 70%`
- a local Stage `4h-r3c` results directory

These items should be treated as:

```text
THREAD_CONTEXT_ONLY
```

not as directly reproducible file-backed evidence from the listed result directories.

## Claims that are supported

The following claims are supported by the consolidated evidence:

1. The fixed-linear MPC was the most suitable active offline deployment candidate among the compared controllers.
2. A PX4 shadow-mode module named `fixed_mpc_shadow` was compiled into the `matek_h743_dummy` firmware target.
3. The module exposes `help/start/status/stop` and runs as a scheduled work-queue task with a 20 ms interval target.
4. The shadow module reads real `vehicle_attitude` and `vehicle_angular_velocity` inputs.
5. The module computes a bounded shadow correction chain:
   - `raw -> amplitude_limited -> rate_limited -> pre_gate -> shadow_correction -> post_gate`
6. The implementation uses an atomic diagnostic snapshot and consistency self-check.
7. The module subscribes to the CADRC/allocator base-command chain in read-only mode using:
   - `hydro_servos`
   - `hydro_torque_setpoint`
8. All post-gate outputs remain zero under the safety block.
9. The implementation preserves CADRC authority and does not publish actuator or hydro control commands.
10. The current zero hydro base command observed on bench is attributable to upstream PX4 mode/setpoint gating, not a `fixed_mpc_shadow` read-path failure.

## Claims that are not supported

The following claims are **not** supported and must not be made:

1. `fixed_mpc_shadow` controlled the robot.
2. `fixed_mpc_shadow` controlled fins or servos.
3. Active output was enabled.
4. Actuator sign was confirmed.
5. A closed-loop MPC experiment was completed.
6. Water-tank active validation was completed.
7. Outdoor active MPC validation was completed.
8. CADRC was replaced.
9. The current candidate shadow output is active-ready.

## Recommended paper wording boundary

Use language such as:

```text
deployed in shadow mode
read-only subscription
bounded shadow correction
post-gate output remained zero
active output remained blocked because actuator sign was unconfirmed
```

Avoid language such as:

```text
controlled
closed-loop validated
replaced the onboard controller
demonstrated active fin control
```
