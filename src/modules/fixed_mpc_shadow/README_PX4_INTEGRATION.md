# fixed_mpc_shadow PX4 Module Skeleton

This is a shadow-mode implementation package. It is not flight-ready active-output firmware.

Default behavior:

- computes `delta_mpc_suggested`;
- logs diagnostics;
- does not publish actuator commands;
- active output is compile-time disabled by `FIXED_MPC_SHADOW_ALLOW_ACTIVE_OUTPUT=0`;
- runtime gates fail closed because `actuator_sign_confirmed=false` by default.

Copy this folder into `PX4-Autopilot/src/modules/fixed_mpc_shadow/` and add it to the PX4 modules build only for compile/integration checks. Matek H743 target naming must be confirmed in the local PX4 tree.
