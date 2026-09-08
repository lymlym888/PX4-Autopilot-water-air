# AMCEO Integration Baseline Audit

> Audit date: 2026-09-07 (Asia/Shanghai)  
> Repository: `/home/linyi666/PX4-Autopilot-water-air`  
> Audited revision: `6abec104d4680a2c2c36499d665aec7ab18a3063` (`AMCEO`, `v1.15.0-beta1-345-g6abec104d4`)  
> Scope: source audit only. No existing source, parameter, build configuration, or algorithm was modified.  
> Confidence convention: **confirmed** means directly established by executable source; **historical configuration** means found in an older parameter/ULog artifact and is not asserted to be the currently loaded flight-controller value.

## 1. Executive Summary

This is a customized **PX4 Autopilot** tree (PX4 v1.15 development lineage) using uORB, PX4 work queues, CMake/Ninja, NuttX, and an ARM Cortex-M7 target. The target represented by the active build cache and firmware artifacts is **`matek_h743_dummy`** (`MATEK_H743`, NuttX, `arm-none-eabi`, Cortex-M7). Its normal build command is:

```sh
make matek_h743_dummy
```

The current CADRC is not in the standard PX4 attitude controller. It is embedded in `HydroRateControl::Run()` and is active only when `nav_state == STAB` and `HY_CADRC_EN != 0`. It controls both roll and pitch through two `ADRCController` instances; this report reconstructs roll, with pitch structurally identical.

The confirmed roll path is:

```text
gyro driver (sensor_gyro[_fifo])
  -> VehicleAngularVelocity filtering/calibration -> vehicle_angular_velocity.xyz[0] = p
EKF2 -> vehicle_attitude.q -> Eulerf::phi() = phi
manual_control_setpoint.roll -> HY_ROLL_MAX -> phi_r
  -> ADRCController::update(phi, phi_r, p, dt_used)
  -> Uc -> HY_CADRC_U_SCALE -> magnitude clip -> TRIM_ROLL -> magnitude clip
  -> hydro_torque_setpoint.xyz[0]
  -> HydroAllocator allocation matrix and nonlinear foil/thruster solve
  -> hydro_servos / hydro_motors
  -> standard ControlAllocator adds these to its own actuator setpoints
  -> final [-1,1] clip -> actuator_servos / actuator_motors
  -> output-function mapping -> physical PWM outputs
```

The most important baseline findings are:

1. **The observer uses the previous raw, unnormalized, unlimited CADRC output.** The rate ESO term is `-_J0 * _Uc[k-1]`. It does not receive the command after `HY_CADRC_U_SCALE`, either magnitude clamp, trim, allocation, final actuator clamp, output reversal, or real actuator dynamics.
2. **There is no effective roll-command slew/rate limiter on the custom hydro contribution.** CADRC has magnitude clamps, but no rate clamp. PX4 control-allocation slew limiting is applied before `hydro_servos`/`hydro_motors` are added, so it does not limit the custom addition.
3. **The reference is not reliably held between manual-input publications.** `HydroRateControl::Run()` calls `_manual_control_setpoint_sub.update()` every gyro-driven iteration and explicitly clears the stored setpoint whenever no new manual message is available. At a nominal 400 Hz control loop and slower RC/joystick publication, `phi_r` can alternate between the latest stick command and zero. This contradicts the nearby comment's apparent intent of clearing only absent/invalid RC data.
4. **`phi_r` may come from RC or MAVLink joystick.** `HydroRateControl` checks only `valid` and `timestamp`, not `data_source`. The default/current documented `COM_RC_IN_MODE=3` keeps the first valid RC or joystick source until reboot.
5. **CADRC uses measured gyro roll rate, not observer-estimated roll rate.** `p` is `vehicle_angular_velocity.xyz[0]`, after calibration, bias removal, notches, and low-pass filtering.
6. **The angle and rate streams are asynchronous.** CADRC is triggered by angular-velocity updates, while `phi` is the last available EKF attitude. With the represented settings, rate data is nominally capped at 400 Hz and EKF attitude prediction is historically configured at 100 Hz.
7. **Switching into STAB resets both CADRC instances to all zeros, but is not bumpless.** No state is initialized from current `phi`, `p`, reference, or prior actuator command. Toggling `HY_CADRC_EN` while already in STAB does not reset the controller; stale state can resume.
8. **The custom CADRC log message omits essential signals and never sets its own `timestamp`.** It records reference, scaled pre-limiter command, and four observer states, but not `phi`, `p`, raw `Uc`, rate reference, final limited command, selector/mode, or saturation flags.
9. **Physical left/right wiring cannot be confirmed from source.** Code labels logical index 0 “left” and index 1 “right”. A historical parameter snapshot maps logical 0 to Servo 5/PWM MAIN12 and logical 1 to Servo 4/PWM MAIN11, but this does not prove physical wiring on the current robot.
10. A baseline rebuild was attempted without source changes and failed in unrelated platform code under the installed GCC 13.2.1 (`platforms/common/spi.cpp:90`, `-Werror=address`). A Python 3.12/SymForce circular-import traceback also appeared during configuration. Pre-existing firmware artifacts exist, but they are older than the current `HydroRateControl.cpp` timestamp and therefore are not accepted as a successful baseline build of the exact audited tree.

The safest future integration point is immediately before the existing `HY_CADRC_U_SCALE` and common output-limit block in `HydroRateControl::Run()`: preserve the CADRC call and arithmetic byte-for-byte in the CADRC selector branch, give every new controller its own object and reset lifecycle, and feed all selected outputs through one explicitly documented common post-controller limiter. No change to `HydroAllocator` is required merely to select among roll controllers.

## 2. Build Baseline

### 2.1 Framework and target

| Item | Audit result | Evidence |
|---|---|---|
| Flight stack | PX4 Autopilot, v1.15 development lineage | root PX4 build system; `git describe`: `v1.15.0-beta1-345-g6abec104d4` |
| RTOS/platform | NuttX | `build/matek_h743_dummy/CMakeCache.txt`: `CONFIG_BOARD_PLATFORM=nuttx`; old ULog reports `sys_os_name: NuttX` |
| Messaging | uORB | subscriptions/publications throughout `HydroRateControl.hpp` and `HydroAllocator.hpp` |
| Scheduling | PX4 `ScheduledWorkItem` | `HydroRateControl` uses `nav_and_controllers`; `HydroAllocator` uses `rate_ctrl` |
| Actual represented target | `matek_h743_dummy` | active CMake cache, `.elf`/`.px4` names, `boards/matek/h743/dummy.px4board` |
| Board/CPU | MATEK H743, Cortex-M7 | CMake cache and configuration output |
| Toolchain | `arm-none-eabi` | CMake cache/configuration; installed compiler is GCC 13.2.1 |
| Build type | `MinSizeRel` | `build/matek_h743_dummy/CMakeCache.txt` |

`boards/matek/h743/dummy.px4board:20-22` explicitly enables `HYDRO_ALLOCATOR`, `HYDRO_ATT_CONTROL`, and `HYDRO_RATE_CONTROL`. `ROMFS/px4fmu_common/init.d/rc.fw_apps:36-39` starts those modules after the standard fixed-wing and standard control-allocation modules. The historical parameter snapshots select `SYS_AUTOSTART=2100`, corresponding to `ROMFS/px4fmu_common/init.d/airframes/2100_standard_plane`.

### 2.2 Correct build command

```sh
make matek_h743_dummy
```

An optional parallel form is `make matek_h743_dummy -jN`; the baseline attempt used `-j2`. The target name is not `matek_h743_default`: the enabled custom modules are in `dummy.px4board`, and the build cache's `PX4_CONFIG` is `matek_h743_dummy`.

### 2.3 Baseline build result

Command executed, with no source edits:

```sh
make matek_h743_dummy -j2
```

Result: **FAIL (exit code 2), 395/1039**, outside CADRC.

Primary fatal error:

```text
platforms/common/spi.cpp:90:59: error: the address of 'px4_spi_buses'
will never be NULL [-Werror=address]
cc1plus: all warnings being treated as errors
```

Environment:

- `arm-none-eabi-g++ 13.2.1 20231009`
- Python `3.12.3`
- Configuration also printed a non-fatal-at-that-point SymForce circular-import traceback: `ImportError: cannot import name 'lambdify' from partially initialized module 'symforce.util'`.

This failure does not establish a CADRC compile defect. CADRC message generation completed during the attempt, but compilation stopped before the CADRC module objects were rebuilt. No workaround was applied because changing warnings, tool versions, generated files, or source was outside this audit.

Pre-existing artifacts:

| Artifact | Timestamp | SHA-256 | Qualification |
|---|---:|---|---|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 2026-09-07 12:39:40 +0800 | `72b579599157e6ab8a35366d83cf0dfa8678f687dff761b5be427fede92b4ef2` | Contains CADRC strings/parameters, but predates current `HydroRateControl.cpp` mtime |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 2026-09-07 12:39:40 +0800 | `0c8205ebae2489b2a1595413dd8167c3185df19cfb83820a1c8e0a726475aaf9` | Same limitation |

Therefore the baseline status for the **exact audited source** is “rebuild attempted, environment/toolchain failure”; it is not inferred from the existence of stale artifacts.

### 2.4 CADRC and directly related files

Direct implementation/configuration:

- `src/modules/hydro_rate_control/CADRC.cpp` — actual observer, adaptive term, controller, internal clamps.
- `src/modules/hydro_rate_control/CADRC.h` — class interface, state layout, constructor defaults.
- `src/modules/hydro_rate_control/HydroRateControl.cpp` — input acquisition, mode gate, reset, reference mapping, CADRC invocation, scaling/limiting, publication.
- `src/modules/hydro_rate_control/HydroRateControl.hpp` — subscriptions, publications, two CADRC instances, parameters.
- `src/modules/hydro_rate_control/hydro_rate_control_params.c` — CADRC/reference parameter definitions and defaults.
- `src/modules/hydro_rate_control/CMakeLists.txt` — compiles `CADRC.cpp` and `CADRC.h` into `modules__hydro_rate_control`.
- `src/modules/hydro_rate_control/Kconfig` — module enable switch.
- `msg/HydroCadrcControlMessage.msg` — current CADRC diagnostic uORB schema.
- `msg/CMakeLists.txt` — includes that message in generation.
- `src/modules/logger/logged_topics.cpp` — default logging of `hydro_cadrc_control_message` at interval 0.
- `boards/matek/h743/dummy.px4board` — enables the module in the represented hardware target.
- `ROMFS/px4fmu_common/init.d/rc.fw_apps` — starts `hydro_rate_control` and downstream allocator.

Files required to understand the complete CADRC output path, but not part of the CADRC algorithm itself:

- `src/modules/hydro_allocator/HydroAllocator.cpp`, `.hpp`, `module.yaml`, `CMakeLists.txt`, `Kconfig`.
- `src/modules/control_allocator/ControlAllocator.cpp`, `.hpp`, `module.yaml`.
- `src/modules/control_allocator/ControlAllocation/ControlAllocation.cpp`, `.hpp`.
- `src/lib/mixer_module/output_functions.yaml`.
- `src/lib/mixer_module/functions/FunctionMotors.hpp`, `FunctionServos.hpp`.
- `msg/VehicleTorqueSetpoint.msg`, `VehicleThrustSetpoint.msg`, `ActuatorMotors.msg`, `ActuatorServos.msg`, `HydroAllocateMessage.msg`.

## 3. Control Call Graph

### 3.1 Full roll path

| Step | File and function | Key variable/topic | C/C++ type | Unit/range | Trigger/rate |
|---:|---|---|---|---|---|
| 1 | gyro driver, board-selected implementation | `sensor_gyro` or `sensor_gyro_fifo` | uORB message, raw fields converted to `float` | driver output is scaled to rad/s before/within sensor pipeline | hardware dependent |
| 2 | `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`; `Run()`, `FilterAngularVelocity()`, `CalibrateAndPublish()` | `_angular_velocity`, `vehicle_angular_velocity.xyz` | `matrix::Vector3f`, `float32[3]` | rad/s, FRD body XYZ | callback driven; `IMU_GYRO_RATEMAX=400` caps nominal publish at 400 Hz |
| 3 | `src/modules/ekf2/EKF2.cpp`; `EKF2::PublishAttitude()` | `vehicle_attitude.q` | `float32[4]` Hamilton quaternion | FRD body to NED earth | estimator driven; represented historical `EKF2_PREDICT_US=10000` gives 100 Hz |
| 4 | `src/modules/hydro_rate_control/HydroRateControl.cpp`; `Run()` | `Eulerf euler_angles`, `roll = euler_angles.phi()` | `Eulerf`, `float` | rad; positive right-hand rotation about body +X | gyro callback loop, but held at last attitude update |
| 5 | same | `rates`, `p = rates(0)` | `Vector3f`, `float` | rad/s; body +X roll rate | fresh `vehicle_angular_velocity` loop |
| 6 | RC path: `rc_update.cpp`; `Run()`, `UpdateManualControlInput()`; selector: `ManualControl*` | `manual_control_setpoint.roll` | `float32` | normalized `[-1,1]`; + means stick right/right side down | source publication rate; not synchronized with gyro |
| 7 | `HydroRateControl::Run()` | `roll_sp = manual.roll * radians(HY_ROLL_MAX)` | `float` | rad; default maximum ±30 deg | recomputed every hydro rate iteration; currently zeroed on iterations with no new manual message |
| 8 | same | `dt`, `dt_used` | `float` | seconds | per gyro iteration; see Section 6 |
| 9 | `CADRC.cpp`; `ADRCController::update()` | `_Xe1`, `_Z1`, `_Wx_p`, `_Xe2`, `_Z2`, `_Uc` | scalar `float`; `Matrix<float,2,1>` | reconstructed in Section 4 | every STAB iteration with CADRC enabled |
| 10 | `HydroRateControl::Run()` | raw `_cadrc_roll.getUc()`; scaled prelimit `Uc*u_scale`; `u_roll` | `float` | raw controller units, then normalized command | same loop |
| 11 | same | `constrain(...,-1,1)`, then add `TRIM_ROLL`, constrain again | `float u_roll` | normalized `[-1,1]` | same loop; magnitude only, no rate limit |
| 12 | same | `_hydro_torque_setpoint.xyz[0]`; topic `hydro_torque_setpoint` | `float32[3]` | normalized torque command | published each active hydro rate iteration |
| 13 | `src/modules/hydro_allocator/HydroAllocator.cpp`; `Run()` | `torque_vector`, `allocate_matrix`, `delta_force_vector` | `Vector3f`, `Matrix<float,4,3>`, `Vector4f` | normalized input multiplied by configured N/normalized ratios -> N-like virtual force increments | callback on hydro torque/thrust; 100 ms backup |
| 14 | same; `optim()` | `thrust_x/z`, `x[side][angle,thrust]` | `float` arrays | force N by implementation comment; foil angle rad; thruster force N | each allocator run |
| 15 | same | `hydro_servos.control[HY_SVn_IDX-1]`, `hydro_motors.control[HY_RTn_IDX-1]` | `float32[]` | servos `[-1,1]`; motors `[0,1]` here | each allocator run when foil enable/mode gate passes |
| 16 | `src/modules/control_allocator/ControlAllocator.cpp`; `publish_actuator_controls()` | existing CA setpoint `+ hydro_servos/motors`, then `constrain` | `float` | final normalized `[-1,1]`; small motor commands ±0.02 become NaN | control allocator update rate |
| 17 | mixer output providers and PWM driver | `actuator_servos`, `actuator_motors` -> `PWM_MAIN_FUNCn` | uORB + hardware output | normalized -> configured PWM microseconds | hardware/output scheduling; historical old ULog showed ~1.25 ms PWM task interval, not a current guarantee |

### 3.2 Important concurrency/data-age behavior

- `HydroRateControl` runs on `vehicle_angular_velocity` callbacks and has a `ScheduleDelayed(20_ms)` backup. Its nominal active rate is therefore sensor-driven, not a fixed periodic timer. The backup provides approximately 50 Hz wakeups during missing gyro notifications, but control still depends on message-copy behavior.
- `vehicle_attitude` is read with `update()` but retained when no new message arrives. Thus CADRC can run several times with the same `phi` and new `p`.
- Manual input behaves differently: when `update()` returns false, `_manual_control_setpoint` is explicitly cleared. The last valid reference is **not** held.
- `HydroAllocator` registers both hydro torque and thrust callbacks, and schedules a 100 ms backup. It uses the most recently retained values of both topics.
- The standard fixed-wing controller and standard PX4 control allocator are also started. `ControlAllocator::publish_actuator_controls()` adds the hydro actuator arrays to the already allocated standard actuator setpoints. Final physical commands can therefore be a sum, not CADRC-only.

## 4. Current CADRC Reconstruction

### 4.1 Symbols and state

For roll at discrete step (k):

- (\phi): `P_current` / `roll`, measured Euler roll angle [rad].
- (\phi_r): `P_target` / `roll_sp`, reference [rad].
- (p): `Wx_meas`, measured body roll rate [rad/s].
- (e_1=\phi_r-\phi): `_Xe1`.
- (Z_1=[z_{11},z_{12}]^T): `_Z1`, outer/position-error ESO state.
- (p_r): `_Wx_p`, generated desired roll rate.
- (e_2=p_r-p): `_Xe2`.
- (Z_2=[z_{21},z_{22}]^T): `_Z2`, inner/rate-error ESO state.
- (U_c): `_Uc`, raw controller output before scaling, trim, or downstream limiting.
- (h): `tau`, the internally clamped integration step.

Both `Z1` and `Z2` are observer states. From the equations, `z11` estimates the outer error state and `z12` estimates the unmodelled/additive term in its derivative (for a moving reference this also absorbs missing reference dynamics). `z21` estimates the rate error and `z22` estimates its additive derivative term. The source never formally declares `z12` or `z22` to be a physical torque disturbance, so interpreting either as a uniquely signed physical disturbance is not justified. In particular, a plant-side disturbance estimate would require a sign and gain convention not documented in code.

### 4.2 Observer gains

For each two-state observer:

\[
L_1=\begin{bmatrix}\beta_{11}\\\beta_{12}\end{bmatrix}
=\begin{bmatrix}2\omega_{o1}\\\omega_{o1}^2\end{bmatrix},\qquad
L_2=\begin{bmatrix}\beta_{21}\\\beta_{22}\end{bmatrix}
=\begin{bmatrix}2\omega_{o2}\\\omega_{o2}^2\end{bmatrix}.
\]

At defaults `omega_o1=omega_o2=15`, both are `[30, 225]^T`.

### 4.3 Outer observer and reference-rate generation

The matrix code at `CADRC.cpp:176-189` is exactly:

\[
\dot Z_1=(A_1-L_1C_1)Z_1+B_1p_k+L_1e_{1,k},
\]

with

\[
A_1=\begin{bmatrix}0&1\\0&0\end{bmatrix},\quad
B_1=\begin{bmatrix}-1\\0\end{bmatrix},\quad
C_1=\begin{bmatrix}1&0\end{bmatrix}.
\]

Expanded for all but the first update after reset:

\[
\dot z_{11}=z_{12}-p_k+\beta_{11}(e_{1,k}-z_{11}),
\]
\[
\dot z_{12}=\beta_{12}(e_{1,k}-z_{11}).
\]

On the first update, the `-p_k` term is deliberately replaced by zero (`_nn > 1 ? _Wx_meas : 0`). Forward Euler is then used:

\[
Z_{1,k+1}=Z_{1,k}+h\dot Z_{1,k}.
\]

With default `alpha=0`, desired rate is:

\[
p_r=k_{p1}z_{11}+z_{12}.
\]

There is no independent reference filter, tracking differentiator, `p_r` clamp, or `\dot p_r`/`\ddot p_r` calculation.

### 4.4 Adaptive/CADRC term

The history length is `m=4`. Let

\[
W=[p_k,p_{k-1},p_{k-2},p_{k-3}],
\]

and

\[
Q=\mathrm{diag}\left(\frac{\ln1}{\ln24},\frac{\ln2}{\ln24},
\frac{\ln3}{\ln24},\frac{\ln4}{\ln24}\right)
\approx\mathrm{diag}(0,0.2181,0.3457,0.4362).
\]

After more than four samples and only if `abs(alpha)>1e-6`:

\[
\Delta=-hWQ,\quad A=\Delta^T\Delta,
\]

and the custom `geninv_c()` computes a damped generalized inverse using Cholesky with damping (10^{-6}). Then:

\[
\Gamma=-\alpha A^{+}_{\lambda}\Delta^Te_1.
\]

The desired rate becomes:

\[
p_r=k_{p1}(z_{11}+\Delta\Gamma)+z_{12}+W\Gamma.
\]

If Cholesky fails, `A_inv`, `Gamma`, and `Delta` are zeroed, disabling the adaptive correction for that iteration. With the shipped default `alpha=0`, this entire adaptive path is inactive and `p_r=k_{p1}z11+z12`.

### 4.5 Inner observer and controller

The code sets (e_2=p_r-p). At `CADRC.cpp:225-254`:

\[
\dot Z_2=(A_2-L_2C_2)Z_2+B_2U_{c,k-1}+L_2e_{2,k}+D_2\dot Z_{1,k},
\]

where

\[
A_2=\begin{bmatrix}0&1\\0&0\end{bmatrix},\quad
B_2=\begin{bmatrix}-J_0\\0\end{bmatrix},\quad
C_2=\begin{bmatrix}1&0\end{bmatrix},\quad
D_2=\begin{bmatrix}k_{p1}&1\\0&0\end{bmatrix}.
\]

Expanded after the first update:

\[
\dot z_{21}=z_{22}-J_0U_{c,k-1}+\beta_{21}(e_{2,k}-z_{21})
+k_{p1}\dot z_{11}+\dot z_{12},
\]
\[
\dot z_{22}=\beta_{22}(e_{2,k}-z_{21}).
\]

The first update replaces (U_{c,k-1}) with zero. Forward Euler is used:

\[
Z_{2,k+1}=Z_{2,k}+h\dot Z_{2,k}.
\]

Immediately afterward only `z22` is clamped:

\[
z_{22}\in[-50,50].
\]

The raw output is:

\[
U_c=\frac{k_{p2}z_{21}+z_{22}}{J_0}.
\]

This is the actual disturbance-compensation sign: `z12` is **added** to the rate reference and `z22` is **added** in the raw control numerator. There is no integral term other than observer/adaptive state evolution and no derivative feedback term outside the observer equations.

Although the parameter comment calls `J0` “inertia-like”, executable code uses `J0` as the input coefficient in `-J0*Uc` and divides the feedback/compensation numerator by it. It therefore behaves mathematically as an assumed control effectiveness/input gain, not unambiguously as physical inertia. This is a comment/interpretation mismatch worth preserving during integration.

### 4.6 Initialization, reset, and finite handling

The constructor initializes weights/gains and calls `reset()`. Reset zeroes:

- `Z1`, `Z2`, the four-rate history, adaptive matrices/vectors;
- sample counter `_nn`;
- `Xe1`, `Xe2`, measured-rate cache, desired rate, raw output;
- saved `Z1_dot`.

Reset is also called on the edge into STAB. It does **not** initialize observer errors from current `phi_r-phi`, current `p`, or the previous actuator command.

After computing `Uc`, if either `Uc` or `Wx_p` is non-finite, the full controller is reset and both outputs are explicitly set to zero. There are no explicit finite checks on inputs before the matrix updates; non-finite state generally propagates until this final check. There is no `Uc` magnitude limit inside `ADRCController`.

### 4.7 Parameter table

| Parameter | Default | Metadata constraint | Actual role |
|---|---:|---|---|
| `HY_CADRC_EN` | 1 | boolean | Enables CADRC only in STAB |
| `HY_CADRC_J0` | 10.1746 | min 0.001 | assumed inner-loop input coefficient; setter clamps to at least 0.001 |
| `HY_CADRC_KP1` | 500.0 | min 0 | outer error-to-rate gain |
| `HY_CADRC_KP2` | 1000.0 | min 0 | inner error-to-raw-control gain |
| `HY_CADRC_O1` | 15.0 | min 0 | outer ESO bandwidth; generates beta 30/225 |
| `HY_CADRC_O2` | 15.0 | min 0 | inner ESO bandwidth; generates beta 30/225 |
| `HY_CADRC_ALP` | 0.0 | min 0 | adaptive correction gain; zero disables adaptive branch |
| `HY_CADRC_TAU` | 0.0025 s | [0, 0.02] | positive value replaces measured loop `dt`; zero selects measured `dt` |
| `HY_CADRC_U_SCALE` | 0.001 | min 1e-6 | converts raw `Uc` to normalized torque-like command |
| `HY_ROLL_MAX` | 30 deg | [0,80] deg | stick-to-roll-reference scale |
| `TRIM_ROLL` | 0.0 | global PX4 parameter | added after first CADRC magnitude clamp |
| `HY_RLL_TO_YAW_FF` | 0.0 | min 0 | sends final roll torque command into yaw; does not alter roll itself |

No post-CADRC ULog/parameter snapshot exists in the repository, so current loaded real-vehicle values cannot be established. The values above are source defaults and generated-build defaults, not claimed live values.

## 5. Signal/Unit Table

| Signal | Source / variable | Type | Confirmed unit/range | Frame/sign | Filtering/qualification |
|---|---|---|---|---|---|
| raw gyro | `sensor_gyro[_fifo]` | uORB fields | converted to rad/s | sensor frame initially | driver/FIFO dependent |
| roll rate (p) | `rates(0)`, `vehicle_angular_velocity.xyz[0]` | `float` | rad/s | FRD body +X; positive right-hand roll, right side down | calibration/rotation, scale, estimated bias removal, notch filters, gyro low-pass; represented cutoff 40 Hz |
| attitude quaternion | `_vehicle_attitude.q` | `float[4]` | unit quaternion | FRD body -> NED earth, Hamilton | EKF-estimated; no added filtering in HydroRateControl |
| roll angle (\phi) | `roll = Eulerf(...).phi()` | `float` | rad | same positive body-X roll convention | last available attitude sample; age unchecked |
| manual roll | `_manual_control_setpoint.roll` | `float` | normalized `[-1,1]` | positive stick right / positive roll | RC calibration/dead-zone if RC; direct `/1000` mapping if MAVLink; no HydroRateControl filter |
| roll reference (\phi_r) | `roll_sp` | `float` | rad, default maximum ±0.523599 | positive follows manual roll | linear scale only; current code clears it to zero without a new manual message |
| outer error | `_Xe1` | `float` | rad | `phi_r - phi` | none |
| `z11` | `_Z1(0,0)` | `float` | error-state dimension, nominal rad | outer ESO | forward-Euler state |
| `z12` | `_Z1(1,0)` | `float` | error-derivative/additive-term dimension, nominal rad/s | additive in `p_r` | not a separately calibrated physical disturbance |
| desired rate | `_Wx_p` | `float` | nominal rad/s by equation | body roll | no clamp/filter/TD |
| inner error | `_Xe2` | `float` | rad/s | `p_r-p` | none |
| `z21` | `_Z2(0,0)` | `float` | rate-error dimension, nominal rad/s | inner ESO | no magnitude clamp |
| `z22` | `_Z2(1,0)` | `float` | rate-error derivative/additive-term dimension, nominal rad/s² | additive in control numerator | hard clamp `[-50,50]` |
| raw controller output | `_Uc`, `getUc()` | `float` | **not physically established**; unnormalized controller units | sign follows equations | finite check only |
| scaled prelimit output | `getUc()*u_scale`; logged `u_roll` | `float` | normalized-command intent, not yet legal-range guaranteed | positive roll command | logged before both magnitude clamps and trim |
| first limited output | local `u_roll` after line 279 | `float` | normalized `[-1,1]` | roll torque axis | magnitude clip |
| actual published roll torque | `_hydro_torque_setpoint.xyz[0]` | `float32` | normalized `[-1,1]` | body roll axis | trim added, second magnitude clip; no slew limit |
| virtual force increments | `delta_force_vector` | `Vector4f` | N-like, due allocator parameter convention | logical `[x0,z0,x1,z1]` | allocation ratios, then negative-X redistribution |
| optimized foil angles | `x[0][0]`, `x[1][0]` | `float` | rad before normalization | logical side 0/1 | constrained to `±HY_WING_MAX_A` |
| hydro servo output | `hydro_servos.control[]` | `float32` | normalized `[-1,1]` | parameter-indexed | foil enable/mode gate; no slew limit |
| final servo setpoint | `actuator_servos.control[]` | `float32` | normalized `[-1,1]` | output-function indexed | standard CA output plus hydro, then final clamp |
| PWM output | `actuator_outputs.output[]` | `float32` | normally microseconds for PWM | board/output channel | configured min/max/reverse/disarmed mapping |

The `VehicleThrustSetpoint.msg` schema says `hydro_thrust_setpoint.xyz` is `[-1,1]`, but `HydroRateControl.cpp:431-434,500-501` explicitly computes and publishes N-valued hydro thrust. This schema/implementation unit mismatch is downstream of roll CADRC but materially affects allocation and should not be copied into a new controller interface.

## 6. Timing and Discretization

### 6.1 Actual control-loop timing

`HydroRateControl` registers a callback on `vehicle_angular_velocity`; it is not a fixed-frequency timer. Source and represented parameters imply:

- requested/capped gyro control publication: `IMU_GYRO_RATEMAX=400`, nominal 400 Hz or 2.5 ms;
- backup schedule: 20 ms (50 Hz) if callbacks do not arrive;
- exact in-flight rate depends on gyro FIFO, sensor driver, uORB scheduling, and work-queue load and cannot be proven from static source alone.

An older, pre-CADRC ULog confirms this hardware family and shows the controller stack running, but it cannot establish the current CADRC runtime frequency. It is used only as historical corroboration.

### 6.2 `dt` acquisition and protection

In `HydroRateControl::Run()`:

1. `dt` is the difference between successive `vehicle_angular_velocity.timestamp_sample` values.
2. It is immediately clamped to `[0.002, 0.04]` seconds.
3. A wall-clock fallback exists if `dt < DT_MIN || dt > DT_MAX`, but because a copied timestamp delta was already clamped inclusively, this branch normally executes only when no angular-velocity message was copied (`dt` remains zero).
4. On the first copied sample, `_last_run` starts at zero, so the measured delta clamps to 0.04 s.
5. `dt_used = HY_CADRC_TAU` whenever that parameter is positive; otherwise it is the measured/clamped `dt`.
6. Inside `ADRCController::update()`, `dt_used` is independently clamped to `[0.0005,0.02]` and called `tau`.

With default `HY_CADRC_TAU=0.0025`, the observer/controller discretization uses a fixed 2.5 ms even if the callback period jitters. With `HY_CADRC_TAU=0`, it becomes variable step, based on gyro sample timestamps but limited first to 2–40 ms and then to 0.5–20 ms; the effective range is therefore 2–20 ms for copied gyro samples.

### 6.3 Discrete method

Both ESOs use explicit/forward Euler:

```text
Z_next = Z + tau * Z_dot
```

The adaptive history uses a hand-written discrete difference-like term `Delta = -tau * Wx_history * Q`; its damped inverse is recomputed algebraically each iteration. There is no Tustin/bilinear transform, zero-order-hold exact discretization, or precomputed discrete state transition matrix.

## 7. Reference Input Path

### 7.1 Source selection

The immediate source is `manual_control_setpoint`, not an attitude-setpoint topic. It is selected by `ManualControl` from `manual_control_input` instances.

- RC route: `input_rc` -> `RCUpdate` -> `manual_control_input` -> `ManualControlSelector` -> `manual_control_setpoint`.
- Joystick route: MAVLink `MANUAL_CONTROL` -> `MavlinkReceiver::handle_message_manual_control()` -> `manual_control_input` -> selector -> `manual_control_setpoint`.
- `HydroRateControl` does not require `data_source == SOURCE_RC`. Therefore `phi_r` is **not necessarily RC-derived**.
- Default `COM_RC_IN_MODE=3` means first valid RC or joystick source is retained until reboot.

### 7.2 RC mapping and dead zone

For a physical RC source, `RCUpdate` maps `RC_MAP_ROLL` to a calibrated input channel. It applies a piecewise-linear mapping:

```text
[RC_MIN, RC_TRIM-RC_DZ, RC_TRIM+RC_DZ, RC_MAX]
                         -> [-1, 0, 0, 1]
```

and then applies `RCn_REV`. `get_rc_value()` finally constrains the result to `[-1,1]`. Historical robot parameters map roll to RC channel 1, with `RC1_MIN=1001`, `TRIM=1500`, `MAX=1991`, `DZ=10`, `REV=+1`. These values predate CADRC and are not asserted as live values.

MAVLink joystick roll is `MANUAL_CONTROL.y / 1000` and does not pass through RC calibration/dead-zone code.

### 7.3 Angle mapping and available derivatives

`HydroRateControl` performs only:

\[
\phi_r=\mathrm{manual.roll}\times\mathrm{radians}(HY\_ROLL\_MAX).
\]

There is no additional dead zone, expo, scale curve, low-pass filter, tracking differentiator, or slew-rate limit in this module. With no accepted manual setpoint it intends to use zero.

However, the implementation treats “no **new** uORB update this iteration” as “no/invalid manual input” and clears the stored message. Thus reference hold is absent and the effective reference can pulse at manual-message rate inside a faster gyro loop.

Availability:

- `phi_r`: available as local `roll_sp` and in `hydro_cadrc_control_message.roll_sp` while CADRC runs.
- `dot(phi_r)`: not computed or published.
- `ddot(phi_r)`: not computed or published.
- `_Wx_p` is a controller-generated desired rate, not a measured or differentiated `dot(phi_r)` and is currently not published (the publication code is commented out).
- `z12` must not be relabelled as `dot(phi_r)`; it is an observer state for the outer error dynamics.

## 8. Actuator Output Path

### 8.1 CADRC output and limit stages

For roll, the variables are:

| Stage | Expression / variable | Legal range | Logged now? |
|---|---|---:|---|
| Raw controller | `_cadrc_roll.getUc()` / `_Uc` | unbounded except finite reset | no |
| Scaled, before limiter | `_cadrc_roll.getUc() * HY_CADRC_U_SCALE` | unbounded | yes, misleadingly named `hydro_cadrc_control_message.u_roll` |
| First magnitude-limited | local `u_roll = constrain(scaled,-1,1)` | `[-1,1]` | no |
| Trimmed/final hydro command | `u_roll = constrain(u_roll + TRIM_ROLL,-1,1)` | `[-1,1]` | indirectly via logged `hydro_torque_setpoint.xyz[0]` |
| Published | `_hydro_torque_setpoint.xyz[0]` | `[-1,1]` | yes |

No roll magnitude limit smaller than 1 is present. No roll rate/slew limiter is present in `HydroRateControl` or `HydroAllocator`.

The standard `ControlAllocation::applySlewRateLimit()` can limit its own `_actuator_sp` according to `CA_Rn_SLEW`/`CA_SVn_SLEW`, but `ControlAllocator::publish_actuator_controls()` adds `_hydro_motors` and `_hydro_servos` **after** allocation, after that slew limiter, and after the standard allocator clip. It then only performs a final `[-1,1]` magnitude clip. Consequently even nonzero CA slew parameters would not slew-limit the custom hydro addition. Historical snapshots also show all relevant CA slew parameters set to zero.

### 8.2 Hydro allocation

`HydroAllocator::Run()` constructs the logical actuator ordering `[x0,z0,x1,z1]` and moment ordering `[roll,pitch,yaw]`. For roll command (u_\phi):

\[
\Delta F = M\begin{bmatrix}u_\phi&u_\theta&u_\psi\end{bmatrix}^T,
\]

where each row of (M) comes from `HY_VXRTn_ROL_R` or `HY_VZRTn_ROL_R` and the pitch/yaw equivalents. These ratios convert normalized moment command to N-like force increments by the module's stated convention.

The resulting side forces enter a Newton solve for foil angle and thruster force. Safety/constraint stages include:

- redistribution preventing negative total/individual `thrust_x`;
- Newton iterate angle clamp `±HY_WING_MAX_A`;
- thruster clamp `[0, x1_max]`, where `x1_max` depends on throttle, `HY_TH_MAX_GAIN`, and `HY_RT_MAX_THRUST`;
- final foil normalization and clip `[-1,1]`;
- final thruster normalization and clip `[0,1]`;
- foil enable AUX threshold, vehicle-mode gate, and `HY_ALLOCATE_MODE` selection.

Source defaults set all roll allocation ratios to zero and `HY_FOIL_AUX=0` with threshold 0, so a factory-default configuration does not establish a working roll-to-foil path. A historical real-robot snapshot uses:

```text
HY_VXRT0_ROL_R = 0       HY_VXRT1_ROL_R = 0
HY_VZRT0_ROL_R = -10     HY_VZRT1_ROL_R = +10
HY_SV0_IDX = 5           HY_SV1_IDX = 4
HY_RT0_IDX = 1           HY_RT1_IDX = 2
HY_WING_MAX_A = 1.134464 rad
HY_RT_MAX_THRUST = 30 N
```

Those historical ratios make positive roll demand create negative logical-side-0 Z increment and positive logical-side-1 Z increment. This is a software allocation fact for that snapshot, not proof of current physical polarity.

### 8.3 Final channels and physical relationship

Code comments label logical side 0 as left and side 1 as right. In the historical snapshot:

| Logical label | Hydro mapping | PX4 actuator function | PWM mapping | PWM endpoints/reversal |
|---|---|---|---|---|
| side 0, code-comment “left” | `HY_SV0_IDX=5` -> `hydro_servos.control[4]` | Servo 5 (function 205) | PWM MAIN12 | 800–2200 us; MAIN12 reversed by historical `PWM_MAIN_REV=6273` |
| side 1, code-comment “right” | `HY_SV1_IDX=4` -> `hydro_servos.control[3]` | Servo 4 (function 204) | PWM MAIN11 | 800–2200 us; not reversed in that snapshot |

The source and repository artifacts do not prove wiring, linkage orientation, or which physical foil is actually attached to MAIN11/MAIN12. **无法从源码确认物理左右关系。** It must be verified against wiring documentation or a safe disarmed actuator test before any new controller is enabled.

The final servo value is also not necessarily equal to the hydro value: standard control-allocation output is added first, then the hydro value is added, then the sum is clamped. Any standard control surface mapped to the same Servo function can interact with CADRC.

### 8.4 Observer control-input proof

The exact call chain is:

```text
ADRCController::_Uc[k-1]
  -> CADRC.cpp:240, Bp2 * _Uc[k-1]
  -> update Z2
  -> CADRC.cpp:254 computes new raw _Uc[k]
  -> HydroRateControl.cpp:279 multiplies by HY_CADRC_U_SCALE and clamps
  -> HydroRateControl.cpp:291 adds TRIM_ROLL and clamps
  -> hydro_torque_setpoint.xyz[0]
  -> HydroAllocator -> hydro_servos/motors
  -> ControlAllocator addition/final clamp
  -> actuator output
```

There is no return path from any post-limiter variable to `ADRCController`. Therefore observer choice is unequivocally option **1: previous unbounded controller command**, not option 2. It is additionally in raw `Uc` units, whereas the downlink receives `Uc * U_SCALE` plus trim. Saturation, foil disable, allocator mode, standard-output addition, PWM reversal, and actuator failure are invisible to the observer.

### 8.5 Manual/automatic switching

| Transition/event | Current behavior |
|---|---|
| any non-STAB -> STAB | `entered_stab` edge resets both roll and pitch CADRC states to zero |
| STAB CADRC active | CADRC updates and overrides hydro roll/pitch torque commands |
| STAB with `HY_CADRC_EN=0` | falls back to existing rate-controller branch |
| STAB -> MANUAL/other | CADRC states are not reset immediately; they stop updating |
| later re-entry to STAB | reset occurs because nav-state edge is detected |
| `HY_CADRC_EN` 1 -> 0 while remaining STAB | state freezes; no reset |
| `HY_CADRC_EN` 0 -> 1 while remaining STAB | stale state resumes; no reset |
| parameter gain change | gains change in place; no observer/controller reset |
| previous actuator initialization | none |
| bumpless transfer | none |

The zero reset prevents old state from surviving a normal mode re-entry, but does not make the transfer bumpless. First-update gyro/control terms are suppressed, while nonzero angle error immediately starts integrating from zero.

## 9. Logging Capability

PX4 `logger`/ULog is the active mechanism. `src/modules/logger/logged_topics.cpp` adds the custom hydro topics by default. Existing useful topics include:

| Quantity | Existing topic/field | Status |
|---|---|---|
| `phi` | `vehicle_attitude.q` | logged; requires quaternion-to-Euler conversion offline |
| `p` | `vehicle_angular_velocity.xyz[0]` | logged, default logger interval 20 ms in normal topic list |
| manual source/stick | `manual_control_setpoint.roll`, `.data_source`, `.valid` | logged, default interval 200 ms; too slow to diagnose the reference-clearing behavior precisely |
| `phi_r` | `hydro_cadrc_control_message.roll_sp` | custom topic configured interval 0 |
| scaled output before limiter | `hydro_cadrc_control_message.u_roll` | available; despite name, it is pre-clamp and pre-trim |
| actual limited hydro `u_phi` | `hydro_torque_setpoint.xyz[0]` | available, though custom topic currently uses default logger scheduling rather than explicit 0 interval |
| observer states | `roll_z11`, `roll_z12`, `roll_z21`, `roll_z22` | available |
| disturbance-related estimates | `z12`, `z22` | available, but semantics/sign must follow Section 4 rather than generic “disturbance” labels |
| allocation intermediates | `hydro_allocate_message.thrust_xz_control`, `.thrust_xz_limited`, `.actuator_optimed` | available |
| hydro actuator arrays | `hydro_servos`, `hydro_motors` | published but not explicitly in the default logger list |
| combined actuator command | `actuator_servos`, `actuator_motors` | logged at 100 ms default interval |
| physical PWM | `actuator_outputs` multi-topic | optional logging at 100 ms |

Current logging defects/gaps:

- `HydroRateControl` never assigns `_hydro_cadrc_control_message.timestamp`; the zero-initialized timestamp remains zero on every publication.
- No `timestamp_sample` exists in the message schema.
- `phi`, `p`, raw `Uc`, `_Wx_p`, `Xe1`, `Xe2`, adaptive correction, selected controller, reset event, and saturation flags are absent.
- `u_roll` is not the final actuator command; it is `raw Uc * U_SCALE` before both clamps and trim.
- Default downsampling of standard attitude/rate/actuator topics prevents exact sample-by-sample correlation with a nominal 400 Hz CADRC topic.

For the next stage, the least intrusive diagnostic publication point is inside `HydroRateControl::Run()`, after controller selection and after the common limiter, immediately before assignment to `_hydro_torque_setpoint.xyz[0]`. One controller-agnostic diagnostic message should carry at least:

```text
timestamp, timestamp_sample, controller_id, reset_counter
phi, p, phi_r, phi_r_dot(if a controller actually supplies it)
raw_controller_output, scaled_prelimit_output, limited_u_phi
saturation flags, dt_measured, dt_used
controller-specific state array / named CADRC z11,z12,z21,z22
estimated_disturbance with explicit sign convention
```

The downstream actuals should be correlated from `hydro_allocate_message`, `hydro_servos`, `hydro_motors`, `actuator_servos`, `actuator_motors`, and `actuator_outputs`. Logging `hydro_servos/hydro_motors` explicitly at a useful interval would expose the otherwise hidden point before standard CA addition.

## 10. Minimal-Change Multi-Controller Integration Plan

This is an interface/integration plan only; it does not design LADRC, AMCEO-ADRC, or the fourth ADRC algorithm.

### 10.1 Preserve the CADRC baseline

1. Freeze the existing `ADRCController` implementation and its exact call branch as the reference. Add regression/golden-vector tests before touching selection logic. Do not rename states, alter equations, change first-sample suppression, change `tau`, or “correct” the observer input as part of initial multi-controller integration.
2. Add one integer selector parameter whose legacy/default value selects existing CADRC. Keep `HY_CADRC_EN` behavior compatible: either retain it as the top-level enable gate or define selector migration so existing stored parameters still activate CADRC identically.
3. Instantiate four independent controller objects in `HydroRateControl`, one for each algorithm. Do not share observer/history/integrator storage.
4. Define a small common input value object populated from already confirmed signals: `phi [rad]`, `p [rad/s]`, `phi_r [rad]`, measured `dt [s]`, selected `dt [s]`, and explicit previous-command fields. Do not silently supply derivatives or actuator feedback that do not exist.
5. Define a small common result: raw roll command plus validity and optional diagnostic fields. Preserve CADRC's raw-unit scaling exactly in its branch; new algorithms must explicitly declare whether their output is raw or already normalized.
6. Place the selector only around controller update/output production. Keep the existing CADRC post-processing order unchanged: `U_SCALE`, first `[-1,1]` clamp, `TRIM_ROLL`, second clamp, `hydro_torque_setpoint`, and existing downstream allocation.

### 10.2 Switching/reset policy

- Maintain an `active_controller` and `previous_controller`.
- On selector change, reset the controller being entered according to that controller's own reset routine. Do not reset or mutate inactive controllers opportunistically.
- Initialize only through an explicit per-controller API using current `phi`, `p`, `phi_r`, and the actual common limited command if the algorithm specification requires it. For existing CADRC, preserve the current all-zero reset unless a separately reviewed behavior change is authorized.
- Do not allow inactive-controller states to update in the background unless explicitly required and flight-tested; background observers can accumulate with the wrong control input.
- Add a switching validity gate: non-finite output falls back to a documented safe/legacy behavior and records a fault. A selector change while armed should initially be prohibited or require a separately reviewed bumpless-transfer implementation.
- Preserve the existing nav-state edge reset and add selector/enable edges explicitly, avoiding stale-state resumption.

### 10.3 Common limiter and actuator feedback

Use one shared, explicit output pipeline for all four selected commands, but initially reproduce existing CADRC arithmetic exactly. If a future rate limiter is authorized, insert it once after normalization/scaling and before publication to `hydro_torque_setpoint`, initialize it from the last actual limited hydro command, and verify that it also covers trim behavior.

Expose both:

- `u_requested`: selected controller request before common limiting;
- `u_applied`: final common limited hydro roll command actually published.

Controllers that require applied-command observer feedback should receive `u_applied` delayed by one controller sample and with a documented normalization/gain convention. **Do not change existing CADRC from raw `_Uc[k-1]` feedback during the first coexistence stage**, because that would change the baseline. Treat a future anti-windup/observer-input correction as a separate flight-control change with independent tests.

### 10.4 Scope boundary

No change to `HydroAllocator` or standard `ControlAllocator` is necessary to add a roll-controller selector because all algorithms should converge on the existing normalized `hydro_torque_setpoint.xyz[0]` contract. Modify downstream modules only if a separately approved actuator-feedback or rate-limiting feature truly requires it. Avoid introducing a new parallel allocation route per controller.

## 11. Risk List

Priority is ordered roughly by immediate real-vehicle hazard.

| Priority | Risk | Current evidence / consequence | Required next-stage control |
|---:|---|---|---|
| Critical | Reference is cleared between manual messages | `update()` false branch zeroes `_manual_control_setpoint`; `phi_r` can pulse between stick target and zero | characterize in ULog; resolve only under separate behavior-change approval; golden-test legacy CADRC path |
| Critical | Wrong output channel/physical side | logical labels and historical parameter mapping do not prove wiring | bench/disarmed channel identification; record physical mapping and polarity |
| Critical | Sign mismatch | FRD/NED/body-X, RC, allocation ratios, PWM reverse, and linkage signs compound | end-to-end low-amplitude sign test for positive `phi_r` and positive `u_phi` |
| Critical | Observer uses unapplied raw command | `_Uc[k-1]` differs by `U_SCALE`, clamps, trim, allocation, enable gates, and actuator dynamics | preserve for CADRC baseline; make command convention explicit for each new observer |
| Critical | No effective hydro slew limit | standard CA slew happens before hydro addition | do not assume `CA_SVn_SLEW` protects water foils; separately approve/test common limiter |
| Critical | Multiple controllers contribute to same final actuator | standard CA actuator setpoint plus hydro output | verify airframe/function mapping and whether standard control-surface output overlaps hydro servo functions |
| High | Unit mismatch | raw `Uc` unknown units; hydro torque normalized; hydro thrust schema says normalized while code sends N | typed/documented interface and unit assertions/tests |
| High | `dt` mismatch | controller callback is variable but default observer step fixed at 2.5 ms; nested clamps differ | log both measured and used `dt`; require each algorithm to declare timing assumption |
| High | Stale controller state on enable/selector changes | `HY_CADRC_EN` toggles in STAB do not reset | explicit enable/selector edge state machine |
| High | Non-bumpless reset/switch | all states zero; no prior actuator initialization | initially disallow armed switching; later implement algorithm-specific reviewed initialization |
| High | Output-range mismatch between algorithms | CADRC raw output requires 0.001 scale; another algorithm may emit normalized output | controller result metadata or per-controller explicit scaling; never reuse scale implicitly |
| High | Saturation invisible to observer | no applied-command feedback or saturation flag | log both requested/applied; controller-specific anti-windup only after review |
| High | NaN/Inf before final check | input/state matrices can become non-finite; only `Uc/Wx_p` checked at end | finite-check all common inputs/outputs and fail safely without changing CADRC golden path unexpectedly |
| High | Attitude/rate sample asynchrony | 100 Hz-ish attitude held in 400 Hz-ish gyro loop | carry timestamps/sample age; reject or diagnose stale attitude per reviewed policy |
| High | Attitude validity/age not checked | zero-initialized/stale quaternion may be converted and used | log attitude age; establish startup validity gate |
| High | Allocation defaults produce no roll authority | default roll ratios zero and foil AUX default disabled | parameter preflight validation for required nonzero/mapped channels |
| High | Allocator solver singular/non-finite behavior | Newton inversion result is not checked before use | solver validity diagnostics and safe bounded fallback in a separate allocator review |
| Medium | Observer-state semantic mislabelling | `z12/z22` are error-dynamics extended states, not automatically physical disturbance | keep equations/sign convention in logs/docs |
| Medium | Gain changes without reset | parameters update live and overwrite gains/J0/alpha | decide and test atomic parameter-update/reset policy |
| Medium | `J0` interpretation error | comment says inertia-like; equations use it as input coefficient | name units/convention in controller interface; do not transpose to `1/J` by assumption |
| Medium | Unbounded `z11`, `z21`, `p_r`, raw `Uc` | only `z22` and downstream output are clamped | stress/golden tests including large step and saturation |
| Medium | Euler numerical stability | high observer/gains with forward Euler and changing step | discrete stability analysis for each allowed `dt`; no silent reuse of CADRC step policy |
| Medium | Adaptive inverse conditioning | rank-one `Delta^T Delta`, damped generalized inverse, Cholesky fallback | test alpha>0 separately; default alpha=0 should remain baseline |
| Medium | Logger timestamp is zero | custom message timestamp never assigned | set timestamp in future diagnostic work and verify ULog sample alignment |
| Medium | Logging downsampling hides limits/switches | standard topics logged at 20–200 ms intervals | controller diagnostic at controller cadence or justified bounded rate |
| Medium | First update suppresses gyro/control terms | reset transient differs from steady observer equation | preserve in baseline tests; document before any initialization change |
| Medium | RC versus joystick behavior differs | RC dead-zone/calibration; MAVLink direct scaling; selector accepts either | log `data_source`; test both intended sources |
| Medium | Control loop may run backup with stale/no gyro copy | work queue has 20 ms backup and fallback timing | verify message freshness and fail-safe semantics |
| Medium | PWM reverse/min/max differ by channel | historical MAIN12 reversed, MAIN11 not | include PWM mapping in commissioning checklist |
| Low | Class naming ambiguity | file/feature called CADRC, class called `ADRCController` | avoid renaming baseline; use unambiguous names only for new classes/files |

## 12. Exact Files Likely to Be Modified in the Next Stage

The following is a minimal expected change set, not authorization to modify it now.

### Existing files likely to require edits

1. `src/modules/hydro_rate_control/HydroRateControl.hpp`  
   Add selector state, independent controller instances, common input/output structs if kept local, and reset/switch bookkeeping.
2. `src/modules/hydro_rate_control/HydroRateControl.cpp`  
   Add parameter loading, selector branch, per-controller reset handling, common output/diagnostic point. The existing CADRC branch should remain arithmetically unchanged.
3. `src/modules/hydro_rate_control/hydro_rate_control_params.c`  
   Add controller selector and algorithm-specific parameters; preserve all existing CADRC names/defaults.
4. `src/modules/hydro_rate_control/CMakeLists.txt`  
   Add new algorithm source/header files.
5. `msg/CMakeLists.txt`  
   Only if a new controller-agnostic diagnostic uORB message is added.
6. `src/modules/logger/logged_topics.cpp`  
   Add that diagnostic topic and, if approved, explicit `hydro_servos`/`hydro_motors` logging.

### New files likely to be added

Use final algorithm names agreed by the controls team; a minimal anticipated layout is:

```text
src/modules/hydro_rate_control/LADRC.h
src/modules/hydro_rate_control/LADRC.cpp
src/modules/hydro_rate_control/AMCEO_ADRC.h
src/modules/hydro_rate_control/AMCEO_ADRC.cpp
src/modules/hydro_rate_control/ADRCVariant.h
src/modules/hydro_rate_control/ADRCVariant.cpp
msg/HydroRollControllerStatus.msg
```

These names are organizational placeholders only and do not define any algorithm.

### Files that should not require edits for controller coexistence

- `src/modules/hydro_rate_control/CADRC.cpp`
- `src/modules/hydro_rate_control/CADRC.h`
- `src/modules/hydro_allocator/HydroAllocator.cpp`
- `src/modules/hydro_allocator/HydroAllocator.hpp`
- `src/modules/control_allocator/ControlAllocator.cpp`
- board PWM/airframe files

If the next stage proposes changing any of these, especially CADRC or allocation, that change should be separated from initial controller coexistence and justified by its own baseline tests and real-vehicle safety review.

### Acceptance checks before flight

- Exact CADRC golden-vector match for fixed input sequences, including reset, first two samples, saturation, `alpha=0`, and `alpha>0`.
- Build `matek_h743_dummy` with the project-supported compiler/Python environment.
- Verify selector default boots into existing CADRC behavior with stored legacy parameters.
- Verify inactive controllers cannot alter outputs or shared state.
- Verify mode/selector/enable transitions while disarmed; keep armed switching disabled until a reviewed bumpless method exists.
- Confirm logical side, Servo function, PWM channel, PWM reversal, and physical foil direction on hardware.
- Log and compare raw request, limited `u_phi`, hydro actuator values, combined actuator values, and physical PWM.
- Inject stale attitude, missing manual updates, NaN/Inf, saturation, and allocation-disable cases on bench/SITL/HITL before water testing.

---

Audit conclusion: the existing CADRC can be retained as the immutable behavioral baseline while adding independent controller implementations around a narrow selector in `HydroRateControl`. The first integration milestone should be coexistence plus observability, not an algorithmic “cleanup” of CADRC. The observer-input mismatch, missing hydro slew limit, manual-reference clearing, asynchronous state samples, and unverified physical channel polarity are baseline facts that must be measured and managed explicitly rather than silently changed during AMCEO integration.
