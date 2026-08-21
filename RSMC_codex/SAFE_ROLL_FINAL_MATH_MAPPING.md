# HAAR SafeRoll 数学与源码映射

## 采样与参考

每个 `vehicle_angular_velocity` callback 为样本 k。原始时间差来自 `timestamp_sample`，有效范围 1–10 ms，见 `SafeRollController.cpp:808-888`。姿态、角速度、allocator timestamp 和 finite 状态在进入新控制数学前检查。

人工命令：

```text
phi_cmd = manual_control_setpoint.roll * radians(HY_ROLL_MAX)
```

源码：`HydroRateControl.cpp:254-258`。mode 0 继续把该未滤波 `roll_sp` 送入 CADRC；modes 1–4 的二阶滤波为：

```text
z1_dot = z2
z2_dot = wn²(phi_cmd-z1)-2*zeta*wn*z2
```

并对 z2、z2_dot 限幅，源码 `SafeRollController.cpp:403-420`。

## 因果创新与估计器

`SafeRollController.cpp:422-493` 使用：

```text
r = (p_k-p_(k-1))/dt - a0*p_(k-1)
u pair = previous_u_effective_model
u_f = lambda*(u_f_previous + u_e,newest - u_e,previous)
r_f = lambda*(r_f_previous + r_newest - r_previous)
xi = r_f - b_hat_previous*u_f
```

只有 excitation/gate 通过才按提示公式更新 b，并依次投影到 prior 与 certified interval。扰动点估计和半径在同一函数中按 `HY_SR_LD`、`HY_SR_DDB*dt/0.0025` 更新；invalid sample 不进入 history。

history 为 40 个固定对象，无 heap。五个 lag `{4,8,16,24,32}` 在 `SafeRollController.cpp:22` 和 `:494-561` 依次求交。每个空交集只置 conflict bit 并保留此前集合；传播量为 `HY_SR_DBB*dt/0.0025`。

## Nominal

`SafeRollController.cpp:328-343`：

```text
e1 = phi_ref-phi
e2 = phi_ref_dot-p
u_nominal =
 (-a0*p-d_hat+phi_ref_ddot+k1*e1+k2*e2)/b_control
```

code 1/3/4 使用 b_hat，code 2 使用 b0。函数内部不饱和。

## 双边鲁棒 DE-HOCBF

`SafeRollController.cpp:345-385` 严格对应：

```text
h_upper = phi_safe-phi
h_lower = phi_safe+phi
beta_upper = g1*g2*h_upper-(g1+g2)*dt*p-dt²(a0*p+d_hat)
beta_lower = g1*g2*h_lower+(g1+g2)*dt*p+dt²(a0*p+d_hat)
b_minus = max(b_floor,b_hat-rho_b)
b_plus = b_hat+rho_b
```

上下界按 `c_lower/c_upper` 的符号选择 `b_plus/b_minus`。code 2 使用固定 `b0` 与 `[b_min,b_max]` 的最坏半径；code 3/4 使用在线证书。code 1 不求交 CBF。

最终 `psi_upper/psi_lower/psi_min` 使用 blended command 重新计算：`SafeRollController.cpp:1002-1034`。

## 硬区间与最小松弛

`SafeRollController.cpp:899-967`：

```text
U_amp  = [-u_max,u_max]
du     = HY_SR_DUMAX*dt/0.0025
U_rate = [u_previous_executed-du,u_previous_executed+du]
```

code 1–3 再与 `U_alloc` 求交；code 4 只使用 amp/rate。`u_previous_executed` 是 `_last_selected`。

allocation-aware 过程位于 `SafeRollController.cpp:583-618`：

1. 共享 `solve()` 得到 hydro candidate；
2. 将最近完成的 generic Servo5/4 加到 hydro 左/右请求；
3. 用共享 `linearize()` 的 `g_left/g_right` 构造 commanded angle/rate interval；
4. 用 `HY_SR_DREM` 收缩角度范围；
5. 左右区间求交。

最终 candidate 通过 `checkCandidate()`、generic+hydro、normalized range、commanded 角度/速率和 `forward()` 校核：`SafeRollController.cpp:620-680`。失败时从上一执行命令向 candidate 固定二分 8 次：`:682-723`。

最小松弛位于 `SafeRollController.cpp:287-297`：

```text
epsilon = max(0,
              0.5*(cbf_lower-cbf_upper),
              cbf_lower-hard_upper,
              hard_lower-cbf_upper)
```

只扩张 CBF，硬区间不放宽。最终为 `project(u_nominal,U_safe)`。

## 模式与输出

- code 0：`HydroRateControl.cpp:396-399` 调用 `observeLegacy()`，稳态直接返回 legacy。
- code 1：filter + estimator + nominal + amp/rate/alloc。
- code 2：b0 + fixed rho_b + robust CBF + hard interval + relaxation。
- code 3：在线 b/d/certificate + robust CBF + allocation-aware interval。
- code 4：同 code 3，但跳过 U_alloc；仅 `HY_SR_ENABLE=2` 可输出。

使能策略和所有前置故障检查见 `SafeRollController.cpp:839-889`。MAP_OK/OWN_OK 不在这些条件中。

## 最终执行与有效输入

ControlAllocator 保持：

```text
final_servo = constrain(generic_servo + hydro_servo,-1,1)
```

源码 `ControlAllocator.cpp:850-876`。最终 Servo5 映射逻辑左、Servo4 映射逻辑右，乘 allocator `max_hydrofoil_angle` 得 commanded delta，再用共享 `forward()` 重构 `u_phi_effective_model`：`ControlAllocator.cpp:926-943`。

`foil_position_feedback_available=false`、`u_effective_is_model_based=true`：`ControlAllocator.cpp:942-944`。PWM/actuator_outputs 从未作为测得翼角。

