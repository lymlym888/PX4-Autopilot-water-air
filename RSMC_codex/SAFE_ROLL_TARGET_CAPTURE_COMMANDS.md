# HAAR safe-roll 目标板采集命令

这些命令由人工在 PX4 NSH 执行并将完整终端输出保存为带日期的文本文件。`actuator_outputs` 只是 PWM/驱动命令，不是水翼位置反馈。任何阶段都保持 `HY_SR_MODE=0`；本文不授权烧写、参数修改或 actuator test。

## A. 上电、未解锁、只读

```sh
ver all
param show HY_SR_*
param show HY_FOIL_*
param show HY_ALLOCATE_MODE
param show HY_WING_MAX_A
param show HY_SV0_IDX
param show HY_SV1_IDX
param show CA_*
param show PWM_MAIN_FUNC11
param show PWM_MAIN_FUNC12
param show PWM_MAIN_MIN11
param show PWM_MAIN_MAX11
param show PWM_MAIN_MIN12
param show PWM_MAIN_MAX12
param show PWM_MAIN_REV
hydro_allocator status
control_allocator status
work_queue status
top
perf
perf latency
listener haar_safe_roll_status 5
listener hydro_allocate_message 5
listener actuator_servos 5
listener actuator_outputs 5
```

PX4 中固件版本命令为 `ver all`；它对应验收项中的“version”。若目标固件另有 `version` 别名，可额外保存其输出，但不要以别名失败替代 `ver all`。

人工保存项：固件文件 SHA-256、参数导出文件 SHA-256、终端原始文本、UTC/本地时间、板号。确认 `HY_SR_MAP_OK=0`、`HY_SR_OWN_OK=0`，不要在此阶段修改。

## B. STAB 但不驱动执行器

只有在空气桨和水下推进器均硬件断电、机体固定后，人工按批准程序进入 STAB。保持控制杆中性，不解锁或使用安全阻止实际驱动。

```sh
hydro_allocator status
listener vehicle_status 5
listener manual_control_setpoint 10
listener haar_safe_roll_status 20
listener hydro_cadrc_control_message 20
listener hydro_allocate_message 20
listener actuator_servos 20
listener actuator_outputs 20
work_queue status
top
perf
```

核对：

- `requested_mode=active_mode=0`；
- generic/hydro/final Servo4/5 及 `generic_hydro_final_consistent`；
- foil AUX selection/value/threshold/gate；
- logical indexes 与 mapping valid；
- `physical_mapping_verified=false`、`servo_ownership_verified=false`、`safe_roll_hardware_ready=false`。

## C. 人工批准的 mode 0 低幅台架测试

本组只负责采集；低幅 MAIN11/MAIN12 动作必须由人工按 `SAFE_ROLL_BENCH_CHECKLIST.md` 独立执行。Codex 不运行 actuator-test 命令。

测试前：

```sh
perf reset
hydro_allocator status
listener haar_safe_roll_status 5
listener hydro_allocate_message 5
listener actuator_servos 5
listener actuator_outputs 5
```

每个五点测试期间，由第二终端循环采集或开启批准的 logger。每完成一个点，至少保存：

```sh
listener haar_safe_roll_status 3
listener hydro_allocate_message 3
listener actuator_servos 3
listener actuator_outputs 3
```

全部点结束并回零后：

```sh
hydro_allocator status
control_allocator status
work_queue status
top
perf
perf latency
```

不要在本阶段执行 `param set HY_SR_MODE 1..4`、`param set HY_SR_MAP_OK 1` 或 `param set HY_SR_OWN_OK 1`。人工确认结果应先进入结果模板和评审。

## 文件命名建议

```text
YYYYMMDD_HHMM_A_disarmed_readonly.txt
YYYYMMDD_HHMM_B_stab_nodrive.txt
YYYYMMDD_HHMM_C_main11_point_<n>.txt
YYYYMMDD_HHMM_C_main12_point_<n>.txt
YYYYMMDD_HHMM_parameters.params
```
