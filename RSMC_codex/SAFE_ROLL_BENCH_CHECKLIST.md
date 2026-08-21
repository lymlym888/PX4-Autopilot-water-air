# HAAR safe-roll 断桨台架检查清单

本文件只供人工执行。Codex 未执行任何步骤。首轮全程保持 `HY_SR_MODE=0`，不要设置 `HY_SR_MAP_OK/HY_SR_OWN_OK=1`，不要进入 modes 1–4。

## 0. 批准与人员

- [ ] 试验负责人批准日期、地点、固件 hash 和参数备份。
- [ ] 至少两人参加：一人操作，一人观察并可立即断电。
- [ ] 填写 `SAFE_ROLL_BENCH_RESULT_TEMPLATE.md`，禁止凭记忆补录。

## 1. 机械与电气安全

- [ ] 拆除所有空气螺旋桨，或物理断开其动力线路。
- [ ] 断开水下推进器动力，或用硬件方式确保其绝不旋转。
- [ ] 将机器人刚性固定；水翼全程不会碰撞支架、人员或线缆。
- [ ] 使用限流电源/安全电源，记录电压与限流值；准备独立总断电。
- [ ] 佩戴护目镜，人员不在翼面运动平面内。
- [ ] 首轮不测全行程，只使用经负责人批准的小幅值。

## 2. 上电后只读确认

- [ ] 保持未解锁，执行 `SAFE_ROLL_TARGET_CAPTURE_COMMANDS.md` 的 A 组并保存原始输出。
- [ ] 核对固件 SHA/版本、`HY_SR_MODE=0`、`HY_SR_MAP_OK=0`、`HY_SR_OWN_OK=0`。
- [ ] 核对 `PWM_MAIN_FUNC11=204`、`PWM_MAIN_FUNC12=205`；若不一致立即停止。
- [ ] 核对 `HY_SV0_IDX/HY_SV1_IDX`、`CA_*`、MIN/MAX/REV；若与试验计划不一致立即停止。
- [ ] 执行 `hydro_allocator status`，记录 AUX selection/value/threshold/gate/allocation mode。

## 3. STAB 无驱动观察

- [ ] 仍不允许推进器通电；按人工批准的方法进入 STAB，但不发送 actuator test。
- [ ] 保持 roll/pitch/yaw/throttle 中性，记录 `haar_safe_roll_status`、`hydro_allocate_message`、`actuator_servos`、`actuator_outputs`。
- [ ] 确认 `active_mode=0`；若不是 0 立即断电并停止。
- [ ] 记录 Servo4/5 的 generic、hydro、final normalized command 与 MAIN11/12 PWM。
- [ ] 检查 `generic_hydro_final_consistent`；false 时停止后续动作。

## 4. 人工批准的低幅单通道测试

以下动作必须由人工在 PX4/QGC 的受控 actuator-test 界面执行；Codex 不执行。每次只动一个输出，持续时间尽可能短，动作后立即回零。先 MAIN11，完全回零并确认静止后再 MAIN12。

- [ ] 试验负责人填写首轮绝对幅值上限：`|normalized| <= ______`。
- [ ] MAIN11：依次测 `-small`、`-small/2`、`0`、`+small/2`、`+small`。
- [ ] MAIN12：依次测 `-small`、`-small/2`、`0`、`+small/2`、`+small`。
- [ ] 每一点记录请求 normalized、generic、hydro、final、PWM、物理角度/方向和稳定时间。
- [ ] 记录 MAIN11 实际驱动左翼还是右翼；MAIN12 同样记录。
- [ ] 记录正命令导致前缘/后缘向何方向，以及所采用物理角正方向。
- [ ] 记录卡滞、回差、死区、异常噪声、电流突升；任一异常立即断电。
- [ ] 禁止扩大到全行程；禁止同时驱动两翼；禁止接通任何推进器。

## 5. 中位与动态

- [ ] 比较 normalized command=0 时的 PWM 与物理水翼零位。
- [ ] 比较历史 generic=-0.1 对应中位与 normalized=0 对应中位。
- [ ] 记录左/右 neutral offset、近似死区、回差和小幅阶跃速率。
- [ ] 根据无碰撞余量给出保守 safe commanded angle range；首轮不得把该范围当作正式机械限位。

## 6. 审核与参数确认

- [ ] 试验人员和审核人员在结果模板签字。
- [ ] 先评审方案 A/B/C；禁止现场临时清除 `-0.1`。
- [ ] 只有物理左右和符号确认后，才可由人工在后续批准流程中设置 `HY_SR_MAP_OK=1`。
- [ ] 只有 ownership/trim 方案落地且 mode 0 物理输出验证后，才可人工设置 `HY_SR_OWN_OK=1`。
- [ ] 本台架即使全部通过，也不代表 B3 gate 正式参数或 B5 WCET 已通过。
