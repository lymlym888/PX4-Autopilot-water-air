# HAAR safe-roll Phase 1.5 H743 实时性计划

本计划测量 Phase 1 基础设施，不代表未来 nominal/DE-HOCBF/proposed 的 WCET。当前没有目标板数据。

## 安全前提与矩阵

全程保持 mode 0；空气桨和水下推进器硬件断电；不执行 actuator test。依次测量：

1. logger 关闭、未解锁、中性输入；
2. logger 开启、未解锁、中性输入；
3. 经人工批准的 STAB 无驱动，logger 关闭；
4. 同一状态 logger 开启。

每个工况预热 30 s、测量至少 120 s，开始前执行 `perf reset`。保存 `perf`、`top`、`work_queue status`、相关 listener 和日志文件。

## 指标与证据源

| 指标 | 证据源 | 说明 |
|---|---|---|
| HydroRateControl cycle avg/max | `perf` 的 `hydro_rate_control: cycle` | counter 创建于 `HydroRateControl.cpp:48`，包围主 Run |
| SafeRoll mode-0 observer avg/max | `perf` 的 `safe_roll: cycle`；`haar_safe_roll_status.cycle_time_us` | 仅基础观察器 |
| HydroAllocator solve/module avg/max | `perf` 的 `hydro_allocator: cycle`；`hydro_allocator status` | 当前 counter 包围完整 Run，不仅 Newton solve，报告须标注 |
| ControlAllocator cycle avg/max | `perf` 的 `control_allocator: cycle` | counter 当前在 `ControlAllocator.cpp:311-477`；不应误称含全部发布尾部 |
| gyro-to-output latency | `perf` 的 `control latency` | output driver 的 `MixingOutput` counter，建立于 `mixer_module.cpp:77` |
| nav_and_controllers load | `work_queue status`、`top` | HydroRateControl 位于该队列 |
| rate_ctrl load | `work_queue status`、`top` | HydroAllocator、ControlAllocator 位于该队列 |
| stack free/high-water | `top` 与平台可用 stack 输出 | 记录各任务/队列最小剩余，不能用静态 frame 代替 |
| 400 Hz dt 分布 | ULog `haar_safe_roll_status.dt_raw` | 统计 count/min/p50/p95/p99/max；异常 flags 计数 |
| logger 影响 | 相同工况 logger off/on 差值 | 对 avg/max、miss、dt tail 和队列负载比较 |

调度位置源码：HydroRateControl 使用 `nav_and_controllers`（`HydroRateControl.cpp:47`）；HydroAllocator 和 ControlAllocator 使用 `rate_ctrl`（各自构造函数 `:43`、`:54`）。

## 建议验收指标

- 400 Hz 下无 deadline miss；
- mode-0 SafeRoll observer 显著小于 250 us；
- allocator 正常 solve/完整 cycle 应显著小于 500 us，并分别说明测量口径；
- 整体 gyro-to-output 链路相对 2.5 ms 周期保留充分裕量；
- 无持续 stack warning，最小剩余满足项目规定；
- logger 开启不造成明显丢周期或 dt tail 恶化。

“显著小于”必须由项目评审量化为放行阈值；首次测量不得自动据平均值放宽最大值要求。

## dt 分析方法

从 ULog 导出 `dt_raw` 和 `fault_flags`，分别统计：

- `dt_raw < 1 ms`；
- `1 ms <= dt_raw <= 10 ms`；
- `dt_raw > 10 ms`；
- timestamp non-monotonic；
- gyro/attitude stale；
- 连续异常的最长长度。

首样本 `dt_raw=0` 要与真实异常分开统计。主机 benchmark 只能用于开发回归，不得填入目标板结果。

## 停止条件

出现执行器意外动作、推进器供电、持续 stack warning、reset、deadline miss、明显 logger 丢周期或温度/电源异常时立即停止并保留原始证据。B5 只能在目标板数据填写、复核和签字后关闭。
