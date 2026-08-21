# HAAR safe-roll Phase 1.5 性能结果模板

状态：**尚待目标板实测**。不得填入主机 benchmark 冒充 H743 数据。

板号/机体号：________________　日期：________________　操作/审核：________________  
固件 version：________________　PX4 SHA-256：________________  
参数文件 SHA-256：________________　SD 卡/logger 配置：________________  
测量时长/采样率：________________　供电/温度：________________

| 工况 | Logger | HydroRate avg/max us | SafeRoll avg/max us | HydroAllocator avg/max us | ControlAllocator avg/max us | Control latency avg/max us |
|---|---|---:|---:|---:|---:|---:|
| 未解锁、中性 | off | | | | | |
| 未解锁、中性 | on | | | | | |
| STAB、无驱动 | off | | | | | |
| STAB、无驱动 | on | | | | | |

## 工作队列与 CPU

`nav_and_controllers` load/interval/overrun：________________________________  
`rate_ctrl` load/interval/overrun：________________________________  
`top` CPU idle/min/max：________________________________  
deadline miss 数：________　持续时间/上下文：________________________________

## dt 分布

有效样本数：________　首样本零值数：________  
min/p50/p95/p99/max：________ / ________ / ________ / ________ / ________ us  
`dt<1 ms`：________　`dt>10 ms`：________　non-monotonic：________  
gyro stale：________　attitude stale：________　最长连续异常：________

## Stack

| Task/work queue | stack total | minimum free/high-water | warning |
|---|---:|---:|---|
| nav_and_controllers | | | |
| rate_ctrl | | | |
| logger | | | |
| output driver | | | |

## Logger 对比

平均开销变化：________________  
最大值/tail 变化：________________  
丢周期/丢日志：________________  
原始 `perf/top/work_queue` 文件：________________  
ULog 文件及 SHA-256：________________

## 评审

无 deadline miss：通过 / 不通过  
SafeRoll observer <<250 us：通过 / 不通过  
allocator <<500 us：通过 / 不通过（口径：solve / full module）  
2.5 ms 周期裕量充分：通过 / 不通过  
stack：通过 / 不通过  
logger 影响：通过 / 不通过  
B5 可关闭：是 / 否

审核意见与签字：____________________________________________________________
