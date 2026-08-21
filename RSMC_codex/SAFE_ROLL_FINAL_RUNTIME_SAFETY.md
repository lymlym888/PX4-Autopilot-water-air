# HAAR SafeRoll 运行时安全

## 启用门

新 code 输出必须同时满足：

- `HY_SR_ENABLE` 允许 requested code；
- requested code 为 1–4；
- STAB 且 `HY_CADRC_EN=1`；
- gyro/attitude timestamp、dt、finite 有效；
- allocator/context 有效且不陈旧；
- foil gate 开启；
- logical Servo4/5 mapping 匹配既有配置；
- config、certificate、hard interval、projection 和 nonlinear candidate 有效；
- candidate total runtime 不超过 2000 us。

实现：`SafeRollController.cpp:808-889`、`:1049-1081`。MAP_OK、OWN_OK、非零 generic trim 不在启用条件中。

## Fault bit

| bit | 含义 | fatal |
|---:|---|---|
| 0 | timestamp non-monotonic | yes |
| 1 | dt too small | yes |
| 2 | dt too large | yes |
| 3 | gyro stale | yes |
| 4 | attitude stale | yes |
| 5 | quaternion nonfinite | yes |
| 6 | angular velocity/input nonfinite | yes |
| 7 | allocator invalid/stale | yes |
| 8 | foil gate disabled | yes |
| 9 | output disabled/not STAB | yes |
| 10 | requested code invalid | yes |
| 11 | logical mapping invalid | yes |
| 12–14 | Phase 1.5 兼容保留 | no new use |
| 15 | invalid configuration | yes |
| 16 | invalid b/d certificate | yes |
| 17 | hard interval empty | yes |
| 18 | nonlinear candidate invalid | yes |
| 19 | projection/psi invalid | yes |
| 20 | rejected certificate lag conflict | no；该 lag 被丢弃 |
| 21 | candidate runtime overrun | yes |
| 22 | fallback latched | state |

定义：`SafeRollController.hpp:42-64`。

## 切换与 fallback

- 请求 code/enable/reset-sensitive 参数/arming/STAB/attitude reset 变化会清空新控制器 history，参考以当前 `phi_cmd` 初始化，输出从上一执行值开始 blend：`SafeRollController.cpp:96-121`、`:790-805`。
- 正常 blend 的每拍输出仍投影到包含 `U_rate` 的 hard interval：`:978-1001`。
- fault fallback 先将实时 legacy 限制到 DUMAX step，再按 blend 计算，并再次限制本拍 step：`:741-754`。
- fault 会锁存；即使数据恢复，也不会自动重入失败 code。操作者改变 MODE 或 RST_ID 后才重新尝试：`:1062-1099`。
- 所有对执行链的结果都在写入前检查 finite；invalid 不发送 NaN candidate。

## Allocator 与执行器

- allocation query 只调用无状态共享库，不发布 uORB、不修改 HydroAllocator warm-start。
- 线性区间使用 generic+hydro 的 final commanded state，不将 PWM 当角度。
- exact check 固定 8 次二分，没有无界求解。
- Hydrofoil model Newton solve 有固定最大迭代和显式 singular/nonfinite/nonconvergence 状态：`HydrofoilAllocationModel.cpp:88-211`。
- 下游仍执行 `constrain(generic+hydro,-1,1)`；SafeRoll 不依赖该 saturation 解决可实现性。

## 固定资源

- history 长度 40、lag 数 5、bisection 8，均为编译期常量。
- SafeRoll 和共享 allocator 中没有 `new/delete/malloc`、STL 动态容器或递归。
- 最大静态 SafeRoll 函数栈 656 B；目标板任务 high-water 尚待人工实测。

## 当前物理语义

- `delta_left/right_commanded` 是 final pre-PWM normalized command 乘模型角度标度。
- `foil_rate_*_commanded` 是相邻 commanded delta 的差分。
- `u_effective_model` 使用 final generic+hydro commanded delta/T 的 forward model。
- 没有水翼位置反馈；绝不应将这些字段解释为 actual/measured angle。

## 实验前仍需验证

本实现消除了 MAP_OK/OWN_OK 软件阻断，依据用户声明接受历史实物链路。它不消除以下工程风险：

- H743 目标实测 WCET、调度余量和 logger/SD 卡吞吐未测；
- 模型参数、扰动界与安全边界仍需渐进式实验确认；
- 上游 allocation query 使用最近完成的下游 generic/final 样本，存在一个控制链样本的同步延迟；
- code 4 明确取消 allocation-aware interval，风险高于 1–3。

