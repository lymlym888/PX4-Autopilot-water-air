# HAAR safe-roll 台架结果模板

状态：**尚待人工填写**。空白项不得推断为通过。

## 基本信息

日期/地点：____________________  
操作员：____________________  观察员：____________________  审核员：____________________  
机体编号：____________________  飞控板：____________________  
PX4 version：____________________  
固件 SHA-256：____________________  
参数备份文件及 SHA-256：____________________  
电源电压/限流：____________________  
空气桨已拆除：是 / 否　水下推进器已硬件断电：是 / 否　机体已固定：是 / 否

## 只读参数快照

`HY_SR_MODE`：____　`HY_SR_MAP_OK`：____　`HY_SR_OWN_OK`：____  
`HY_FOIL_AUX`：____　当前 AUX 值：____　`HY_FOIL_THR`：____　gate：开 / 关  
`HY_ALLOCATE_MODE`：____　`HY_WING_MAX_A`：____  
`HY_SV0_IDX`：____　`HY_SV1_IDX`：____  
`PWM_MAIN_FUNC11`：____　`PWM_MAIN_FUNC12`：____　`PWM_MAIN_REV`：____  
MAIN11 MIN/MAX：____/____　MAIN12 MIN/MAX：____/____  
CS3 trim/effect：____________________　CS4 trim/effect：____________________

## 通道物理映射

MAIN11 = physical left / physical right / UNKNOWN  
MAIN12 = physical left / physical right / UNKNOWN  
软件逻辑 `control[4]=left, control[3]=right` 与实物一致：是 / 否 / UNKNOWN

MAIN11 正命令：前缘向____ / 后缘向____ / 物理角度____  
MAIN12 正命令：前缘向____ / 后缘向____ / 物理角度____  
统一物理角正方向定义：________________________________________________

## 五点测量

人工批准 `small` 幅值：________ normalized；每点最大持续：________ s。

| Channel | 请求值 | generic | hydro | final normalized | PWM us | 物理角度 | 方向/现象 | 稳定时间 |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| MAIN11 | `-small` | | | | | | | |
| MAIN11 | `-small/2` | | | | | | | |
| MAIN11 | `0` | | | | | | | |
| MAIN11 | `+small/2` | | | | | | | |
| MAIN11 | `+small` | | | | | | | |
| MAIN12 | `-small` | | | | | | | |
| MAIN12 | `-small/2` | | | | | | | |
| MAIN12 | `0` | | | | | | | |
| MAIN12 | `+small/2` | | | | | | | |
| MAIN12 | `+small` | | | | | | | |

## 中位、死区和速率

逻辑左 mechanical neutral：________ deg；对应 final normalized/PWM：________ / ________ us  
逻辑右 mechanical neutral：________ deg；对应 final normalized/PWM：________ / ________ us  
左 neutral offset：________　右 neutral offset：________  
左 approximate dead zone：________　右 approximate dead zone：________  
左 approximate rate：________ deg/s　右 approximate rate：________ deg/s  
建议 safe commanded angle range：左 [____, ____] deg；右 [____, ____] deg  
卡滞：无 / 有（说明）________　回差：________　异常噪声/电流：________

## 软件诊断

`generic_hydro_final_consistent` 全程 true：是 / 否  
`active_mode` 全程 0：是 / 否  
有无 saturation：____________________  
command=0 与机械零位一致：是 / 否；说明：________________________________  
历史 `-0.1` 是否有明确物理中位用途：是 / 否 / UNKNOWN；证据：____________

## 决策与签字

建议方案：A / B / C / 暂不决策  
`HY_SR_MAP_OK` 可设 1：是 / 否（必须有映射和符号证据）  
`HY_SR_OWN_OK` 可设 1：是 / 否（必须有 ownership/trim 落地证据）  
遗留风险：________________________________________________________________

操作员签字/日期：____________________  
观察员签字/日期：____________________  
审核员签字/日期：____________________
