# IMU 第二阶段：常温验收 → 恒温验收 → 再考虑云台闭环

新增上层读取接口和可选简洁记录 `imu 5`，见 [IMU_API.md](IMU_API.md)。原模式1～4不变。

当前实物流程：先在取出的 A 板上测试，确认正常后才上云台。
此前 `imu 1/2` 连续三分钟正常，证明采集初步稳定，不等于姿态/温漂已经验收。
本版不修改电机反馈、PID、Pitch 前馈、标定表或 CAN 控制。

## 当前就做这几步（没有 24 V 的独立 A 板）

1. 烧录本工程 `build/Debug/G3508_Test.elf`，串口仍为 USART2 115200 8N1。
2. 发送 `imu 4` 和换行，把板固定静止；再发送 `imucal` 和换行。
3. 保持静止约 7 秒。前三列应从 `1,0,0...` 走到 `3,0,1`：
   state=1 采集静止零偏约 3 秒；state=2 姿态启动约 3 秒；state=3 可观察姿态。
   progress 是静止窗口比例 0..1，不是总启动进度。
4. 发送 `imu 3`。检查抬头 pitch_up 增大，左转 yaw_left 增大，反向相反。
   板子回原位置后 pitch/roll 应接近原值；保持静止再观察 1 分钟。
5. 返回 `imu 2`，确认新增算法后仍约 1000 Hz，错误/溢出/丢帧计数未持续增加。

请反馈 `imu 3` 静止数据和转动方向即可；若不进入 state=3，先发 `imu 4` 的一行，
不要先改增益。静止校准需要真正固定板子，检测算法不能区分慢速恒定转动和零偏。
第一次测试使用独立板，不连接执行机构。**原 main 中 `Pitch_Cal_Start()` 仍保留，
它会启动原有云台标定；后续上云台前需要明确测试模式，不能直接认为烧录后不会运动。**

## 与第一阶段输出的区别

`imu 1` 的前三列一直是原始 XYZ **角速度 rad/s**，不是三个姿态角。
`imu 2` 的原 12 列不变。`imu 0` 返回原云台输出。

`imu 3`（9 列）：

`roll_deg, pitch_up_deg, yaw_left_deg, corrected_gx_rad_s, corrected_gy_rad_s,
corrected_gz_rad_s, attitude_state, attitude_error, valid`

姿态角单位为度，角速度仍为芯片右手坐标系 rad/s，故抬头时 corrected_gy 为负，
pitch_up 为正。这不是符号不一致的错误：只在最终显示接口把 Euler pitch 取负。
内部 Fusion 四元数不翻单轴，避免破坏右手系。roll 符号沿用实测 X 轴。
当前根据实测假定芯片轴与枪口的前/左/上对应；安装改变后必须复验三轴映射。
板子随 Pitch 转动，输出的是板子相对重力的姿态，不是电机相对底座的编码器角度。
没有磁力计，yaw 是本次校准后的相对航向，会漂移，范围 ±180°；不提供绝对航向。
大俯仰接近 ±90° 时 Euler 表达有奇异性，内部四元数仍可用。

`imu 4`（12 列）：

`attitude_state, attitude_error, progress, bias_x_rad_s, bias_y_rad_s,
bias_z_rad_s, calibration_temp_c, current_temp_c, heater_duty_0_to_1,
heater_state, heater_error, thermal_stable`

注意 bias 是所测陀螺仪零偏；没有做加速度计六面标定，不能据此宣称姿态精度达标。

| 状态 | 含义 |
|---|---|
| attitude_state 0 / 1 / 2 / 3 / 4 | 等恒温 / 静止校准 / 姿态启动 / 可观察 / 故障锁定 |
| attitude_error 0 / 1 / 2 / 3 / 4 | 无错误 / 30秒校准超时 / 数据断续 / 偏离校准温度超过2℃ / 数值异常 |
| heater_state 0 / 1 / 2 / 3 | 关闭 / 升温或调节 / 恒温 / 故障锁定 |
| heater_error 0 / 1 / 2 / 3 / 4 | 无错误 / 测温或时序异常 / ≥55℃ / 60秒无明显升温 / 180秒未稳定 |

`valid=1` 仅表示本版校准与数据时序检查通过，不是上车/射击工况验收通过。
数据缺帧或温度变化后的姿态故障不会悄悄清零航向并恢复：排查后重新 `imucal`。
`imureset` 重新初始化采集与姿态，会丢弃本次偏置/航向。偏置没有写入 Flash。

## 加热：功能保留并默认启用，常温测试不必等待加热

默认目标 **45℃**，PI 初始参数为 duty 单位 Kp=0.15/℃、Ki=0.025/(℃·s)，
占空比上限 80%，≥47℃暂时停止输出，≥55℃锁定保护。
在 45±0.5℃ 连续 5 秒后 thermal_stable=1，自动开始静止校准。
这只是初始参数，温升、超调、稳态温差与电源能力仍需实板验证。

没有加热供电时，`imucal` 允许直接用当前稳定室温校准；不会等到45℃才采样。
此时加热尝试 60 秒仍没有升温会报告 heater_error=3 并停止输出，**并不代表 MPU 采集失败**。
若需要停止此次加热尝试，可用 `imuheat 0`；排查供电后 `imuheat 1` 可重新启用并清除热故障。
不要在真实过温或供电未查明时循环清故障。`imuheat 1` 不自动清姿态错误。
常温校准后又升温，温差超过2℃会使姿态失效；稳定后重新 `imucal`，不继续用冷机零偏。

接好具有加热供电的硬件后，再做以下验收：

- 全程观察 `imu 4` 的温度、占空比、热错误；预期逐渐升至45℃附近，占空比从高值降下来。
- 45±0.5℃连续5秒且板子固定后，应自动走完校准/姿态启动（此前若手动校准或已故障，手动 `imucal`）。
- 恒温后复查静止角速度、姿态漂移以及 `imu 2` 错误计数；增加电机动作/振动是后续独立验收。

## 数据链与文件

`SPI DMA → 采样队列 → IMU_Service → 每帧回调 IMU_AttitudeProcess`
`→ 静止窗口均值/方差 → gyro 去零偏 → Fusion 六轴解算 → 姿态/valid`

回调在主循环，不在 DMA/EXTI 中断做浮点融合。消费队列的每一帧，不是轮询最新帧凑1kHz。
静止窗口3秒，检查角速度幅值、方差、各轴加速度变化、加速度模长和温度跨度；运动会重置窗口，
30秒未成功有明确失败状态。开始时重力确定 roll/pitch，yaw 设相对零，随后还有约3秒 Fusion 启动期。
融合输入 gyro 从 rad/s 转 deg/s，accel 从 m/s² 转 g，dt 使用 DRDY 的 DWT 无符号差。
Fusion 的方向残差拒绝之外，加了加速度模长偏离g超过1.5m/s²时的重力修正拒绝。
短时动态加速度仍会影响姿态，六轴不能消除全部运动加速度误差。

`IMU_GetSample → IMU_HeaterService → IMU_HeaterPortWrite → PB5/TIM3_CH2`

温控与姿态分离：失温不伪装为数据断流，失温后的零偏温度条件由姿态自身检查。
加热输出采用 TIM3 **PWM2 单脉冲**，1MHz计数、20ms窗口：先低后高，溢出回零后自动停止且输出低。
每轮授权最多16ms加热；主循环阻塞或普通断点暂停不应留下永久高电平。
不得在调试器中开启 TIM3 freeze，否则会冻结输出电平。软件无法防止 MOS 管短路或供电电气故障。
真实脉冲波形尚未用示波器验证；可在 PB5 验证周期/极性/停更后低电平。

无需当前修改 CubeMX：.ioc 保持 PB5 上电低电平 GPIO，随后 USER CODE 内的
`IMU_HeaterInit` 切到 AF2。TIM3 完全由 `imu_heater_port.c` 管理；**不要另加 MX_TIM3_Init 或把 TIM3
分给其他用途**。这是有意独立管理的板级端口，不是遗漏一个 HAL_TIM 启动。

主要文件：`Core/Application/Devices/IMU/imu_attitude.c`（校准/融合），
`imu_heater.c`（热状态机），`imu_heater_port.c`（PB5单脉冲），
`main.c`（调用与显示），`app.c`（命令），`ThirdParty/Fusion`（固定版本原版源码与MIT许可证）。

## 验证与来源

执行 `powershell -NoProfile -ExecutionPolicy Bypass -File tests/imu/run.ps1`：
原采集测试 + 逐帧回调/重启测试 + 偏置/倾斜初始化/90°航向/30°俯仰/重复帧/
运动和温变拒绝/超时/掉帧/NaN/计时回绕/加热保护测试。
这些测试与 ARM 编译不代替实物验收；没有测得闭环精度、温度曲线或总线上实际时序。

Fusion 上游：https://github.com/xioTechnologies/Fusion
固定 commit `a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`，原版权见 `ThirdParty/Fusion/LICENSE.md`。
定时器模式参考 ST RM0090 的 TIM2～TIM5 PWM / One-pulse mode 章节。
