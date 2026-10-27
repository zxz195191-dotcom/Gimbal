# A 板 MPU6500：第一阶段上板验收

> 本文保留第一阶段采集说明。已增加第二阶段零偏、姿态和温控，
> 当前功能与验收入口请看 [IMU_ATTITUDE.md](IMU_ATTITUDE.md)。
> 下文“加热关闭、不解算姿态”仅描述第一阶段历史版本。

本版建立六轴采集和诊断。A 板随 Pitch 运动，但输出仍为 MPU6500 芯片原始 XYZ，
尚未转换到枪口坐标系。gyro 为 rad/s，accel 为 m/s²，温度为 ℃。
`sample.valid` 只表示当前采样可读，不表示完成零偏标定或可用于姿态闭环。
本版不解算姿态、不启用 IST8310、不写 Flash；PB5 被明确拉低，加热关闭。

## 数据链与入口

`main: MX_SPI5_Init → DWT_Init → IMU_Init`（实际 DWT_Init 在 USER CODE 2）
→ 主循环 `IMU_Service` 执行分步复位、身份校验、寄存器配置与回读
→ PB8 上升沿 → `IMU_OnDataReady` → SPI5 DMA
→ `IMU_OnTransferComplete` 入队（含 DRDY 时刻的 DWT 和毫秒时间戳）
→ `IMU_Service` 检查/解码/单位换算 → `IMU_GetSample` / `g_imu_debug`。

原云台仍按电机编码器反馈运行。IMU 初始化失败会报告错误，主循环继续运行。
传感器复位和唤醒等待通过状态机完成，没有 HAL_Delay；初始化寄存器传输有 2 ms 超时。
数据读取采用 DMA，队列可用 7 帧，每轮最多处理 4 帧。超过 5 ms 的旧帧丢弃，
10 ms 无 DMA 完成、超过 2 ms DMA 未完成、连续 10 个坏帧会锁定 FAULT。
错误不自动隐藏；排除原因后用 `imureset` 重试，该命令会重置诊断计数。
本阶段未检测所有“合理数值但传感器冻结”的故障，不能仅凭 valid 判定可闭环。

## CubeMX 对照（已写入 ioc 和源文件，无需先重新生成）

| 项目 | 配置 |
|---|---|
| PF6 | GPIO 输出，高电平，MPU6500_CS |
| PF7 / PF8 / PF9 | SPI5 SCK / MISO / MOSI，AF5 |
| SPI5 | Full Duplex Master，8 bit，MSB first，CPOL Low，CPHA 1 Edge，软件 NSS |
| SPI 分频 | 128：PCLK2=90 MHz 时为 703.125 kHz，配置和采样都用这个速率 |
| PB8 | GPIO_EXTI8，上升沿，下拉，MPU6500_INT |
| RX DMA | DMA2 Stream3 / Channel2，Normal，Byte，Memory Increment，Very High |
| TX DMA | DMA2 Stream4 / Channel2，Normal，Byte，Memory Increment，Very High |
| NVIC | EXTI9_5、DMA2_Stream3、DMA2_Stream4、SPI5，抢占优先级 4，子优先级 0 |
| PB5 | GPIO 输出，低电平，IMU_HEATER；此阶段不用 TIM3 |

MPU6500：WHO_AM_I=0x70；±2000°/s（16.4 LSB/°/s）、±8g（4096 LSB/g）、
1 kHz 输出，陀螺仪和加速度 DLPF 均为 92 Hz。滤波参数是初始配置，尚未通过云台振动测试。
SPI HAL 来自本机参考工程，与当前 HAL 主版本文件完全一致（HAL 1.8.5）；保留 ST 文件原版权。

CubeMX 重新生成前提交或备份当前工程；.ioc 已同步，但本次没有通过 CubeMX GUI 验证生成。
请确认生成后的 USER CODE 区仍有 IMU_Init/IMU_Service 和 HAL 回调转发，CMake 中仍保留
Core/Application/Devices/IMU 的两个源文件。不要再复制参考工程里另外一套 HAL 回调。

## 上板操作

当前 main.c 仍调用 `Pitch_Cal_Start()`。第一次验证仅给 A 板供电，并断开执行机构的 CAN，
避免原有自动标定带动云台。驱动本身不会请求电机运动。

1. 编译/烧录 `build/Debug/G3508_Test.elf`。串口仍用原 USART2、115200、8N1。
2. 发送 `imu 2` 加换行，等待 2 秒。正常为 `state=6, error=0, id=112`，
   采样率约 1000 Hz，drdy/dma/valid 计数持续增加。
3. 发送 `imu 1` 加换行，看静止加速度模长约 9.81 m/s²，温度合理且连续，
   手转板子时角速度随方向反转。此时未恒温/去零偏，静止角速度不保证为零。
4. 分别做抬头/低头、左转/右转，记录是哪根 gyro 轴变化、正负号。
   加速度测量的是比力：板子平放时向上的芯片轴通常约 +g；不能按“重力朝下”直接改符号。
5. 连续跑至少 10 分钟，记录采样率、SPI错误、溢出、坏帧、丢帧计数。
   然后再在受控条件下加入电机动作/振动比较；此时仍不把 IMU 接入闭环。
6. `imu 0` 恢复原六通道云台曲线。`imureset` 可在调试断点恢复后重新启动 IMU。

串口命令仅在 `VISION_PROTOCOL_ENABLED=0` 的原调试模式可用。
输出沿用 100 ms 周期，约 10 Hz；这是串口显示频率，传感器本身仍按 1 kHz 采样。
一次只选择一组 CSV，不混合云台/IMU 文本。若串口无输出，先看调试器中的 g_imu_debug，
区分“IMU采集失败”和“UART没发送”；请让 MCU 继续运行，暂停断点会使采样失去实时性。

### CSV 顺序

`imu 1` 共 9 列：
`gx_rad_s, gy_rad_s, gz_rad_s, ax_mps2, ay_mps2, az_mps2, temperature_c, dt_ms, valid`

`imu 2` 共 12 列：
`state, error, who_am_i, sample_rate_hz, drdy_count, dma_count, spi_error_count,
queue_overrun_count, invalid_frame_count, sample_age_ms, busy_drop_count, stale_drop_count`

首帧 dt=0，因为没有前一帧；随后正常约 1 ms。间隔不在 0.5～1.5 ms 内会计入 timing_gap_count
并将该帧标为无效。DWT 在 180 MHz 时约 23.86 秒回绕，本驱动先作无符号差再转换为秒，
且绝不重置全局 DWT。调试 g_imu_debug 可额外查看 raw、flags、dt_min/max_cycles 等。

### 错误排查

| error | 含义及下一步 |
|---|---|
| 0 | 无锁存错误；还需看 state 和 sample.valid |
| 1 | DWT 未启用：核对 main 调用顺序 |
| 2 | SPI错误：看 hal_spi_error，检查 SPI5/DMA/片选配置 |
| 3 | ID不等于0x70：检查供电、芯片型号、片选和 SPI 引脚 |
| 4 | 寄存器回读不同：看 failed_register、expected_value、readback_value |
| 5 | 无数据：ID正确但drdy为0时，先核对实物板版本和 MPU INT 是否接 PB8 |
| 6 | DMA超时：核对 DMA2 时钟、中断向量、Stream/Channel |
| 7 | 连续坏帧：看 sample.flags，检查数据就绪、总线、量程和采样周期 |

state：0 OFF、1 RESET、2 WAIT_RESET、3 WAIT_WAKE、4 CONFIGURE、5 WAIT_DATA、6 RUNNING、7 FAULT。
flags 按位：1 无DRDY、2 全0/全FF、4 原始值接近量程端点、8 dt异常。

## 验证范围

`powershell -NoProfile -ExecutionPolicy Bypass -File tests/imu/run.ps1` 使用 gcc 跑主机测试，
覆盖大小端、有符号值、量程换算、DWT/SysTick回绕、初始化回读失败、断流、
DMA忙/超时、SPI错误、队列溢出、旧数据拒绝及重启；它不模拟真实 SPI 电气与中断延迟。
ARM 编译验证通过不等于上板验收通过。上板数据确认后才做恒温、零偏、坐标映射和姿态融合。
