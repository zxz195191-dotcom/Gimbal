# CLion目标为空、OpenOCD配置看似丢失

2026-10-06检查：当前打开的是外层`D:/Clion_32/Gimbal_Shooter_A`，没有顶层CMakeLists.txt；真实工程和原运行配置都在`test`。原配置没有丢失，OpenOCD也没有被删除。

1. CLion选择“文件 → 打开”，选 `D:/Clion_32/Gimbal_Shooter_A/test/CMakeLists.txt`，选择“作为项目打开”。不要只将main.c作为文件打开。
2. 等CMake加载完成，使用已有Debug预设（原IDE名称`Debug - Debug`）。
3. 运行配置选择原来的“OpenOCD 下载并运行 / G3508_Test”，目标和可执行文件均为`G3508_Test`。

已核实的路径：

- OpenOCD：`C:/OpenOcd/xpack-openocd-0.12.0-7/bin/openocd.exe`
- 板配置：`C:/OpenOcd/stm32f4_daplink.cfg`
- 固件：`D:/Clion_32/Gimbal_Shooter_A/test/build/Debug/G3508_Test.elf`
- 原运行配置：`test/.idea/workspace.xml`，仍含目标、Debug预设和板配置。

板配置使用CMSIS-DAP、SWD、1000kHz、stm32f4x；没有改成ST-Link。原端口3333/4444。OpenOCD版本命令正常，配置只做不连接硬件的解析检查；没有执行下载、复位或电机运动。

如果继续手动新建配置，先打开正确工程，再填上述板配置并选择G3508_Test目标。外层空项目中补一个OpenOCD条目不会自动建立CMake目标。

命令行编译（PowerShell）：

```powershell
Set-Location D:/Clion_32/Gimbal_Shooter_A/test
cmake --preset Debug
cmake --build --preset Debug --parallel
```

工程恢复后使用新41°版本：最低姿态上电并首次`pe`建立零点，`pcal`采集0～41°，完成后`pscan`比较。默认`ptest`现在20～41°往返。普通pd/pe沿用已有零点，不会重新置零；补偿表在RAM中，重新烧录/重启后需重新pcal。

参考：https://www.jetbrains.com/help/clion/creating-new-project-from-scratch.html
