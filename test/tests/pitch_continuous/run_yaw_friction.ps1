$ErrorActionPreference = 'Stop'
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$gimbal = Join-Path $project 'Core\Application\Module\Gimbal'
$motor = Join-Path $project 'Core\Application\Devices\Motor'
$imu = Join-Path $project 'Core\Application\Devices\IMU'
$out = Join-Path $project 'build\friction-tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
& python "$PSScriptRoot\extract_mit.py" "$motor\dm_motor.c" "$out\mit_wire.c"
if ($LASTEXITCODE -ne 0) { throw 'MIT extraction failed' }
foreach ($variant in @('normal','reverse','quantized')) {
    $defines = @('-DGIMBAL_YAW_DIRECTION=1.0f')
    if ($variant -eq 'reverse') { $defines = @('-DGIMBAL_YAW_DIRECTION=-1.0f') }
    if ($variant -eq 'quantized') { $defines += @('-DYAW_TEST_FRAME_PERIOD=5U','-DYAW_TEST_QUANTIZED_FEEDBACK=1') }
    & gcc -std=c11 -O2 -Wall -Wextra -Werror @defines "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
        "$PSScriptRoot\test_yaw_friction.c" "$out\mit_wire.c" `
        "$gimbal\gimbal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" `
        "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" `
        "$gimbal\pitch_calibration.c" "$gimbal\pitch_continuous_cal.c" `
        -o "$out\friction_${variant}.exe" -lm
    if ($LASTEXITCODE -ne 0) { throw 'Friction test compilation failed' }
    & "$out\friction_${variant}.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Friction test failed' }
}
