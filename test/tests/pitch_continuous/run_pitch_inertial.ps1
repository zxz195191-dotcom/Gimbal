$ErrorActionPreference = 'Stop'
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$gimbal = Join-Path $project 'Core\Application\Module\Gimbal'
$motor = Join-Path $project 'Core\Application\Devices\Motor'
$imu = Join-Path $project 'Core\Application\Devices\IMU'
$out = Join-Path $project 'build\pitch-inertial-tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
foreach ($variant in @('normal','reverse')) {
    $defines = @()
    if ($variant -eq 'reverse') { $defines = @('-DPITCH_ENCODER_ROTATION_SIGN=-1.0f') }
    & gcc -std=c11 -O2 -Wall -Wextra -Werror @defines `
        "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
        "$PSScriptRoot\test_pitch_inertial.c" "$gimbal\gimbal.c" `
        "$gimbal\pitch_inertial.c" "$gimbal\yaw_inertial.c" `
        "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" `
        "$gimbal\pitch_calibration.c" "$gimbal\pitch_continuous_cal.c" `
        "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" `
        -o "$out\pitch_${variant}.exe" -lm
    if ($LASTEXITCODE -ne 0) { throw 'Pitch inertial compilation failed' }
    & "$out\pitch_${variant}.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Pitch inertial tests failed' }
}
