$ErrorActionPreference = 'Stop'
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$gimbal = Join-Path $project 'Core\Application\Module\Gimbal'
$motor = Join-Path $project 'Core\Application\Devices\Motor'
$imu = Join-Path $project 'Core\Application\Devices\IMU'
$testBuild = Join-Path $project 'build\diagnostic-tests'
New-Item -ItemType Directory -Path $testBuild -Force | Out-Null
& python "$PSScriptRoot\extract_mit.py" "$motor\dm_motor.c" "$testBuild\mit_wire.c"
if ($LASTEXITCODE -ne 0) { throw 'MIT extraction failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
    "$PSScriptRoot\test_yaw_diagnostics.c" "$testBuild\mit_wire.c" `
    "$gimbal\gimbal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" `
    "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" `
    "$gimbal\pitch_calibration.c" "$gimbal\pitch_continuous_cal.c" `
    -o "$testBuild\diagnostics.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Diagnostic test compilation failed' }
& "$testBuild\diagnostics.exe" "$testBuild"
if ($LASTEXITCODE -ne 0) { throw 'Diagnostic tests failed' }
foreach ($speed in @(1,5,20)) {
    & python "$PSScriptRoot\decode_yaw_diag.py" "$testBuild\yaw_diag_${speed}dps.log" `
        -o "$testBuild\yaw_diag_${speed}dps.csv"
    if ($LASTEXITCODE -ne 0) { throw 'Diagnostic capture decoding failed' }
}
