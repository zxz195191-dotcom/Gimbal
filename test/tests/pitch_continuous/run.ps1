$ErrorActionPreference = 'Stop'
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$gimbal = Join-Path $project 'Core\Application\Module\Gimbal'
$motor = Join-Path $project 'Core\Application\Devices\Motor'
$imu = Join-Path $project 'Core\Application\Devices\IMU'
$testBuild = Join-Path $project 'build\pitch-continuous-tests'
New-Item -ItemType Directory -Path $testBuild -Force | Out-Null
& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
  "$PSScriptRoot\test_continuous.c" "$gimbal\gimbal.c" "$gimbal\pitch_calibration.c" `
  "$gimbal\pitch_continuous_cal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" -o "$testBuild\test_continuous.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Pitch continuous test compilation failed' }
& "$testBuild\test_continuous.exe"
if ($LASTEXITCODE -ne 0) { throw 'Pitch continuous tests failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
  "$PSScriptRoot\test_yaw_inertial.c" "$gimbal\gimbal.c" "$gimbal\pitch_calibration.c" `
  "$gimbal\pitch_continuous_cal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" -o "$testBuild\test_yaw_inertial.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Yaw inertial test compilation failed' }
& "$testBuild\test_yaw_inertial.exe"
if ($LASTEXITCODE -ne 0) { throw 'Yaw inertial tests failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror '-DYAW_IMU_DIRECTION=-1.0f' `
  "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
  "$PSScriptRoot\test_yaw_inertial.c" "$gimbal\gimbal.c" "$gimbal\pitch_calibration.c" `
  "$gimbal\pitch_continuous_cal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" -o "$testBuild\test_yaw_reverse.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Yaw reverse test compilation failed' }
& "$testBuild\test_yaw_reverse.exe"
if ($LASTEXITCODE -ne 0) { throw 'Yaw reverse tests failed' }

& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$gimbal" `
    "$PSScriptRoot\test_load.c" "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" `
    -o "$testBuild\test_load.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Load model test compilation failed' }
& "$testBuild\test_load.exe"
if ($LASTEXITCODE -ne 0) { throw 'Load model tests failed' }
& python "$PSScriptRoot\extract_mit.py" "$motor\dm_motor.c" "$testBuild\mit_wire.c"
if ($LASTEXITCODE -ne 0) { throw 'MIT source extraction failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
    "$testBuild\mit_wire.c" "$PSScriptRoot\test_load_integration.c" "$gimbal\gimbal.c" "$gimbal\pitch_calibration.c" `
    "$gimbal\pitch_continuous_cal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" `
    "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" -o "$testBuild\test_load_integration.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Load integration test compilation failed' }
& "$testBuild\test_load_integration.exe"
if ($LASTEXITCODE -ne 0) { throw 'Load integration tests failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror '-DGIMBAL_YAW_DIRECTION=-1.0f' "-I$PSScriptRoot" "-I$gimbal" "-I$motor" "-I$imu" `
    "$testBuild\mit_wire.c" "$PSScriptRoot\test_load_integration.c" "$gimbal\gimbal.c" "$gimbal\pitch_calibration.c" `
    "$gimbal\pitch_continuous_cal.c" "$gimbal\yaw_inertial.c" "$gimbal\pitch_inertial.c" "$gimbal\yaw_diagnostics.c" "$gimbal\yaw_friction_goal.c" "$gimbal\load_compensation.c" "$gimbal\yaw_load_cal.c" `
    -o "$testBuild\test_load_reverse.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Load reverse compilation failed' }
& "$testBuild\test_load_reverse.exe"
if ($LASTEXITCODE -ne 0) { throw 'Load reverse tests failed' }
