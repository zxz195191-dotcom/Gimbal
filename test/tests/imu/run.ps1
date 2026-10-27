$ErrorActionPreference = 'Stop'
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$driver = Join-Path $project 'Core\Application\Devices\IMU'
$testBuild = Join-Path $project 'build\imu-tests'
New-Item -ItemType Directory -Path $testBuild -Force | Out-Null
& gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$PSScriptRoot\fakes" "-I$driver" `
    "$PSScriptRoot\test_imu.c" "$driver\imu_frame.c" "$driver\imu_mpu6500.c" `
    -o "$testBuild\test_imu.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'IMU test compilation failed' }
& "$testBuild\test_imu.exe"
if ($LASTEXITCODE -ne 0) { throw 'IMU tests failed' }
& gcc -std=c11 -O2 -Wall -Wextra -Werror -DFUSION_USE_NORMAL_SQRT `
    "-I$driver" "-I$project\ThirdParty\Fusion" `
    "$PSScriptRoot\test_attitude.c" "$driver\imu_attitude.c" "$driver\imu_heater.c" `
    "$project\ThirdParty\Fusion\FusionAhrs.c" -o "$testBuild\test_attitude.exe" -lm
if ($LASTEXITCODE -ne 0) { throw 'Attitude test compilation failed' }
& "$testBuild\test_attitude.exe"
if ($LASTEXITCODE -ne 0) { throw 'Attitude tests failed' }
