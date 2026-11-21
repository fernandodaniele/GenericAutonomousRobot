$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'build/imu_1'
New-Item -ItemType Directory -Force $out | Out-Null
& gcc -std=c11 -Wall -Wextra -Werror -O0 -g -I "$PSScriptRoot/../safety_b11/mocks" -I "$root/Core/Inc" "$PSScriptRoot/test_imu.c" -lm -o "$out/test_imu.exe"
if ($LASTEXITCODE -ne 0) { throw 'IMU native compilation failed' }
& "$out/test_imu.exe"
if ($LASTEXITCODE -ne 0) { throw 'IMU tests failed' }
