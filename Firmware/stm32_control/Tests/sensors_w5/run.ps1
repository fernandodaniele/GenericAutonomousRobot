$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'build/sensors_w5'
New-Item -ItemType Directory -Force $out | Out-Null
& gcc -std=c11 -Wall -Wextra -Werror -O0 -g -I "$PSScriptRoot/mocks" -I "$PSScriptRoot/../safety_b11/mocks" -I "$root/Core/Inc" "$PSScriptRoot/test_sensors.c" -o "$out/test_sensors.exe"
if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }
& "$out/test_sensors.exe"
if ($LASTEXITCODE -ne 0) { throw 'Sensor tests failed' }
