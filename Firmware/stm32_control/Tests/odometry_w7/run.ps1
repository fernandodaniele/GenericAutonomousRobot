$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'build/odometry_w7'
New-Item -ItemType Directory -Force $out | Out-Null
& gcc -std=c11 -Wall -Wextra -Werror -O0 -g -I "$PSScriptRoot/../safety_b11/mocks" -I "$root/Core/Inc" "$PSScriptRoot/test_odometry.c" -o "$out/test_odometry.exe"
if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }
& "$out/test_odometry.exe"
if ($LASTEXITCODE -ne 0) { throw 'Odometry tests failed' }
