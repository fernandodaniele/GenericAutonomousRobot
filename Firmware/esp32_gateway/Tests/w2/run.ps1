$ErrorActionPreference = 'Stop'
$firmware = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$build = Join-Path $firmware '.pio/build/w2-native'
New-Item -ItemType Directory -Force -Path $build | Out-Null
# Headers puente: el mismo mock del SDK, sin APIs de prueba en produccion.
$mocks = Join-Path $build 'mocks'
foreach ($header in @('esp_err.h','esp_log.h','esp_timer.h','esp_http_server.h','freertos/FreeRTOS.h','freertos/task.h','driver/uart.h','driver/gpio.h')) {
    $path = Join-Path $mocks $header
    New-Item -ItemType Directory -Force -Path (Split-Path $path) | Out-Null
    Set-Content -LiteralPath $path -Value '#include "sdk.h"' -Encoding Ascii
}
$assembly = ".section .rdata,`"dr`"`n"
foreach ($name in @('index.html','style.css','app.js')) {
    $symbol = $name.Replace('.','_')
    $asset = (Join-Path $firmware "web/$name").Replace('\','/')
    $assembly += ".global _binary_${symbol}_start`n_binary_${symbol}_start:`n.incbin `"$asset`"`n.global _binary_${symbol}_end`n_binary_${symbol}_end:`n"
}
$assets = Join-Path $build 'assets.S'
Set-Content -LiteralPath $assets -Value $assembly -Encoding Ascii
$exe = Join-Path $build 'test_status.exe'
& gcc -std=gnu11 -Wall -Wextra -Werror -O0 -g -I "$PSScriptRoot/mocks" -I $mocks -I "$firmware/include" -I "$firmware/src" "$PSScriptRoot/test_status.c" $assets -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Fallo compilacion nativa' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Fallo pruebas nativas' }
