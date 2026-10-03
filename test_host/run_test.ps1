# Builds and runs the PC test of kaiaai-esp32/safety.h.
# Needs zig as a C++ compiler: python -m pip install --target <dir> ziglang
param(
  [string]$Python = "C:\Users\Dark\AppData\Local\Programs\Python\Python313\python.exe",
  [string]$ZigDir = "D:\Robot\fw_build\_zig"
)

$here = $PSScriptRoot
$build = Join-Path $here "build"
New-Item -ItemType Directory -Force $build | Out-Null

# safety.h includes its neighbours by name, so it has to sit next to the stubs
Copy-Item (Join-Path $here "stubs\host_stubs.h") $build -Force
Copy-Item (Join-Path $here "..\kaiaai-esp32\safety.h") $build -Force
Copy-Item (Join-Path $here "test_safety.cpp") $build -Force

$stub = '#include "host_stubs.h"'
foreach ($name in 'WiFi.h','esp_sleep.h','robot_config.h','util.h','motors.h','lidar.h','ros.h','adc.h') {
  Set-Content (Join-Path $build $name) $stub
}
New-Item -ItemType Directory -Force (Join-Path $build "driver") | Out-Null
Set-Content (Join-Path $build "driver\gpio.h") '#include "../host_stubs.h"'

$env:PYTHONPATH = $ZigDir
$exe = Join-Path $build "test_safety.exe"
& $Python -m ziglang c++ -std=c++17 -Wall -Wno-nullability-completeness -I $build (Join-Path $build "test_safety.cpp") -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $exe
exit $LASTEXITCODE
