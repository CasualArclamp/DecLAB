# made by CasualArclamp
# Builds single-file DecLAB executables for 64-bit Windows 10/11 that need nothing
# installed: the C runtime is linked statically, so no Visual C++ redistributable.
# The results go to exe\ (git-ignored): declab-gui.exe and
# declab.exe (command line). After DecDAB's script.
#
#   powershell -ExecutionPolicy Bypass -File scripts\build-portable.ps1
#
# Builds in target\portable, so the usual target directory and its cache stay as they
# are.

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$triple = "x86_64-pc-windows-msvc"

# With an explicit --target the flags reach only the executables, not build scripts
# and proc macros.
$env:RUSTFLAGS = "-C target-feature=+crt-static"
$env:CARGO_TARGET_DIR = Join-Path $root "target\portable"
if (-not $env:CARGO_BUILD_JOBS) { $env:CARGO_BUILD_JOBS = "6" }

Push-Location $root
try {
    cargo build --release --target $triple -p declab-gui -p declab-cli
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed ($LASTEXITCODE)" }
    $out = Join-Path $root "exe"
    New-Item -ItemType Directory -Force $out | Out-Null
    foreach ($exe in "declab-gui.exe", "declab.exe") {
        Copy-Item (Join-Path $env:CARGO_TARGET_DIR "$triple\release\$exe") $out -Force
        $mb = (Get-Item (Join-Path $out $exe)).Length / 1MB
        Write-Host ("{0,-16} {1,6:N1} MB" -f $exe, $mb)
    }
    # TETRA speech: the ACELP codec is a separate, privately built library
    # (ETSI's code is not GPL), in ..\DecTETRA-acelp. Built here with the
    # same static C runtime as the exes, and copied next to them.
    $acelpToml = Join-Path (Split-Path -Parent $root) "DecTETRA-acelp/Cargo.toml"
    if (Test-Path $acelpToml) {
        cargo build --release --target $triple --manifest-path $acelpToml
        if ($LASTEXITCODE -ne 0) { throw "tetra_acelp build failed ($LASTEXITCODE)" }
        Copy-Item (Join-Path $env:CARGO_TARGET_DIR "$triple/release/tetra_acelp.dll") $out -Force
        Write-Host "tetra_acelp.dll built and copied (TETRA speech)"
    } else {
        Write-Host "DecTETRA-acelp not found: TETRA speech will not play"
    }
    Write-Host "written to $out"
} finally {
    Pop-Location
}
