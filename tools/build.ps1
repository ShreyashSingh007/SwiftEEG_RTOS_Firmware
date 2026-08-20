<#
    SwiftEEG build helper.

    Why this exists rather than `nrfutil ... toolchain launch -- cmake ...`:

    The nrfutil toolchain launcher injects its own -S pointing at the repo
    root and mangles Windows drive paths (-DBOARD_ROOT=D:/foo arrives as
    "D:"). The result is that any build whose application is NOT the repo
    root - every test suite - silently configures the main app instead, with
    the wrong prj.conf. It looks like it worked; it did not.

    So the toolchain environment is set up explicitly and cmake is called
    directly.

    Usage:
        .\tools\build.ps1                 # main application
        .\tools\build.ps1 -Target proto   # protocol test suite
        .\tools\build.ps1 -Pristine       # wipe the build dir first
#>
param(
    [ValidateSet("app", "proto")]
    [string]$Target = "app",
    [switch]$Pristine
)

# NOT "Stop": cmake and ninja write ordinary progress to stderr, and with
# Stop PowerShell turns that into a fatal NativeCommandError. Success is
# judged by $LASTEXITCODE instead, which is what actually matters.
$ErrorActionPreference = "Continue"

$Toolchain = "C:\ncs\toolchains\dcbdc366a1"
$NcsVersion = "v3.4.0"
$RepoRoot = Split-Path -Parent $PSScriptRoot

$env:PATH = "$Toolchain\opt\bin;$Toolchain\opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin;$env:PATH"
$env:ZEPHYR_BASE = "C:\ncs\$NcsVersion\zephyr"
$env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = "$Toolchain\opt\zephyr-sdk"

switch ($Target) {
    "app"   { $SourceDir = $RepoRoot }
    "proto" { $SourceDir = Join-Path $RepoRoot "tests\proto" }
}
$BuildDir = Join-Path $SourceDir "build"

if ($Pristine -and (Test-Path $BuildDir)) {
    Write-Host "wiping $BuildDir"
    Remove-Item $BuildDir -Recurse -Force
}

Write-Host "building '$Target' from $SourceDir"

& "$Toolchain\opt\bin\cmake.exe" -B $BuildDir -S $SourceDir -GNinja -DBOARD=swifteeg
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

& "$Toolchain\opt\bin\ninja.exe" -C $BuildDir
if ($LASTEXITCODE -ne 0) { throw "build failed" }

Write-Host ""
Write-Host "artifact: $BuildDir\zephyr\zephyr.hex"
