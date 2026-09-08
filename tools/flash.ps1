<#
    Flash + reset helper.

    Why not just `openocd -c "program ... verify reset exit"`:

      * `program` runs its own reset, and the ST-Link's SRST does not
        reliably drive nRESET on this board - it times out.
      * The flash write algorithm runs code on the target. If resident
        firmware keeps taking interrupts (BLE especially) the algorithm is
        disrupted and the write fails partway, leaving a corrupt image.

    So: mass-erase first via NVMC (direct register writes, no target code),
    then write, verify, and reset through SCB->AIRCR SYSRESETREQ, which works
    where SRST does not.

    Usage:
        .\tools\flash.ps1                                  # main app
        .\tools\flash.ps1 -Hex tests\proto\build\zephyr\zephyr.hex
        .\tools\flash.ps1 -NoErase                         # skip mass erase
#>
param(
    [string]$Hex = "build\zephyr\zephyr.hex",
    [switch]$NoErase
)

$ErrorActionPreference = "Continue"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$OpenOcd  = "D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe"
$Cfg      = Join-Path $RepoRoot "openocd\swifteeg.cfg"

if (-not [System.IO.Path]::IsPathRooted($Hex)) {
    $Hex = Join-Path $RepoRoot $Hex
}
if (-not (Test-Path $Hex)) { throw "hex not found: $Hex" }

Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force

# Forward slashes: OpenOCD's TCL treats backslash as an escape.
$hexFwd = $Hex.Replace([char]92, '/')
$cmds = @("-f", $Cfg, "-c", "init", "-c", "halt")
if (-not $NoErase) {
    # Re-halt after the erase: with flash blank the core runs garbage off an
    # empty vector table, and a running/faulting core disrupts the flash
    # write algorithm ("timeout waiting for algorithm").
    # After erasing, reset and re-halt. With flash blank the core runs off an
    # empty vector table and hard-faults; a running or faulting core disrupts
    # the flash write algorithm ("timeout waiting for algorithm"). Resetting
    # then halting puts it in a known stopped state before the write.
    $cmds += @(
        "-c", "nrf5 mass_erase",
        "-c", "mww 0xE000ED0C 0x05FA0004",
        "-c", "sleep 300",
        "-c", "halt",
        "-c", "sleep 100",
        "-c", "halt"
    )
}
$cmds += @(
    "-c", "flash write_image erase $hexFwd",
    "-c", "verify_image $hexFwd",
    # SYSRESETREQ: a real system reset without relying on SRST.
    "-c", "mww 0xE000ED0C 0x05FA0004",
    "-c", "exit"
)

Write-Host "flashing $Hex"
& $OpenOcd @cmds 2>&1 |
    Select-String -Pattern "Mass erase|wrote|verified|Error:" |
    ForEach-Object { "  $_" }
