# SwiftEEG — Zephyr RTOS Firmware

8-channel wireless EEG acquisition on nRF52840 + ADS1299, built on the
nRF Connect SDK. Designed as a raw BCI tool: full hardware control, on-chip
DSP, precise timestamps, and a transport-agnostic binary API.

> **Status: M1 complete, verified on hardware.**
>
> | Check | Result |
> |---|---|
> | Board port, flashing, RTT logging | pass |
> | Both LEDs blinking | pass |
> | IMU over SPI | pass, `chip id 0x70` |
> | AFE absent handled, firmware continues | pass |
> | VDD rail | 3315 mV |
> | USB CDC ACM | enumerates as a serial port |
> | BLE advertising as "SwiftEEG" | pass |
> | Unit tests on target | 26/26 |
>
> Verified on a board built **without** the ADS1299. Acquisition and DSP
> (M2) need the populated board.

---

## 0. Where we are  (read this first)

**M1 is done.** Everything below was verified on the board built **without**
an ADS1299.

### Built and working on hardware
- Board port, flashing, RTT logging
- Both LEDs blinking
- IMU responds over SPI (`chip id 0x70`) - driver initialises only, no
  sample pipeline yet, and none is planned until M2 phase 2
- ADS1299 absence detected; firmware carries on, so one binary runs on all
  three boards
- VDD rail 3315 mV
- USB CDC ACM enumerates as a serial port
- BLE advertises as "SwiftEEG"
- 26/26 unit tests pass on target

### Built, tested, but NOT wired into the application
These compile and pass tests, but nothing calls them yet:
- `src/proto` - binary protocol codec (9 tests)
- `src/sys/ringbuf` - lock-free SPSC ring (5 tests)
- `src/dsp` - DC removal, biquad cascade, filter design (12 tests)

They are also **not in the app's CMakeLists** yet - test builds only.

### What BLE can actually do today
Connect, and write bytes to the Control characteristic. Those bytes are
**logged and discarded**. There is no command set and no parameter control.
The codec exists but is not connected to either transport.

### Next, per the plan
1. **Command layer** - define commands, wire the codec into BLE and USB.
   Doable without hardware. Commands touching the AFE can be accepted and
   stored but not applied until the populated board is connected.
2. **M2, acquisition + DSP** - needs the board with the ADS1299 fitted.

### Blocked on the user
- `wsl --install` (admin + reboot) so tests can run without the board.
  Optional; tests currently run on target instead.
- Confirm the ADS1299 board got the **same solder-jumper fix**. Its
  +/-2.5 V analog rails come off the supply that read 2.26 V here.

### Scope reminder
Milestone order is fixed: **(1) bring-up with USB+BLE -> (2) raw ADC + DSP
-> (3) validate DSP -> SD last, may not happen.** No IMU streaming, no
extra features, until the step that calls for them.

---

## 1. Hardware

| Part | Role |
|---|---|
| Raytac MDBT50Q-1MV2 (nRF52840) | MCU, BLE, USB |
| TI ADS1299IPAGR | 8-ch 24-bit EEG AFE |
| ST LSM6DSV16XTR | 6-axis IMU (motion-artifact reference) |
| microSD (SPI mode) | storage |
| MCP73832T | Li-ion charger (no fuel gauge) |

Rails: AVDD +2.5 V, AVSS −2.5 V, DVDD 3.3 V. Analog span 5.0 V, so the
ADS1299 internal 4.5 V reference is usable. Full scale at gain 24 is
**±187.5 mV**; LSB ≈ **22.35 nV**.

### 1.1 Pin map

Derived from `SwiftEEG.kicad_sch` and cross-checked against the PCB netlist.

| Signal | Pin | | Signal | Pin |
|---|---|---|---|---|
| ADS_DRDY | P0.04 | | SD_SCK | P0.12 |
| ADS_MISO | P0.05 | | SD_MISO | P0.14 |
| ADS_GPIO1 | P0.06 | | SD_MOSI | P0.26 |
| ADS_SCLK | P0.07 | | SD_CS | P0.27 |
| ADS_CS | P0.08 | | SD_DETECT | P0.30 |
| ADS_MOSI | P1.08 | | IMU_MISO | P0.20 |
| LED_Y | P1.10 | | IMU_SCK | P0.22 |
| LED_B | P1.11 | | IMU_MOSI | P0.23 |
| LFXO | P0.00/01 | | IMU_INT2 | P0.24 |
| nRESET | P0.18 | | IMU_CS | P1.00 |

LEDs are **active high** — the MCU drives the anode.

### 1.2 Board quirks that shape the firmware

1. **`START` (AFE pin 38) is unconnected.** Conversions are driven by the SPI
   opcode. A floating CMOS input can read high, in which case `STOP` has no
   effect. The AFE driver self-tests for this at boot and reports a hardware
   fault rather than producing silently-bad data.
2. **`RESET` and `PWDN` are pull-ups to DVDD.** Neither can be driven; reset is
   by SPI opcode only, and the AFE cannot be power-cycled by the MCU.
3. **`CLKSEL` is tied high** — the AFE runs its own internal oscillator, spec'd
   ±2 %. Its sample clock is therefore *asynchronous* to the MCU, so true
   sample rate must be measured against the 32.768 kHz crystal, not assumed.
4. **No battery sense.** `VDDH` sits on the regulated rail, so the `VDDHDIV5`
   trick reads the LDO, not the cell. A divider jumper to a free AIN pin is
   planned.
5. **No user button.** SW1 is wired to nRESET only; all control is over BLE/USB.
6. **SD is SPI-only** — 4-bit SDIO is not wired.
7. **Only IMU INT2 is routed**; INT1 is not connected.

### 1.3 Montage (SRB1 referential)

`INxP` is the scalp electrode; `INxN` is switched to SRB1 internally.

| Ch | Site | | Ch | Site |
|---|---|---|---|---|
| 1 | C4 | | 5 | AFz |
| 2 | P4 | | 6 | F3 |
| 3 | F4 | | 7 | C3 |
| 4 | Oz | | 8 | P3 |

---

## 2. Repo layout

```
boards/shreyash/swifteeg/   out-of-tree board port (HWMv2)
dts/bindings/               ti,ads1299 binding
src/board/                  LEDs, power, battery
src/afe/                    ADS1299 driver
src/imu/                    LSM6DSV16X driver
src/timebase/               PPI capture, 64-bit clock, drift, host sync
src/dsp/                    filter chain (CMSIS-DSP)
src/pipeline/               acquisition -> DSP -> sink fan-out
src/transport/              BLE + USB behind one sink interface
src/proto/                  binary codec (pure, host-testable)
src/storage/                SD block layer
src/sys/                    ring buffers, error handling
tests/                      ztest suites
tools/                      Python: golden vectors, host client, R&D harness
openocd/                    ST-Link runner config
```

---

## 3. Flash map

1 MB internal, no external flash, so both OTA images live on-chip.

| Region | Offset | Size |
|---|---|---|
| mcuboot | `0x00000` | 48 K |
| image-0 (slot0) | `0x0C000` | 464 K |
| image-1 (slot1) | `0x80000` | 464 K |
| storage | `0xF4000` | 48 K |

The application must fit in **464 K** alongside Zephyr, the SoftDevice
Controller, USB, CMSIS-DSP and FatFs. Tracked from M1 — finding out late means
repartitioning and re-flashing every unit over SWD.

---

## 4. Building

```
.\tools\build.ps1                 # main application
.\tools\build.ps1 -Target proto   # protocol test suite
.\tools\build.ps1 -Pristine       # wipe the build dir first
```

### 4.1 Do not build via the nrfutil toolchain launcher

`nrfutil sdk-manager toolchain launch -- cmake ...` **silently builds the
wrong thing.** It injects its own `-S` pointing at the repo root and truncates
Windows drive paths (`-DBOARD_ROOT=D:/foo` arrives as `D:`). The symptom is
subtle: a test suite configures with the main application's `prj.conf`, builds
without complaint, and produces a binary that is not the test.

`tools/build.ps1` sets the toolchain environment explicitly and calls cmake
directly, which avoids this. Use it.

`west build` is not an option here either - this repo is an application, not a
west workspace, so west has no `build` command available.

---

### 5.3 The debugger stops BLE

Halting the core stops advertising. Every `openocd ... halt` - including the
RTT attach in `rtt_halted.cfg` - freezes the radio, so a phone scanning at
that moment sees nothing. If BLE looks dead, reset the board, detach the
debugger entirely, and scan again before suspecting the firmware.

## 5. Flashing and logs

**Probe:** ST-Link V2 over SWD to header **J4** (`1=GND 2=nRESET 3=SWDIO 4=SWDCLK`).
**OpenOCD:** xPack 0.12.0 at `D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe`
— not on PATH, and not bundled with NCS (which ships J-Link tooling instead).

OpenOCD 0.12+ moved vendor configs into subdirectories, so the target is
`target/nordic/nrf52.cfg`, **not** `target/nrf52.cfg`.

### 5.1 APPROTECT

The nRF52840 can ship with APPROTECT enabled, locking the debug port until a
mass erase over the CTRL-AP. Reaching CTRL-AP needs **raw DAP** access, so the
config uses `transport select swd`, not `hla_swd` — OpenOCD's own `nrf52.cfg`
warns that HLA adapters cannot reach CTRL-AP, making `nrf52_recover` silently
useless. Raw DAP needs ST-Link firmware V2J28+; if the adapter is older, the
Raspberry Pi 5 bit-banging raw SWD is the fallback.

### 5.2 Commands

Probe — is the chip alive, is it locked:

```
openocd -f openocd/swifteeg.cfg -c "init; targets; exit"
```

Unlock a locked chip (mass erase, destroys all flash):

```
openocd -f openocd/swifteeg.cfg -c "init; nrf52_recover; exit"
```

Flash:

```
openocd -f openocd/swifteeg.cfg -c "program build/zephyr/zephyr.hex verify reset exit"
```

RTT logs — the only log path, since no UART is spare:

```
openocd -f openocd/swifteeg.cfg -c "init; reset halt; rtt setup 0x20000000 0x40000 \"SEGGER RTT\"; rtt start; rtt server start 9090 0; resume"
```

then `telnet localhost 9090`.

---

## 6. Milestones

- **M1 — bring-up. DONE.** Board port, RTT logging, USB + BLE up, IMU ID
  verified over SPI, AFE absence handled, both LEDs blinking.
- **M2 — acquisition + DSP.** PPI-latched `DRDY` timestamps, SPIM3 DMA
  acquisition 250 SPS→16 kSPS, IMU alignment, DSP chain, binary protocol over
  BLE and USB.
- **M3 — validation.** Golden vectors against a Python/SciPy reference,
  injected-signal hardware tests, noise floor, impedance sweep.
- **M4 — SD.** Deferred; block layer stubbed from M1.

---

## 7. DSP design principle

Firmware does only what *must* happen on-chip — DC removal for numeric
headroom, mains notch, anti-alias decimation, IMU artifact removal — and
exposes everything else as a **host-programmable biquad cascade**. No
filtering opinion is baked in.

Notably, the chain deliberately avoids a fixed high-pass above ~0.1 Hz:
aggressive causal high-pass filtering distorts slow ERP components such as
P300, which is exactly what this device is meant to measure well.

Total group delay of the active configuration is computed and reported in
telemetry, so the host can correct sample timestamps exactly.
