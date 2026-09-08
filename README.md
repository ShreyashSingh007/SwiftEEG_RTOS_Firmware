# SwiftEEG — Zephyr RTOS Firmware

8-channel wireless EEG acquisition on nRF52840 + ADS1299, built on the
nRF Connect SDK. Designed as a raw BCI tool: full hardware control, on-chip
DSP, precise timestamps, and a transport-agnostic binary API.

> **Status: M1 complete, fully verified on both board builds.**
>
> | Check | No-AFE board | AFE board |
> |---|---|---|
> | Board port, flashing, RTT logging | pass | pass |
> | Both LEDs blinking | pass | pass |
> | IMU over SPI | `chip id 0x70` | `chip id 0x70` |
> | ADS1299 over SPI | absent, handled | **present, ID 0x3e, 8 ch** |
> | ADS1299 `START` not stuck high | n/a | **pass** |
> | VDD rail | 3315 mV | 3309 mV |
> | USB CDC ACM | COM3 | COM4 |
> | BLE advertising as "SwiftEEG" | pass | pass |
> | Unit tests on target | 26/26 | - |
>
> One binary runs on both builds. Acquisition and DSP are M2.

---

## 0. Where we are  (read this first)

**M1 is done. M2 is under way** - the hardware acquisition path is proven up
to the point of actually fetching the samples.

### M2 progress: samples are flowing through the DMA path

DRDY drives the whole acquisition cycle in hardware. One PPI channel carries
the falling edge to two tasks - `TIMER1` capture, which timestamps the
sample, and `SPIM3 TASKS_START`, which fetches it. The CPU does nothing
until 27 bytes are already in RAM.

Measured on the board at 250 SPS:

```
capture: 247 edges in 1000 ms -> 250 SPS (interval mean 3996 us, min 3996, max 3997)
stream:  247 frames in 1000 ms (237 measured) -> 250 SPS (gap mean 3996 us, min 3996, max 3997)
stream:  status word 0xc00000, bad 0, overruns 0
stream:  ch1 shorted-input noise 31 counts p-p (~692 nV), min -977 max -946
```

What each line is worth:

- **1 us jitter.** That is the 1 MHz timer's own resolution. No interrupt
  latency appears in a timestamp because no interrupt is involved in taking
  it.
- **`status word 0xc00000`, 0 bad in 237.** The ADS1299 hard-wires the top
  four status bits to `1100`. Every frame carrying `0xC` means none slipped
  by a byte - the failure mode to fear when hardware, not code, starts the
  transfer.
- **0 overruns.** No transfer was still running when the next DRDY arrived.
- **692 nV peak-to-peak, shorted inputs, gain 24.** The datasheet's
  input-referred noise is ~0.14 uV RMS at this setting, which is roughly
  0.9 uV peak-to-peak for Gaussian noise. The analog front end is behaving.

The mean gap of 3996 us is ~250.2 SPS against a nominal 250 - the AFE's
internal oscillator running about 0.1 % fast, well inside its +/-2 % spec,
and exactly the offset the drift estimator exists to measure.

BLE still advertises through all of this, which is the real check that the
PPI and GPIOTE channels came from the shared allocators rather than ones
MPSL reserves for the radio.

### Two things the SPI bring-up settled

**SPIM3 is driven through the HAL, not Zephyr's SPI API.** The transfer has
to be started by PPI on the DRDY edge with no code in between, which that
API cannot express. The `spi3` node is left disabled for Zephyr's driver and
`src/afe` owns the peripheral; pins still come from devicetree via pinctrl,
and `spi2` keeps using the normal driver for the IMU.

**Hardware chip select is not used, and cannot be.** SPIM3 is the only
instance with one, and it is configured correctly - `PSEL.CSN`, `CSNPOL`,
`CSNDUR` all read back right - but the part never answers through it. Its
guard time tops out around 4 us and the ADS1299 needs longer between CS
falling and the first clock. So CS is a plain GPIO: toggled around register
access, and held low for the whole streaming session, which is the
arrangement the datasheet describes for continuous read.

Still to do for M2: ring buffer between the interrupt and a DSP thread, the
DSP chain itself, and streaming out over BLE and USB.

---

Everything below is M1, verified on the board with the ADS1299 fitted as
well as the one without.

### Built and working on hardware
- Board port, flashing, RTT logging
- Both LEDs blinking
- IMU responds over SPI (`chip id 0x70`) - driver initialises only, no
  sample pipeline yet, and none is planned until M2 phase 2
- **ADS1299 detected and identified: ID `0x3e`** - reserved bit set,
  `DEV_ID` = ADS1299, `NU_CH` = 8 channels. Absence is also handled, so one
  binary runs on all three boards.
- **`START` pin is not stuck high** - the floating-pin risk is closed, no
  hardware change needed. See section on risks.
- VDD rail 3315 mV (no-AFE board) / 3309 mV (AFE board)
- USB CDC ACM enumerates as a serial port, on both boards
  (`VBUSDETECT=1 OUTPUTRDY=1` -> `usbd_cdc_acm: Configuration enabled`)
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
1. **Ring buffer** - move frames from the transfer-complete interrupt to a
   DSP thread. `src/sys/ringbuf` is written and tested but not yet wired in.
2. **DSP chain**, then streaming, then the command layer.

### Blocked on the user
- `wsl --install` (admin + reboot) so tests can run without the board.
  Optional; tests currently run on target instead.

Nothing else. The solder-jumper question is **resolved**: the AFE board reads
3309 mV, so its supply is healthy and it never had the fault that the other
board had.

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
   **Measured on the AFE board: not stuck high — `STOP` is honoured.** The
   test issues `STOP`, then watches `DRDY` for 10 ms; at the reset default of
   250 SPS a free-running part would pulse every 4 ms, so silence is proof.
   No pull-down wire is needed. The self-test stays in, because a floating
   input can behave differently with temperature or an enclosure fitted.
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

Written, and compiled into the application:

```
boards/shreyash/swifteeg/   out-of-tree board port (HWMv2)
dts/bindings/               ti,ads1299 binding
src/board/                  LEDs, VDD self-test
src/afe/                    ADS1299 probe + START self-test (probe only)
src/transport/              BLE GATT service, USB CDC ACM
openocd/                    ST-Link runner config
tools/                      build, flash, RTT, Python oracles
```

Written and tested, but **not yet in the application's CMakeLists** - test
builds only, nothing calls them:

```
src/proto/                  binary codec          (9 tests)
src/dsp/                    DC removal, biquads   (12 tests)
src/sys/                    lock-free SPSC ring   (5 tests)
```

**Empty placeholders** - directories exist to fix the shape of the tree, but
there is no code in them yet:

```
src/timebase/               M2: PPI capture, 64-bit clock, drift, host sync
src/pipeline/               M2: acquisition -> DSP -> sink fan-out
src/storage/                M4: SD block layer
src/imu/                    M2 phase 2: artifact reference input
```

There is no SwiftEEG IMU driver and none is needed for M1 - the `chip id
0x70` in the boot log comes from Zephyr's in-tree `LSM6DSV16X` driver, bound
in devicetree.

Test layout does not mirror `src/`: the **ringbuf suite lives inside
`tests/dsp/`**, since both link the same test binary. `tests/ringbuf/` and
`tests/timebase/` are empty directories - ignore them.

```
tests/proto/                proto suite            9 tests
tests/dsp/                  dsp + ringbuf suites   17 tests
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

## 5. Flashing and logs

**Probe:** ST-Link V2 over SWD to header **J4** (`1=GND 2=nRESET 3=SWDIO 4=SWDCLK`).
**OpenOCD:** xPack 0.12.0 at `D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe`
— not on PATH, and not bundled with NCS (which ships J-Link tooling instead).

OpenOCD 0.12+ moved vendor configs into subdirectories, so the target is
`target/nordic/nrf52.cfg`, **not** `target/nrf52.cfg`.

### 5.1 Transport: HLA, and what it costs

This ST-Link (V2J46S7) **fails raw-DAP** access on this board — it reports
status 0x0b and never attaches. The config therefore uses
`interface/stlink-hla.cfg` with `transport select hla_swd`, which works
reliably.

The price: HLA cannot reach the nRF52's CTRL-AP, so **`nrf52_recover` does
not work here**. That is the only way back from an APPROTECT lock.

> **Never enable UICR APPROTECT.** There is no recovery path with this probe.

Check it is clear (expect `0xffffffff`):

```bash
openocd -f openocd/swifteeg.cfg -c init -c halt -c "mdw 0x10001208" -c exit
```

Both boards flashed so far read `0xffffffff`. Arduino-flashed boards have not
had APPROTECT set either, so a board arriving with vendor firmware is fine.

### 5.2 SWD speed: faster is more reliable

Counter-intuitive, but measured: **950–1200 kHz works, 125–480 kHz fails.**
If SWD is flaky, do not "helpfully" slow the adapter down — that makes it
worse. `adapter speed 950` is set in `openocd/swifteeg.cfg`.

### 5.3 Reset is by SYSRESETREQ, not SRST

The ST-Link's SRST does not reliably drive nRESET on this board, so
`reset halt`, `reset run`, and OpenOCD's `program ... reset` all time out.
Reset is issued through the Cortex-M's own AIRCR register instead:

```
mww 0xE000ED0C 0x05FA0004
```

Do **not** start firmware by writing PC/SP directly. That leaves a non-zero
exception number in xPSR, and the kernel then trips
`ASSERTION FAIL [!arch_is_in_isr()]` at boot. It looks like a firmware bug
and is not one.

### 5.4 Commands

Flash (mass-erase, write, verify, reset):

```bash
powershell -File tools/flash.ps1
```

`tools/flash.ps1` mass-erases first because the flash write algorithm runs
code *on the target*: resident firmware that keeps taking interrupts — BLE
especially — corrupts the write partway through. It also brace-quotes the
hex path, because the repo path contains spaces and OpenOCD's TCL would
otherwise split it (`Error: Invalid command argument`).

Read the log — RTT is the only path, no UART is spare:

```bash
python tools/rtt.py
```

That halts the core, dumps the buffer, and **leaves the target halted**.
Reflash, or reset it with the `mww` command above. To follow a running
target instead:

```bash
python tools/rtt.py --live 20
```

Both are verified working on this rig. Live mode is the less dependable of
the two - RTT reads race the running target over HLA - so prefer the halted
dump unless you need to watch something that happens after boot.

### 5.5 `usbd_ch9: not supported` is benign

Windows asks every new device for a Microsoft OS descriptor. This firmware
does not implement one, so channel 9 logs a protocol error and Windows moves
on. Enumeration completes normally - `usbd_cdc_acm: Configuration enabled`
follows a few hundred ms later, and the port appears. Not a fault.

Also note the VID/PID are **Zephyr's test IDs** (0x2FE3/0x0001). They must be
changed before this ships to anyone.

### 5.5 The debugger stops BLE

Halting the core stops advertising. Every `openocd ... halt` — including the
default `tools/rtt.py` — freezes the radio, so a phone scanning at that
moment sees nothing. If BLE looks dead, reset the board, detach the debugger
entirely, and scan again before suspecting the firmware.

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
