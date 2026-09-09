# SwiftEEG — Zephyr RTOS Firmware

8-channel wireless EEG acquisition on nRF52840 + ADS1299, built on the
nRF Connect SDK. Designed as a raw BCI tool: full hardware control, on-chip
DSP, precise timestamps, and a transport-agnostic binary API.

> **Status: M1 and M2 complete. M3 measured and passing.**
>
> | Check | Result |
> |---|---|
> | Board port, flashing, RTT | pass, both board builds |
> | ADS1299 / IMU over SPI | `ID 0x3e` / `chip id 0x70` |
> | USB CDC ACM | COM4 |
> | BLE advertising | pass, control not wired yet |
> | DRDY timestamp jitter | **1 us**, hardware latched |
> | Sample rate | 250.35 SPS, 0 sequence gaps |
> | Shorted-input noise | **130-149 nV RMS** (datasheet ~140) |
> | Test-signal amplitude | **0.17-0.22 % error**, 0.05 % channel spread |
> | Golden vectors vs reference | worst 3.4 nV over 512 frames x 8 ch |
> | Unit tests on target | 42/42 |
>
> Streaming works over USB. BLE streaming and full device control are next.

---

## 0. Where we are  (read this first)

**M1 and M2 are done, and M3's measurable parts pass.** Samples come off the
ADS1299 by DMA, run through the DSP chain, and stream to a PC over USB.

### Verified on hardware

```
test signal    CH1-8  3.742-3.744 mV p-p, 0.17-0.22 % off, 0.05 % spread
               0.98 Hz on every channel
noise floor    CH1-8  130-149 nV RMS, inputs shorted, gain 24
timing         250.35 SPS, 0.4 us inter-batch jitter, 0 sequence gaps
protocol       0 CRC failures
golden vectors worst 3.4 nV vs the Python reference, 512 frames x 8 ch
```

`python tools/verify.py` re-runs all of it and compares each number against
the datasheet or the protocol definition.

**The noise floor is the result that matters most.** TI quotes ~140 nV RMS
input-referred at gain 24 and 250 SPS; the eight channels measure 130-149.
The analog front end, the +/-2.5 V rails, the reference and the layout are
all sound. A test signal cannot tell you this - a large injected signal rides
straight over a noisy front end.

### What works

- DRDY drives everything in hardware: one PPI channel timestamps the sample
  and starts its SPI transfer, so the CPU wakes with 27 bytes already in RAM
- Lock-free ring from the interrupt to a DSP thread; 0 drops at 250 SPS
- Chain: 24-bit decode, integer DC removal, microvolt scaling, mains notch
- Binary protocol out over USB CDC, 16 samples a frame
- Commands in: stream start/stop, encoding, input mux, test signal,
  register read
- `tools/swifteeg_scope.py`, a live viewer with a test-signal toggle

### What does not work yet

- **BLE carries no data.** It advertises and connects; the Control
  characteristic still logs and discards. This is the current priority.
- **The bias drive (DRL) is off.** `CONFIG3.PD_BIAS` is 0. Fine for a bench
  test against the internal generator, not fine for electrodes on a head -
  see the note in section 1.2.
- Sample rate is fixed at 250 SPS; there is no command to change it.
- Per-channel gain, mux and lead-off are all-or-nothing, not per channel.
- No SD card. Last item, may not happen.

### Scope, as it now stands

The plan changed on 2026-09-09, deliberately. Full wireless control is the
priority; native platform GUIs are deferred. See section 6.

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
3. **`BIASIN`/`BIASOUT` are shorted and wired to J1.5/6** - the driven
   right leg. **It is currently switched off in firmware**
   (`CONFIG3.PD_BIAS` = 0), which is fine on a bench against the internal
   generator and not fine on a head.

   The DRL amplifier drives the body to cancel common-mode, mostly mains.
   Without it, 50 Hz appears on every channel as a large common signal that
   the notch then has to remove, and any of it that the front end cannot
   reject as common-mode is simply gone. `BIASREF` is grounded on this board,
   which is correct - mid-supply is 0 V between the +/-2.5 V rails - so
   `CONFIG3.BIASREF_INT` stays 0 and the reference comes from the pin.

   Enabling it needs `PD_BIAS` set plus `BIAS_SENSP`/`BIAS_SENSN` choosing
   which channels feed the amplifier. That is M4 work.

4. **`CLKSEL` is tied high** — the AFE runs its own internal oscillator, spec'd
   ±2 %. Its sample clock is therefore *asynchronous* to the MCU, so true
   sample rate must be measured against the 32.768 kHz crystal, not assumed.
5. **No battery sense.** `VDDH` sits on the regulated rail, so the `VDDHDIV5`
   trick reads the LDO, not the cell. A divider jumper to a free AIN pin is
   planned.
6. **No user button.** SW1 is wired to nRESET only; all control is over BLE/USB.
7. **SD is SPI-only** — 4-bit SDIO is not wired.
8. **Only IMU INT2 is routed**; INT1 is not connected.

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

Compiled into the application:

```
boards/shreyash/swifteeg/   out-of-tree board port (HWMv2)
dts/bindings/               ti,ads1299 binding
src/board/                  LEDs, VDD self-test
src/afe/                    ADS1299 driver: probe, config, DMA streaming
src/timebase/               1 MHz TIMER1, 64-bit extension, PPI capture task
src/pipeline/               capture (GPIOTE+PPI), chain (DSP), pipeline (thread)
src/dsp/                    DC removal, biquads, filter design
src/sys/                    lock-free SPSC ring
src/proto/                  binary codec
src/transport/              USB CDC, BLE, stream batching, command handling
openocd/                    ST-Link runner config
```

Still empty placeholders:

```
src/storage/                M4-era: SD block layer. Last, may not happen.
src/imu/                    no SwiftEEG driver - the IMU uses Zephyr's
                            in-tree LSM6DSV16X, bound in devicetree
```

Host tools:

```
tools/build.ps1             builds app or any test suite
tools/flash.ps1             mass-erase, write, verify, reset
tools/rtt.py                read the log (RTT is the only log path)
tools/verify.py             hardware acceptance checks, pass/fail
tools/swifteeg_scope.py     live viewer with a test-signal toggle
tools/proto_ref.py          protocol oracle, shared with the host tools
tools/dsp_ref.py            DSP primitive oracle + golden vectors
tools/pipeline_ref.py       whole-chain oracle + golden vectors
```

Tests do not mirror `src/`. The ringbuf suite lives inside `tests/dsp`, and
`tests/ringbuf` is an empty directory - ignore it.

```
tests/proto/                 9 tests
tests/dsp/                  17 tests  (dsp + ringbuf suites)
tests/timebase/              8 tests
tests/pipeline/              8 tests  (whole-chain golden vectors)
```

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

### 5.6 The debugger stops BLE

Halting the core stops advertising. Every `openocd ... halt` — including the
default `tools/rtt.py` — freezes the radio, so a phone scanning at that
moment sees nothing. If BLE looks dead, reset the board, detach the debugger
entirely, and scan again before suspecting the firmware.

---

## 6. Milestones

Revised **2026-09-09**. The original plan was firmware-only, with host GUIs
deferred to a separate plan entirely. That changed on purpose: a host
application is now the instrument used to find the right DSP settings, and
those settings are then pushed down into the firmware. The end state is
unchanged - all processing on the device - but the route there now goes
through a PC application, because filter choices are worth *seeing* before
they are committed to firmware.

- **M1 — bring-up. DONE.** Board port, RTT, USB + BLE up, sensors
  identified, LEDs blinking.
- **M2 — acquisition + DSP. DONE.** PPI-latched DRDY timestamps, SPIM3 DMA,
  ring buffer, DSP chain, binary protocol over USB.
- **M3 — validation. Measurable parts DONE.** Golden vectors against the
  Python reference, injected-signal check, noise floor, timing. Outstanding:
  alpha blocking on a real head, and the impedance sweep.

### M4 — full wireless control  (current)

**Streaming and rate control work over BLE.** Measured to a Windows host:

```
 250 SPS ->  250.5 SPS actual,   8.9 kB/s, 0 bad frames, 0 sequence gaps
 500 SPS ->  500.1 SPS actual,  17.7 kB/s, 0 bad frames, 0 sequence gaps
1000 SPS -> 1002.4 SPS actual,  35.6 kB/s, 0 bad frames, 0 sequence gaps
```

Both links carry byte-identical frames and both are fed at once, so
unplugging USB mid-session does not interrupt BLE. Batches are six samples,
sized so a frame fits one notification at a 247-byte MTU - a notification
over the MTU is dropped by the stack without complaint.

1 kSPS is the ceiling worth having over BLE. 16 kSPS is 432 kB/s and stays
USB-only.

Still to do here:

- **Bias drive (DRL).** Off today. Required before electrodes on a head mean
  anything - see section 1.2.
- Per-channel gain, mux, enable and lead-off, rather than all-or-nothing
- SRB2 routing, notch frequency 50/60, packed int24 encoding
- 30-minute soak with the USB cable out

### Three mistakes worth keeping

**Verbose logging is not free.** The USB stack logs every packet at INFO,
and `LOG_MODE_IMMEDIATE` formats on the calling thread. Shrinking the batch
from 16 samples to 6 tripled the write rate, and the extra logging pushed
AFE register access past its timeout - register reads started failing with
no change to the AFE code at all. Those modules are now at error level.

**Do not do slow work on the Bluetooth thread.** Handling commands inline in
the GATT write callback was fine until one of them restarted the pipeline,
which stops a thread and reconfigures the AFE and takes most of a second.
That starves the link layer and the central drops the connection. Commands
are queued to the command thread now, both links handled the same way.

**The PPI task slot is attached once, not per start.** Restarting
acquisition re-attached the SPI start task to a channel that already had it,
which returns `-EBUSY` and failed every rate change. The attach is now
idempotent.

### M5 — Windows application

The instrument for finding the right DSP settings, and the thing that gets
worn-headset data on screen.

- Controls for every device setting M4 exposes
- Host-side DSP chain, adjustable live: mains notch (50/60), drift removal,
  a re-referencing stage so channels sit on a common zero rather than
  wandering, and a configurable band-pass
- Live plot with real units and a stable Y axis
- Records raw to disk, so a session can be re-analysed with different
  settings afterwards

**Accept:** headset on, USB unplugged, clean traces on screen with alpha
visible on eyes-closed.

### M6 — push the validated chain into the firmware

The settings found in M5 become the device's own, which is what the original
plan always called for.

The mechanism already exists in the design: the DSP chain's last stage is a
**host-programmable biquad cascade**. The host uploads coefficients; firmware
runs them. Nothing needs redesigning - the host application becomes the tool
that designs the coefficients it then uploads.

**Accept:** with the host chain bypassed, on-device output matches what the
host chain produced from the same raw input, within tolerance.

### M7 — native platform GUIs.  Deferred.

The user's own plan, later. Not blocked by anything here: the protocol is
transport-agnostic and documented, so any platform can speak it.

### SD card — last, may not happen.

---

## 7. DSP design principle

Firmware does only what *must* happen on-chip - DC removal for numeric
headroom, mains notch, anti-alias decimation, IMU artifact removal - and
exposes everything else as a **host-programmable biquad cascade**. No
filtering opinion is baked in.

Total group delay of the active configuration is computed and reported, so
the host can correct sample timestamps exactly.

### 7.1 Getting a signal worth trusting

Two things need separating, because they pull in opposite directions.

**What makes a trace look clean is not what makes it correct.** A 1 Hz
high-pass makes EEG look tidy on screen and flattens the wandering baseline
completely. It also distorts exactly the slow components a P300 speller
depends on - this is well established in the ERP methods literature, and it
is why the original plan refused to bake in a fixed high-pass above ~0.1 Hz.

So the chain is split:

- **On the device, minimal and reversible.** DC removal at ~0.08 Hz, which
  exists to fit the signal in a float32 mantissa rather than to shape the
  band, plus the mains notch. What is recorded stays close to what the
  electrodes saw.
- **On the host, whatever the task needs.** View it through a 1 Hz high-pass
  if that is what makes it readable; analyse the same recording through
  0.1 Hz when the slow components matter. Because the recording is not
  already high-passed, both are still possible.

### 7.2 Channels wandering on the Y axis

Three separate causes, worth telling apart rather than filtering harder:

1. **Electrode half-cell offset** - a DC level per electrode, up to tens of
   millivolts, differing between channels. This is the DC removal's job and
   it already handles it.
2. **Slow drift** - offsets moving as gel settles and skin impedance
   changes, over seconds to minutes. A gentle high-pass handles this; the
   corner is the trade-off above.
3. **Common-mode movement** - all channels rising and falling together,
   usually mains or body potential. Filtering per channel does not fix this
   because it is not per channel. Two things do: the **bias drive**, which
   cancels it at the body before the ADC sees it, and a **common average
   reference**, which subtracts the mean across channels afterwards.

The instinct that channels should sit on a shared zero is right, and CAR is
the stage that does it. It is already stage 5 of the design.

**The largest single factor is none of the above.** It is electrode contact
impedance. Below about 5 kOhm, with the bias drive running, EEG turns up
looking like EEG. Above about 20 kOhm no filter chain rescues it. This is
why the impedance measurement is an acceptance item and not a nicety.

### 7.3 The route from host to device

M5 tunes the chain on the host, where a change is visible in a second. M6
moves the settled chain onto the device, where it belongs.

That transfer needs no new architecture. The chain's last stage is a
programmable biquad cascade: the host designs coefficients and uploads them.
The host application becomes the tool that designs what it then installs, and
the acceptance test for M6 is that the device reproduces, from the same raw
input, what the host chain produced.
