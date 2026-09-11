# SwiftEEG — Zephyr RTOS Firmware

8-channel wireless EEG acquisition on nRF52840 + ADS1299, built on the
nRF Connect SDK. Designed as a raw BCI tool: full hardware control, on-chip
DSP, precise timestamps, and a transport-agnostic binary API.

> **Status: M1-M4 done, M5 Windows app working, motion sensor streaming on
> the EEG's clock. Next: recordings of a moving subject, then motion-artifact
> cleanup.**
>
> | Check | Result |
> |---|---|
> | Board port, flashing, RTT | pass, both board builds |
> | ADS1299 / IMU over SPI | `ID 0x3e` / `WHO_AM_I 0x70` |
> | USB CDC ACM | COM4 |
> | BLE streaming + control | 250 / 500 / 1000 SPS, 0 sequence gaps |
> | BLE soak, 30 min at 1 kSPS | 1,802,484 samples, 0 gaps, 0 disconnects |
> | DRDY timestamp jitter | **1 us**, hardware latched |
> | IMU on the EEG clock | 240 / 480 / 960 Hz, 0 gaps, **sample times within 1 us** |
> | Shorted-input noise | **130-149 nV RMS** (datasheet ~140) |
> | Test-signal amplitude | **0.17-0.22 % error**, 0.05 % channel spread |
> | Golden vectors vs reference | worst 3.4 nV over 512 frames x 8 ch |
> | Unit tests on target | 42/42 |
>
> Everything streams and is controllable over Bluetooth: eight EEG channels
> and six motion axes, on one clock.

---

## 0. Where we are  (read this first)

**M1-M4 are done and M5 works.** Samples come off the ADS1299 by DMA, run
through the DSP chain, and stream to a PC over Bluetooth or USB alongside the
motion sensor's, where the Windows application plots and filters them.

### Verified on hardware

```
test signal    CH1-8  3.742-3.744 mV p-p, 0.17-0.22 % off, 0.05 % spread
               0.98 Hz on every channel
noise floor    CH1-8  130-149 nV RMS, inputs shorted, gain 24
timing         250.35 SPS, 0.4 us inter-batch jitter, 0 sequence gaps
protocol       0 CRC failures
golden vectors worst 3.4 nV vs the Python reference, 512 frames x 8 ch
motion         240 / 480 / 960 Hz over BLE with EEG streaming: 0 gaps, sample
               times within 1.0 us of a straight line, gravity 0.984 g,
               sensor clock -1.84 % measured (-1.82 % by its own estimate)
```

`python tools/verify.py` re-runs the EEG checks and compares each number
against the datasheet or the protocol definition.

**The noise floor is the result that matters most.** TI quotes ~140 nV RMS
input-referred at gain 24 and 250 SPS; the eight channels measure 130-149.
The analog front end, the +/-2.5 V rails, the reference and the layout are
all sound. A test signal cannot tell you this - a large injected signal rides
straight over a noisy front end.

### What works

- DRDY drives everything in hardware: one PPI channel timestamps the sample
  and starts its SPI transfer, so the CPU wakes with 27 bytes already in RAM
- Lock-free ring from the interrupt to a DSP thread; 0 drops up to 1 kSPS
- Chain: 24-bit decode, integer DC removal, microvolt scaling, mains notch
- Binary protocol, byte-identical over BLE and USB, 6 samples a frame,
  packed 24-bit by default
- Every device control over either link, answered while streaming: rate
  250/500/1000 SPS, per-channel gain / input / enable / SRB2, bias drive,
  lead-off, 50/60 Hz notch, test signal, register read, config readback
- Motion: LSM6DSV16X accelerometer and gyroscope at 60-960 Hz, each batch
  anchored on TIMER1 - the EEG's clock - by a PPI capture of INT2
- `tools/swifteeg_app.py`, the Windows application (section 6, M5)

### What does not work yet

- **No motion-artifact cleanup yet.** The motion stream is in place; the
  method needs real recordings of a moving subject to be built against.
- **Not yet checked on a head:** alpha with eyes closed at Oz/P3/P4. Blinks
  have been seen on the frontal channels.
- The on-device filter chain is not yet matched to the host chain (M6).
- No impedance measurement - lead-off detection only.
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
   right leg. It is on: `CONFIG3.PD_BIAS` set, and `BIAS_SENSP` and
   `BIAS_SENSN` both `0xFF`.

   The DRL amplifier drives the body to cancel common-mode, mostly mains.
   `BIASREF` is grounded on this board, which is correct - mid-supply is 0 V
   between the +/-2.5 V rails - so `CONFIG3.BIASREF_INT` stays 0.

   `BIAS_SENSN` has to include the negative inputs. SRB1 sits on them, and
   with it outside the loop the amplifier oscillated: 124 mV of common-mode
   and 28 % of samples clipped, against 2.4 mV and none with it inside.

4. **`CLKSEL` is tied high** — the AFE runs its own internal oscillator, spec'd
   ±2 %. Its sample clock is therefore *asynchronous* to the MCU, so true
   sample rate must be measured against the 32.768 kHz crystal, not assumed.
5. **No battery sense.** `VDDH` sits on the regulated rail, so the `VDDHDIV5`
   trick reads the LDO, not the cell. A divider jumper to a free AIN pin is
   planned.
6. **No user button.** SW1 is wired to nRESET only; all control is over BLE/USB.
7. **SD is SPI-only** — 4-bit SDIO is not wired.
8. **Only IMU INT2 is routed**; INT1 is not connected. Confirmed with the
   hardware owner, 2026-09-11.

### 1.3 Montage (SRB1 referential)

Every channel's negative input is tied to **SRB1** internally
(`MISC1.SRB1 = 1`), so all eight are measured against one reference.

| Wire | Goes to | Purpose |
|---|---|---|
| **SRB1** | **right mastoid** | the reference every channel is measured against |
| **BIAS** (BIASOUT) | **left mastoid** | driven right leg - cancels common-mode |

Mastoids are the conventional choice: close to the head, electrically quiet,
and far enough from the scalp sites to carry little EEG of their own.

Putting the reference on one side and the bias on the other is deliberate.
The bias amplifier drives a correction signal into the body, and keeping it
off the reference electrode avoids feeding that correction straight back into
what every channel is measured against.

| Ch | Site | Use | Ch | Site | Use |
|---|---|---|---|---|---|
| 1 | C4 | motor imagery, right | 5 | AFz | frontal / EOG |
| 2 | P4 | P300 | 6 | F3 | frontal |
| 3 | F4 | frontal | 7 | C3 | motor imagery, left |
| 4 | Oz | SSVEP | 8 | P3 | P300 |

The board, and with it the IMU, is worn on the head, so the motion sensor
measures head movement.

## 2. Repo layout

Compiled into the application:

```
boards/shreyash/swifteeg/   out-of-tree board port (HWMv2)
dts/bindings/               ti,ads1299 binding
src/board/                  LEDs, VDD self-test
src/afe/                    ADS1299 driver: probe, config, DMA streaming
src/imu/                    LSM6DSV16X driver: FIFO, INT2 watermark on TIMER1
src/timebase/               1 MHz TIMER1, 64-bit extension, PPI capture tasks
src/pipeline/               capture (GPIOTE+PPI), chain (DSP), pipeline (thread)
src/dsp/                    DC removal, biquads, filter design
src/sys/                    lock-free SPSC ring
src/proto/                  binary codec
src/transport/              USB CDC, BLE, stream batching, command handling
openocd/                    ST-Link runner config
```

Still an empty placeholder:

```
src/storage/                M4-era: SD block layer. Last, may not happen.
```

Host tools:

```
tools/build.ps1             builds app or any test suite
tools/flash.ps1             mass-erase, write, verify, reset
tools/rtt.py                read the log (RTT is the only log path)
tools/verify.py             hardware acceptance checks, pass/fail
tools/swifteeg_app.py       the Windows application (M5)
tools/swifteeg_link.py      USB and BLE links behind one interface
tools/eeg_dsp.py            host filter chain
tools/soak.py               long BLE run, counts sequence gaps
tools/swifteeg_scope.py     early USB-only viewer, superseded by the app
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

`build.ps1` puts the NCS toolchain first on `PATH` for the session it runs
in, and that toolchain brings its own Python - one without `bleak`. Run the
host tools from a fresh shell, not the one that just built.

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

### M4 — full wireless control  (done)

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

Also done: bias drive (DRL), per-channel gain / input / enable / SRB2,
lead-off detection, 50/60 Hz notch, packed 24-bit encoding, and the
30-minute soak with the USB cable out (see M5). The IMU, left over from the
original M2, now streams too - see the motion section below.

### The device stays responsive while streaming

Measured over BLE at 1 kSPS, commands issued while data was flowing:

```
ping           18 ms      set channels   13-30 ms
read register  23 ms      set bias       22 ms
set notch      18 ms      set rate       restarts acquisition
7 of 7 answered.  Streaming through it: 1033 SPS, 0 bad, 0 sequence gaps
```

Nothing was dropped while the AFE was reconfigured mid-stream. Three things
make that true:

- **Commands never run on a thread that cannot afford to block.** The GATT
  write callback only queues bytes; a low-priority thread does the work.
- **The command thread sleeps on a semaphore**, woken by whichever link
  received bytes, rather than polling. Polling every 20 ms was costing
  90 ms round trips on its own.
- **The DSP thread outranks the command thread**, so reconfiguration can
  never delay a sample.

A short connection interval is requested on connect (7.5-15 ms). It is
advisory - the central decides - and iOS will refuse anything under 15 ms.

### Mistakes worth keeping

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

**The Bluetooth receive stack was too small, intermittently.** "BT RX WQ"
runs every connection, subscription and command-write callback on a
1024-byte default stack. Immediate-mode logging formats on it, and with FPU
sharing an interrupt landing mid-callback stacks the FPU registers there
too. It worked for weeks, then faulted with a stack overflow the moment a
central connected. `CONFIG_BT_RX_STACK_SIZE` is now 4096.

**More than one thread writes to USB.** The DSP thread's samples, the IMU
thread's motion frames and the command thread's replies all go into one
ring buffer that is only safe for a single writer. Writers now take a lock,
and a frame goes in whole or not at all.

### M5 — Windows application  (built)

```bash
python tools/swifteeg_app.py
```

Connects over Bluetooth or USB, exposes every device setting, runs the filter
chain on the host, plots EEG and motion, and records raw EEG and motion to
CSV.

Three files:

- `tools/swifteeg_link.py` - USB and BLE behind one interface. bleak is
  async and tkinter is not, so BLE runs its event loop on its own thread and
  the two sides talk over queues; nothing in the UI awaits anything.
- `tools/eeg_dsp.py` - the filter chain, self-tested by property.
- `tools/swifteeg_app.py` - the window.

**30-minute wireless soak passed** before the app was built: 1,802,484
samples at 1001.0 SPS over BLE, 0 sequence gaps, 0 CRC failures, 0
disconnects. `python tools/soak.py --minutes 30 --sps 1000`.

#### Why the app reads the device's configuration on connect

It asks `CMD_GET_CONFIG` the moment the link comes up and moves its own
controls to match, rather than assuming its defaults are what the device is
doing.

That is not tidiness. The first version asked on a 1.2-second timer, which
expires long before a Bluetooth scan finishes, so the request was lost and
the app kept its defaults. It believed 250 SPS while the device ran at 1000,
designed every filter for the wrong sample rate, and produced 3390 uV of
peak-to-peak noise on a channel that should have been quiet. With the
configuration actually read, the same channel sits at 0.07 uV mean and
7.4 uV peak to peak.

A filter designed for the wrong sample rate does not fail loudly. It just
does not work.

#### What the host chain does

High-pass, notch (with its harmonic), common average reference, low-pass -
in that order. The high-pass first so nothing downstream works on a drifting
signal; the re-reference after it so per-channel offsets do not contaminate
the average.

The common average is what pulls the traces onto a shared zero. It subtracts
what every channel has in common, which is where mains and body potential
live - and those are not reachable by any per-channel filter, because they
are not per channel.

Defaults are 0.5 Hz, 45 Hz, 50 Hz notch, CAR on. The 0.5 Hz high-pass is a
viewing choice: it makes a trace look clean and it distorts slow ERP
components. The device still records at 0.08 Hz, so the recording keeps what
the display throws away.

#### Three things that made a low drift cut look unusable

- **The chain restarted itself every few seconds.** Re-aiming the notch at
  the measured mains frequency rebuilt and reset every stage, and each reset
  put every electrode's DC offset back through the high-pass. In simulation:
  13.8 mV RMS of error at a 0.1 Hz corner, 4.6 mV at 1 Hz. The notch now
  retunes in place, keeping its state.
- **Filters started from zero.** The first sample was a step the size of the
  electrode offset, which a 0.1 Hz high-pass takes most of a minute to
  forget. Every stage now primes on its first sample.
- **A bad electrode was part of everyone's reference.** One floating
  electrode put 32 uV RMS on every other channel through the common average,
  in simulation. Each channel now has an "in average" tick, and a channel
  pinned near the converter limit for 2 s leaves the average on its own -
  still drawn in colour, labelled "near limit" or "CLIPPING".

Recordings also had wrong timestamps: every row of a delivery was stamped
from its first batch, a microsecond apart, tens of milliseconds out. Each
row now takes its own batch's hardware timestamp plus its place in it.

#### The plot is paced by the monitor

It redraws once per screen refresh - measured 142-144 fps on a 144 Hz
monitor - by blocking in `DwmFlush` with the Windows timer at 1 ms. Canvas
items are made once and moved, samples are reduced to about a point per
pixel with min/max bins pinned to sample numbers, and the display trails the
newest sample by 50-300 ms so Bluetooth's bursts become steady scrolling.
The stats panel shows the frame rate it is actually getting.

### Motion: IMU streaming, then motion-artifact cleanup  (current)

The requirement: a signal that stays usable on a moving, walking subject.
The original plan always had this - DSP stage 4, IMU-referenced artifact
removal, and an R&D phase to find the method.

1. **IMU streaming. DONE**, tested on the board over BLE (below).
2. **In the app. DONE.** Motion lanes under the EEG, placed by device time;
   rate and range controls; motion recorded to `<name>_motion.csv` beside
   the EEG file, on the same clock.
3. **Recordings, headset on.** A still baseline, head turns and nods,
   walking, chewing.
4. **Cleanup built on the host** and judged on those recordings. Candidates:
   IMU-referenced adaptive filtering, gait-locked template subtraction
   (Gwin et al., 2010), and subspace methods such as ASR. Chosen by
   measurement - motion-locked power removed, blinks and alpha intact.
5. **Onto the device in M6**, as stage 4 of the chain.

It will reduce motion artifact, not remove it. Dry electrodes shift on the
skin in ways a head-mounted IMU only partly sees.

#### How a motion sample gets its time

A canceller fed a motion reference at an unknown offset from the EEG cannot
cancel, so motion is timed on TIMER1, the clock DRDY is latched on:

- the sensor batches samples in its FIFO and raises INT2 at a watermark;
- PPI latches that edge into TIMER1 capture channel 3, in hardware;
- the watermark falls on a known sample of the batch, so each sample is
  placed relative to it at the sensor's sample period;
- that period is measured edge to edge against the crystal. The sensor's own
  clock is not trusted: on this board it runs 1.84 % slow, which at a
  nominal 240 Hz would put the stream 74 ms adrift every 4 seconds.

Measured with EEG streaming at 250 SPS:

```
240 Hz  +/-8 g    0 gaps, times within 0.9 us of a straight line
480 Hz  +/-4 g    0 gaps, within 1.0 us
960 Hz  +/-16 g   0 gaps, within 1.0 us
EEG               0 gaps at every IMU rate; board still: 0.984 g
```

The first build read the FIFO one word per SPI transfer. At 480 Hz that was
slow enough for new words to push the FIFO back over the watermark while it
was being emptied, and that edge was then taken for the next batch's own:
samples up to 25 ms out. The FIFO is now read in one transfer, an edge that
lands during a read is discarded, and a batch without an edge of its own is
placed from the last good one at the measured period.

#### On the wire

A new frame type, `0x05`, device to host. Little-endian payload:

```
u64 ts_us         TIMER1 time of the first sample
u32 seq           sensor sample index of the first sample
u32 period_us_q8  sample period, 1/256 us
u16 accel_g       full scale; one count = accel_g / 32768 g
u16 gyro_dps      full scale; one count = gyro_dps / 32768 deg/s
u8  axes          6: accel x y z, then gyro x y z, int16 each
u8  flags         0x01 timed from the poll (no edge yet), 0x02 FIFO overrun
u16 count         samples that follow, at most 17 - one BLE notification
```

`CMD_SET_IMU` (`0x0F`): enable, rate in Hz (60-960), accel range in g (2-16),
gyro range in deg/s (125-4000). `CMD_GET_CONFIG` appends the sensor's state
after the channel settings: flags (bit 0 on, bit 1 fitted), rate and ranges.
Motion frames flow while the stream is enabled, like the EEG's.

Zephyr's own LSM6DSV16X driver is switched off in `prj.conf`; `src/imu` owns
the chip.

### M6 — push the validated chain into the firmware

The settings found in M5, and the motion cleanup, become the device's own,
which is what the original plan always called for.

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
