# SwiftEEG — Zephyr RTOS Firmware

8-channel wireless EEG acquisition on nRF52840 + ADS1299, built on the
nRF Connect SDK. Designed as a raw BCI tool: full hardware control, on-chip
DSP, precise timestamps, and a transport-agnostic binary API.

> **Status: M1 (bring-up), in progress.** Not yet built or flashed — the NCS
> toolchain is not installed on the dev machine yet. Everything here is
> authored but **unverified against a compiler**.

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

Requires the nRF Connect SDK. Pin the version in `west.yml` before building —
the value there is currently a **placeholder**.

```
west build -b swifteeg
```

---

## 5. Flashing and logs

ST-Link V2 over SWD. There is no spare UART, so **RTT is the only log path**.

```
west flash
```

or directly:

```
openocd -f openocd/swifteeg.cfg -c "program build/zephyr/zephyr.hex verify reset exit"
```

---

## 6. Milestones

- **M1 — bring-up.** Board port, RTT logging, USB + BLE up, AFE and IMU IDs
  verified over SPI, both LEDs blinking.
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
