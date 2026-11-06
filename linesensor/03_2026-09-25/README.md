# PicoLineFollower

PlatformIO project for a **Raspberry Pi Pico (RP2040)**, opened in CLion via the
PlatformIO plugin. Ported from `lineBeta3.ino` + `main.cpp`:

- **Sensing / line math / dual-core structure** — taken wholesale from `lineBeta3.ino`
  (`processLineFast`, fixed-point `atan2`, calibration, `setup/loop` on core0 +
  `setup1/loop1` on core1).
- **LED pattern** — taken from `main.cpp`'s `__setPixelColor()` (green→white fade).
- **CAN → I2C** — the MCP2515/CAN bus is gone; the Pico is now a **transmit-only
  I2C slave** on `Wire1`. It only answers read requests — it never registers an
  `onReceive` handler, so it cannot be made to act on anything written to it.
- **Multiplexer → direct A0–A3** — the 4-bit analog mux is gone; the 4 sensors
  are read straight off `A0`–`A3`.

## Layout

```
include/
  config.h      constants: pins, I2C address, sensor count, lookup tables
  functions.h   all logic (sensor reads, line/angle math, LED patterns, I2C)
src/
  main.cpp      global state + setup()/loop()/setup1()/loop1()
```

## Opening in CLion

Install the "PlatformIO for CLion" plugin, then open this folder — CLion detects
`platformio.ini` and generates the CMake/compile-db integration automatically.
No manual CMakeLists.txt is needed.

## Pin map

| Signal        | GPIO       | Notes                     |
|---------------|------------|----------------------------|
| Sensor 0      | A0 (GP26)  | direct ADC input, no mux  |
| Sensor 1      | A1 (GP27)  | direct ADC input, no mux  |
| Sensor 2      | A2 (GP28)  | direct ADC input, no mux  |
| Sensor 3      | A3 (GP29)  | direct ADC input, no mux  |
| NeoPixel data | GP13       | 6-pixel indicator strip   |
| I2C1 SDA      | GP14       | slave data line           |
| I2C1 SCL      | GP15       | slave clock line          |

The old CAN pins and mux select pins are freed and unused.

### About the 4-sensor direction vectors

`sensVecs_fixed` in `include/config.h` assumes the 4 sensors sit 90° apart
around a circle (A0→0°, A1→90°, A2→180°, A3→270°), matching the original
"ball sensor" layout with fewer points. **If your sensors are mounted in a
straight line instead**, replace the vectors with something like
`{-3,0},{-1,0},{1,0},{3,0}` — a weighted-position average would then be more
meaningful than `atan2`.

## I2C protocol (replaces the old CAN exchange)

Pico listens as a slave at address `0x42` (`I2C_SLAVE_ADDR`), **read-only**:
it does not accept or process any data written to it — only
`Wire1.onRequest()` is registered.

- Host **reads** 3 bytes whenever it wants fresh data:

  | Byte | Meaning                    |
  |------|-----------------------------|
  | 0    | `line` (0/1, line detected) |
  | 1    | `angle` low byte            |
  | 2    | `angle` high byte           |

Calibration re-trigger (previously driven by CAN writes) has no I2C
equivalent now — `cmd` only changes via the internal `0xFF`/`0xFE` state
machine in `loop()`.

## Build

```bash
pio run
```

## Upload

```bash
pio run -t upload
```

Or drag `.pio/build/pico/firmware.uf2` onto the `RPI-RP2` mass-storage drive
that appears when the Pico is plugged in while holding BOOTSEL.
