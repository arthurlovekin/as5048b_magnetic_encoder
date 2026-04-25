# ESP32-C6 examples

Examples that exercise the `as5048b_magnetic_encoder` driver on an ESP32-C6 dev board. Each example is a separate binary in `src/bin/`.

## Prerequisites

- ESP32-C6 dev board
- `riscv32imac-unknown-none-elf` Rust target installed
  (`rustup target add riscv32imac-unknown-none-elf`)
- [`espflash`](https://github.com/esp-rs/espflash) on `PATH`
  (`cargo install espflash`)

The configured runner (`.cargo/config.toml`) is
`espflash flash --monitor --chip esp32c6 --log-format defmt`, so `cargo run`
flashes the board and streams `defmt` logs over USB.

## Wiring

All examples use:

| Signal | GPIO |
| ------ | ---- |
| SDA    | 3    |
| SCL    | 2    |

Pull-ups (typically 4.7 kΩ to 3V3) on SDA and SCL are required.

The 7-bit I²C address is set by the AS5048B's A1/A2 pins:

| A1  | A2  | Address |
| --- | --- | ------- |
| GND | GND | `0x40`  |
| GND | VDD | `0x41`  |
| VDD | GND | `0x42`  |
| VDD | VDD | `0x43`  |

(The OTP can change bits 2–6, but bits 0–1 always come from A1/A2.)

## Examples

### `simple` — single sensor, angle only

Reads the angle in degrees from one AS5048B at `0x40` (A1=A2=GND) every 200 ms.

```sh
cargo run --release --bin simple
```

### `two_sensors_full_read` — two sensors, full read

Reads diagnostics, raw magnitude, and angle (degrees) from two AS5048B sensors sharing the same I²C bus. Sensor A is at `0x40` (A1=A2=GND) and sensor B is at `0x41` (A1=VDD, A2=GND). The bus is shared between the two driver instances via `embedded_hal_bus::i2c::RefCellDevice`.

```sh
cargo run --release --bin two_sensors_full_read
```

To use different addresses, edit the `SENSOR_A_ADDR` / `SENSOR_B_ADDR`
constants at the top of `src/bin/two_sensors_full_read.rs`.
