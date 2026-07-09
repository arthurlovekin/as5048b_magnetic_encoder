### Flashing the KB2040

Flashing the KB2040 is done via UF2 (no debug probe required). First, install the RP2040 toolchain:

```bash
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs
```

To flash, press **RESET** while holding **BOOT** to get the board into bootloader mode, then from `examples/adafruit_kb2040`, run:
```bash
cargo run --release --bin simple
```   
This uses `elf2uf2-rs` to convert the `.elf` to `.uf2` and copy it to the mounted RP2040 bootloader drive.

You can also one-time-program the zero position at the current magnet pose (irreversible) by running:
```bash
cargo run --release --bin program_zero_position
```
This script runs automatically so make sure to flash another script after programming so you don't accidentally program any other boards.

### Reading the data
After flashing, the KB2040 will appear as a USB device at `/dev/ttyACM*` on Linux. You will have to find which port is used using `ls /dev/ttyACM*`. Then, read the data using `cat /dev/ttyACM0` (or use a serial monitor of you choice, like `screen` or `minicom`).
