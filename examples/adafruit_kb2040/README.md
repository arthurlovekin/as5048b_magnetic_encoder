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

### Reading the data
After flashing, the KB2040 will appear on the host as a serial device.

**Linux**

Find which port is used with `ls /dev/ttyACM*`, then read the data using `cat /dev/ttyACM0` (or use a serial monitor of your choice, like `screen` or `minicom`).

**Windows**

Find the COM port from PowerShell:
```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```
(or check Device Manager under "Ports (COM & LPT)").

Then read the data with [PuTTY](https://www.putty.org/): open PuTTY, set **Connection type** to `Serial`, enter the COM port (e.g. `COM3`) under **Serial line**, and set the baud rate to 115200. Click **Open** to start streaming the output.
