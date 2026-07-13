# AS5048B Magnetic Encoder
This is a driver library for the AS5048B, a 14-bit magnetic rotary position sensor produced by AMS OSRAM. This library only supports the I2C interface.

## Features
- [x] Read the angle (raw 14-bit and degrees)
- [x] Read the magnitude of the CORDIC algorithm
- [x] Read diagnostics: offset compensation finished, cordic overflow, magnetic field too strong, magnetic field too weak
- [x] One-Time-Program the I2C address
- [x] One-Time-Program the zero position
- [x] `defmt` support (optional `defmt` feature)
- [x] tested with examples on ESP32-C6 DevKitC and Adafruit KB2040 microcontrollers

## Contributing
Pull requests for new examples, bug reports and feature ideas are welcome.

The relevant parts of the datasheet are summarized in [datasheet_i2c.md](datasheet_i2c.md), but please consult the [official AMS datasheet](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf) before making changes.

Before submitting a pull request, please ensure that:
1. You've added or updated unit tests (mocked I²C via `embedded-hal-mock`) for driver logic. All tests should pass with `cargo test`.
2. You've created a minimal example that tests the changes on the hardware you're working on. Hardware examples live under `examples/<microcontroller_name>`, e.g. `examples/esp32c6_devkitc`). All examples should "just work" when a user runs `cargo run --release --bin <example_name>` (assuming they have the necessary hardware).
3. You've added or updated documentation in the code and README.md file

By contributing, you agree that your contributions are licensed under the MIT license (see [LICENSE-MIT](LICENSE-MIT)).
