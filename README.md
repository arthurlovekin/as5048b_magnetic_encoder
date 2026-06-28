# AS5048B Magnetic Encoder
This is a driver library for the AS5048B, a 14-bit magnetic rotary position sensor produced by AMS OSRAM. This library only supports the I2C interface.

## Features
- [x] Read the angle (raw 14-bit and degrees)
- [x] Read the magnitude of the CORDIC algorithm
- [x] Read diagnostics: offset compensation finished, cordic overflow, magnetic field too strong, magnetic field too weak
- [x] One-Time-Program the I2C address
- [x] One-Time-Program the zero position
- [x] `defmt` support (optional `defmt` feature)
- [x] tested with examples on ESP32-C6 DevKitC microcontroller

## Development
The relevant parts of the datasheet are summarized in the [datasheet_i2c.md](datasheet_i2c.md) file.

## TODO
- Add example and tests for:
    - One-Time-Program the zero position of one sensor