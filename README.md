# AS5048B Magnetic Encoder
This is a driver library for the AS5048B magnetic rotary encoder.

## Features
- Read the angle
- Read the magnitude of the CORDIC algorithm
- Read diagnostics: offest compensation finished, cordic overflow, magnetic field too strong, magnetic field too weak
- One-Time-Program the I2C address (minimal testing on real hardware)
- One-Time-Program the zero position (untested on real hardware)

## Development
The relevant parts of the datasheet are summarized in the [datasheet_i2c.md](datasheet_i2c.md) file.

## TODO
- Add examples that comprehensively cover all features:
    - Read the angle, magnitude, and diagnostics from 2 sensors
    - One-Time-Program the I2C address of one sensor
    - One-Time-Program the zero position of one sensor