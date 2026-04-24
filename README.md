# AS5048B Magnetic Encoder
This is a driver library for the AS5048B magnetic rotary encoder.

## Features
- Read the angle
- Read the magnitude of the magnetic field
- Read diagnostics: offest compensation finished, cordic overflow, magnetic field too strong, magnetic field too weak
- One-Time-Program the I2C address
- One-Time-Program the zero position

## Development
The relevant parts of the datasheet are summarized in the [datasheet_i2c.md](datasheet_i2c.md) file.