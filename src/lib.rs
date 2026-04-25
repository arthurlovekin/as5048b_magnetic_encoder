#![cfg_attr(not(test), no_std)]

//! Driver for the AMS AS5048B magnetic rotary encoder over I²C.
//!
//! Built on top of the [`embedded-hal`] v1.0 traits so it works with any HAL
//! that provides [`embedded_hal::i2c::I2c`] and [`embedded_hal::delay::DelayNs`].
//! See `datasheet_i2c.md` for the parts of the datasheet this driver relies on.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

// ----- Register map (subset; see datasheet_i2c.md) -----
const REG_PROGRAMMING_CONTROL: u8 = 0x03;
const REG_I2C_ADDRESS: u8 = 0x15;
// TODO: implement OTP zero-position programming via 0x16 / 0x17.
// const REG_OTP_ZERO_HI: u8 = 0x16;
// const REG_OTP_ZERO_LO: u8 = 0x17;
const REG_DIAGNOSTICS: u8 = 0xFB;
const REG_MAGNITUDE_MSB: u8 = 0xFC;
const REG_ANGLE_MSB: u8 = 0xFE;

// Programming-control byte values used during the OTP burn sequence.
const OTP_CTRL_PROGRAMMING_MODE: u8 = 0xFD;
const OTP_CTRL_BURN: u8 = 0x08;
const OTP_CTRL_DISABLE: u8 = 0x00;

/// Default 7-bit I²C address with A1/A2 strapped low and an unprogrammed OTP.
pub const DEFAULT_ADDRESS: u8 = 0x40;

/// Maximum raw value of the 14-bit angle / magnitude registers.
pub const RAW_MAX: u16 = 16_383;

/// Decoded diagnostic flags from register `0xFB`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Diagnostics {
    /// Raw register byte (bits 7–4 are unused per datasheet).
    pub raw: u8,
}

impl Diagnostics {
    /// OCF — high once the offset-compensation algorithm has completed (after power-up).
    #[inline]
    pub const fn offset_compensation_finished(self) -> bool {
        (self.raw & 0b0001) != 0
    }
    /// COF — CORDIC overflow. Angle and magnitude are invalid when set.
    #[inline]
    pub const fn cordic_overflow(self) -> bool {
        (self.raw & 0b0010) != 0
    }
    /// Comp Low — magnetic field is high (too-strong magnet).
    #[inline]
    pub const fn magnetic_field_too_strong(self) -> bool {
        (self.raw & 0b0100) != 0
    }
    /// Comp High — magnetic field is weak.
    #[inline]
    pub const fn magnetic_field_too_weak(self) -> bool {
        (self.raw & 0b1000) != 0
    }
}

/// Errors from the OTP I²C-address programming sequence.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProgramError<E> {
    /// `new_address` differs from `old_address` in bits 0–1 (set by pins A1/A2, not OTP).
    HardwareBitsDiffer { old: u8, new: u8 },
    /// 7-bit addresses must have bit 7 clear.
    AddressOutOf7BitRange(u8),
    /// Reserved I²C low addresses (`0x00..=0x07`).
    AddressReserved,
    /// `old_address == new_address` — nothing to program.
    OldNewAddressesIdentical,
    I2cPreVerify(E),
    I2cWriteStep1(E),
    I2cWriteStep2(E),
    I2cWriteStep3(E),
    I2cWriteStep4(E),
    I2cPostVerify(E),
}

/// AS5048B driver bound to a single 7-bit I²C address.
pub struct As5048b<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> As5048b<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a driver bound to `address`. No I²C traffic is generated here.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Release the underlying I²C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Currently-bound 7-bit I²C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Raw 14-bit angle (`0..=16_383`) from registers `0xFE`/`0xFF`.
    pub fn read_angle_raw(&mut self) -> Result<u16, E> {
        read_u14(&mut self.i2c, self.address, REG_ANGLE_MSB)
    }

    /// Angle in degrees, `0.0..360.0`.
    pub fn read_angle_degrees(&mut self) -> Result<f32, E> {
        Ok(raw_to_degrees(self.read_angle_raw()?))
    }

    /// Raw 14-bit CORDIC magnitude from registers `0xFC`/`0xFD`.
    pub fn read_magnitude_raw(&mut self) -> Result<u16, E> {
        read_u14(&mut self.i2c, self.address, REG_MAGNITUDE_MSB)
    }

    /// Diagnostic byte from register `0xFB`.
    pub fn read_diagnostics(&mut self) -> Result<Diagnostics, E> {
        let mut b = [0u8; 1];
        self.i2c
            .write_read(self.address, &[REG_DIAGNOSTICS], &mut b)?;
        Ok(Diagnostics { raw: b[0] })
    }

    /// Burn a new 7-bit I²C address into OTP. **Irreversible.**
    ///
    /// On success, `self.address()` returns `new_address`. After step 1 the
    /// chip already responds on `new_address`, so on later-step failure the
    /// driver is left bound to `new_address` for diagnostic reads.
    /// `new_address` must agree with the current address in bits 0–1 (those
    /// are pin-strapped via A1/A2 and cannot be changed by OTP).
    pub fn program_i2c_address<D: DelayNs>(
        &mut self,
        delay: &mut D,
        new_address: u8,
    ) -> Result<(), ProgramError<E>> {
        validate_new_address(self.address, new_address)?;

        // Pre-verify the old address responds.
        self.read_angle_raw().map_err(ProgramError::I2cPreVerify)?;

        let otp_data = otp_five_bits_for_7bit_address(new_address);
        let old_address = self.address;

        // 1. Load new address into 0x15 while still talking to old_address.
        self.i2c
            .write(old_address, &[REG_I2C_ADDRESS, otp_data])
            .map_err(ProgramError::I2cWriteStep1)?;
        delay.delay_ms(2);

        // From here on, the chip listens on new_address.
        self.address = new_address;

        // 2. Enable programming mode.
        self.i2c
            .write(self.address, &[REG_PROGRAMMING_CONTROL, OTP_CTRL_PROGRAMMING_MODE])
            .map_err(ProgramError::I2cWriteStep2)?;
        delay.delay_ms(2);

        // 3. Burn the fuse.
        self.i2c
            .write(self.address, &[REG_PROGRAMMING_CONTROL, OTP_CTRL_BURN])
            .map_err(ProgramError::I2cWriteStep3)?;
        delay.delay_ms(30);

        // 4. Exit programming mode.
        self.i2c
            .write(self.address, &[REG_PROGRAMMING_CONTROL, OTP_CTRL_DISABLE])
            .map_err(ProgramError::I2cWriteStep4)?;
        delay.delay_ms(2);

        // Post-verify on the new address.
        self.read_angle_raw().map_err(ProgramError::I2cPostVerify)?;
        Ok(())
    }
}

/// Convert a raw 14-bit angle reading to degrees in `0.0..360.0`.
pub fn raw_to_degrees(raw: u16) -> f32 {
    (raw as f32) / (RAW_MAX as f32) * 360.0
}

/// Compute the 5-bit OTP value for register `0x15` that produces the desired
/// 7-bit I²C address (bit 4 of the address is inverted relative to OTP bit 4;
/// bits 0–1 come from pins A1/A2 and are ignored here).
fn otp_five_bits_for_7bit_address(addr_7: u8) -> u8 {
    ((addr_7 >> 2) ^ 0x10) & 0x1F
}

fn validate_new_address<E>(old: u8, new: u8) -> Result<(), ProgramError<E>> {
    if old == new {
        return Err(ProgramError::OldNewAddressesIdentical);
    }
    if new & 0x80 != 0 {
        return Err(ProgramError::AddressOutOf7BitRange(new));
    }
    if new < 0x08 {
        return Err(ProgramError::AddressReserved);
    }
    if (old & 0x03) != (new & 0x03) {
        return Err(ProgramError::HardwareBitsDiffer { old, new });
    }
    Ok(())
}

fn read_u14<I2C, E>(i2c: &mut I2C, address: u8, msb_reg: u8) -> Result<u16, E>
where
    I2C: I2c<Error = E>,
{
    let mut buf = [0u8; 2];
    i2c.write_read(address, &[msb_reg], &mut buf)?;
    Ok(((buf[0] as u16) << 6) | ((buf[1] as u16) & 0x3F))
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};

    #[test]
    fn otp_five_bits_matches_datasheet_examples() {
        // Examples taken from `program.rs` doc comments.
        assert_eq!(otp_five_bits_for_7bit_address(0x44), 0x01);
        assert_eq!(otp_five_bits_for_7bit_address(0x47), 0x01);
        assert_eq!(otp_five_bits_for_7bit_address(0x20), 0x18);
        assert_eq!(otp_five_bits_for_7bit_address(0x7C), 0x0F);
        assert_eq!(otp_five_bits_for_7bit_address(0x48), 0x02);
        assert_eq!(otp_five_bits_for_7bit_address(0x4C), 0x03);
        assert_eq!(otp_five_bits_for_7bit_address(0x50), 0x04);
        assert_eq!(otp_five_bits_for_7bit_address(0x54), 0x05);
        // Note: program.rs doc comment claims 0x58 -> 0x07, but the verified
        // formula gives 0x06 (((0x58 >> 2) ^ 0x10) & 0x1F = 0x16 ^ 0x10 = 0x06),
        // which matches the addresses successfully programmed on hardware.
        assert_eq!(otp_five_bits_for_7bit_address(0x58), 0x06);
    }

    #[test]
    fn validate_new_address_rejects_invalid() {
        assert_eq!(
            validate_new_address::<()>(0x40, 0x40),
            Err(ProgramError::OldNewAddressesIdentical)
        );
        assert_eq!(
            validate_new_address::<()>(0x40, 0x80),
            Err(ProgramError::AddressOutOf7BitRange(0x80))
        );
        assert_eq!(
            validate_new_address::<()>(0x40, 0x05),
            Err(ProgramError::AddressReserved)
        );
        // Bits 0–1 differ -> A1/A2 strapping mismatch.
        assert_eq!(
            validate_new_address::<()>(0x40, 0x45),
            Err(ProgramError::HardwareBitsDiffer { old: 0x40, new: 0x45 })
        );
        assert_eq!(validate_new_address::<()>(0x40, 0x44), Ok(()));
    }

    #[test]
    fn raw_to_degrees_endpoints() {
        assert_eq!(raw_to_degrees(0), 0.0);
        let max = raw_to_degrees(RAW_MAX);
        assert!((max - 360.0).abs() < 1e-3);
        let mid = raw_to_degrees(RAW_MAX / 2);
        assert!((mid - 180.0).abs() < 0.05);
    }

    #[test]
    fn read_angle_raw_decodes_msb_lsb() {
        // MSB=0xAA (0b1010_1010), LSB=0x15 (0b0001_0101) -> (0xAA << 6) | (0x15 & 0x3F) = 10901.
        let expected: u16 = (0xAA_u16 << 6) | (0x15 & 0x3F);
        let expectations = [Transaction::write_read(
            DEFAULT_ADDRESS,
            vec![REG_ANGLE_MSB],
            vec![0xAA, 0x15],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut dev = As5048b::new(&mut i2c, DEFAULT_ADDRESS);

        assert_eq!(dev.read_angle_raw().unwrap(), expected);
        i2c.done();
    }

    #[test]
    fn read_angle_raw_masks_lsb_upper_bits() {
        // Datasheet: only low 6 bits of the LSB register are angle data.
        // MSB=0x01, LSB=0xFF -> (1 << 6) | (0xFF & 0x3F) = 64 | 63 = 127.
        let expectations = [Transaction::write_read(
            DEFAULT_ADDRESS,
            vec![REG_ANGLE_MSB],
            vec![0x01, 0xFF],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut dev = As5048b::new(&mut i2c, DEFAULT_ADDRESS);

        assert_eq!(dev.read_angle_raw().unwrap(), 127);
        i2c.done();
    }

    #[test]
    fn read_magnitude_raw_uses_magnitude_register() {
        let expectations = [Transaction::write_read(
            0x44,
            vec![REG_MAGNITUDE_MSB],
            vec![0x10, 0x20],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut dev = As5048b::new(&mut i2c, 0x44);

        let expected = (0x10_u16 << 6) | (0x20 & 0x3F);
        assert_eq!(dev.read_magnitude_raw().unwrap(), expected);
        i2c.done();
    }

    #[test]
    fn read_diagnostics_decodes_each_flag() {
        let expectations = [Transaction::write_read(
            DEFAULT_ADDRESS,
            vec![REG_DIAGNOSTICS],
            vec![0b0000_1111],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut dev = As5048b::new(&mut i2c, DEFAULT_ADDRESS);

        let d = dev.read_diagnostics().unwrap();
        assert_eq!(d.raw, 0b0000_1111);
        assert!(d.offset_compensation_finished());
        assert!(d.cordic_overflow());
        assert!(d.magnetic_field_too_strong());
        assert!(d.magnetic_field_too_weak());
        i2c.done();
    }

    #[test]
    fn read_diagnostics_with_only_ocf_set() {
        let expectations = [Transaction::write_read(
            DEFAULT_ADDRESS,
            vec![REG_DIAGNOSTICS],
            vec![0b0000_0001],
        )];
        let mut i2c = I2cMock::new(&expectations);
        let mut dev = As5048b::new(&mut i2c, DEFAULT_ADDRESS);

        let d = dev.read_diagnostics().unwrap();
        assert!(d.offset_compensation_finished());
        assert!(!d.cordic_overflow());
        assert!(!d.magnetic_field_too_strong());
        assert!(!d.magnetic_field_too_weak());
        i2c.done();
    }

    #[test]
    fn program_i2c_address_happy_path() {
        let old = 0x40;
        let new = 0x44; // shares low 2 bits (00) with 0x40
        let otp = otp_five_bits_for_7bit_address(new);

        let expectations = [
            // Pre-verify (read angle on old).
            Transaction::write_read(old, vec![REG_ANGLE_MSB], vec![0x00, 0x00]),
            // Step 1: write OTP byte to 0x15 on old address.
            Transaction::write(old, vec![REG_I2C_ADDRESS, otp]),
            // Step 2: enable programming mode on new address.
            Transaction::write(new, vec![REG_PROGRAMMING_CONTROL, OTP_CTRL_PROGRAMMING_MODE]),
            // Step 3: burn.
            Transaction::write(new, vec![REG_PROGRAMMING_CONTROL, OTP_CTRL_BURN]),
            // Step 4: exit programming.
            Transaction::write(new, vec![REG_PROGRAMMING_CONTROL, OTP_CTRL_DISABLE]),
            // Post-verify (read angle on new).
            Transaction::write_read(new, vec![REG_ANGLE_MSB], vec![0x00, 0x00]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut delay = NoopDelay::new();
        let mut dev = As5048b::new(&mut i2c, old);

        dev.program_i2c_address(&mut delay, new).unwrap();
        assert_eq!(dev.address(), new);
        i2c.done();
    }

    #[test]
    fn program_i2c_address_validation_does_not_touch_bus() {
        let mut i2c = I2cMock::new(&[]);
        let mut delay = NoopDelay::new();
        let mut dev = As5048b::new(&mut i2c, 0x40);

        // Bit-0/1 mismatch — caught before any I²C traffic.
        let err = dev.program_i2c_address(&mut delay, 0x45).unwrap_err();
        assert_eq!(err, ProgramError::HardwareBitsDiffer { old: 0x40, new: 0x45 });
        assert_eq!(dev.address(), 0x40);
        i2c.done();
    }
}
