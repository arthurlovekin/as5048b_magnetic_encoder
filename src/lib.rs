// convert 14-bit raw angle to degrees (0-360)
fn raw_angle_to_degrees(angle: u16) -> f32 {
    (angle as f32) / 16383.0 * 360.0
}

struct As5048bDiagnostics {
    offset_compensation_finished: bool,
    cordic_overflow: bool,
    magnetic_field_too_strong: bool,
    magnetic_field_too_weak: bool,
}

struct As048bMagneticEncoder {
    i2c_address: u8,
    angle: f32,
    magnitude: f32,
    diagnostics: As5048bDiagnostics,
}


