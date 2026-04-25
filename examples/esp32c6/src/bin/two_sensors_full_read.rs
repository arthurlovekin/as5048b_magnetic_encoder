#![no_std]
#![no_main]

use core::cell::RefCell;

use as5048b_magnetic_encoder::As5048b;
use defmt::info;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{BusTimeout, Config as I2cMasterConfig, I2c, SoftwareTimeout};
use esp_hal::main;
use esp_hal::time::{Duration, Rate};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

// Two sensors on the same I2C bus, distinguished by A1/A2 strapping.
// Sensor A: A1=A2=GND  -> 0x40
// Sensor B: A1=VDD, A2=GND -> 0x41
const SENSOR_A_ADDR: u8 = 0x40;
const SENSOR_B_ADDR: u8 = 0x41;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let i2c_cfg = I2cMasterConfig::default()
        .with_frequency(Rate::from_khz(100))
        .with_timeout(BusTimeout::BusCycles(4096))
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(10)));
    let i2c = I2c::new(peripherals.I2C0, i2c_cfg)
        .expect("I2C master config")
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

    // Share the single bus between two driver instances.
    let bus = RefCell::new(i2c);
    let mut sensor_a = As5048b::new(RefCellDevice::new(&bus), SENSOR_A_ADDR);
    let mut sensor_b = As5048b::new(RefCellDevice::new(&bus), SENSOR_B_ADDR);

    let delay = Delay::new();

    let mut loop_count = 0;
    loop {
        read_and_log("A", &mut sensor_a, loop_count);
        read_and_log("B", &mut sensor_b, loop_count);

        loop_count += 1;
        delay.delay_millis(200);
    }
}

fn read_and_log<I2C, E>(label: &str, sensor: &mut As5048b<I2C>, loop_count: u32)
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: defmt::Format,
{
    match (
        sensor.read_diagnostics(),
        sensor.read_magnitude_raw(),
        sensor.read_angle_degrees(),
    ) {
        (Ok(d), Ok(m), Ok(a)) => info!(
            "{}: sensor {} (0x{:02X}) ocf={} cof={} mag_strong={} mag_weak={} mag={} angle={} deg",
            loop_count,
            label,
            sensor.address(),
            d.offset_compensation_finished(),
            d.cordic_overflow(),
            d.magnetic_field_too_strong(),
            d.magnetic_field_too_weak(),
            m,
            a,
        ),
        (diag, mag, angle) => defmt::warn!(
            "{}: sensor {} (0x{:02X}) read failed: diag_ok={} mag_ok={} angle_ok={}",
            loop_count,
            label,
            sensor.address(),
            diag.is_ok(),
            mag.is_ok(),
            angle.is_ok(),
        ),
    }
}
