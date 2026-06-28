#![no_std]
#![no_main]

use as5048b_magnetic_encoder::As5048b;
use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{BusTimeout, Config as I2cMasterConfig, I2c, SoftwareTimeout};
use esp_hal::main;
use esp_hal::time::{Duration, Rate};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

// A1=A2=GND → default 7-bit I2C address.
const AS5048B_ADDR: u8 = 0x40;

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

    let mut sensor = As5048b::new(i2c, AS5048B_ADDR);
    let delay = Delay::new();

    let mut loop_count = 0;
    loop {
        match sensor.read_angle_degrees() {
            Ok(degrees) => info!(
                "{} degrees {:?}",
                loop_count,
                degrees
            ),
            Err(e) => defmt::warn!("AS5048B I2C read failed: {}", e),
        }

        loop_count += 1;
        delay.delay_millis(200);
    }
}
