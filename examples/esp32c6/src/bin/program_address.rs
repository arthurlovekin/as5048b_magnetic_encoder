#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! One-time program a new I²C address into an AS5048B.
//!
//! **Irreversible.** This runs the OTP address-programming sequence
//! ([`As5048b::program_i2c_address`]) exactly once, then turns the onboard
//! WS2812 RGB LED (GPIO8) solid red and idles — the red LED is just a visual
//! marker so you can tell at a glance that *this* firmware (not one of the
//! read-only examples) is the one flashed on the board.
//!
//! Edit [`I2C_ADDR_OLD`] / [`I2C_ADDR_NEW`] to match your wiring and the address
//! you want burned in. `NEW` must agree with `OLD` in bits 0–1, since those come
//! from the A1/A2 strapping pins and cannot be changed by OTP.

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{BusTimeout, Config as I2cMasterConfig, I2c, SoftwareTimeout};
use esp_hal::main;
use esp_hal::rmt::Rmt;
use esp_hal::time::{Duration, Rate};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smart_leds::{RGB8, SmartLedsWrite, brightness, gamma};
use {esp_backtrace as _, esp_println as _};

use as5048b_magnetic_encoder::As5048b;

esp_bootloader_esp_idf::esp_app_desc!();

// Default OTP + A1/A2 strapped low → 7-bit address 0x40.
const I2C_ADDR_OLD: u8 = 0x43;
// Address to burn in. Must share bits 0–1 with `OLD` (A1/A2 pins).
// with A1=A2=low (default: 0x40):  0x44, 0x48, 0x4C, 0x50, 0x54, 0x58 ...
// With A1=A2=high (default: 0x43): 0x47, 0x4B, 0x4F, 0x53, 0x57, 0x5B ...
const I2C_ADDR_NEW: u8 = 0x4B;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    //////// LED: solid red marks this firmware as flashed ////////
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut rmt_buffer = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, &mut rmt_buffer);
    let red = RGB8 { r: 255, g: 0, b: 0 };
    let bright = 1; // 0-255
    led.write(brightness(gamma([red].into_iter()), bright))
        .expect("LED write");

    //////// I2C master ////////
    // A bounded hardware BusTimeout plus a software SoftwareTimeout so a missing
    // sensor fails fast instead of appearing to hang.
    let i2c_cfg = I2cMasterConfig::default()
        .with_frequency(Rate::from_khz(100))
        .with_timeout(BusTimeout::BusCycles(4096))
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(10)));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_cfg)
        .expect("I2C master config")
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

    let mut delay = Delay::new();

    //////// One-time program the new address. Irreversible. ////////
    // Keep `dev` alive for the confirmation loop below. On success it is bound to
    // the new address; the driver also rebinds to the new address after step 1,
    // so `dev.address()` reflects wherever the chip actually ended up.
    let mut dev = As5048b::new(&mut i2c, I2C_ADDR_OLD);
    match dev.program_i2c_address(&mut delay, I2C_ADDR_NEW) {
        Ok(()) => info!(
            "Programmed AS5048B 0x{:02x} -> 0x{:02x}. It now responds on the new address.",
            I2C_ADDR_OLD,
            dev.address(),
        ),
        Err(e) => defmt::error!("Programming failed: {}", e),
    }

    // Confirm everything works: read the angle on the (new) address each second.
    // The LED stays red the whole time so the board state remains obvious.
    loop {
        let addr = dev.address();
        match dev.read_angle_degrees() {
            Ok(degrees) => info!("addr=0x{:02x}: angle={} deg", addr, degrees),
            Err(e) => defmt::warn!("addr=0x{:02x}: angle read failed: {}", addr, e),
        }
        delay.delay_millis(1000);
    }
}
