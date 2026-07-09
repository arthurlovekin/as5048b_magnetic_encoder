#![no_std]
#![no_main]

//! One-time program the zero position of an AS5048B.
//!
//! **Irreversible.** Place the magnet at the desired mechanical zero, then this
//! runs the OTP zero-position sequence ([`As5048b::program_zero_position`])
//! exactly once. It logs the angle before and after programming over USB CDC —
//! afterwards the reading at the current magnet position should be ~0°. It then
//! idles reading the angle.

use core::fmt::Write;

use adafruit_kb2040::entry;
use adafruit_kb2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        timer::Timer,
        usb::UsbBus,
        watchdog::Watchdog,
        Sio,
    },
    XOSC_CRYSTAL_FREQ,
};
use as5048b_magnetic_encoder::As5048b;
use fugit::RateExtU32;
use heapless::String;
use panic_halt as _;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// A1=A2=GND → default 7-bit I2C address.
const AS5048B_ADDR: u8 = 0x40;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = adafruit_kb2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(USB_CLASS_CDC)
        .build();

    // I2C0 on the KB2040's labelled SDA (GPIO12) / SCL (GPIO13) pins.
    let mut i2c = adafruit_kb2040::hal::I2C::i2c0(
        pac.I2C0,
        pins.sda.reconfigure(),
        pins.scl.reconfigure(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut sensor = As5048b::new(&mut i2c, AS5048B_ADDR);

    // Give the host a moment to open the CDC port before we log.
    {
        let start = timer.get_counter().ticks();
        let delay_microsec = 5_000_000;
        while timer.get_counter().ticks() - start < delay_microsec {
            usb_dev.poll(&mut [&mut serial]);
        }
    }

    //////// Angle before programming ////////
    usb_dev.poll(&mut [&mut serial]);
    match sensor.read_angle_degrees() {
        Ok(degrees) => {
            let mut line = String::<128>::new();
            let _ = write!(line, "Before programming: angle={} deg\r\n", degrees);
            let _ = serial.write(line.as_bytes());

            //////// One-time program the zero position at the current magnet pose. Irreversible. ////////
            match sensor.program_zero_position(&mut timer) {
                Ok(()) => {
                    let _ = serial.write(
                        b"Programmed zero position. Current magnet pose is now ~0 deg.\r\n",
                    );
                }
                Err(e) => {
                    let mut line = String::<128>::new();
                    let _ = write!(line, "Programming failed: {:?}\r\n", e);
                    let _ = serial.write(line.as_bytes());
                }
            }

            //////// Angle after programming (should read ~0 deg at the current pose) ////////
            usb_dev.poll(&mut [&mut serial]);
            match sensor.read_angle_degrees() {
                Ok(degrees) => {
                    let mut line = String::<128>::new();
                    let _ = write!(line, "After programming: angle={} deg\r\n", degrees);
                    let _ = serial.write(line.as_bytes());
                }
                Err(e) => {
                    let mut line = String::<128>::new();
                    let _ = write!(line, "After programming: angle read failed: {:?}\r\n", e);
                    let _ = serial.write(line.as_bytes());
                }
            }
        }
        Err(e) => {
            let mut line = String::<128>::new();
            let _ = write!(
                line,
                "Before programming: angle read failed ({:?}); skipping OTP programming\r\n",
                e
            );
            let _ = serial.write(line.as_bytes());
        }
    }

    // Confirm everything works: read the angle each second.
    loop {
        usb_dev.poll(&mut [&mut serial]);

        match sensor.read_angle_degrees() {
            Ok(degrees) => {
                let mut line = String::<128>::new();
                let _ = write!(line, "angle={} deg\r\n", degrees);
                let _ = serial.write(line.as_bytes());
            }
            Err(e) => {
                let mut line = String::<128>::new();
                let _ = write!(line, "angle read failed: {:?}\r\n", e);
                let _ = serial.write(line.as_bytes());
            }
        }

        let start = timer.get_counter().ticks();
        while timer.get_counter().ticks() - start < 1_000_000 {
            usb_dev.poll(&mut [&mut serial]);
        }
    }
}
