#![no_std]
#![no_main]

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut sensor = As5048b::new(&mut i2c, AS5048B_ADDR);

    let mut loop_count = 0;
    loop {
        usb_dev.poll(&mut [&mut serial]);

        match sensor.read_angle_degrees() {
            Ok(degrees) => {
                let mut line = String::<128>::new();
                let _ = write!(line, "{} degrees {}\r\n", loop_count, degrees);
                let _ = serial.write(line.as_bytes());
            }
            Err(e) => {
                let mut line = String::<128>::new();
                let _ = write!(line, "AS5048B I2C read failed: {:?}\r\n", e);
                let _ = serial.write(line.as_bytes());
            }
        }

        loop_count += 1;

        let start = timer.get_counter().ticks();
        while timer.get_counter().ticks() - start < 200_000 {
            usb_dev.poll(&mut [&mut serial]);
        }
    }
}
