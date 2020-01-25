#![no_main]
#![no_std]

use crate::hal::{prelude::*, stm32};
use cortex_m_rt::entry;
use panic_reset as _;
use stm32_usbd::UsbBus;
use stm32f0xx_hal as hal;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

enum TogCount {
    On(usize),
    Off(usize),
}

#[entry]
fn main() -> ! {
    if let Some(p) = stm32::Peripherals::take() {
        let (usb, rcc, crs, mut flash, gpioa, gpiob) =
            (p.USB, p.RCC, p.CRS, p.FLASH, p.GPIOA, p.GPIOB);

        let (usb_dm, usb_dp, mut led) = cortex_m::interrupt::free(|cs| {
            let mut rcc = rcc
                .configure()
                .hsi48()
                .enable_crs(crs)
                .sysclk(48.mhz())
                .pclk(24.mhz())
                .freeze(&mut flash);

            let gpioa = gpioa.split(&mut rcc);
            let gpiob = gpiob.split(&mut rcc);

            let usb_dm = gpioa.pa11;
            let usb_dp = gpioa.pa12;

            // (Re-)configure PA1 as output
            (usb_dm, usb_dp, gpiob.pb13.into_push_pull_output(cs))
        });

        for _ in 0..3 {
            // Turn PA1 on ten million times in a row
            for _ in 0..180_000 {
                led.set_high().ok();
            }
            // Then turn PA1 off a million times in a row
            for _ in 0..240_000 {
                led.set_low().ok();
            }
        }

        led.set_high().ok();

        let usb_bus = UsbBus::new(usb, (usb_dm, usb_dp));

        let mut serial = SerialPort::new(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27DD))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        let mut tog = TogCount::Off(0);
        led.set_low().ok();
        let mut togs = 0;

        'reset: loop {
            if !usb_dev.poll(&mut [&mut serial]) {
                use TogCount::*;
                tog = match tog {
                    Off(n) if n >= 200_000 => {
                        led.set_high().ok();
                        togs += 1;
                        On(0)
                    }
                    On(n) if n >= 200_000 => {
                        led.set_low().ok();
                        togs += 1;
                        Off(0)
                    }
                    On(n) => On(n + 1),
                    Off(n) => Off(n + 1),
                };

                assert!(togs <= 60);

                continue;
            } else {
                tog = TogCount::Off(0);
                led.set_low().ok();
            }

            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    led.set_high().ok(); // Turn on

                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if *c == b'z' {
                            break 'reset;
                        }

                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }

            led.set_low().ok(); // Turn off
        }
    }

    panic!();
}
