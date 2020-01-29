#![no_main]
#![no_std]

use crate::hal::{prelude::*, stm32, i2c::I2c, delay::Delay};
use cortex_m_rt::entry;
use panic_persist::{self as _, get_panic_message_bytes};
use stm32_usbd::UsbBus;
use stm32f0xx_hal as hal;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use adafruit_seesaw as seesaw;
use cortex_m;
use embedded_hal::blocking::{
    i2c::{WriteRead, Write},
    delay::DelayUs,
};

enum TogCount {
    On(usize),
    Off(usize),
}

#[entry]
fn main() -> ! {
    if let (Some(p), Some(cp)) = (stm32::Peripherals::take(), cortex_m::Peripherals::take()) {
        //////////////////////////////////////////////////////////////////////
        // Set up the hardware!
        //////////////////////////////////////////////////////////////////////
        let (usb, rcc, crs, mut flash, gpioa, gpiob, i2c1) =
            (p.USB, p.RCC, p.CRS, p.FLASH, p.GPIOA, p.GPIOB, p.I2C1);

        let (usb_dm, usb_dp, mut led, i2c, mut delay) = cortex_m::interrupt::free(|cs| {
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

            let scl = gpiob.pb6.into_alternate_af1(&cs);
            let sda = gpiob.pb7.into_alternate_af1(&cs);

            let i2c = I2c::i2c1(i2c1, (scl, sda), 100.khz(), &mut rcc);

            let delay = Delay::new(cp.SYST, &rcc);

            (usb_dm, usb_dp, gpiob.pb13.into_push_pull_output(cs), i2c, delay)
        });


        //////////////////////////////////////////////////////////////////////
        // Blink a few times to show we've [re]-booted
        //////////////////////////////////////////////////////////////////////
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

        //////////////////////////////////////////////////////////////////////
        // Set up USB as a CDC ACM Serial Port
        //////////////////////////////////////////////////////////////////////
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

        // for _ in 0..7 {
        //     // Turn PA1 on ten million times in a row
        //     for _ in 0..1_800_000 {
        //         led.set_high().ok();
        //     }
        //     // Then turn PA1 off a million times in a row
        //     for _ in 0..2_400_000 {
        //         led.set_low().ok();
        //     }
        // }

        // delay.delay_us(1_000_000u32);

        let mut trellis = seesaw::SeeSaw {
            i2c,
            delay,
            address: 0x2E // TODO: FIXME
        };

        // for _ in 0..2 {
        //     // Turn PA1 on ten million times in a row
        //     for _ in 0..1_800_000 {
        //         led.set_high().ok();
        //     }
        //     // Then turn PA1 off a million times in a row
        //     for _ in 0..2_400_000 {
        //         led.set_low().ok();
        //     }
        // }


        // if let Ok(i) = trellis.keypad_get_count() {
        //     for _ in 0..(i as usize) {
        //         // Turn PA1 on ten million times in a row
        //         for _ in 0..180_000 {
        //             led.set_high().ok();
        //         }
        //         // Then turn PA1 off a million times in a row
        //         for _ in 0..240_000 {
        //             led.set_low().ok();
        //         }
        //     }
        // } else {
        //     for _ in 0..3 {
        //         // Turn PA1 on ten million times in a row
        //         for _ in 0..1_800_000 {
        //             led.set_high().ok();
        //         }
        //         // Then turn PA1 off a million times in a row
        //         for _ in 0..2_400_000 {
        //             led.set_low().ok();
        //         }
        //     }
        // }

        // panic!();

        let mut once = false;

        'reset: loop {
            //////////////////////////////////////////////////////////////////
            // If the USB port is idle, blink to show we are bored
            //////////////////////////////////////////////////////////////////
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

                //////////////////////////////////////////////////////////////
                // After a while, just crash and reboot so we can press the
                // DFU button on reboot
                //////////////////////////////////////////////////////////////
                assert!(togs <= 60);

                continue;
            } else {
                //////////////////////////////////////////////////////////////
                // When active, reset the idle counter
                //////////////////////////////////////////////////////////////
                tog = TogCount::Off(0);
                led.set_low().ok();
            }

            let mut buf = [0u8; 64];

            //////////////////////////////////////////////////////////////////
            // Loopback as upper case, use 'z' to force a reboot of the device
            //////////////////////////////////////////////////////////////////
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    led.set_high().ok(); // Turn on

                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if *c == b'z' {
                            break 'reset;
                        }

                        if !once && *c == b'p' {
                            once = true;

                            while serial.write(&[b'\r', b'\n']).is_err() {}

                            if let Some(msg_b) = get_panic_message_bytes() {
                                let mut write_offset = 0;
                                let count = msg_b.len();
                                while write_offset < count {
                                    match serial.write(&msg_b[write_offset..count]) {
                                        Ok(len) if len > 0 => {
                                            write_offset += len;
                                        }
                                        _ => {}
                                    }
                                }
                            } else {
                                let msg_b = "No panic.".as_bytes();
                                let count = msg_b.len();
                                let mut write_offset = 0;

                                while write_offset < count {
                                    match serial.write(&msg_b[write_offset..count]) {
                                        Ok(len) if len > 0 => {
                                            write_offset += len;
                                        }
                                        _ => {}
                                    }
                                }
                            }

                            while serial.write(&[b'\r', b'\n']).is_err() {}

                            continue;
                        }

                        if *c == b't' {
                            while serial.write(&[b'\r', b'\n']).is_err() {}

                            if let Ok(i) = trellis.keypad_get_count() {
                                let msg_b = "Good Trellis.".as_bytes();
                                let count = msg_b.len();
                                let mut write_offset = 0;

                                while write_offset < count {
                                    match serial.write(&msg_b[write_offset..count]) {
                                        Ok(len) if len > 0 => {
                                            write_offset += len;
                                        }
                                        _ => {}
                                    }
                                }
                            } else {
                                let msg_b = "Bad Trellis.".as_bytes();
                                let count = msg_b.len();
                                let mut write_offset = 0;

                                while write_offset < count {
                                    match serial.write(&msg_b[write_offset..count]) {
                                        Ok(len) if len > 0 => {
                                            write_offset += len;
                                        }
                                        _ => {}
                                    }
                                }
                            }

                            while serial.write(&[b'\r', b'\n']).is_err() {}
                        }

                        if b'a' <= *c && *c <= b'z' {
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

    panic!("reset requested");
}
