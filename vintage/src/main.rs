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

        // Force
        usb_dev.bus().force_reenumeration(|| {
            delay.delay_us(1_000_000u32);
        });

        let mut tog = TogCount::Off(0);
        led.set_low().ok();
        let mut togs = 0;


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
        use heapless::String;
        use heapless::consts::*;
        use core::fmt::Write;

        let mut string_buf: String<U1024> = String::new();

        let mut panic_report_once = false;
        let mut any_input = false;
        let mut demo_neo: u8 = 0;
        let mut color: u32 = 0b1001_0010_0100_1001;

        'outer: loop {
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
                if !any_input && (togs >= 10) {
                    panic!("Input Timeout");
                }

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
                    any_input = true;


                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if *c == b'z' {
                            panic!("reset requested");
                        }

                        if !panic_report_once && *c == b'p' {
                            panic_report_once = true;

                            println_blocking(&mut serial, "");

                            if let Some(msg_b) = get_panic_message_bytes() {
                                println_blocking_bytes(&mut serial, msg_b);
                            } else {
                                println_blocking(&mut serial, "No Panic.");
                            }

                            continue 'outer;
                        }

                        if *c == b't' {
                            println_blocking(&mut serial, "");

                            if let Ok(i) = trellis.keypad_get_count() {
                                println_blocking(&mut serial, "Good Trellis.");

                                string_buf.clear();

                                if let Ok(_) = write!(&mut string_buf, "{} buttons", i) {
                                    println_blocking(&mut serial, &string_buf);
                                }

                            } else {
                                println_blocking(&mut serial, "Bad Trellis.");
                            }

                            continue 'outer;
                        }

                        if *c == b'k' {
                            for i in 0..16 {
                                // ???
                                let key = (((i) / 4) * 8 + ((i) % 4));

                                trellis.keypad_set_event_raw(
                                    key,
                                    adafruit_seesaw::keypad::Edge::Rising,
                                    adafruit_seesaw::keypad::Status::Enable,
                                ).unwrap();

                                trellis.keypad_set_event_raw(
                                    key,
                                    adafruit_seesaw::keypad::Edge::Falling,
                                    adafruit_seesaw::keypad::Status::Enable,
                                ).unwrap();
                            }
                            println_blocking(&mut serial, "\r\nSet Key Events");
                            continue 'outer;
                        }

                        if *c == b'l' {
                            for i in 0..16 {
                                // ???
                                let key = (((i) / 4) * 8 + ((i) % 4));

                                trellis.keypad_set_event_raw(
                                    key,
                                    adafruit_seesaw::keypad::Edge::Rising,
                                    adafruit_seesaw::keypad::Status::Disable,
                                ).unwrap();

                                trellis.keypad_set_event_raw(
                                    key,
                                    adafruit_seesaw::keypad::Edge::Falling,
                                    adafruit_seesaw::keypad::Status::Disable,
                                ).unwrap();
                            }
                            println_blocking(&mut serial, "\r\nUnset Key Events");
                            continue 'outer;
                        }

                        if *c == b'r' {
                            if let Ok(i) = trellis.keypad_get_count() {
                                let mut buf = [0u8; 32];

                                let evts = &mut buf[..(i as usize)];

                                if trellis.keypad_read_raw(evts).is_ok() {
                                    string_buf.clear();
                                    write!(&mut string_buf, "Events: ").ok();

                                    for e in evts {
                                        write!(&mut string_buf, "{:02X} ", e).ok();
                                    }
                                    println_blocking(&mut serial, &string_buf);
                                }

                            } else {
                                println_blocking(&mut serial, "\r\nNo Events");
                            }
                        }

                        if *c == b's' {
                            type SResult<T> = Result<T, adafruit_seesaw::Error>;

                            #[derive(Debug)]
                            struct Status {
                                hw_id: SResult<u8>,
                                version: SResult<u32>,
                                options: SResult<u32>,
                                temp_raw: SResult<u32>,
                            }

                            println_blocking(&mut serial, "");
                            string_buf.clear();

                            let hw_id = trellis.status_get_hwid();
                            let version = trellis.status_get_version();
                            let options = trellis.status_get_options();
                            let temp_raw = trellis.status_get_temp_raw();

                            let status = Status {
                                hw_id,
                                version,
                                options,
                                temp_raw,
                            };

                            let res = write!(
                                &mut string_buf,
                                "{:08X?}",
                                status
                            );

                            if res.is_ok() {
                                println_blocking(&mut serial, &string_buf);
                            } else {
                                println_blocking(&mut serial, "Format error");
                            }

                            continue 'outer;
                        }

                        if *c == b'm' {
                            println_blocking(&mut serial, "");
                            demo_neo += 1;
                            if demo_neo >= 16 {
                                demo_neo = 0;
                            }

                            let colors = color.to_le_bytes();

                            trellis.neopixel_write_buf_raw(
                                (demo_neo as u16) * 3,
                                &[colors[0], colors[1], colors[2]]
                            ).unwrap();
                            trellis.neopixel_show().unwrap();

                            color = color.rotate_left(1);


                        }

                        if *c == b'n' {
                            println_blocking(&mut serial, "");

                            if trellis.neopixel_set_speed(adafruit_seesaw::neopixel::Speed::Khz800).is_err() {
                                println_blocking(&mut serial, "Bad Speed");
                            } else if trellis.neopixel_set_buf_length_bytes(3 * 16).is_err() {
                                println_blocking(&mut serial, "Bad Buf Set");
                            } else if trellis.neopixel_set_pin(3).is_err() {
                                println_blocking(&mut serial, "Bad Pin Set");
                            } else {
                                println_blocking(&mut serial, "Neopixel init!");
                            }

                            continue 'outer;
                        }

                        if b'a' <= *c && *c <= b'z' {
                            *c &= !0x20;
                        }
                    }

                    print_blocking_bytes(&mut serial, &buf);
                }
                _ => {}
            }

            led.set_low().ok(); // Turn off
        }
    }

    loop {
        continue;
    }
}

use core::borrow::BorrowMut;

fn print_blocking<B, RS, WS>(usb: &mut SerialPort<B, RS, WS>, msg: &str)
where
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    print_blocking_bytes(usb, msg.as_bytes())
}

fn println_blocking<B, RS, WS>(usb: &mut SerialPort<B, RS, WS>, msg: &str)
where
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    println_blocking_bytes(usb, msg.as_bytes())
}

fn print_blocking_bytes<B, RS, WS>(usb: &mut SerialPort<B, RS, WS>, msg_b: &[u8])
where
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    let count = msg_b.len();
    let mut write_offset = 0;

    while write_offset < count {
        match usb.write(&msg_b[write_offset..count]) {
            Ok(len) if len > 0 => {
                write_offset += len;
            }
            _ => {}
        }
    }
}

fn println_blocking_bytes<B, RS, WS>(usb: &mut SerialPort<B, RS, WS>, msg_b: &[u8])
where
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    print_blocking_bytes(usb, msg_b);
    print_blocking_bytes(usb, &[b'\r', b'\n']);
}
