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
use adafruit_neotrellis as neotrellis;

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
        // let usb_bus = UsbBus::new(usb, (usb_dm, usb_dp));


        // let mut serial = SerialPort::new(&usb_bus);


        // let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27DD))
        //     .manufacturer("Fake company")
        //     .product("Serial port")
        //     .serial_number("TEST")
        //     .device_class(USB_CLASS_CDC)
        //     .build();

        // Force
        // usb_dev.bus().force_reenumeration(|| {
        //     delay.delay_us(1_000_000u32);
        // });

        let mut tog = TogCount::Off(0);
        led.set_low().ok();
        let mut togs = 0;


        let mut trellis = neotrellis::NeoTrellis::new(i2c, delay, None).unwrap();

        use heapless::String;
        use heapless::consts::*;
        use core::fmt::Write;

        let mut string_buf: String<U1024> = String::new();

        let mut panic_report_once = false;
        let mut any_input = false;
        let mut demo_neo: u8 = 0;
        let mut color: u32 = 0b1001_0010_0100_1001;

        trellis
            .neopixels()
            .set_speed(neotrellis::Speed::Khz800).unwrap()
            .set_pixel_count(16).unwrap()
            .set_pixel_type(neotrellis::ColorOrder::GRB).unwrap()
            .set_pin(3).unwrap();

        for c in 0..4 {
            let cols = match c {
                0 => [0x00, 0x10, 0x00],
                1 => [0x10, 0x00, 0x00],
                2 => [0x00, 0x00, 0x10],
                _ => [0x00, 0x00, 0x00],
            };

            for i in 0..16 {
                trellis
                    .keypad()
                    .enable_key_event(i, neotrellis::Edge::Rising).unwrap()
                    .enable_key_event(i, neotrellis::Edge::Falling).unwrap();

                trellis
                    .neopixels()
                    .set_pixel_rgb(i, cols[1], cols[0], cols[2])
                    .unwrap();
            }

            trellis.neopixels().show().unwrap();

            led.set_high().ok();
            trellis.seesaw().delay_us(150_000u32);
            led.set_low().ok();
            trellis.seesaw().delay_us(150_000u32);
        }

        let mut sticky = [false; 16];


        'outer: loop {

            //////////////////////////////////////////////////////////////////
            // If the USB port is idle, blink to show we are bored
            //////////////////////////////////////////////////////////////////
            // if !usb_dev.poll(&mut [&mut serial]) {
            //     use TogCount::*;
            //     tog = match tog {
            //         Off(n) if n >= 200_000 => {
            //             led.set_high().ok();
            //             togs += 1;
            //             On(0)
            //         }
            //         On(n) if n >= 200_000 => {
            //             led.set_low().ok();
            //             togs += 1;
            //             Off(0)
            //         }
            //         On(n) => On(n + 1),
            //         Off(n) => Off(n + 1),
            //     };

            //     //////////////////////////////////////////////////////////////
            //     // After a while, just crash and reboot so we can press the
            //     // DFU button on reboot
            //     //////////////////////////////////////////////////////////////
            //     if !any_input && (togs >= 10) {
            //         panic!("Input Timeout");
            //     }
            // } else {
            //     //////////////////////////////////////////////////////////////
            //     // When active, reset the idle counter
            //     //////////////////////////////////////////////////////////////
            //     tog = TogCount::Off(0);
            //     led.set_low().ok();
            // }

            // let mut buf = [0u8; 64];

            //////////////////////////////////////////////////////////////////
            // Process button presses
            //////////////////////////////////////////////////////////////////

            led.set_high().ok();
            trellis.seesaw().delay_us(10_000u32);
            led.set_low().ok();
            trellis.seesaw().delay_us(10_000u32);


            for evt in trellis.keypad().get_events().unwrap().as_slice() {
                if evt.key == 15 && evt.event == neotrellis::Edge::Falling {
                    panic!()
                }

                if evt.event == neotrellis::Edge::Rising {

                    if sticky[evt.key as usize] {
                        trellis.neopixels().set_pixel_rgb(evt.key, 0, 0, 0).unwrap();
                        sticky[evt.key as usize] = false;
                    } else {
                        let colors = color.to_le_bytes();

                        trellis.neopixels().set_pixel_rgb(evt.key, colors[1], colors[0], colors[2]).unwrap();

                        color = color.rotate_left(1);
                        sticky[(evt.key as usize)] = true;
                    }
                }
            }

            trellis.neopixels().show().unwrap();

            //////////////////////////////////////////////////////////////////
            // Loopback as upper case, use 'z' to force a reboot of the device
            //////////////////////////////////////////////////////////////////
            // match serial.read(&mut buf) {
            //     Ok(count) if count > 0 => {
            //         any_input = true;

            //         for c in buf[0..count].iter_mut() {
            //             if *c == b'z' {
            //                 panic!("reset requested");
            //             }

            //             if b'a' <= *c && *c <= b'z' {
            //                 *c &= !0x20;
            //             }
            //         }
            //         print_blocking_bytes(&mut serial, &buf);
            //     }
            //     _ => {}
            // }

            // led.set_low().ok(); // Turn off
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
