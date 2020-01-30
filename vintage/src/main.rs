#![no_main]
#![no_std]

use panic_persist as _;

use adafruit_neotrellis::{self as neotrellis, NeoTrellis};
use embedded_hal::digital::v2::OutputPin;
use rtfm::app;
use stm32_usbd::UsbBus;
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB6, PB7},
        Alternate, PushPull, AF1,
    },
    i2c::I2c,
    stm32f0::stm32f0x2::I2C1,
    usb::Peripheral,
    prelude::*
};
use usb_device::{
    bus::{self, UsbBusAllocator},
    prelude::*,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        serial: SerialPort<'static, UsbBus<Peripheral>>,
        led: PB13<stm32f0xx_hal::gpio::Output<PushPull>>,
        trellis: NeoTrellis<I2c<I2C1, PB6<Alternate<AF1>>, PB7<Alternate<AF1>>>, Delay>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;

        //////////////////////////////////////////////////////////////////////
        // Set up the hardware!
        //////////////////////////////////////////////////////////////////////
        let (usb, rcc, crs, mut flash, gpioa, gpiob, i2c1, syst) = (
            cx.device.USB,
            cx.device.RCC,
            cx.device.CRS,
            cx.device.FLASH,
            cx.device.GPIOA,
            cx.device.GPIOB,
            cx.device.I2C1,
            cx.core.SYST,
        );

        let (usb_dm, usb_dp, mut led, i2c, delay) = cortex_m::interrupt::free(|cs| {
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

            let mut usb_dp = usb_dp.into_push_pull_output(&cs);
            usb_dp.set_low().ok();
            cortex_m::asm::delay(48_000_000 / 100);
            let usb_dp = usb_dp.into_floating_input(&cs);

            let scl = gpiob.pb6.into_alternate_af1(&cs);
            let sda = gpiob.pb7.into_alternate_af1(&cs);

            let i2c = I2c::i2c1(i2c1, (scl, sda), 100.khz(), &mut rcc);

            let delay = Delay::new(syst, &rcc);

            (
                usb_dm,
                usb_dp,
                gpiob.pb13.into_push_pull_output(cs),
                i2c,
                delay,
            )
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

        // //////////////////////////////////////////////////////////////////////
        // // Set up USB as a CDC ACM Serial Port
        // //////////////////////////////////////////////////////////////////////
        *USB_BUS = Some(UsbBus::new(Peripheral {
            usb,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        }));

        let mut serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let mut usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27DD))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        let trellis = neotrellis::NeoTrellis::new(i2c, delay, None).unwrap();

        usb_dev.poll(&mut [&mut serial]);

        init::LateResources {
            usb_dev,
            serial,
            led,
            trellis,
        }
    }

    #[task(binds = USB, resources = [usb_dev, serial, led])]
    fn usb_tx(mut cx: usb_tx::Context) {
        cx.resources.led.set_high().ok();
        usb_poll(&mut cx.resources.usb_dev, &mut cx.resources.serial);
        cx.resources.led.set_low().ok();
    }

    #[idle(resources = [trellis])]
    fn idle(mut cx: idle::Context) -> ! {
        match inner_idle(&mut cx) {
            Ok(_) => loop { continue; },
            Err(_) => panic!(),
        };
    }
};

fn inner_idle(cx: &mut idle::Context) -> Result<(), neotrellis::Error> {
    let mut color: u32 = 0b1001_0010_0100_1001;

    cx.resources
        .trellis
        .neopixels()
        .set_speed(neotrellis::Speed::Khz800)?
        .set_pixel_count(16)?
        .set_pixel_type(neotrellis::ColorOrder::GRB)?
        .set_pin(3)?;

    for c in 0..4 {
        let cols = match c {
            0 => [0x00, 0x10, 0x00],
            1 => [0x10, 0x00, 0x00],
            2 => [0x00, 0x00, 0x10],
            _ => [0x00, 0x00, 0x00],
        };

        for i in 0..16 {
            cx.resources
                .trellis
                .keypad()
                .enable_key_event(i, neotrellis::Edge::Rising)?
                .enable_key_event(i, neotrellis::Edge::Falling)?;

            cx.resources
                .trellis
                .neopixels()
                .set_pixel_rgb(i, cols[1], cols[0], cols[2])?;
        }

        cx.resources.trellis.neopixels().show()?;
        cx.resources.trellis.seesaw().delay_us(150_000u32);
    }

    let mut sticky = [false; 16];

    'outer: loop {
        // let mut buf = [0u8; 64];

        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////

        cx.resources.trellis.seesaw().delay_us(10_000u32);
        cx.resources.trellis.seesaw().delay_us(10_000u32);

        for evt in cx
            .resources
            .trellis
            .keypad()
            .get_events()?
            .as_slice()
        {
            if evt.key == 15 && evt.event == neotrellis::Edge::Falling {
                panic!()
            }

            if evt.event == neotrellis::Edge::Rising {
                if sticky[evt.key as usize] {
                    cx.resources
                        .trellis
                        .neopixels()
                        .set_pixel_rgb(evt.key, 0, 0, 0)?;
                    sticky[evt.key as usize] = false;
                } else {
                    let colors = color.to_le_bytes();

                    cx.resources
                        .trellis
                        .neopixels()
                        .set_pixel_rgb(evt.key, colors[1], colors[0], colors[2])?;

                    color = color.rotate_left(1);
                    sticky[(evt.key as usize)] = true;
                }
            }
        }

        cx.resources.trellis.neopixels().show()?;
    }
}

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
) {
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 8];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if *c == b'z' {
                        panic!();
                    }

                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                serial.write(&buf[0..count]).ok();
            }
            _ => {}
        }
    }
}
