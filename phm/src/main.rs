#![no_main]
#![no_std]

use panic_persist as _;

use adafruit_neotrellis::{self as neotrellis, NeoTrellis};
use embedded_hal::digital::v2::OutputPin;
use heapless::{
    i::Queue as ConstQueue,
    spsc::{Consumer, Producer, Queue},
};
use rtfm::app;
use stm32_usbd::UsbBus;
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB6, PB7},
        Alternate, PushPull, AF1,
    },
    i2c::I2c,
    prelude::*,
    stm32f0::stm32f0x2::I2C1,
    usb::Peripheral,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use phm_icd::{
    cobs_buffer::{consts::*, Buffer},
    DeviceToHostMessages, HostToDeviceMessages,
};

// mod trellis;
mod usb;
mod i2c_rpc;

pub struct UsbChannels {
    incoming: Producer<'static, HostToDeviceMessages, U16, u8>,
    outgoing: Consumer<'static, DeviceToHostMessages, U16, u8>,
}

pub struct ClientChannels {
    incoming: Consumer<'static, HostToDeviceMessages, U16, u8>,
    outgoing: Producer<'static, DeviceToHostMessages, U16, u8>,
}

#[app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        serial: SerialPort<'static, UsbBus<Peripheral>>,
        led: PB13<stm32f0xx_hal::gpio::Output<PushPull>>,
        // trellis: NeoTrellis<, Delay>,
        delay: Delay,
        i2c: I2c<I2C1, PB6<Alternate<AF1>>, PB7<Alternate<AF1>>>,
        buffer: Buffer<U256>,
        usb_chan: UsbChannels,
        cli_chan: ClientChannels,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
        static mut HOST_TO_DEVICE: Queue<HostToDeviceMessages, U16, u8> = Queue(ConstQueue::u8());
        static mut DEVICE_TO_HOST: Queue<DeviceToHostMessages, U16, u8> = Queue(ConstQueue::u8());

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

        //////////////////////////////////////////////////////////////////////
        // Set up USB as a CDC ACM Serial Port
        //////////////////////////////////////////////////////////////////////
        *USB_BUS = Some(UsbBus::new(Peripheral {
            usb,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        }));

        let mut serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let mut usb_dev =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27DD))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();

        //////////////////////////////////////////////////////////////////////
        // Set up channels
        //////////////////////////////////////////////////////////////////////
        let (host_tx, host_rx) = HOST_TO_DEVICE.split();
        let (devc_tx, devc_rx) = DEVICE_TO_HOST.split();

        // let trellis = neotrellis::NeoTrellis::new(i2c, delay, None).unwrap();

        usb_dev.poll(&mut [&mut serial]);

        init::LateResources {
            usb_dev,
            serial,
            led,
            i2c,
            delay,
            buffer: Buffer::new(),
            usb_chan: UsbChannels {
                incoming: host_tx,
                outgoing: devc_rx,
            },
            cli_chan: ClientChannels {
                incoming: host_rx,
                outgoing: devc_tx,
            },
        }
    }

    #[task(binds = USB, resources = [usb_dev, serial, buffer, usb_chan])]
    fn usb_tx(mut cx: usb_tx::Context) {
        crate::usb::usb_poll(&mut cx);
    }

    #[idle(resources = [cli_chan, i2c, led, delay])]
    fn idle(mut cx: idle::Context) -> ! {
        match i2c_rpc::task(&mut cx) {
            Ok(_) => loop { continue; },
            Err(_) => panic!(),
        }
    }
};
