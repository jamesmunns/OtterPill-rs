#![no_main]
#![no_std]

use panic_persist as _;

use adafruit_neotrellis::{self as neotrellis, NeoTrellis};
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
    stm32::TIM7,
    stm32f0::stm32f0x2::I2C1,
    timers::{Event, Timer},
    usb::Peripheral,
    watchdog::Watchdog,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use vintage_icd::{
    cobs_buffer::{consts::*, Buffer},
    DeviceToHostMessages, HostToDeviceMessages,
};

mod clock;
mod colors;
mod trellis;
mod usb;

pub struct UsbChannels {
    incoming: Producer<'static, HostToDeviceMessages, U16, u8>,
    outgoing: Consumer<'static, DeviceToHostMessages, U16, u8>,
}

pub struct ClientChannels {
    incoming: Consumer<'static, HostToDeviceMessages, U16, u8>,
    outgoing: Producer<'static, DeviceToHostMessages, U16, u8>,
}

type HalI2C1 = I2c<I2C1, PB6<Alternate<AF1>>, PB7<Alternate<AF1>>>;

#[app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        serial: SerialPort<'static, UsbBus<Peripheral>>,
        led: PB13<stm32f0xx_hal::gpio::Output<PushPull>>,
        trellis: NeoTrellis<HalI2C1, Delay>,
        buffer: Buffer<U256>,
        usb_chan: UsbChannels,
        cli_chan: ClientChannels,
        ms_timer: Timer<TIM7>,
        toggle: bool,
        stepdown: u32,
        wdog: Watchdog,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
        static mut HOST_TO_DEVICE: Queue<HostToDeviceMessages, U16, u8> = Queue(ConstQueue::u8());
        static mut DEVICE_TO_HOST: Queue<DeviceToHostMessages, U16, u8> = Queue(ConstQueue::u8());

        //////////////////////////////////////////////////////////////////////
        // Set up the hardware!
        //////////////////////////////////////////////////////////////////////
        let (usb, rcc, crs, mut flash, gpioa, gpiob, i2c1, syst, tim7, wdog) = (
            cx.device.USB,
            cx.device.RCC,
            cx.device.CRS,
            cx.device.FLASH,
            cx.device.GPIOA,
            cx.device.GPIOB,
            cx.device.I2C1,
            cx.core.SYST,
            cx.device.TIM7,
            cx.device.IWDG,
        );

        let (usb_dm, usb_dp, mut led, i2c, delay, ms_timer, wdog) =
            cortex_m::interrupt::free(|cs| {
                let mut rcc = rcc
                    .configure()
                    .hsi48()
                    .enable_crs(crs)
                    .sysclk(48.mhz())
                    .pclk(24.mhz())
                    .freeze(&mut flash);

                let gpioa = gpioa.split(&mut rcc);
                let gpiob = gpiob.split(&mut rcc);

                let led = gpiob.pb13.into_push_pull_output(cs);

                let usb_dm = gpioa.pa11;
                let usb_dp = gpioa.pa12;

                let mut usb_dp = usb_dp.into_push_pull_output(&cs);
                usb_dp.set_low().ok();
                cortex_m::asm::delay(48_000_000 / 500);
                let usb_dp = usb_dp.into_floating_input(&cs);

                let scl = gpiob.pb6.into_alternate_af1(&cs);
                let sda = gpiob.pb7.into_alternate_af1(&cs);

                let i2c = I2c::i2c1(i2c1, (scl, sda), 100.khz(), &mut rcc);

                let delay = Delay::new(syst, &rcc);

                // TODO: https://github.com/stm32-rs/stm32f0xx-hal/issues/90
                // This timer seems to fire at twice the expected rate, for now
                // just half the time
                let mut ms_timer = Timer::tim7(tim7, 500.hz(), &mut rcc);
                ms_timer.listen(Event::TimeOut);

                let mut wdog = Watchdog::new(wdog);
                wdog.start(2.hz());

                (usb_dm, usb_dp, led, i2c, delay, ms_timer, wdog)
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

        let trellis = neotrellis::NeoTrellis::new(i2c, delay, None).unwrap();

        usb_dev.poll(&mut [&mut serial]);

        init::LateResources {
            usb_dev,
            serial,
            led,
            trellis,
            buffer: Buffer::new(),
            usb_chan: UsbChannels {
                incoming: host_tx,
                outgoing: devc_rx,
            },
            cli_chan: ClientChannels {
                incoming: host_rx,
                outgoing: devc_tx,
            },
            ms_timer,
            toggle: false,
            stepdown: 0,
            wdog,
        }
    }

    #[task(binds = TIM7, resources = [led, ms_timer, toggle, stepdown])]
    fn clock_tick(mut cx: clock_tick::Context) {
        clock::tick(&mut cx);
    }

    #[task(binds = USB, resources = [usb_dev, serial, buffer, usb_chan])]
    fn usb_tx(mut cx: usb_tx::Context) {
        crate::usb::usb_poll(&mut cx);
    }

    #[idle(resources = [trellis, cli_chan, wdog])]
    fn idle(mut cx: idle::Context) -> ! {
        match trellis::trellis_task(&mut cx) {
            Ok(_) => panic!(),
            Err(e) => panic!("{:?}", e),
        };
    }
};

// use cortex_m::register::msp;
// use cortex_m_rt::pre_init;
//
// #[pre_init]
// unsafe fn before_main() {
//     extern "C" {
//         static mut _panic_dump_start: u8;
//     }

//     let start_ptr = &mut _panic_dump_start as *mut u8;

//     // Panic-persist sets a flag to the start of the dump region
//     // when a panic occurs
//     if 0x0FACADE0 == core::ptr::read_unaligned(start_ptr.cast::<usize>()) {
//         // Clear the flag
//         start_ptr.cast::<usize>().write_unaligned(0x00000000);

//         // The DFU bootloader's reset vector and initial stack pointer
//         const SYSMEM_MSP: u32 = 0x1fffC800;
//         const SYSMEM_RESET: u32 = 0x1fffC804;

//         let dfu_msp = core::ptr::read(SYSMEM_MSP as *const u32);
//         let putter: *const fn() = SYSMEM_RESET as *const fn();

//         msp::write(dfu_msp);
//         (*putter)();
//     }
// }
