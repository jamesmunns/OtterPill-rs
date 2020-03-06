#![no_main]
#![no_std]

use panic_persist as _;

use rtfm::app;
use stm32_usbd::UsbBus;
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB13, PB6, PB7},
        Alternate, PushPull, AF1,
        Pin, Input, Output, OpenDrain, Floating,
    },
    i2c::I2c,
    prelude::*,
    stm32::TIM7,
    stm32f0::stm32f0x2::I2C1,
    timers::{Event, Timer},
    usb::Peripheral,
    watchdog::Watchdog,
    stm32::Interrupt,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};

use usbd_hid::hid_class::{HIDClass};
use usbd_hid::descriptor::MouseReport;
use usbd_hid::descriptor::generator_prelude::*;

use heapless::{
    i::Queue as ConstQueue,
    spsc::{Consumer, Producer, Queue},
    consts::*,
};

pub struct Button {
    pub switch: Pin<Input<Floating>>,
    pub led: Pin<Output<OpenDrain>>,
}

pub struct Panel {
    pub buttons: [Button; 3]
}

type HalI2C1 = I2c<I2C1, PB6<Alternate<AF1>>, PB7<Alternate<AF1>>>;

#[app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        hid: HIDClass<'static, UsbBus<Peripheral>>,
        led: PB13<stm32f0xx_hal::gpio::Output<PushPull>>,
        i2c: HalI2C1,
        ms_timer: Timer<TIM7>,
        toggle: bool,
        stepdown: u32,
        wdog: Watchdog,
        panel: Panel,
        rpt_tx: Producer<'static, MouseReport, U32, u8>,
        rpt_rx: Consumer<'static, MouseReport, U32, u8>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
        static mut REPORTS: Queue<MouseReport, U32, u8> = Queue(ConstQueue::u8());

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

        let (usb_dm, usb_dp, mut led, mut i2c, _delay, ms_timer, wdog, butt_0, butt_1, butt_2, led_b0, led_b1, led_b2) =
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

                let butt_0 = gpiob.pb15.into_floating_input(cs).downgrade();
                let butt_1 = gpioa.pa8.into_floating_input(cs).downgrade();
                let butt_2 = gpiob.pb1.into_floating_input(cs).downgrade();

                let mut led_b0 = gpiob.pb8.into_open_drain_output(cs).downgrade();
                let mut led_b1 = gpioa.pa15.into_open_drain_output(cs).downgrade();
                let mut led_b2 = gpiob.pb2.into_open_drain_output(cs).downgrade();

                led_b0.set_high().ok();
                led_b1.set_high().ok();
                led_b2.set_high().ok();

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

                let mut ms_timer = Timer::tim7(tim7, 1000.hz(), &mut rcc);
                ms_timer.listen(Event::TimeOut);

                let mut wdog = Watchdog::new(wdog);
                wdog.start(2.hz());

                (usb_dm, usb_dp, led, i2c, delay, ms_timer, wdog, butt_0, butt_1, butt_2, led_b0, led_b1, led_b2)
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

        // Set the Gyroscope to 50Hz, active, 250deg/sec resolution
        i2c.write(0x21, &[0x0D, 0b00_0_00_0_11]).unwrap();
        i2c.write(0x21, &[0x13, 0b000_100_10]).unwrap();

        //////////////////////////////////////////////////////////////////////
        // Set up USB as a CDC ACM Serial Port
        //////////////////////////////////////////////////////////////////////
        *USB_BUS = Some(UsbBus::new(Peripheral {
            usb,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        }));

        let hid = HIDClass::new(
            USB_BUS.as_ref().unwrap(),
            MouseReport::desc(),
            60
        );

        let usb_dev =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27DD))
                .manufacturer("FerrousSystems")
                .product("SplitMouse")
                .serial_number("peter001")
                .device_class(0xEF)
                .build();

        let panel = Panel {
            buttons: [
                Button { switch: butt_0, led: led_b0 },
                Button { switch: butt_1, led: led_b1 },
                Button { switch: butt_2, led: led_b2 },
            ]
        };

        let (rpt_tx, rpt_rx) = REPORTS.split();

        //////////////////////////////////////////////////////////////////////
        // Set up channels
        //////////////////////////////////////////////////////////////////////

        init::LateResources {
            usb_dev,
            hid,
            led,
            i2c,
            ms_timer,
            toggle: false,
            stepdown: 0,
            wdog,
            panel,
            rpt_tx,
            rpt_rx,
        }
    }

    #[task(binds = TIM7, resources = [
        led,
        ms_timer,
        toggle,
        stepdown,
        i2c,
        rpt_tx,
        panel,
        wdog,
    ])]
    fn clock_tick(cx: clock_tick::Context) {
        static mut BUTT_STATE: [bool; 3] = [false; 3];
        static mut SCROLL_ACTIVE: bool = false;
        static mut DELAY: u8 = 0;

        const LEFT_BUTTON: usize   = 0;
        const RIGHT_BUTTON: usize  = 1;
        const MIDDLE_BUTTON: usize = 2;

        // Clear interrupt
        cx.resources.ms_timer.wait().ok();

        // Update button state
        let new_butt = [
            cx.resources.panel.buttons[0].switch.is_low().unwrap(),
            cx.resources.panel.buttons[1].switch.is_low().unwrap(),
            cx.resources.panel.buttons[2].switch.is_low().unwrap(),
        ];

        let diff_butt = new_butt != *BUTT_STATE;

        // Check for rising edge on middle button
        if !BUTT_STATE[MIDDLE_BUTTON] && new_butt[MIDDLE_BUTTON] {
            *SCROLL_ACTIVE = !*SCROLL_ACTIVE;
        }

        // Update LEDs
        if new_butt[LEFT_BUTTON] {
            cx.resources.panel.buttons[LEFT_BUTTON].led.set_low().unwrap();
        } else {
            cx.resources.panel.buttons[LEFT_BUTTON].led.set_high().unwrap();
        }
        if new_butt[RIGHT_BUTTON] {
            cx.resources.panel.buttons[RIGHT_BUTTON].led.set_low().unwrap();
        } else {
            cx.resources.panel.buttons[RIGHT_BUTTON].led.set_high().unwrap();
        }
        if *SCROLL_ACTIVE {
            cx.resources.panel.buttons[MIDDLE_BUTTON].led.set_low().unwrap();
        } else {
            cx.resources.panel.buttons[MIDDLE_BUTTON].led.set_high().unwrap();
        }

        let butt_report = {
            let mut rpt = 0u8;
            if new_butt[LEFT_BUTTON] {
                rpt |= 0b001;
            }
            if new_butt[RIGHT_BUTTON] {
                rpt |= 0b010;
            }
            if *SCROLL_ACTIVE {
                rpt |= 0b100;
            }
            rpt
        };

        *BUTT_STATE = new_butt;

        // Is there data ready?
        let mut out = [0u8; 6];
        cx.resources.i2c.write_read(
            0x21,
            &[0x07],
            &mut out[..1]
        ).unwrap();

        if (out[0] & 0b0000_1000) != 0 {
            cx.resources.i2c.write_read(
                0x21,
                &[0x01],
                &mut out
            ).unwrap();

            // Tweak data
            let x_raw = i8::from_le_bytes([out[2]]);
            let y_raw = i8::from_le_bytes([out[0]]);

            let x_notch = match x_raw {
                    -128 ..= -32 => -128,
                x @  -31 ..=  -8 => x * 4,
                x @   -7 ..=  -2 => x * 4,
                      -1 ..=   1 => 0,
                x @    2 ..=   7 => x * 4,
                x @    8 ..=  31 => x * 4,
                      32 ..= 127 => 127,
            };

            let y_notch = match y_raw {
                    -128 ..= -32 => -128,
                y @  -31 ..=  -8 => y * 4,
                y @   -7 ..=  -2 => y * 4,
                      -1 ..=   1 => 0,
                y @    2 ..=   7 => y * 4,
                y @    8 ..=  31 => y * 4,
                      32 ..= 127 => 127,
            };

            let mr = MouseReport {
                x: x_notch,
                y: y_notch,
                buttons: butt_report,
            };

            cx.resources.wdog.feed();
            cx.resources.rpt_tx.enqueue(mr).ok();
            rtfm::pend(Interrupt::USB);
            *DELAY = 0;
        }
        // else if diff_butt || *DELAY >= 5 {
        //     // No gyro data, but buttons changed
        //     let mr = MouseReport {
        //         x: 0,
        //         y: 0,
        //         buttons: butt_report,
        //     };
        //     cx.resources.rpt_tx.enqueue(mr).ok();
        //     rtfm::pend(Interrupt::USB);
        //     *DELAY = 0;
        // }
        else {
            *DELAY += 1;
        }
    }

    #[task(binds = USB, resources = [
        usb_dev,
        hid,
        rpt_rx,
    ])]
    fn usb_tx(cx: usb_tx::Context) {
        if let Some(mr) = cx.resources.rpt_rx.dequeue() {
            cx.resources.hid.push_input(&mr).ok();
        }

        cx.resources.usb_dev.poll(&mut [cx.resources.hid]);
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
};

#[cfg_attr(feature = "panic-dfu", cortex_m_rt::pre_init)]
#[allow(dead_code)]
unsafe fn before_main() {
    extern "C" {
        static mut _panic_dump_start: u8;
    }

    use cortex_m::register::msp;

    let start_ptr = &mut _panic_dump_start as *mut u8;

    // Panic-persist sets a flag to the start of the dump region
    // when a panic occurs
    if 0x0FACADE0 == core::ptr::read_unaligned(start_ptr.cast::<usize>()) {
        // Clear the flag
        start_ptr.cast::<usize>().write_unaligned(0x00000000);

        // The DFU bootloader's reset vector and initial stack pointer
        const SYSMEM_MSP: u32 = 0x1fffC800;
        const SYSMEM_RESET: u32 = 0x1fffC804;

        let dfu_msp = core::ptr::read(SYSMEM_MSP as *const u32);
        let putter: *const fn() = SYSMEM_RESET as *const fn();

        msp::write(dfu_msp);
        (*putter)();
    }
}
