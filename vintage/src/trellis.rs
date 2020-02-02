use adafruit_neotrellis as neotrellis;
use vintage_icd::{DeviceToHostMessages, HostToDeviceMessages};
use crate::clock::RollingClock;
use libm::{sinf, cosf, fabsf};
use stm32f0xx_hal::stm32::Interrupt;
use embedded_hal::watchdog::Watchdog;
use heapless::{Vec, consts::*};
use core::cmp::{min, max};

pub fn trellis_task(cx: &mut crate::idle::Context) -> Result<(), neotrellis::Error> {
    let mut color: u32 = 0b1001_0010_0100_1001;
    let trellis = &mut *cx.resources.trellis;
    let incoming = &mut cx.resources.cli_chan.incoming;
    let outgoing = &mut cx.resources.cli_chan.outgoing;
    let wdog = &mut cx.resources.wdog;

    wdog.feed();

    trellis
        .neopixels()
        .set_speed(neotrellis::Speed::Khz800)?
        .set_pixel_count(16)?
        .set_pixel_type(neotrellis::ColorOrder::GRB)?
        .set_pin(3)?;

    // Cycle the board through some initial colors
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
                .enable_key_event(i, neotrellis::Edge::Rising)?
                .enable_key_event(i, neotrellis::Edge::Falling)?;

            trellis
                .neopixels()
                .set_pixel_rgb(i, cols[1], cols[0], cols[2])?;
        }

        trellis.neopixels().show()?;
        trellis.seesaw().delay_us(150_000u32);
        wdog.feed();
    }

    let mut sticky = [false; 16];

    let mut script: [Sequence; 16] = [
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),

        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),

        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),

        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
        Sequence::empty(),
    ];

    script[0].set(
        &[
            Action::new(Actions::Static(StayColor::new(1000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Sin(Cycler::new(2000.0, 2000, RGB8 { r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Sin(Cycler::new(2000.0, 2000, RGB8 { r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Sin(Cycler::new(2000.0, 2000, RGB8 { r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(3000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[1].set(
        &[
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[2].set(
        &[
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[3].set(
        &[
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[4].set(
        &[
            Action::new(Actions::Static(StayColor::new(1000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(3000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[5].set(
        &[
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[6].set(
        &[
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[7].set(
        &[
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[8].set(
        &[
            Action::new(Actions::Static(StayColor::new(1000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(3000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[9].set(
        &[
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[10].set(
        &[
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[11].set(
        &[
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[12].set(
        &[
            Action::new(Actions::Static(StayColor::new(1000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(3000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[13].set(
        &[
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[14].set(
        &[
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(2000, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    script[15].set(
        &[
            Action::new(Actions::Static(StayColor::new(2500, RGB8 { r: 0x00, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0xFF, g: 0x00, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0xFF, b: 0x00})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_up(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Fade(FadeColor::new_fade_down(1000, RGB8{ r: 0x00, g: 0x00, b: 0xFF})), Behavior::OneShot),
            Action::new(Actions::Static(StayColor::new(1500, RGB8 { r: 0xFF, g: 0xFF, b: 0xFF})), Behavior::OneShot),
        ],
        Behavior::LoopN { current: 0, cycles: 5 },
    );

    'outer: loop {
        wdog.feed();

        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////
        if let Some(msg) = incoming.dequeue() {
            use DeviceToHostMessages::*;
            use HostToDeviceMessages::*;

            if Ping == msg {
                outgoing.enqueue(Ack).ok();
                rtfm::pend(Interrupt::USB);
            } else if Reset == msg {
                panic!();
            }
        }

        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////
        trellis.seesaw().delay_us(20_000u32);

        for i in 0..16 {
            if let Some(mut pix) = script[i].poll() {
                pix.gamma_correct();

                trellis.neopixels().set_pixel_rgb(
                    i as u8,
                    pix.r,
                    pix.g,
                    pix.b,
                )?;
            }
        }

        trellis.neopixels().show()?;
    }
}


#[derive(Copy, Clone)]
struct RGB8 {
    r: u8,
    g: u8,
    b: u8,
}

impl RGB8 {
    fn gamma_correct(&mut self) {
        // From the smart-leds-rs crate
        const GAMMA8: [u8; 256] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4,
            4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11,
            12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
            22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37,
            38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58,
            59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85,
            86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
            115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144,
            146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177, 180,
            182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220,
            223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
        ];

        self.r = GAMMA8[self.r as usize];
        self.g = GAMMA8[self.g as usize];
        self.b = GAMMA8[self.b as usize];
    }
}

struct Sequence {
    seq: Vec<Action, U8>,
    position: usize,
    behavior: Behavior,
}

impl Sequence {
    fn empty() -> Self {
        Self {
            seq: Vec::new(),
            position: 0,
            behavior: Behavior::OneShot,
        }
    }

    fn clear(&mut self) {
        self.seq.clear();
        self.position = 0;
    }

    fn set(&mut self, actions: &[Action], behavior: Behavior) {
        let amt = min(self.seq.capacity(), actions.len());
        self.seq.clear();
        self.seq.extend_from_slice(&actions[..amt]).ok();
        self.behavior = behavior;
        self.position = 0;
    }

    fn poll(&mut self) -> Option<RGB8> {
        if self.seq.is_empty() {
            return None;
        } else if self.position >= self.seq.len() {
            return None;
        }

        let behavior = &mut self.behavior;
        let seq = &mut self.seq;
        let position = &mut self.position;

        use Behavior::*;
        match behavior {
            OneShot => {
                seq[*position]
                    .poll()
                    .or_else(|| {
                        *position += 1;
                        if *position < seq.len() {
                            seq[*position].reinit();
                            seq[*position].poll()
                        } else {
                            None
                        }
                    })
            }
            LoopForever => {
                seq[*position]
                    .poll()
                    .or_else(|| {
                        *position += 1;

                        if *position >= seq.len() {
                            *position = 0;
                        }

                        seq[*position].reinit();
                        seq[*position].poll()
                    })
            }
            LoopN{ ref mut current, cycles } => {
                seq[*position]
                    .poll()
                    .or_else(|| {
                        *position += 1;

                        if *position >= seq.len() {
                            if *current < *cycles {
                                *position = 0;
                                *current += 1;
                                seq[*position].reinit();
                                seq[*position].poll()
                            } else {
                                None
                            }
                        } else {
                            seq[*position].reinit();
                            seq[*position].poll()
                        }


                    })
            }
        }
    }
}

#[derive(Clone)]
struct Action {
    action: Actions,
    behavior: Behavior,
}

impl Action {
    fn new(action: Actions, behavior: Behavior) -> Self {
        Self {
            action,
            behavior,
        }
    }

    fn reinit(&mut self) {
        self.action.reinit();

        use Behavior::*;
        match &mut self.behavior {
            OneShot => {},
            LoopForever => {},
            LoopN { ref mut current, .. } => {
                *current = 0;
            }
        }
    }

    fn poll(&mut self) -> Option<RGB8> {
        use Behavior::*;
        use Actions::*;

        let action = &mut self.action;
        let behavior = &mut self.behavior;

        match behavior {
            OneShot => {
                action.poll()
            },
            LoopForever => {
                action.poll()
                    .or_else(|| {
                        action.reinit();
                        action.poll()
                    })
            },
            LoopN { ref mut current, cycles } => {
                action.poll()
                    .or_else(|| {
                        if *current < *cycles {
                            *current += 1;
                            action.poll()
                        } else {
                            None
                        }
                    })
            }
        }
    }
}

#[derive(Clone)]
enum Behavior {
    OneShot,
    LoopForever,
    LoopN {
        current: usize,
        cycles: usize,
    },
}

#[derive(Clone)]
enum Actions {
    Sin(Cycler),
    Static(StayColor),
    Fade(FadeColor),
}

impl Actions {
    fn reinit(&mut self) {
        use Actions::*;

        match self {
            Sin(s) => s.reinit(),
            Static(s) => s.reinit(),
            Fade(f) => f.reinit(),
        }
    }

    fn poll(&self) -> Option<RGB8> {
        use Actions::*;
        match self {
            Sin(s) => s.poll(),
            Static(s) => s.poll(),
            Fade(f) => f.poll(),
        }
    }
}

#[derive(Clone)]
struct Cycler {
    start_ms: u32,
    period_ms: f32,
    duration_ms: u32,
    color: RGB8,
    func: fn(f32) -> f32,
}

// Methods:
//
// reinit(): reinitialize with the current time
// poll() -> Option<RGB8>: Some if updated color, None if action is complete

impl Cycler {
    fn new(period_ms: f32, duration_ms: u32, color: RGB8) -> Self {
        // Since we "rectify" the sine wave, it actually has a period that
        // looks half as long.
        let period_ms = period_ms * 2.0;

        Self {
            start_ms: 0,
            period_ms,
            duration_ms,
            color,
            func: sinf,
        }
    }

    fn reinit(&mut self) {
        self.start_ms = RollingClock::get_ms();
    }

    fn poll(&self) -> Option<RGB8> {
        if RollingClock::since(self.start_ms) >= self.duration_ms {
            return None;
        }

        let delta = RollingClock::since(self.start_ms);

        let deltaf = (delta as f32);
        let normalized = deltaf / self.period_ms;
        let rad_norm = normalized * 2.0 * core::f32::consts::PI;
        let out_norm = (self.func)(rad_norm);
        let abs_out = fabsf(out_norm);

        let retval = RGB8 {
            r: (abs_out * (self.color.r as f32)) as u8,
            g: (abs_out * (self.color.g as f32)) as u8,
            b: (abs_out * (self.color.b as f32)) as u8,
        };

        Some(retval)
    }

    fn start_high(&mut self) {
        self.func = cosf
    }

    fn start_low(&mut self) {
        self.func = sinf
    }
}

#[derive(Clone)]
struct StayColor {
    start_ms: u32,
    duration_ms: u32,
    color: RGB8,
}

impl StayColor {
    fn new(duration_ms: u32, color: RGB8) -> Self {
        Self {
            start_ms: 0,
            duration_ms,
            color,
        }
    }

    fn reinit(&mut self) {
        self.start_ms = RollingClock::get_ms();
    }

    fn poll(&self) -> Option<RGB8> {
        if RollingClock::since(self.start_ms) >= self.duration_ms {
            return None;
        } else {
            Some(self.color)
        }
    }
}

#[derive(Clone)]
struct FadeColor {
    cycler: Cycler,
}

impl FadeColor {
    fn new_fade_up(duration_ms: u32, color: RGB8) -> Self {
        let period_ms = (duration_ms as f32) * 2.0;

        let mut cycler = Cycler::new(
            period_ms,
            duration_ms,
            color,
        );
        cycler.start_low();

        Self {
            cycler
        }
    }

    fn new_fade_down(duration_ms: u32, color: RGB8) -> Self {
        let period_ms = (duration_ms as f32) * 2.0;

        let mut cycler = Cycler::new(
            period_ms,
            duration_ms,
            color,
        );
        cycler.start_high();

        Self {
            cycler
        }
    }

    fn reinit(&mut self) {
        self.cycler.reinit();
    }

    fn poll(&self) -> Option<RGB8> {
        self.cycler.poll()
    }
}
