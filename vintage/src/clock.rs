use core::sync::atomic::{AtomicU32, Ordering};
use embedded_hal::digital::v2::OutputPin;
use stm32f0xx_hal::prelude::*;

pub struct RollingClock(());

impl RollingClock {
    pub fn get_ms() -> u32 {
        ROLLING_MILLIS.load(Ordering::Acquire)
    }

    pub fn since(before: u32) -> u32 {
        Self::get_ms().wrapping_sub(before)
    }
}

static ROLLING_MILLIS: AtomicU32 = AtomicU32::new(0);

pub fn tick(cx: &mut crate::clock_tick::Context) {
    cx.resources.ms_timer.wait().ok();

    // Update the millisecond clock. We only have load and store
    // on thumbv6, but we are the only writer.
    let now = ROLLING_MILLIS.load(Ordering::Acquire);
    ROLLING_MILLIS.store(now.wrapping_add(1), Ordering::Release);

    *cx.resources.stepdown += 1;

    if *cx.resources.stepdown >= 1000 {
        *cx.resources.stepdown = 0;
    } else {
        return;
    }

    if *cx.resources.toggle {
        cx.resources.led.set_low().ok();
    } else {
        cx.resources.led.set_high().ok();
    }
    *cx.resources.toggle = !*cx.resources.toggle
}
