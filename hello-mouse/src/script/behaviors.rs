use crate::clock::RollingClock;
use crate::colors::RGB8;
use libm::{cosf, fabsf, sinf};

#[derive(Clone, Debug, Default)]
pub struct StayColor {
    start_ms: u32,
    pub duration_ms: u32,
    pub color: RGB8,
}

impl StayColor {
    pub fn new(duration_ms: u32, color: RGB8) -> Self {
        Self {
            start_ms: RollingClock::get_ms(),
            duration_ms,
            color,
        }
    }

    pub fn reinit(&mut self) {
        self.start_ms = RollingClock::get_ms();
    }

    pub fn poll(&self) -> Option<RGB8> {
        if RollingClock::since(self.start_ms) >= self.duration_ms {
            None
        } else {
            Some(self.color)
        }
    }
}

#[derive(Clone, Debug)]
pub struct Cycler {
    start_ms: u32,
    pub period_ms: f32,
    pub duration_ms: u32,
    pub color: RGB8,
    func: fn(f32) -> f32,
}

// Methods:
//
// reinit(): reinitialize with the current time
// poll() -> Option<RGB8>: Some if updated color, None if action is complete

impl Cycler {
    pub fn new(period_ms: f32, duration_ms: u32, color: RGB8) -> Self {
        // Since we "rectify" the sine wave, it actually has a period that
        // looks half as long.
        let period_ms = period_ms * 2.0;

        Self {
            start_ms: RollingClock::get_ms(),
            period_ms,
            duration_ms,
            color,
            func: sinf,
        }
    }

    pub fn reinit(&mut self) {
        self.start_ms = RollingClock::get_ms();
    }

    pub fn poll(&self) -> Option<RGB8> {
        let delta = RollingClock::since(self.start_ms);

        if delta >= self.duration_ms {
            return None;
        }

        let deltaf = delta as f32;
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

    pub fn start_high(&mut self) {
        self.func = cosf
    }

    pub fn start_low(&mut self) {
        self.func = sinf
    }
}

#[derive(Clone, Debug)]
pub struct FadeColor {
    pub cycler: Cycler,
}

impl FadeColor {
    pub fn new_fade_up(duration_ms: u32, color: RGB8) -> Self {
        let period_ms = (duration_ms as f32) * 2.0;

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_low();

        Self { cycler }
    }

    pub fn new_fade_down(duration_ms: u32, color: RGB8) -> Self {
        let period_ms = (duration_ms as f32) * 2.0;

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_high();

        Self { cycler }
    }

    pub fn reinit(&mut self) {
        self.cycler.reinit();
    }

    pub fn poll(&self) -> Option<RGB8> {
        self.cycler.poll()
    }

    pub fn inner_mut(&mut self) -> &mut Cycler {
        &mut self.cycler
    }
}
