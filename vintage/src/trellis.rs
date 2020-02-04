use crate::clock::RollingClock;
use crate::colors;
use adafruit_neotrellis::{self as neotrellis, NeoTrellis};
use core::cmp::{max, min};
use embedded_hal::watchdog::Watchdog as _;
use heapless::{
    consts::*,
    spsc::{Consumer, Producer},
    String, Vec,
};
use libm::{cosf, fabsf, sinf};
use panic_persist::get_panic_message_bytes;
use stm32f0xx_hal::stm32::Interrupt;
use stm32f0xx_hal::watchdog::Watchdog;
use vintage_icd::{CurrentState, DeviceToHostMessages, HostToDeviceMessages, StatusMessage};

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::i2c::{Read, Write};

enum State {
    Idle {
        last_ping: u32,
    },
    TimingSelected {
        pin: u8,
        start_ms: u32,
        last_ping: u32,
    },
    Timeout {
        start_ms: u32,
        last_ping: u32,
    },
}

const WORK_DURATION_MS: u32 = 1000 * 60 * 30;
const TIMEOUT_NOTIFICATION_MS: u32 = 1000 * 60 * 5;
const PING_INTERVAL_MS: u32 = 500;

fn setup<I: Read + Write, D: DelayUs<u32>>(
    trellis: &mut NeoTrellis<I, D>,
    initial_colors: &mut [RGB8; 16],
    wdog: &mut Watchdog,
) -> Result<(), neotrellis::Error> {
    wdog.feed();

    trellis
        .neopixels()
        .set_speed(neotrellis::Speed::Khz800)?
        .set_pixel_count(16)?
        .set_pixel_type(neotrellis::ColorOrder::GRB)?
        .set_pin(3)?;

    for i in 0..16 {
        trellis
            .keypad()
            .enable_key_event(i, neotrellis::Edge::Rising)?
            .enable_key_event(i, neotrellis::Edge::Falling)?;
    }

    initial_colors.iter_mut().for_each(|mut c| {
        c.gamma_correct();
        c.r >>= 2;
        c.g >>= 2;
        c.b >>= 2;
    });

    Ok(())
}

fn step<I: Read + Write, D: DelayUs<u32>>(
    trellis: &mut NeoTrellis<I, D>,
    initial_colors: &[RGB8; 16],
    last_color: &mut [RGB8; 16],
    script: &mut [Sequence; 16],
    state: &mut State,
    incoming: &mut Consumer<HostToDeviceMessages, U16, u8>,
    outgoing: &mut Producer<DeviceToHostMessages, U16, u8>,
) -> Result<(), neotrellis::Error> {
    //////////////////////////////////////////////////////////////////
    // Process USB messages
    //////////////////////////////////////////////////////////////////
    if let Some(msg) = incoming.dequeue() {
        let responded = match msg {
            HostToDeviceMessages::Ping => {
                outgoing.enqueue(DeviceToHostMessages::Ack).ok();
                true
            }
            HostToDeviceMessages::GetPanic => {
                let string = if let Some(msg_b) = get_panic_message_bytes() {
                    let mut buf = Vec::new();
                    buf.extend_from_slice(msg_b).ok();
                    unsafe { String::from_utf8_unchecked(buf) }
                } else {
                    let mut msg = String::new();
                    msg.push_str("No Panic :)").ok();
                    msg
                };

                outgoing.enqueue(DeviceToHostMessages::Panic(string)).ok();
                true
            }
            HostToDeviceMessages::Reset => panic!(),
            _ => false,
        };

        if responded {
            rtfm::pend(Interrupt::USB);
        }
    }

    //////////////////////////////////////////////////////////////////
    // Any pending outgoing messages?
    //////////////////////////////////////////////////////////////////
    let (next, msg) = match state {
        State::Idle { last_ping } => {
            if RollingClock::since(*last_ping) >= PING_INTERVAL_MS {
                let now = RollingClock::get_ms();
                (
                    Some(State::Idle { last_ping: now }),
                    Some(DeviceToHostMessages::Status(StatusMessage {
                        current_tick: now,
                        state: CurrentState::Idle,
                    })),
                )
            } else {
                (None, None)
            }
        }
        State::TimingSelected {
            start_ms,
            pin,
            last_ping,
        } => {
            if RollingClock::since(*last_ping) >= PING_INTERVAL_MS {
                let now = RollingClock::get_ms();
                (
                    Some(State::TimingSelected {
                        start_ms: *start_ms,
                        pin: *pin,
                        last_ping: now,
                    }),
                    Some(DeviceToHostMessages::Status(StatusMessage {
                        current_tick: now,
                        state: CurrentState::Timing {
                            pin: *pin,
                            elapsed: RollingClock::since(*start_ms),
                        },
                    })),
                )
            } else {
                (None, None)
            }
        }
        State::Timeout {
            start_ms,
            last_ping,
        } => {
            if RollingClock::since(*last_ping) >= PING_INTERVAL_MS {
                let now = RollingClock::get_ms();

                (
                    Some(State::Timeout {
                        start_ms: *start_ms,
                        last_ping: now,
                    }),
                    Some(DeviceToHostMessages::Status(StatusMessage {
                        current_tick: now,
                        state: CurrentState::Timeout {
                            elapsed: RollingClock::since(*start_ms),
                        },
                    })),
                )
            } else {
                (None, None)
            }
        }
    };

    if let Some(next) = next {
        *state = next;
    }

    if let Some(msg) = msg {
        outgoing.enqueue(msg).ok();
        rtfm::pend(Interrupt::USB);
    }

    //////////////////////////////////////////////////////////////////
    // Process button presses
    //////////////////////////////////////////////////////////////////
    // Check for button events
    for evt in trellis.keypad().get_events()?.as_slice() {
        if evt.event == neotrellis::Edge::Rising {
            let next = match state {
                State::Idle { .. } => {
                    select_action(evt.key, script, &initial_colors);
                    let now = RollingClock::get_ms();

                    Some(State::TimingSelected {
                        pin: evt.key,
                        start_ms: now,
                        last_ping: now,
                    })
                }
                State::TimingSelected { .. } => {
                    idle_action(script, &initial_colors);
                    let now = RollingClock::get_ms();
                    Some(State::Idle { last_ping: now })
                }
                State::Timeout { .. } => {
                    idle_action(script, &initial_colors);
                    let now = RollingClock::get_ms();
                    Some(State::Idle { last_ping: now })
                }
            };

            if let Some(next) = next {
                *state = next;
            }
        }
    }

    // Check for timeout events
    let next = match state {
        State::Idle { .. } => None,
        State::TimingSelected { start_ms, pin, .. } => {
            if RollingClock::since(*start_ms) >= WORK_DURATION_MS {
                timeout_action(*pin, script, &initial_colors);

                let now = RollingClock::get_ms();
                Some(State::Timeout {
                    start_ms: now,
                    last_ping: now,
                })
            } else {
                None
            }
        }
        State::Timeout { start_ms, .. } => {
            if RollingClock::since(*start_ms) >= TIMEOUT_NOTIFICATION_MS {
                idle_action(script, &initial_colors);
                Some(State::Idle {
                    last_ping: RollingClock::get_ms(),
                })
            } else {
                None
            }
        }
    };

    if let Some(next) = next {
        *state = next;
    }

    // Update the screen
    let mut any = false;
    for i in 0..16 {
        if let Some(pix) = script[i].poll() {
            if last_color[i] != pix {
                any = true;
                trellis
                    .neopixels()
                    .set_pixel_rgb(i as u8, pix.r, pix.g, pix.b)?;
                last_color[i] = pix;
            }
        }
    }

    if any {
        trellis.neopixels().show()?;
    }

    Ok(())
}

pub fn trellis_task(cx: &mut crate::idle::Context) -> Result<(), neotrellis::Error> {
    let trellis = &mut *cx.resources.trellis;
    let incoming = &mut cx.resources.cli_chan.incoming;
    let outgoing = &mut cx.resources.cli_chan.outgoing;
    let wdog = &mut cx.resources.wdog;

    let mut initial_colors = [
        colors::ORANGE_RED,
        colors::CRIMSON,
        colors::DARK_ORANGE,
        colors::DARK_SALMON,
        colors::GOLD,
        colors::YELLOW,
        colors::FOREST_GREEN,
        colors::LAWN_GREEN,
        colors::BLUE,
        colors::DODGER_BLUE,
        colors::DARK_VIOLET,
        colors::HOT_PINK,
        colors::VIOLET,
        colors::INDIGO,
        colors::DARK_GRAY,
        colors::LIGHT_YELLOW,
    ];

    let mut last_color = [RGB8 { r: 0, g: 0, b: 0 }; 16];

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

    let mut state = State::Idle {
        last_ping: RollingClock::get_ms(),
    };

    setup(trellis, &mut initial_colors, wdog)?;

    idle_action(&mut script, &initial_colors);

    loop {
        wdog.feed();
        let start = RollingClock::get_ms();

        step(
            trellis,
            &initial_colors,
            &mut last_color,
            &mut script,
            &mut state,
            incoming,
            outgoing,
        )?; // TODO

        // // For profiling the event loop
        // outgoing
        //     .enqueue(DeviceToHostMessages::CycleTime(RollingClock::since(start)))
        //     .ok();
        // rtfm::pend(Interrupt::USB);

        loop {
            if RollingClock::since(start) >= 20 {
                break;
            }
        }
    }
}

struct IPos {
    x: i8,
    y: i8,
}

impl IPos {
    #[allow(dead_code)]
    fn move_xy_pin(&self, dx: i8, dy: i8) -> Option<u8> {
        let x = self.x + dx;
        let y = self.y + dy;
        if ((x < 0) || (x > 3)) || ((y < 0) || (y > 3)) {
            None
        } else {
            Some(IPos { x, y }.to_pin())
        }
    }

    fn from_pin(pin: u8) -> Self {
        IPos {
            x: (pin & 0b11) as i8,
            y: (pin >> 2) as i8,
        }
    }

    fn distance_from_pin(&self, other_pin: u8) -> u8 {
        let other = IPos::from_pin(other_pin);
        let dx = (self.x - other.x).abs() as u8;
        let dy = (self.y - other.y).abs() as u8;
        max(dx, dy)
    }

    fn to_pin(&self) -> u8 {
        ((self.y as u8) << 2) | (self.x as u8)
    }
}

fn timeout_action(key: u8, script: &mut [Sequence; 16], colors: &[RGB8; 16]) {
    let k_u = key as usize;

    for p in 0..16 {
        let p_u = p as usize;

        let color = &colors[k_u];

        script[p_u].set(
            &[
                Action::new(
                    Actions::Static(StayColor::new(300, *color)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Static(StayColor::new(600, colors::BLACK)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Static(StayColor::new(300, *color)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Static(StayColor::new(2400, colors::BLACK)),
                    Behavior::OneShot,
                ),
            ],
            Behavior::LoopForever,
        );
    }
}

fn idle_action(script: &mut [Sequence; 16], colors: &[RGB8; 16]) {
    for p in 0..16 {
        let p_u = p as usize;

        let color = script[p_u].poll().unwrap_or_else(|| colors::BLACK);

        script[p_u].set(
            &[
                Action::new(
                    Actions::Fade(FadeColor::new_fade_down(100, color)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Static(StayColor::new(200, colors::BLACK)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Fade(FadeColor::new_fade_up(100, colors[p_u])),
                    Behavior::OneShot,
                ),
            ],
            Behavior::OneShot,
        );
    }
}

fn select_action(key: u8, script: &mut [Sequence; 16], colors: &[RGB8; 16]) {
    let pos = IPos::from_pin(key);
    let k_u = key as usize;

    let mut soft_color = colors[k_u];
    soft_color.r >>= 1;
    soft_color.g >>= 1;
    soft_color.b >>= 1;

    for p in 0..16 {
        let p_u = p as usize;

        script[p_u].clear();

        match pos.distance_from_pin(p) {
            0 => script[p_u].set(
                &[
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(400, colors::BLACK)), // 4
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Sin(Cycler::new(10000.0, 10000, soft_color)),
                        Behavior::LoopForever,
                    ),
                ],
                Behavior::OneShot,
            ),
            1 => script[p_u].set(
                &[
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors::BLACK)), // 1
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(300, colors::BLACK)), // 3
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Sin(Cycler::new(10000.0, 10000, soft_color)),
                        Behavior::LoopForever,
                    ),
                ],
                Behavior::OneShot,
            ),
            2 => script[p_u].set(
                &[
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(200, colors::BLACK)), // 2
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(200, colors::BLACK)), // 2
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Sin(Cycler::new(10000.0, 10000, soft_color)),
                        Behavior::LoopForever,
                    ),
                ],
                Behavior::OneShot,
            ),
            3 => script[p_u].set(
                &[
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(300, colors::BLACK)), // 3
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(100, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(100, colors::BLACK)), // 1
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Sin(Cycler::new(10000.0, 10000, soft_color)),
                        Behavior::LoopForever,
                    ),
                ],
                Behavior::OneShot,
            ),
            _ => panic!(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGB8 {
    pub r: u8,
    pub g: u8,
    pub b: u8,
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
        self.clear();

        self.seq.extend_from_slice(&actions[..amt]).ok();
        self.behavior = behavior;
    }

    fn poll(&mut self) -> Option<RGB8> {
        if self.seq.is_empty() || (self.position >= self.seq.len()) {
            return None;
        }

        let behavior = &mut self.behavior;
        let seq = &mut self.seq;
        let position = &mut self.position;

        use Behavior::*;
        match behavior {
            OneShot => seq[*position].poll().or_else(|| {
                *position += 1;
                if *position < seq.len() {
                    seq[*position].reinit();
                    seq[*position].poll()
                } else {
                    None
                }
            }),
            LoopForever => seq[*position].poll().or_else(|| {
                *position += 1;

                if *position >= seq.len() {
                    *position = 0;
                }

                seq[*position].reinit();
                seq[*position].poll()
            }),
            LoopN {
                ref mut current,
                cycles,
            } => seq[*position].poll().or_else(|| {
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
            }),
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
        Self { action, behavior }
    }

    fn reinit(&mut self) {
        self.action.reinit();

        use Behavior::*;
        match &mut self.behavior {
            OneShot => {}
            LoopForever => {}
            LoopN {
                ref mut current, ..
            } => {
                *current = 0;
            }
        }
    }

    fn poll(&mut self) -> Option<RGB8> {
        use Behavior::*;

        let action = &mut self.action;
        let behavior = &mut self.behavior;

        match behavior {
            OneShot => action.poll(),
            LoopForever => action.poll().or_else(|| {
                action.reinit();
                action.poll()
            }),
            LoopN {
                ref mut current,
                cycles,
            } => action.poll().or_else(|| {
                if *current < *cycles {
                    *current += 1;
                    action.poll()
                } else {
                    None
                }
            }),
        }
    }
}

#[derive(Clone)]
enum Behavior {
    OneShot,
    LoopForever,

    #[allow(dead_code)]
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
            start_ms: RollingClock::get_ms(),
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
            start_ms: RollingClock::get_ms(),
            duration_ms,
            color,
        }
    }

    fn reinit(&mut self) {
        self.start_ms = RollingClock::get_ms();
    }

    fn poll(&self) -> Option<RGB8> {
        if RollingClock::since(self.start_ms) >= self.duration_ms {
            None
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

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_low();

        Self { cycler }
    }

    fn new_fade_down(duration_ms: u32, color: RGB8) -> Self {
        let period_ms = (duration_ms as f32) * 2.0;

        let mut cycler = Cycler::new(period_ms, duration_ms, color);
        cycler.start_high();

        Self { cycler }
    }

    fn reinit(&mut self) {
        self.cycler.reinit();
    }

    fn poll(&self) -> Option<RGB8> {
        self.cycler.poll()
    }
}
