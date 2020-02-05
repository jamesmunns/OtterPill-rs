use core::cmp::max;

use adafruit_neotrellis::{self as neotrellis, NeoTrellis};
use embedded_hal::{
    blocking::{
        delay::DelayUs,
        i2c::{Read, Write},
    },
    watchdog::Watchdog as _,
};
use heapless::{
    consts::*,
    spsc::{Consumer, Producer},
    String, Vec,
};
use panic_persist::get_panic_message_bytes;
use stm32f0xx_hal::{stm32::Interrupt, watchdog::Watchdog};
use vintage_icd::{CurrentState, DeviceToHostMessages, HostToDeviceMessages, StatusMessage};

use crate::{
    clock::RollingClock,
    colors::{self, RGB8},
    script::{
        engine::{Action, Actions, Behavior, Sequence},
        behaviors::{Cycler, StayColor, FadeColor},
    },
};

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

        // For profiling the event loop
        outgoing
            .enqueue(DeviceToHostMessages::CycleTime(RollingClock::since(start)))
            .ok();
        rtfm::pend(Interrupt::USB);

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
                    Actions::Fade(FadeColor::new_fade_down(150, color)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Static(StayColor::new(300, colors::BLACK)),
                    Behavior::OneShot,
                ),
                Action::new(
                    Actions::Fade(FadeColor::new_fade_up(150, colors[p_u])),
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
                        Actions::Fade(FadeColor::new_fade_down(150, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(600, colors::BLACK)), // 4
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
                        Actions::Fade(FadeColor::new_fade_down(150, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors::BLACK)), // 1
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(450, colors::BLACK)), // 3
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
                        Actions::Fade(FadeColor::new_fade_down(150, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(300, colors::BLACK)), // 2
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(300, colors::BLACK)), // 2
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
                        Actions::Fade(FadeColor::new_fade_down(150, colors[p_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(450, colors::BLACK)), // 3
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_up(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Fade(FadeColor::new_fade_down(150, colors[k_u])),
                        Behavior::OneShot,
                    ),
                    Action::new(
                        Actions::Static(StayColor::new(150, colors::BLACK)), // 1
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
