use core::cmp::min;

use heapless::{consts::*, Vec};

use crate::{
    colors::RGB8,
    script::behaviors::{Cycler, FadeColor, StayColor},
};

#[derive(Clone, Debug, Default)]
pub struct Sequence {
    seq: Vec<Action, U8>,
    position: usize,
    behavior: Behavior,
}

#[derive(Clone, Debug)]
pub struct Action {
    action: Actions,
    behavior: Behavior,
}

#[derive(Clone, Debug)]
pub enum Actions {
    Sin(Cycler),
    Static(StayColor),
    Fade(FadeColor),
}

#[derive(Clone, Debug)]
pub enum Behavior {
    OneShot,
    LoopForever,

    #[allow(dead_code)]
    LoopN {
        current: usize,
        cycles: usize,
    },
    Nop,
}

#[derive(Debug)]
pub struct ActionBuilder(Action);

impl Default for Behavior {
    fn default() -> Self {
        Behavior::Nop
    }
}

impl Default for Actions {
    fn default() -> Self {
        Actions::Static(StayColor::new(0, RGB8 {r: 0, g: 0, b: 0 }))
    }
}

impl Sequence {
    pub fn empty() -> Self {
        Self {
            seq: Vec::new(),
            position: 0,
            behavior: Behavior::OneShot,
        }
    }

    pub fn clear(&mut self) {
        self.seq.clear();
        self.position = 0;
    }

    pub fn set(&mut self, actions: &[Action], behavior: Behavior) {
        let amt = min(self.seq.capacity(), actions.len());
        self.clear();

        self.seq.extend_from_slice(&actions[..amt]).ok();
        self.behavior = behavior;
    }

    pub fn poll(&mut self) -> Option<RGB8> {
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
            Nop => None,
        }
    }
}

impl Action {
    #![allow(dead_code)]

    pub fn new(action: Actions, behavior: Behavior) -> Self {
        Self { action, behavior }
    }

    pub fn build() -> ActionBuilder {
        ActionBuilder::new()
    }

    pub fn reinit(&mut self) {
        self.action.reinit();

        use Behavior::*;
        match &mut self.behavior {
            OneShot => {}
            LoopForever => {}
            Nop => {}
            LoopN {
                ref mut current, ..
            } => {
                *current = 0;
            }
        }
    }

    pub fn poll(&mut self) -> Option<RGB8> {
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
            Nop => None,
        }
    }
}

// Builder Methods
impl ActionBuilder {
    #![allow(dead_code)]

    pub fn new() -> Self {
        Self(Action {
            action: Actions::default(),
            behavior: Behavior::default(),
        })
    }

    pub fn finish(self) -> Action {
        self.0
    }

    pub fn times(mut self, ct: usize) -> Self {
        self.0.behavior = Behavior::LoopN { current: 0, cycles: ct };
        self
    }

    pub fn once(mut self) -> Self {
        self.0.behavior = Behavior::OneShot;
        self
    }

    pub fn forever(mut self) -> Self {
        self.0.behavior = Behavior::LoopForever;
        self
    }

    pub fn color(mut self, color: RGB8) -> Self {
        match &mut self.0.action {
            Actions::Sin(a) => a.color = color,
            Actions::Static(a) => a.color = color,
            Actions::Fade(a) => a.inner_mut().color = color,
        }
        self
    }

    pub fn for_ms(mut self, duration: u32) -> Self {
        match &mut self.0.action {
            Actions::Sin(a) => a.duration_ms = duration,
            Actions::Static(a) => a.duration_ms = duration,
            Actions::Fade(a) => {
                a.inner_mut().duration_ms = duration;
                a.inner_mut().period_ms = (duration as f32) * 2.0;
            }
        }
        self
    }

    pub fn period_ms(mut self, duration: f32) -> Self {
        match &mut self.0.action {
            Actions::Sin(a) => a.period_ms = duration,
            Actions::Static(_) => {},
            Actions::Fade(a) => a.inner_mut().period_ms = duration,
        }
        self
    }

    pub fn sin(mut self) -> Self {
        self.0.action = match self.0.action {
            s @ Actions::Sin(_) => s,
            Actions::Static(StayColor { color, duration_ms, .. }) => Actions::Sin(Cycler::new(1.0f32, duration_ms, color)),
            Actions::Fade(FadeColor { mut cycler }) => {
                cycler.start_low();
                Actions::Sin(cycler)
            },
        };
        self
    }

    pub fn solid(mut self) -> Self {
        self.0.action = match self.0.action {
            Actions::Sin(cycler) => Actions::Static(StayColor::new(cycler.duration_ms, cycler.color)),
            s @ Actions::Static(_) => s,
            Actions::Fade(FadeColor { cycler }) => Actions::Static(StayColor::new(cycler.duration_ms, cycler.color)),
        };
        self
    }

    pub fn fade_up(mut self) -> Self {
        self.0.action = match self.0.action {
            Actions::Sin(cycler) => Actions::Fade(FadeColor::new_fade_up(cycler.duration_ms, cycler.color)),
            Actions::Static(stat) => Actions::Fade(FadeColor::new_fade_up(stat.duration_ms, stat.color)),
            Actions::Fade(FadeColor { cycler }) => Actions::Fade(FadeColor::new_fade_up(cycler.duration_ms, cycler.color)),
        };
        self
    }

    pub fn fade_down(mut self) -> Self {
        self.0.action = match self.0.action {
            Actions::Sin(cycler) => Actions::Fade(FadeColor::new_fade_down(cycler.duration_ms, cycler.color)),
            Actions::Static(stat) => Actions::Fade(FadeColor::new_fade_down(stat.duration_ms, stat.color)),
            Actions::Fade(FadeColor { cycler }) => Actions::Fade(FadeColor::new_fade_down(cycler.duration_ms, cycler.color)),
        };
        self
    }
}

impl Actions {
    pub fn reinit(&mut self) {
        use Actions::*;

        match self {
            Sin(s) => s.reinit(),
            Static(s) => s.reinit(),
            Fade(f) => f.reinit(),
        }
    }

    pub fn poll(&self) -> Option<RGB8> {
        use Actions::*;
        match self {
            Sin(s) => s.poll(),
            Static(s) => s.poll(),
            Fade(f) => f.poll(),
        }
    }
}
