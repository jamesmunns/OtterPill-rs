use adafruit_neotrellis as neotrellis;
use vintage_icd::{DeviceToHostMessages, HostToDeviceMessages};
use crate::clock::RollingClock;

enum CyclePeriod {
    Millis64,
    Millis128,
    Millis256,
    Millis512,
    Seconds1,
    Seconds2,
    Seconds4,
    Seconds8,
}

struct Cycler {
    start: u32,
    period: CyclePeriod,
}

impl Cycler {
    fn new(period: CyclePeriod) -> Self {
        Self {
            start: RollingClock::get_ms(),
            period,
        }
    }

    fn with_start(start: u32, period: CyclePeriod) -> Self {
        Self {
            start,
            period,
        }
    }

    fn get_u8(&self) -> u8 {
        let delta = RollingClock::since(self.start);

        use CyclePeriod::*;
        let x = match self.period {
            Millis64 => {
                match ((delta & 0x3F) << 2) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Millis128 => {
                match ((delta & 0x7F) << 1) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Millis256 => {
                match (delta & 0xFF) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Millis512 => {
                match ((delta & 0x1FE) >> 1) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Seconds1 => {
                match ((delta & 0x3FC) >> 2) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Seconds2 => {
                match ((delta & 0x7F8) >> 3) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Seconds4 => {
                match ((delta & 0xFF0) >> 4) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            Seconds8 => {
                match ((delta & 0x1FE0) >> 5) as u8 {
                    x @ 0x00..=0x7F => x << 1,
                    0x80 => 0xFF,
                    x @ _ => {
                        ((x as i8).abs() as u8) << 1
                    }
                }
            }
            _ => unimplemented!()
        };

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
        GAMMA8[x as usize]
    }
}

pub fn trellis_task(cx: &mut crate::idle::Context) -> Result<(), neotrellis::Error> {
    let mut color: u32 = 0b1001_0010_0100_1001;
    let trellis = &mut *cx.resources.trellis;
    let incoming = &mut cx.resources.cli_chan.incoming;
    let outgoing = &mut cx.resources.cli_chan.outgoing;

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
    }

    let mut sticky = [false; 16];

    let loopers = [
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),
        Cycler::new(CyclePeriod::Seconds2),

    ];

    'outer: loop {
        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////
        if let Some(msg) = incoming.dequeue() {
            use DeviceToHostMessages::*;
            use HostToDeviceMessages::*;

            if Ping == msg {
                outgoing.enqueue(Ack).ok();
            }
        }

        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////
        trellis.seesaw().delay_us(20_000u32);

        for i in 0..16 {
            trellis.neopixels().set_pixel_rgb(
                i as u8,
                loopers[i].get_u8(),
                0,
                0
            )?;
        }

        trellis.neopixels().show()?;
    }
}
