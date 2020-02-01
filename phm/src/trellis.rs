use adafruit_neotrellis as neotrellis;
use phm_icd::{DeviceToHostMessages, HostToDeviceMessages};

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

        for evt in trellis.keypad().get_events()?.as_slice() {
            if evt.key == 15 && evt.event == neotrellis::Edge::Falling {
                panic!()
            }

            if evt.event == neotrellis::Edge::Rising {
                if sticky[evt.key as usize] {
                    trellis.neopixels().set_pixel_rgb(evt.key, 0, 0, 0)?;
                    sticky[evt.key as usize] = false;
                } else {
                    let colors = color.to_le_bytes();

                    trellis
                        .neopixels()
                        .set_pixel_rgb(evt.key, colors[1], colors[0], colors[2])?;

                    color = color.rotate_left(1);
                    sticky[(evt.key as usize)] = true;
                }
            }
        }

        trellis.neopixels().show()?;
    }
}
