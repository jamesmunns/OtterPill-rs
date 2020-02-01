use serialport::prelude::*;
use postcard::{from_bytes, to_slice_cobs, to_stdvec_cobs};
use phm_icd::{
    HostToDeviceMessages,
    DeviceToHostMessages,
    I2CCommand,
    I2CResponse,
    cobs_buffer::{
        Buffer,
        FeedResult,
        consts::*,
    }
};
use std::sync::mpsc::{Sender, Receiver, TryRecvError};
use std::time::{
    Duration,
    Instant,
};
use std::io::prelude::*;
use std::io;
use embedded_hal::blocking::i2c;
use embedded_hal::blocking::delay::DelayUs;
use adafruit_neotrellis::{self as neotrellis, NeoTrellis, Error};
use rand::prelude::*;

fn rand_exp() -> u8 {
    thread_rng().gen_range(32, 96)
}

impl UsbRpcI2c {
    fn get_message_timeout(&mut self, timeout: Duration) -> Result<DeviceToHostMessages, ()> {
        let mut cobs_buf: Buffer<U256> = Buffer::new();
        let mut raw_buf = [0u8; 256];
        let now = Instant::now();

        while now.elapsed() < timeout {
            let buf = match self.serial.read(&mut raw_buf) {
                Ok(ct) => {
                    &raw_buf[..ct]
                },
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    io::stdout().flush().ok().expect("Could not flush stdout");
                    continue;
                },
                Err(e) => {
                    eprintln!("{:?}", e);
                    return Err(());
                }
            };

            let mut window = &buf[..];

            'cobs: while !window.is_empty() {
                use FeedResult::*;
                window = match cobs_buf.feed::<DeviceToHostMessages>(&window) {
                    Consumed => break 'cobs,
                    OverFull(new_wind) => new_wind,
                    DeserError(new_wind) => new_wind,
                    Success { data, remaining } => {
                        return Ok(data);
                    }
                };
            }
        }
        Err(())
    }

    fn send_message(&mut self, msg: HostToDeviceMessages) -> Result<(), ()> {
        let cobs = to_stdvec_cobs(&msg).map_err(drop)?;
        self.serial.write_all(&cobs).map_err(drop)?;
        self.serial.flush().map_err(drop)
    }
}

impl i2c::Read for UsbRpcI2c {
    type Error = ();

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let msg = HostToDeviceMessages::I2c(I2CCommand::Read { address: addr, count: buffer.len() });
        self.send_message(msg)?;

        let resp = self.get_message_timeout(Duration::from_secs(5))?;
        if let DeviceToHostMessages::I2c(I2CResponse::ReadResponse(resp)) = resp {
            buffer[..resp.len()].copy_from_slice(resp.as_ref());
            Ok(())
        } else {
            println!("Wrong Resp Rd! {:?}", resp);
            Err(())
        }
    }
}

impl i2c::Write for UsbRpcI2c {
    type Error = ();

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut hbuf = heapless::Vec::new();
        hbuf.extend_from_slice(bytes);

        let msg = HostToDeviceMessages::I2c(I2CCommand::Write { address: addr, data: hbuf });
        self.send_message(msg)?;

        let resp = self.get_message_timeout(Duration::from_secs(5))?;
        if DeviceToHostMessages::I2c(I2CResponse::WriteAcknowledge) == resp {
            Ok(())
        } else {
            println!("Wrong Resp Wr! {:?}", resp);
            Err(())
        }
    }
}

struct UsbRpcI2c {
    pub serial: Box<dyn SerialPort>,
}

struct Sleepy;

impl DelayUs<u32> for Sleepy {
    fn delay_us(&mut self, us: u32) {
        std::thread::sleep(Duration::from_micros(us.into()))
    }
}

fn main() -> Result<(), ()> {
    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(1000);
    settings.baud_rate = 115_200;

    let mut port = match serialport::open_with_settings("/dev/ttyACM0", &settings) {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", "/dev/ttyACM0", e);
            ::std::process::exit(1);
        }
    };

    let mut rpc = UsbRpcI2c { serial: port };
    let mut acks = 0;

    while acks < 5 {
        rpc.send_message(HostToDeviceMessages::Ping).unwrap();
        if let Ok(DeviceToHostMessages::Ack) = rpc.get_message_timeout(Duration::from_secs(1)) {
            acks += 1;
        }
    }

    while rpc.get_message_timeout(Duration::from_secs(1)).is_ok() {
        continue;
    }

    // for _ in 0..6 {
    //     let mut bench = 0usize;
    //     let start = Instant::now();

    //     while start.elapsed() < Duration::from_secs(10) {
    //         rpc.send_message(HostToDeviceMessages::Ping).unwrap();
    //         if let Ok(DeviceToHostMessages::Ack) = rpc.get_message_timeout(Duration::from_secs(1)) {
    //             bench += 1;
    //         }
    //     }

    //     print!("{} loops, {} per second, ", bench, bench / 10);
    //     println!("average rtt: {:.02}us", 1_000_000.0 / ((bench / 10) as f64));
    // }

    trellis(NeoTrellis::new(
        rpc,
        Sleepy,
        None,
    ).unwrap()).map_err(drop)?;


    Ok(())
}

fn trellis(mut trellis: NeoTrellis<UsbRpcI2c, Sleepy>) -> Result<(), Error> {
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
                .disable_key_event(i, neotrellis::Edge::Rising)?
                .disable_key_event(i, neotrellis::Edge::Falling)?
                .disable_key_event(i, neotrellis::Edge::High)?
                .disable_key_event(i, neotrellis::Edge::Low)?
                .enable_key_event(i, neotrellis::Edge::Rising)?;

            trellis
                .neopixels()
                .set_pixel_rgb(i, cols[1], cols[0], cols[2])?;
        }

        trellis.neopixels().show()?;
        trellis.seesaw().delay_us(150_000u32);
    }

    let mut sticky = [false; 16];
    let mut last_event = Instant::now();
    let mut last = None;

    'outer: loop {
        //////////////////////////////////////////////////////////////////
        // Process button presses
        //////////////////////////////////////////////////////////////////
        trellis.seesaw().delay_us(20_000u32);

        let mut any = false;
        for evt in trellis.keypad().get_events()?.as_slice() {
            any = true;
            if evt.event == neotrellis::Edge::Rising {
                last_event = Instant::now();

                let red = rand_exp();
                let grn = rand_exp();
                let blu = rand_exp();

                last = Some((evt.key, red, grn, blu));

                select_animation(
                    &mut trellis,
                    evt.key,
                    red,
                    grn,
                    blu,
                )?;
            }
        }
        if !any {
            if let Some((key, red, grn, blu)) = last {
                selected_animation(
                    &mut trellis,
                    key,
                    red,
                    grn,
                    blu,
                )?;
            }
        }

        if last_event.elapsed() > Duration::from_secs(30) {
            let start = Instant::now();
            let mut iter = (0..16).cycle();

            while trellis.seesaw().keypad_get_count()? == 0 {
                trellis.seesaw().delay_us((random::<u32>() % 10) * 1000 + 100_000);

                if let Some(_i) = iter.next() {
                    let i = random::<u8>() % 16;

                    sticky[i as usize] = true;

                    trellis
                        .neopixels()
                        .set_pixel_rgb(i, rand_exp(), rand_exp(), rand_exp())?;

                    trellis.neopixels().show()?;
                }

                if (random::<u16>() / 4) == 0 {
                    for i in 0..16 {
                        sticky[i as usize] = false;
                        trellis
                            .neopixels()
                            .set_pixel_rgb(i, 0, 0, 0)?;
                    }
                }
            }
        }

        trellis.neopixels().show()?;
    }
}

struct IPos {
    x: i8,
    y: i8,
}

impl IPos {
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

    fn to_pin(&self) -> u8 {
        ((self.y as u8) << 2) | (self.x as u8)
    }
}

trait DelayUntilPending {
    type Error;
    fn delay_up_us(&mut self, us: u32) -> Result<(), Error>;
}

impl<I, D> DelayUntilPending for NeoTrellis<I, D>
where
    I: i2c::Read + i2c::Write,
    D: DelayUs<u32>,
{
    type Error = Error;

    fn delay_up_us(&mut self, us: u32) -> Result<(), Error> {
        let start = Instant::now();
        let end = Duration::from_micros(us as u64);

        while start.elapsed() < end {
            if self.seesaw().keypad_get_count()? > 0 {
                return Ok(());
            }
        }
        Ok(())
    }
}

fn selected_animation(trellis: &mut NeoTrellis<UsbRpcI2c, Sleepy>, pin: u8, red: u8, green: u8, blue: u8) -> Result<(), Error> {
    use core::cmp::max;

    let coord = IPos::from_pin(pin);
    let deltas = &[
        (-1, -1), (-1, 0), (-1, 1),
        ( 0, -1),          ( 0, 1),
        ( 1, -1), ( 1, 0), ( 1, 1),
    ];

    let rls = 8 - red.leading_zeros();
    let gls = 8 - green.leading_zeros();
    let bls = 8 - blue.leading_zeros();

    let steps = max(max(rls, gls), bls);
    let steps = max(steps, 7);

    if steps <= 2 {
        return Ok(());
    }

    for i in (2..steps).rev() {
        for (dx, dy) in deltas {
            if let Some(pin) = coord.move_xy_pin(*dx, *dy) {
                trellis.neopixels().set_pixel_rgb(
                    pin,
                    red >> i,
                    green >> i,
                    blue >> i,
                )?;
            }
        }
        trellis.neopixels().show()?;
        trellis.delay_up_us(20_000 * (i as u32))?;
    }

    trellis.delay_up_us(250_000)?;

    for i in (2..steps) {
        for (dx, dy) in deltas {
            if let Some(pin) = coord.move_xy_pin(*dx, *dy) {
                trellis.neopixels().set_pixel_rgb(
                    pin,
                    red >> i,
                    green >> i,
                    blue >> i,
                )?;
            }
        }
        trellis.neopixels().show()?;
        trellis.delay_up_us(20_000 * (i as u32))?;
    }

    trellis.delay_up_us(250_000)?;

    Ok(())
}

fn select_animation(trellis: &mut NeoTrellis<UsbRpcI2c, Sleepy>, pin: u8, red: u8, green: u8, blue: u8) -> Result<(), Error> {
    // We want to "spread out" the selection. This takes at most four steps.
    let coord = IPos::from_pin(pin);

    // Step one: Set the pin
    trellis.neopixels().set_pixel_rgb(pin, red, green, blue)?;
    trellis.neopixels().show()?;
    trellis.delay_up_us(10_000)?;

    for d in 1..4 {
        let div = 1 << (d as u8);
        for dx in -d..=d {
            for dy in -d..=d {
                if (dx != d) && (dy != d) && (dx != -d) && (dy != -d) {
                    continue;
                } else if let Some(pin) = coord.move_xy_pin(dx, dy) {
                    trellis.neopixels().set_pixel_rgb(pin, red / div, green / div, blue / div)?;
                }

            }
        }
        trellis.neopixels().show()?;
        trellis.delay_up_us(20_000 * (d as u32))?;
    }

    trellis.delay_up_us(250_000)?;

    for d in (1..4).rev() {
        for dx in -d..=d {
            for dy in -d..=d {
                if (dx != d) && (dy != d) && (dx != -d) && (dy != -d) {
                    continue;
                } else if let Some(pin) = coord.move_xy_pin(dx, dy) {
                    trellis.neopixels().set_pixel_rgb(pin, 0, 0, 0)?;
                }

            }
        }
        trellis.neopixels().show()?;
        trellis.delay_up_us(30_000 * (d as u32))?;
    }

    // trellis.neopixels().set_pixel_rgb(15, 255, 0, 0)?;
    // trellis.neopixels().show()?;

    Ok(())
}
