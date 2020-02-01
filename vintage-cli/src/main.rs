use serialport::prelude::*;
use postcard::{from_bytes, to_slice_cobs, to_stdvec_cobs};
use vintage_icd::{
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
                        remaining
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

    trellis(NeoTrellis::new(
        rpc,
        Sleepy,
        None,
    ).unwrap()).map_err(drop)?;


    Ok(())
}

fn trellis(mut trellis: NeoTrellis<UsbRpcI2c, Sleepy>) -> Result<(), Error> {
    let mut color = 0b1001_0010_0100_1000u32;

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

// fn inner_loop(port: &mut Box<dyn SerialPort>) -> Result<(), ()> {
//     let mut cobs_buf: Buffer<U256> = Buffer::new();
//     let mut raw_buf = [0u8; 256];
//     let mut now = Instant::now();

//     loop {
//         if now.elapsed() >= Duration::from_secs(2) {
//             now = Instant::now();

//             let msg = HostToDeviceMessages::Ping;

//             if let Ok(slice) = postcard::to_slice_cobs(&msg, &mut raw_buf) {
//                 port.write_all(slice).map_err(drop)?;
//                 port.flush().map_err(drop)?;
//             }

//             println!("\nSENT: {:?}", msg);
//         }

//         let buf = match port.read(&mut raw_buf) {
//             Ok(ct) => {
//                 &raw_buf[..ct]
//             },
//             Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
//                 print!(".");
//                 io::stdout().flush().ok().expect("Could not flush stdout");
//                 continue;
//             },
//             Err(e) => {
//                 eprintln!("{:?}", e);
//                 return Err(());
//             }
//         };

//         let mut window = &buf[..];

//         'cobs: while !window.is_empty() {
//             use FeedResult::*;
//             window = match cobs_buf.feed::<DeviceToHostMessages>(&window) {
//                 Consumed => break 'cobs,
//                 OverFull(new_wind) => new_wind,
//                 DeserError(new_wind) => new_wind,
//                 Success { data, remaining } => {
//                     println!("\nGOT: {:?}", data);
//                     remaining
//                 }
//             };
//         }
//     }


//         // if let Ok(count) = serial.read(&mut buf) {
//         //     let mut window = &buf[..count];

//         //     'cobs: while !window.is_empty() {
//         //         use FeedResult::*;
//         //         window = match buffer.feed::<HostToDeviceMessages>(&window) {
//         //             Consumed => break 'cobs,
//         //             OverFull(new_wind) => new_wind,
//         //             DeserError(new_wind) => new_wind,
//         //             Success { data, remaining } => {
//         //                 usb_chan.incoming.enqueue(data).ok();
//         //                 remaining
//         //             }
//         //         };
//         //     }
//         // }
// }



    // trellis
    //     .neopixels()
    //     .set_speed(neotrellis::Speed::Khz800)?
    //     .set_pixel_count(16)?
    //     .set_pixel_type(neotrellis::ColorOrder::GRB)?
    //     .set_pin(3)?;

    // // Cycle the board through some initial colors
    // for c in 0..4 {
    //     let cols = match c {
    //         0 => [0x00, 0x10, 0x00],
    //         1 => [0x10, 0x00, 0x00],
    //         2 => [0x00, 0x00, 0x10],
    //         _ => [0x00, 0x00, 0x00],
    //     };

    //     for i in 0..16 {
    //         trellis
    //             .keypad()
    //             .enable_key_event(i, neotrellis::Edge::Rising)?
    //             .enable_key_event(i, neotrellis::Edge::Falling)?;

    //         trellis
    //             .neopixels()
    //             .set_pixel_rgb(i, cols[1], cols[0], cols[2])?;
    //     }

    //     trellis.neopixels().show()?;
    //     trellis.seesaw().delay_us(150_000u32);
    // }

    // let mut sticky = [false; 16];

    // 'outer: loop {
    //     //////////////////////////////////////////////////////////////////
    //     // Process button presses
    //     //////////////////////////////////////////////////////////////////
    //     if let Some(msg) = incoming.dequeue() {
    //         use DeviceToHostMessages::*;
    //         use HostToDeviceMessages::*;

    //         if Ping == msg {
    //             outgoing.enqueue(Ack).ok();
    //         }
    //     }

    //     //////////////////////////////////////////////////////////////////
    //     // Process button presses
    //     //////////////////////////////////////////////////////////////////
    //     trellis.seesaw().delay_us(20_000u32);

    //     for evt in trellis.keypad().get_events()?.as_slice() {
    //         if evt.key == 15 && evt.event == neotrellis::Edge::Falling {
    //             panic!()
    //         }

    //         if evt.event == neotrellis::Edge::Rising {
    //             if sticky[evt.key as usize] {
    //                 trellis.neopixels().set_pixel_rgb(evt.key, 0, 0, 0)?;
    //                 sticky[evt.key as usize] = false;
    //             } else {
    //                 let colors = color.to_le_bytes();

    //                 trellis
    //                     .neopixels()
    //                     .set_pixel_rgb(evt.key, colors[1], colors[0], colors[2])?;

    //                 color = color.rotate_left(1);
    //                 sticky[(evt.key as usize)] = true;
    //             }
    //         }
    //     }

    //     trellis.neopixels().show()?;
    // }
