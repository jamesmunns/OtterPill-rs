use serialport::prelude::*;
use postcard::{from_bytes, to_slice_cobs};
use vintage_icd::{
    HostToDeviceMessages,
    DeviceToHostMessages,
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

    inner_loop(&mut port).map_err(drop)?;

    Ok(())
}

fn inner_loop(port: &mut Box<dyn SerialPort>) -> Result<(), ()> {
    let mut cobs_buf: Buffer<U256> = Buffer::new();
    let mut raw_buf = [0u8; 256];
    let mut now = Instant::now();

    loop {
        if now.elapsed() >= Duration::from_secs(2) {
            now = Instant::now();

            let msg = HostToDeviceMessages::Ping;

            if let Ok(slice) = postcard::to_slice_cobs(&msg, &mut raw_buf) {
                port.write_all(slice).map_err(drop)?;
                port.flush().map_err(drop)?;
            }

            println!("SENT: {:?}", msg);
        }

        let buf = match port.read(&mut raw_buf) {
            Ok(ct) => {
                println!("Got {} bytes", ct);
                &raw_buf[..ct]
            },
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                println!("Timeout");
                continue;
            },
            Err(e) => {
                eprintln!("{:?}", e);
                return Err(());
            }
        };

        let mut window = &buf[..];

        'cobs: while !window.is_empty() {
            println!("{:?}", window);
            use FeedResult::*;
            window = match cobs_buf.feed::<DeviceToHostMessages>(&window) {
                Consumed => break 'cobs,
                OverFull(new_wind) => new_wind,
                DeserError(new_wind) => new_wind,
                Success { data, remaining } => {
                    println!("GOT: {:?}", data);
                    remaining
                }
            };
        }
    }


        // if let Ok(count) = serial.read(&mut buf) {
        //     let mut window = &buf[..count];

        //     'cobs: while !window.is_empty() {
        //         use FeedResult::*;
        //         window = match buffer.feed::<HostToDeviceMessages>(&window) {
        //             Consumed => break 'cobs,
        //             OverFull(new_wind) => new_wind,
        //             DeserError(new_wind) => new_wind,
        //             Success { data, remaining } => {
        //                 usb_chan.incoming.enqueue(data).ok();
        //                 remaining
        //             }
        //         };
        //     }
        // }
}
