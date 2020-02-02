use postcard::to_slice_cobs;
use vintage_icd::{cobs_buffer::FeedResult, HostToDeviceMessages};

pub fn usb_poll(cx: &mut crate::usb_tx::Context) {
    let usb = &mut *cx.resources.usb_dev;
    let serial = &mut *cx.resources.serial;
    let buffer = &mut *cx.resources.buffer;
    let incoming = &mut cx.resources.usb_chan.incoming;
    let outgoing = &mut cx.resources.usb_chan.outgoing;

    let mut buf = [0u8; 128];


    if usb.poll(&mut [serial]) {
        if let Ok(count) = serial.read(&mut buf) {
            let mut window = &buf[..count];

            'cobs: while !window.is_empty() {
                use FeedResult::*;
                window = match buffer.feed::<HostToDeviceMessages>(&window) {
                    Consumed => break 'cobs,
                    OverFull(new_wind) => new_wind,
                    DeserError(new_wind) => new_wind,
                    Success { data, remaining } => {
                        incoming.enqueue(data).ok();
                        remaining
                    }
                };
            }
        }
    }

    while let Some(msg) = outgoing.dequeue() {

        to_slice_cobs(&msg, &mut buf)
        .map_err(drop)
        .and_then(|mut slice| {
            while let Ok(count) = serial.write(slice) {
                if count == slice.len() {
                    break;
                } else {
                    slice = &mut slice[count..];
                }
            }
            Ok(())
        })
        .unwrap();
    }

}
