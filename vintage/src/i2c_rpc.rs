use embedded_hal::blocking::i2c::{Read, Write};
use vintage_icd::{
    HostToDeviceMessages,
    DeviceToHostMessages,
    I2CResponse,
    I2CCommand,
};
use embedded_hal::blocking::delay::DelayUs;
use heapless::{Vec, consts::*};
use stm32f0xx_hal::stm32::Interrupt;
use embedded_hal::digital::v2::OutputPin;

pub fn task(cx: &mut crate::idle::Context) -> Result<(), ()> {
    let mut hbuf: Vec<u8, U32> = Vec::new();
    let mut toggle = false;
    cx.resources.delay.delay_us(3_000_000u32);

    loop {
        if let Some(msg) = cx.resources.cli_chan.incoming.dequeue() {
            if toggle {
                cx.resources.led.set_low().ok();
            } else {
                cx.resources.led.set_high().ok();
            }

            toggle = !toggle;

            let resp = match msg {
                HostToDeviceMessages::I2c(I2CCommand::Read { address, count } ) => {
                    hbuf.resize(count, 0).ok();
                    if cx.resources.i2c.read(address, hbuf.as_mut()).is_ok() {
                        Some(DeviceToHostMessages::I2c(I2CResponse::ReadResponse(hbuf.clone())))
                    } else {
                        Some(DeviceToHostMessages::I2c(I2CResponse::ReadError))
                    }
                }
                HostToDeviceMessages::I2c(I2CCommand::Write { address, data }) => {
                    if cx.resources.i2c.write(address, &data).is_ok() {
                        Some(DeviceToHostMessages::I2c(I2CResponse::WriteAcknowledge))
                    } else {
                        Some(DeviceToHostMessages::I2c(I2CResponse::WriteError))
                    }
                }
                HostToDeviceMessages::I2c(I2CCommand::DelayUs(dly)) => {
                    cx.resources.delay.delay_us(dly);
                    Some(DeviceToHostMessages::I2c(I2CResponse::DelayAcknowledge))
                }
                HostToDeviceMessages::Ping => {
                    Some(DeviceToHostMessages::Ack)
                }
                _ => panic!(),
            };

            if let Some(resp) = resp {
                if !cx.resources.cli_chan.outgoing.enqueue(
                    resp
                ).is_ok() {
                    cx.resources.delay.delay_us(1_000_000u32);
                } else {
                    rtfm::pend(Interrupt::USB);
                }
            }
        }
    }
}
