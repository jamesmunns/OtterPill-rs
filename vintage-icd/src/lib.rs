#![cfg_attr(not(test), no_std)]

use serde::{Serialize, Deserialize};
use heapless::{Vec, consts::*};

pub mod cobs_buffer;

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct Message<T>
{
    id: u32,
    payload: T
}


#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum HostToDeviceMessages {
    Ping,
    StatusLed(LedCommand),
    I2c(I2CCommand),
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum DeviceToHostMessages {
    Ack,
    I2c(I2CResponse)
}

// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum LedCommand {
    On,
    Off,
    Toggle,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum I2CCommand {
    Read{address: u8, count: usize},
    Write{address: u8, data: Vec<u8, U32>},
    DelayUs(u32),
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum I2CResponse {
    ReadResponse(Vec<u8, U32>),
    ReadError,
    WriteAcknowledge,
    WriteError,
    DelayAcknowledge,
}
