#![no_std]

use serde::{Serialize, Deserialize};

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
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum DeviceToHostMessages {
    Ack
}

// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum LedCommand {
    On,
    Off,
    Toggle,
}
