#![cfg_attr(not(test), no_std)]

use serde::{Serialize, Deserialize};
use heapless::{String, consts::*};

pub mod cobs_buffer;

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct Message<T>
{
    pub id: u32,
    pub payload: T
}


#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum HostToDeviceMessages {
    Ping,
    Reset,
    Ack,
    GetPanic,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum DeviceToHostMessages {
    Ack,
    Ping,
    Status(StatusMessage),
    Panic(String<U64>),
    CycleTime(u32),
}

// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StatusMessage {
    pub current_tick: u32,
    pub state: CurrentState,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum CurrentState {
    Idle,
    Timing { pin: u8, elapsed: u32 },
    Timeout { elapsed: u32 },
}
