//! Driver for the Adafruit Seesaw.
//!
//! Totally untested so far

#![no_std]

use ehal::blocking::{
    delay::DelayUs,
    i2c::{Read, Write},
};
use embedded_hal as ehal;

// TODO: Some kind of shared-bus thing for sharing i2c?
pub struct SeeSaw<I2C, DELAY> {
    pub i2c: I2C,
    pub delay: DELAY,
    pub address: u8,
}

#[derive(Debug)]
pub enum Error {
    I2c,
    SeeSaw(SeeSawError),
}

#[derive(Debug)]
pub enum SeeSawError {
    SizeError,
    InvalidArgument,
    Unimplemented,
}

const BUFFER_MAX: usize = 32;
const PAYLOAD_MAX: usize = BUFFER_MAX - 2;
const DEFAULT_DELAY_US: u32 = 125;

impl<I2C, DELAY> SeeSaw<I2C, DELAY>
where
    I2C: Read + Write,
    DELAY: DelayUs<u32>,
{
    fn write(&mut self, base: u8, function: u8, buf: &[u8]) -> Result<(), Error> {
        if buf.len() > PAYLOAD_MAX {
            return Err(Error::SeeSaw(SeeSawError::SizeError));
        }

        let mut tx_buf: [u8; 32] = [0u8; 32];

        let end = 2 + buf.len();

        tx_buf[0] = base;
        tx_buf[1] = function;
        tx_buf[2..end].copy_from_slice(buf);

        self.i2c
            .write(self.address, &tx_buf[..end])
            .map_err(|_| Error::I2c)
    }

    fn read(&mut self, base: u8, function: u8, delay_us: u32, buf: &mut [u8]) -> Result<(), Error> {
        self.write(base, function, &[])?;
        self.delay.delay_us(delay_us);
        self.i2c.read(self.address, buf).map_err(|_| Error::I2c)
    }

    /// Get the count of pending key events on the keypad
    pub fn keypad_get_count(&mut self) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read(
            keypad::BASE,
            keypad::functions::COUNT,
            500,
            &mut buf,
        )?;
        Ok(buf[0])
    }

    /// Enable or disable the interrupt
    pub fn keypad_set_interrupt(&mut self, enable: bool) -> Result<(), Error> {
        use keypad::functions::{INTENCLR, INTENSET};

        let func = if enable { INTENSET } else { INTENCLR };
        self.write(keypad::BASE, func, &[1])
    }

    /// Set or clear the trigger event on a given key.
    ///
    /// I'm unclear on how these key numbers are typically set. For the Neotrellis,
    /// there is some complicated bit mask/shifting to go from 0-16 to the
    /// actual u8 values
    pub fn keypad_set_event_raw(
        &mut self,
        key_raw: u8,
        edge: keypad::Edge,
        status: keypad::Status,
    ) -> Result<(), Error> {
        let stat: u8 = (1 << ((edge as u8) + 1)) | (status as u8);
        self.write(keypad::BASE, keypad::functions::EVENT, &[key_raw, stat])
    }

    /// Read an event on a given key.
    ///
    /// I'm unclear on how these key numbers are typically set. For the Neotrellis,
    /// there is some complicated bit mask/shifting to go from 0-16 to the
    /// actual u8 values
    ///
    /// Additionally theres some shenanigans to convert the raw bufer to (key + event)
    pub fn keypad_read_raw(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.read(keypad::BASE, keypad::functions::FIFO, 1000, buf)
    }

    pub fn neopixel_set_pin(&mut self, pin: u8) -> Result<(), Error> {
        if pin >= neopixel::NUM_PINS {
            return Err(Error::SeeSaw(SeeSawError::InvalidArgument));
        }

        self.write(neopixel::BASE, neopixel::functions::PIN, &[pin])
    }

    pub fn neopixel_set_speed(&mut self, speed: neopixel::Speed) -> Result<(), Error> {
        self.write(neopixel::BASE, neopixel::functions::SPEED, &[speed as u8])
    }

    pub fn neopixel_set_buf_length_bytes(&mut self, len: u16) -> Result<(), Error> {
        if len > neopixel::MAX_BUF_BYTES {
            return Err(Error::SeeSaw(SeeSawError::InvalidArgument));
        }

        let bytes: [u8; 2] = len.to_be_bytes();
        self.write(neopixel::BASE, neopixel::functions::BUF_LENGTH, &bytes)
    }

    pub fn neopixel_set_buf_length_pixels(
        &mut self,
        ct: usize,
        order: neopixel::ColorOrder,
    ) -> Result<(), Error> {
        use neopixel::ColorOrder::*;

        let bpp = match order {
            RGB | GRB => 3,
            RGBW | GRBW => 4,
        };

        let count = ct * bpp;

        if count <= (u16::max_value() as usize) {
            self.neopixel_set_buf_length_bytes(count as u16)
        } else {
            Err(Error::SeeSaw(SeeSawError::InvalidArgument))
        }
    }

    pub fn neopixel_show(&mut self) -> Result<(), Error> {
        self.write(neopixel::BASE, neopixel::functions::SHOW, &[])
    }

    pub fn neopixel_write_buf_raw(&mut self, idx: u16, buf: &[u8]) -> Result<(), Error> {
        if buf.len() > neopixel::MAX_BUF_WRITE_BYTES {
            return Err(Error::SeeSaw(SeeSawError::InvalidArgument));
        }

        let mut tx_buf = [0u8; 32];

        let tx_buf_len = 2 + buf.len();

        tx_buf[..2].copy_from_slice(&idx.to_be_bytes());
        tx_buf[2..tx_buf_len].copy_from_slice(buf);

        self.write(neopixel::BASE, neopixel::functions::BUF, &tx_buf[..tx_buf_len])
    }

    pub fn status_get_hwid(&mut self) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read(status::BASE, status::functions::HW_ID, DEFAULT_DELAY_US, &mut buf)
            .map_err(|_| Error::I2c)?;
        Ok(buf[0])
    }

    pub fn status_get_version(&mut self) -> Result<u32, Error> {
        let mut buf = [0u8; 4];
        self.read(status::BASE, status::functions::VERSION, DEFAULT_DELAY_US, &mut buf)
            .map_err(|_| Error::I2c)?;
        Ok(u32::from_be_bytes(buf))
    }

    pub fn status_get_options(&mut self) -> Result<u32, Error> {
        let mut buf = [0u8; 4];
        self.read(status::BASE, status::functions::OPTIONS, DEFAULT_DELAY_US, &mut buf)
            .map_err(|_| Error::I2c)?;
        Ok(u32::from_be_bytes(buf))
    }

    // Get raw temperature. To convert to celcius, divide by (1 << 16)
    pub fn status_get_temp_raw(&mut self) -> Result<u32, Error> {
        let mut buf = [0u8; 4];
        self.read(status::BASE, status::functions::TEMP, 1000, &mut buf)
            .map_err(|_| Error::I2c)?;
        Ok(u32::from_be_bytes(buf))
    }

    pub fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us);
    }
}

pub mod status {
    pub const BASE: u8 = 0x00;

    pub mod functions {
        pub const HW_ID: u8 = 0x01;
        pub const VERSION: u8 = 0x02;
        pub const OPTIONS: u8 = 0x03;
        pub const TEMP: u8 = 0x04;
        pub const SWRST: u8 = 0x7F;
    }

    pub const HW_ID_CODE: u8 = 0x55;
}

pub mod neopixel {
    pub const BASE: u8 = 0x0E;

    pub mod functions {
        pub const PIN: u8 = 0x01;
        pub const SPEED: u8 = 0x02;
        pub const BUF_LENGTH: u8 = 0x03;
        pub const BUF: u8 = 0x04;
        pub const SHOW: u8 = 0x05;
    }

    pub const NUM_PINS: u8 = 32;
    pub const MAX_BUF_BYTES: u16 = 63 * 3;

    pub const MAX_BUF_WRITE_BYTES: usize = 30;
    pub const MAX_RGB_WRITE_PIXELS: usize = MAX_BUF_WRITE_BYTES / 3;
    pub const MAX_RGBW_WRITE_PIXELS: usize = MAX_BUF_WRITE_BYTES / 4;

    #[derive(Debug, Copy, Clone)]
    pub enum ColorOrder {
        RGB,
        GRB,
        RGBW,
        GRBW,
    }

    #[derive(Debug, Copy, Clone)]
    #[repr(u8)]
    pub enum Speed {
        Khz400 = 0x00,
        Khz800 = 0x01,
    }
}

pub mod keypad {
    pub const BASE: u8 = 0x10;

    pub mod functions {
        pub const STATUS: u8 = 0x00;
        pub const EVENT: u8 = 0x01;
        pub const INTENSET: u8 = 0x02;
        pub const INTENCLR: u8 = 0x03;
        pub const COUNT: u8 = 0x04;
        pub const FIFO: u8 = 0x10;
    }

    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct KeyEvent {
        pub key: u8,
        pub event: Edge,
    }

    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    #[repr(u8)]
    pub enum Edge {
        /// Indicates that the key is currently pressed
        High = 0x00,

        /// Indicates that the key is currently released
        Low = 0x01,

        /// Indicates that the key was recently released
        Falling = 0x02,

        /// Indicates that the key was recently pressed
        Rising = 0x03,
    }

    impl Edge {
        pub fn from_u8(val: u8) -> Result<Self, crate::Error> {
            match val {
                0 => Ok(Edge::High),
                1 => Ok(Edge::Low),
                2 => Ok(Edge::Falling),
                3 => Ok(Edge::Rising),
                _ => Err(crate::Error::SeeSaw(crate::SeeSawError::InvalidArgument)),
            }
        }
    }

    #[repr(u8)]
    pub enum Status {
        Disable = 0x00,
        Enable = 0x01,
    }
}
