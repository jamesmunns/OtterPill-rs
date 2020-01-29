#![no_std]

use embedded_hal::blocking::{
    i2c::{Read, Write},
    delay::DelayUs,
};

pub use adafruit_seesaw::{
    SeeSaw,
    neopixel::{
        self, Speed, ColorOrder,
    },
    keypad::{
        self, KeyEvent, Edge, Status,
    },
    Error, SeeSawError,
};

pub type Result<T> = core::result::Result<T, Error>;

pub struct NeoTrellis<I2C, DELAY> {
    pub seesaw: SeeSaw<I2C, DELAY>,
    pub neopixel_settings: NeoPixelSettings,
}

pub struct NeoPixels<'a, I2C, DELAY> {
    seesaw: &'a mut SeeSaw<I2C, DELAY>,
    settings: &'a mut NeoPixelSettings,
}

pub struct KeyPad<'a, I2C, DELAY> {
    seesaw: &'a mut SeeSaw<I2C, DELAY>,
}

pub struct NeoPixelSettings {
    leds_ct: u8,
    led_type: ColorOrder,
}

impl<I2C, DELAY> NeoTrellis<I2C, DELAY>
where
    I2C: Read + Write,
    DELAY: DelayUs<u32>,
{
    pub fn new(i2c: I2C, delay: DELAY, address: Option<u8>) -> Result<Self> {
        let neopixel_settings = NeoPixelSettings {
            leds_ct: 16,
            led_type: ColorOrder::GRB
        };

        let mut seesaw = SeeSaw {
            i2c,
            delay,
            address: address.unwrap_or_else(|| 0x2E),
        };

        if seesaw.status_get_hwid()? == 0x55 {
            Ok(Self {
                seesaw,
                neopixel_settings,
            })
        } else {
            Err(Error::I2c)
        }
    }

    pub fn neopixels(&mut self) -> NeoPixels<I2C, DELAY> {
        NeoPixels {
            seesaw: &mut self.seesaw,
            settings: &mut self.neopixel_settings,
        }
    }

    pub fn keypad(&mut self) -> KeyPad<I2C, DELAY> {
        KeyPad {
            seesaw: &mut self.seesaw,
        }
    }

    pub fn seesaw(&mut self) -> &mut SeeSaw<I2C, DELAY> {
        &mut self.seesaw
    }
}

impl<'a, I2C, DELAY> NeoPixels<'a, I2C, DELAY>
where
    I2C: Read + Write,
    DELAY: DelayUs<u32>,
{
    pub fn set_speed(&'a mut self, speed: Speed) -> Result<&'a mut Self> {
        self.seesaw.neopixel_set_speed(speed)?;
        Ok(self)
    }

    pub fn set_pin(&'a mut self, pin: u8) -> Result<&'a mut Self> {
        self.seesaw.neopixel_set_pin(pin)?;
        Ok(self)
    }

    pub fn set_pixel_type(&'a mut self, ty: ColorOrder) -> Result<&'a mut Self> {
        self.settings.led_type = ty;

        self.seesaw.neopixel_set_buf_length_pixels(
            self.settings.leds_ct as usize,
            self.settings.led_type
        )?;

        Ok(self)
    }

    pub fn set_pixel_count(&'a mut self, count: u8) -> Result<&'a mut Self> {
        self.settings.leds_ct = count;

        self.seesaw.neopixel_set_buf_length_pixels(
            self.settings.leds_ct as usize,
            self.settings.led_type
        )?;

        Ok(self)
    }

    pub fn set_pixel_rgb(&'a mut self, index: u8, red: u8, green: u8, blue: u8) -> Result<&'a mut Self> {
        match self.settings.led_type {
            ColorOrder::RGB => self.seesaw.neopixel_write_buf_raw(3 * (index as u16), &[red, green, blue]),
            ColorOrder::GRB => self.seesaw.neopixel_write_buf_raw(3 * (index as u16), &[green, red, blue]),
            ColorOrder::RGBW => self.seesaw.neopixel_write_buf_raw(4 * (index as u16), &[red, green, blue, 0]),
            ColorOrder::GRBW => self.seesaw.neopixel_write_buf_raw(4 * (index as u16), &[green, red, blue, 0]),
        }?;

        Ok(self)
    }

    pub fn show(&'a mut self) -> Result<&'a mut Self> {
        self.seesaw.neopixel_show()?;
        Ok(self)
    }
}

impl<'a, I2C, DELAY> KeyPad<'a, I2C, DELAY>
where
    I2C: Read + Write,
    DELAY: DelayUs<u32>,
{
    pub fn pending_events(&mut self) -> Result<u8> {
        self.seesaw.keypad_get_count()
    }

    pub fn get_event(&mut self) -> Result<Option<KeyEvent>> {
        if self.pending_events()? > 0 {
            let mut evt_raw = [0u8; 1];

            self.seesaw.keypad_read_raw(&mut evt_raw)?;

            let event = evt_raw[0] & 0b0000_0011;
            let event = Edge::from_u8(event)?;

            let key = evt_raw[0] >> 2;

            // What is this math even? Copy/paste from adafruit code
            let key = ((key) / 8) * 4 + ((key) % 8);

            Ok(Some(KeyEvent { key, event }))
        } else {
            Ok(None)
        }
    }

    pub fn get_events(&mut self) -> Result<Events> {
        use core::cmp::min;
        let ct = self.pending_events()?;
        let mut retval = Events::new();

        if ct == 0 {
            return Ok(retval);
        }

        let mut evt_raw = [0u8; MAX_EVENTS_USIZE];
        let ct = min(ct, MAX_EVENTS_U8);

        self.seesaw.keypad_read_raw(&mut evt_raw[..ct as usize])?;

        for i in 0..(ct as usize) {
            let event = evt_raw[i] & 0b0000_0011;
            let event = Edge::from_u8(event)?;

            let key = evt_raw[i] >> 2;

            // What is this math even? Copy/paste from adafruit code
            let key = ((key) / 8) * 4 + ((key) % 8);

            retval.events[i] = KeyEvent { key, event };
        }

        retval.count = ct;

        Ok(retval)
    }

    pub fn enable_key_event(&'a mut self, key: u8, event: Edge) -> Result<&'a mut Self> {
        // What is this math even? Copy/paste from adafruit code
        let key = ((key) / 4) * 8 + ((key) % 4);

        self.seesaw.keypad_set_event_raw(key, event, Status::Enable)?;

        Ok(self)
    }

    pub fn disable_key_event(&'a mut self, key: u8, event: Edge) -> Result<&'a mut Self> {
        // What is this math even? Copy/paste from adafruit code
        let key = ((key) / 4) * 8 + ((key) % 4);

        self.seesaw.keypad_set_event_raw(key, event, Status::Disable)?;

        Ok(self)
    }
}

const MAX_EVENTS_USIZE: usize = 16;
const MAX_EVENTS_U8: u8 = 16;

pub struct Events {
    count: u8,
    events: [KeyEvent; MAX_EVENTS_USIZE]
}

impl Events {
    pub fn new() -> Self {
        Self {
            count: 0,
            events: [KeyEvent { key: 0, event: Edge::Falling }; MAX_EVENTS_USIZE],
        }
    }

    pub fn as_slice(&self) -> &[KeyEvent] {
        if self.count == 0 {
            &[]
        } else {
            &self.events[..self.count as usize]
        }
    }
}
