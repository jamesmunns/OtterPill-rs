# vintage

[Harvest] from the [Trellis].

Powered by the [OtterPill] and [Embedded Rust].

[Harvest]: https://www.getharvest.com/
[Trellis]: https://www.adafruit.com/product/3954

[OtterPill]: https://github.com/Jan--Henrik/OtterPill
[Embedded Rust]: https://github.com/rust-embedded/wg

## Notes

* Trellis board is based on the Adafruit Seesaw I2C protocol
    * [NeoPixel endpoints](https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/neopixel)
    * [KeyPad endpoints](https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/master/adafruit_seesaw/keypad.py) - From CircuitPython examples
* [Trellis Schematic](https://learn.adafruit.com/assets/62094)
* [NeoTrellis CircuitPython Drivers](https://github.com/adafruit/Adafruit_CircuitPython_NeoTrellis)
    * Uses Keypad, KeyEvent, and NeoPixel from adafruit_seesaw
* Driver should be something like:
    * NeoPixel Trellis
        * Adafruit Seesaw
