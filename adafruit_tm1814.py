# SPDX-FileCopyrightText: 2022 Jeff Epler, written for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`tm1814` - TM1814 LED strip driver for RP2 microcontrollers
===========================================================

* Author(s): Jeff Epler for Adafruit Industries

The class defined here is largely compatible with the standard NeoPixel class,
except that it always is writing data to the LEDs in the background.

Not only do TM1814 pixels require continuous updating, writing the LED data in
the background will allow more time for your Python code to run.

Because the pixelbuf storage is also being written out 'live', it is possible
to experience tearing, where the LEDs are a combination of old and new values
at the same time.
"""

import struct
import time
import adafruit_pixelbuf
from rp2pio import StateMachine
from adafruit_pioasm import Program

# Pixel stream is very similar to NeoPixel WS2812B, but inverted.
#
# Pixel time is 1.25us (800kHz)
#
# Datasheet low times for a "0" bit are 310 (min) 360 (typ) 410 (max) ns
# Datasheet high times for a "1" bit are 650 (min) 720 (typ) 1000 (max) ns
#
# Operating PIO at 14x the bit clock lets us achieve nominal 357ns and 714ns
DEBUG = 0 # Set to 1 to output OPPOSITE signal on the next higher pin
_program = Program(
    f"""
.side_set {2 if DEBUG else 1}
.wrap_target
    pull block          side 1
    out y, 32           side 1      ; get count of pixel bits

bitloop:
    pull ifempty        side 1      ; drive low
    out x 1             side 1 [4]
    jmp !x do_zero      side {2 if DEBUG else 0} [3]  ; drive low and branch depending on bit val
    jmp y--, bitloop    side {2 if DEBUG else 0} [3]  ; drive low for a one (long pulse)
    jmp end_sequence    side 1      ; sequence is over

do_zero:
    jmp y--, bitloop    side 1 [3]  ; drive high for a zero (short pulse)

end_sequence:
    pull block          side 1      ; get fresh delay value
    out y, 32           side 1      ; get delay count
wait_reset:
    jmp y--, wait_reset side 1      ; wait until delay elapses
.wrap
        """
)

def convert_brightness(x):
    x = int(x * 63) + 13
    x |= (x << 8)
    return x | (x << 16)

class TM1814PixelBackground(  # pylint: disable=too-few-public-methods
    adafruit_pixelbuf.PixelBuf
):
    def __init__(
        self, pin, n, *, brightness=1.0
    ):
        pixel_order = "RGBW"

        n = max(n, 4) # minimum of 4 pixels

        byte_count = 4 * n
        bit_count = byte_count * 8 + 64 # count the 64 brightness bits
        padding_count = -byte_count % 4

        self._brightness = brightness
        raw_brightness = convert_brightness(brightness)
        print(f"{raw_brightness=:08x}")
        # backwards, so that dma byteswap corrects it!
        header = struct.pack(">LLL", bit_count - 1, raw_brightness, raw_brightness ^ 0xffffffff)
        trailer = b"\0" * padding_count + struct.pack(">L", 38400) # Delay is about 3ms

        self._sm = StateMachine(
            _program.assembled,
            auto_pull=False,
            first_sideset_pin=pin,
            out_shift_right=False,
            pull_threshold=32,
            frequency=800_000 * 14,
            **_program.pio_kwargs,
        )

        self._buf = None
        super().__init__(
            n,
            brightness=1.0,
            byteorder=pixel_order,
            auto_write=False,
            header=header,
            trailer=trailer,
        )

        self.show()

    @property
    def auto_write(self):
        return True

    @property
    def brightness(self):
        return self._brightness

    @brightness.setter
    def brightness(self, brightness):
        raw_brightness = convert_brightness(brightness)
        self._brightness = brightness
        struct.pack_into("<LL", self._buf, 4, raw_brightness, raw_brightness ^ 0xffffffff)

    def _transmit(self, buf):
        self._buf = buf
        self._sm.background_write(loop=memoryview(buf).cast("L"), swap=True)


if __name__ == "__main__":
    import board
    import rainbowio
    import supervisor

    NEOPIXEL = board.A0
    NUM_PIXELS = 150
    pixels = TM1814PixelBackground(NEOPIXEL, NUM_PIXELS, brightness=0.1)
    buf = memoryview(pixels._buf).cast('L')
    for _ in range(1000):
        # Around 1 cycle per second
        pixels.fill(rainbowio.colorwheel(supervisor.ticks_ms() // 4))
        print(*(f"{pixel:08x}" for pixel in buf[:8]), "...", f"{buf[-1]:08x}")
