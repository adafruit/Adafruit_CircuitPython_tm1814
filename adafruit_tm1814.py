# SPDX-FileCopyrightText: 2022 Jeff Epler, written for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
``adafruit_tm1814`` - TM1814 LED strip driver for RP2 microcontrollers
======================================================================

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
from adafruit_pioasm import Program
from rp2pio import StateMachine

# Pixel stream is very similar to NeoPixel WS2812B, but inverted.
#
# Pixel time is 1.25us (800kHz)
#
# Datasheet low times for a "0" bit are 310 (min) 360 (typ) 410 (max) ns
# Datasheet high times for a "1" bit are 650 (min) 720 (typ) 1000 (max) ns
#
# Operating PIO at 14x the bit clock lets us achieve nominal 357ns and 714ns
_pio_source = ()

TM1814_MIN_CURRENT = 6.5
TM1814_MAX_CURRENT = 38
TM1814_CURRENT_SCALE = 2


def _convert_one_current(value):
    if value < TM1814_MIN_CURRENT or value > TM1814_MAX_CURRENT:
        raise ValueError("Current control out of range")
    return round((value - TM1814_MIN_CURRENT) * TM1814_CURRENT_SCALE)


def _current_control_word(arg):
    if isinstance(arg, (int, float)):
        arg = arg, arg, arg, arg
    result = [_convert_one_current(value) for value in arg]
    result += [value ^ 0xFF for value in result]
    return result


class TM1814PixelBackground(  # pylint: disable=too-few-public-methods
    adafruit_pixelbuf.PixelBuf
):
    """
    A sequence of TM1814 addressable pixels

    Except as noted, provides all the functionality of
    `adafruit_pixelbuf.PixelBuf`, particularly
    `adafruit_pixelbuf.PixelBuf.fill` and
    `adafruit_pixelbuf.PixelBuf.__setitem__`.

    As the strip always auto-written, there is no need to call the `show` method.

    :param ~microcontroller.Pin pin: The pin to output neopixel data on.
    :param int n: The number of neopixels in the chain
    :param float brightness: Brightness of the pixels between 0.0 and 1.0 where 1.0 is full
      brightness. This brightness value is software-multiplied with raw pixel values.
    :param float|tuple[float,float,float] current_control: TM1814 current
      control register. See documentation of the ``current_control`` property
      below.
    :param str pixel_order: Set the pixel color channel order. WRGB is set by
      default. Only 4-bytes-per-pixel formats are supported.
    :param bool inverted: True to invert the polarity of the output signal.
    """

    def __init__(  # noqa: PLR0913
        self,
        pin,
        n: int,
        *,
        brightness: float = 1.0,
        pixel_order: str = "WRGB",
        current_control: float | tuple[float, float, float, float] = 38.0,
        inverted: bool = False,
    ):
        if len(pixel_order) != 4:
            raise ValueError("Invalid pixel_order")

        _program = Program(f"""
    .side_set 1
    .wrap_target
        pull block          side {not inverted:1d}
        out y, 32           side {not inverted:1d}      ; get count of pixel bits

    bitloop:
        pull ifempty        side {not inverted:1d}      ; drive low
        out x 1             side {not inverted:1d} [4]
        jmp !x do_zero      side {inverted:1d}     [3]  ; drive low and branch depending on bit val
        jmp y--, bitloop    side {inverted:1d}     [3]  ; drive low for a one (long pulse)
        jmp end_sequence    side {not inverted:1d}      ; sequence is over

    do_zero:
        jmp y--, bitloop    side {not inverted:1d} [3]  ; drive high for a zero (short pulse)

    end_sequence:
        pull block          side {not inverted:1d}      ; get fresh delay value
        out y, 32           side {not inverted:1d}      ; get delay count
    wait_reset:
        jmp y--, wait_reset side {not inverted:1d}      ; wait until delay elapses
    .wrap
            """)

        byte_count = 4 * n
        bit_count = byte_count * 8 + 64  # count the 64 brightness bits

        self._current_control = current_control

        # backwards, so that dma byteswap corrects it!
        header = struct.pack(">L8B", bit_count - 1, *_current_control_word(current_control))
        trailer = struct.pack(">L", 38400)  # Delay is about 3ms

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
            brightness=brightness,
            byteorder=pixel_order,
            auto_write=False,
            header=header,
            trailer=trailer,
        )

        super().show()

    def show(self) -> None:
        """Does nothing, because the strip is always auto-written"""

    @property
    def current_control(self) -> float | tuple[float, float, float, float]:
        """Access the current control register of the TM1814

        The TM1814 has a per-channel current control register that is shared across
        the entire strip.

        The current regulation range is from 6.5mA to 38mA in 0.5mA increments.
        Out of range values will throw ValueError.

        The relationship between human perception & LED current value is highly
        nonlinear: The lowest setting may appear only slightly less bright than the
        brightest setting, not 6x brighter as you might expect.

        If this property is set to a single number, then the same value is used for
        each channel. Otherwise, it must be a tuple of 4 elements where each element
        is applied to a different channel.
        """
        return self._current_control

    @current_control.setter
    def current_control(self, value: float | tuple[float, float, float]) -> None:
        struct.pack_into("8B", self._buf, 4, *_current_control_word(value))
        self._current_control = value

    def deinit(self) -> None:
        """Deinitialize the object"""
        self._sm.deinit()
        super().deinit()

    @property
    def auto_write(self) -> bool:
        """Returns True because the strip is always auto-written.

        Any value assigned to auto_write is ignored."""
        return True

    @auto_write.setter
    def auto_write(self, value: bool) -> None:
        pass

    def _transmit(self, buf: bytes) -> None:
        self._buf = buf
        self._sm.background_write(loop=memoryview(buf).cast("L"), swap=True)
