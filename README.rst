Introduction
============


.. image:: https://readthedocs.org/projects/circuitpython-tm1814/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/tm1814/en/latest/
    :alt: Documentation Status


.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/adafruit/Adafruit_CircuitPython_tm1814/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_tm1814/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

TM1814 RGB(W) LED driver for RP2 microcontrollers


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.



Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install adafruit_tm1814

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import board
    import rainbowio
    import supervisor
    from adafruit_tm1814 import TM1814PixelBackground

    # The pin where the LED strip data line is connected
    TM1814 = board.A0
    # The number of TM1814 controllers
    NUM_PIXELS = 100
    pixels = TM1814PixelBackground(TM1814, NUM_PIXELS, brightness=0.1)

    # Cycle the rainbow at about 1 cycle every 4 seconds
    while True:
        pixels.fill(rainbowio.colorwheel(supervisor.ticks_ms() // 16))


Documentation
=============
API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/tm1814/en/latest/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_tm1814/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
