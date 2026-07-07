===============================
Adafruit Feather RP2350
===============================

.. tags:: chip:rp2350

The `Adafruit Feather RP2350 <https://learn.adafruit.com/adafruit-feather-rp2350>`_
is a Feather-form-factor board supplied by Adafruit, built around the same
RP2350A / QFN-60 silicon as ``raspberrypi-pico-2``.  This board definition
templates off ``raspberrypi-pico-2`` and changes only the Feather-specific
pin map and console wiring.

Features
========

* RP2350A microcontroller chip (QFN-60 package)
* Dual-core ARM Cortex M33 processor, flexible clock running up to 150 MHz
* 520kB of SRAM, and 8MB of on-board QSPI Flash memory
* 8MB of on-board QSPI PSRAM (chip select on GPIO8; not yet usable - see
  Supported Capabilities)
* STEMMA QT / Qwiic I2C connector
* USB 1.1 Host and Device support
* Feather-standard pinout and form factor

Serial Console
==============

By default a serial console appears on GPIO0 (UART0 TX) and GPIO1
(UART0 RX). This console runs at 115200-8N1.  This is the console used
by the ``nsh`` configuration, and is confirmed working on real hardware.

The board can also be configured to use the USB connection as the serial
console (the ``usbnsh`` configuration).  This configuration builds and
links, but **does not currently enumerate on real rp23xx hardware** - the
chip's D+ pull-up never asserts (see the "Known issues" note below).  Use
``nsh`` (UART0) until that lands.

Buttons and LEDs
================

A single red user LED is controlled by GPIO7 and is configured as autoled
by default.

A BOOTSEL button, which if held down when power is first applied to the
board (or while tapping RESET), will cause the board to boot into
programming mode and appear as a storage device to the computer connected
via USB.  Saving a ``.uf2`` file to this device will replace the Flash ROM
contents on the board.  The board can also be flashed without the BOOTSEL
button over SWD (e.g. a Raspberry Pi Debug Probe or any CMSIS-DAP adapter).

Pin Mapping
===========

The Feather form factor does not use the Pico's numbered-castellated-pad
layout; pins are identified by silkscreened GPIO/function labels.

===== ================== ==========================================
GPIO  Silk label          Notes
===== ================== ==========================================
GPIO0 TX                  Default TX for UART0 serial console
GPIO1 RX                  Default RX for UART0 serial console
GPIO2 SDA                 Default SDA for I2C0 (also STEMMA QT)
GPIO3 SCL                 Default SCL for I2C0 (also STEMMA QT)
GPIO7 --                  On-board red user LED (autoled)
GPIO8 PCS                 PSRAM chip select; reserved when PSRAM
                          is enabled
GPIO20 MISO               Default RX for SPI
GPIO21 --                 On-board NeoPixel (powered from 3V3)
GPIO22 SCK                Default SCK for SPI
GPIO23 MOSI               Default TX for SPI
===== ================== ==========================================

Power Supply
============

The board can be powered via the USB-C connector.  The RP2350 chip runs
on 3.3 volts, supplied by an onboard voltage regulator.

Supported Capabilities
=======================

NuttX supports the following capabilities on this board:

* UART (console port)

  * GPIO0 (UART0 TX) and GPIO1 (UART0 RX) are used for the console.

* GPIO (LED via autoled)
* PIO (RP2350 Programmable I/O)
* Flash ROM Boot
* USB device

  * The ``usbnsh`` configuration builds and produces a valid ``.uf2``, but
    does not currently enumerate on real hardware - this is a chip-wide
    rp23xx driver bug (the USB-CDC D+ pull-up write lands before the
    controller/PHY are brought up, and is separately clobbered by an
    unconditional register write right after), not specific to this
    board.  Tracked and fixed upstream separately from this board
    addition.

There is currently no PSRAM support wired into this board's configs (the
rp23xx PSRAM driver itself was deleted from upstream by a prior refactor
and is being recovered separately); ``GPIO8``/``PCS`` is reserved for
this once it lands.

Installation
============

1. Download Raspberry Pi Pico SDK

.. code-block:: console

  $ git clone -b 2.2.0 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git

2. Download and install ``picotool``

   .. note::

      If not found at build time, this tool will also be automatically compiled
      from the SDK sources. Manually downloading or compiling it is
      `preferred <https://github.com/raspberrypi/pico-sdk/issues/1827>`__, though.

   Instructions can be found here: https://github.com/raspberrypi/picotool

3. Set PICO_SDK_PATH environment variable

.. code-block:: console

  $ export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

4. Configure and build NuttX

.. code-block:: console

  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh adafruit-feather-rp2350:nsh
  $ make V=1

5. Connect the board to a USB port while pressing BOOTSEL.
   The board will be detected as a USB Mass Storage Device.
   Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

6. To access the console, connect a USB-serial converter (or a debug
   probe's UART bridge) to GPIO0/GPIO1.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled on UART0, at 115200 bps).
Confirmed booting on real hardware.

usbnsh
------

Basic NuttShell configuration (console enabled via USB CDC/ACM). Builds
and produces a valid ``.uf2``, but see the "Known issues" note above -
the console does not currently come up over native USB on real rp23xx
hardware.
