/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_psram.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <stddef.h>

#include "arm_internal.h"

#include "hardware/rp23xx_qmi.h"
#include "hardware/rp23xx_io_qspi.h"
#include "hardware/rp23xx_xip.h"

#include "rp23xx_gpio.h"
#include "rp23xx_psram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO function select 9 on the CS1 pin (e.g. GPIO8 on the Adafruit
 * Feather RP2350, GPIO47 on the Pimoroni Pico Plus 2) muxes it to XIP_CS1,
 * the QMI's second chip select.  This alt-function index is pin-dependent
 * on rp23xx (the same FUNCSEL value means a different signal on other
 * GPIOs), so no generic RP23XX_GPIO_FUNC_xxx macro applies here.
 */

#define PSRAM_CS_FUNCSEL         9

/* PSRAM commands (Quad SPI, APS6404-class devices) */

#define PSRAM_CMD_QPI_ENABLE     0x35
#define PSRAM_CMD_READ_ID        0x9f

/* PSRAM Kernel/Good-Die IDs, read back via CMD_READ_ID */

#define PSRAM_KGD_PASS           0x5d

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psram_set_qmi_timing
 *
 * Description:
 *   Bring up the QMI M0 (flash) window timing for full-speed XIP.  This
 *   briefly reprograms the *flash's own* QMI window while the CPU is
 *   fetching instructions through it, so this function (and the dummy XIP
 *   read that settles the change) must run and complete entirely from
 *   RAM, never from flash.
 *
 ****************************************************************************/

static void locate_code(".data") noinline_function
psram_set_qmi_timing(void)
{
  volatile uint32_t *xip_nocache;

  /* Make sure flash is deselected - QMI doesn't appear to have a busy
   * flag for this.
   */

  while ((getreg32(RP23XX_IO_QSPI_GPIO_QSPI_SS_STATUS) &
          RP23XX_IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_MASK) !=
         RP23XX_IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_MASK)
    {
    }

  /* For > 133 MHz */

  putreg32(0x40000202, RP23XX_QMI_M0_TIMING);

  /* Force a read through the XIP no-allocate/no-cache alias to ensure the
   * new timing is applied before this function returns to flash-resident
   * code.
   */

  xip_nocache = (volatile uint32_t *)0x14000000;
  (void)*xip_nocache;
}

/****************************************************************************
 * Name: psram_detect
 *
 * Description:
 *   Probe the PSRAM chip select for a valid Kernel Good Die ID and return
 *   its size in bytes, or 0 if no PSRAM answered.
 *
 ****************************************************************************/

static size_t locate_code(".data") noinline_function
psram_detect(void)
{
  irqstate_t flags;
  size_t psram_size = 0;
  uint8_t kgd = 0;
  uint8_t eid = 0;
  size_t i;

  flags = enter_critical_section();

  /* Try and read the PSRAM ID via direct_csr. */

  putreg32((30 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN, RP23XX_QMI_DIRECT_CSR);

  /* Need to poll for the cooldown on the last XIP transfer to expire (via
   * direct-mode BUSY flag) before it is safe to perform the first
   * direct-mode operation.
   */

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
          RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  /* Exit out of QMI in case we've inited already.  modifyreg32() is an
   * out-of-line function; a direct call to it from this RAM-resident code
   * would be too far from its .text address for a Thumb BL/BLX (RAM and
   * flash are ~256 MiB apart, well past the ±16 MiB reach of a Thumb-2
   * branch-with-link), so read-modify-write is inlined by hand via the
   * getreg32()/putreg32() macros instead.
   */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

  /* Transmit as quad */

  putreg32(RP23XX_QMI_DIRECT_TX_OE |
           (2 << RP23XX_QMI_DIRECT_TX_IWIDTH_SHIFT) |
           0xf5, RP23XX_QMI_DIRECT_TX);

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
          RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  getreg32(RP23XX_QMI_DIRECT_RX);

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

  /* Read the id */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

  for (i = 0; i < 7; i++)
    {
      putreg32(i == 0 ? PSRAM_CMD_READ_ID : 0xff, RP23XX_QMI_DIRECT_TX);

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_TXEMPTY) == 0)
        {
        }

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
        {
        }

      if (i == 5)
        {
          kgd = getreg32(RP23XX_QMI_DIRECT_RX);
        }
      else if (i == 6)
        {
          eid = getreg32(RP23XX_QMI_DIRECT_RX);
        }
      else
        {
          getreg32(RP23XX_QMI_DIRECT_RX);
        }
    }

  /* Disable direct csr */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~(RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N | RP23XX_QMI_DIRECT_CSR_EN),
           RP23XX_QMI_DIRECT_CSR);

  if (kgd == PSRAM_KGD_PASS)
    {
      uint8_t size_id = eid >> 5;

      psram_size = 1024 * 1024; /* 1 MiB */

      if (eid == 0x26 || size_id == 2)
        {
          psram_size *= 8; /* 8 MiB */
        }
      else if (size_id == 0)
        {
          psram_size *= 2; /* 2 MiB */
        }
      else if (size_id == 1)
        {
          psram_size *= 4; /* 4 MiB */
        }
    }

  leave_critical_section(flags);
  return psram_size;
}

/****************************************************************************
 * Name: psram_init
 *
 * Description:
 *   Detect the PSRAM chip (the CS pin must already be muxed to XIP_CS1 by
 *   the caller) and bring up the QMI M1 window (timing, read/write command
 *   formats, QPI mode) for memory-mapped access.  Returns the detected
 *   PSRAM size in bytes, or 0 if no PSRAM answered.
 *
 ****************************************************************************/

static size_t locate_code(".data") noinline_function
psram_init(void)
{
  size_t psram_size;

  psram_size = psram_detect();
  if (psram_size == 0)
    {
      return 0;
    }

  psram_set_qmi_timing();

  /* Enable direct mode, PSRAM CS, clkdiv of 10 */

  putreg32((10 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN |
           RP23XX_QMI_DIRECT_CSR_AUTO_CS1N, RP23XX_QMI_DIRECT_CSR);
  while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
          RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  /* Enable QPI mode on the PSRAM */

  putreg32(RP23XX_QMI_DIRECT_TX_NOPUSH | PSRAM_CMD_QPI_ENABLE,
           RP23XX_QMI_DIRECT_TX);
  while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
          RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  /* Set PSRAM timing for APS6404:
   * - Max select assumes a sys clock speed >= 120 MHz
   * - Min deselect assumes a sys clock speed <= 138 MHz
   * - Clkdiv of 1 is OK up to 133 MHz.
   */

  putreg32((1  << RP23XX_QMI_TIMING_COOLDOWN_SHIFT) |
           (2  << RP23XX_QMI_TIMING_PAGEBREAK_SHIFT) |
           (15 << RP23XX_QMI_TIMING_MAX_SELECT_SHIFT) |
           (2  << RP23XX_QMI_TIMING_MIN_DESELECT_SHIFT) |
           (2  << RP23XX_QMI_TIMING_RXDELAY_SHIFT) |
           1,   /* CLKDIV occupies bits[7:0], shift 0 */
           RP23XX_QMI_M1_TIMING);

  /* Set PSRAM read command and format: quad prefix/addr/suffix/dummy/data,
   * 8-bit prefix length, 6 dummy cycles (in units of 4 bits).
   */

  putreg32(2 | /* PREFIX_WIDTH occupies bits[1:0], shift 0 */
           (2 << RP23XX_QMI_FMT_ADDR_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_SUFFIX_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_DUMMY_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_DATA_WIDTH_SHIFT) |
           RP23XX_QMI_FMT_PREFIX_LEN |
           (6 << RP23XX_QMI_FMT_DUMMY_LEN_SHIFT),
           RP23XX_QMI_M1_RFMT);

  putreg32(0xeb, RP23XX_QMI_M1_RCMD);

  putreg32(2 | /* PREFIX_WIDTH occupies bits[1:0], shift 0 */
           (2 << RP23XX_QMI_FMT_ADDR_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_SUFFIX_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_DUMMY_WIDTH_SHIFT) |
           (2 << RP23XX_QMI_FMT_DATA_WIDTH_SHIFT) |
           RP23XX_QMI_FMT_PREFIX_LEN,
           RP23XX_QMI_M1_WFMT);

  putreg32(0x38, RP23XX_QMI_M1_WCMD);

  /* Disable direct mode */

  putreg32(0, RP23XX_QMI_DIRECT_CSR);

  /* Enable writes to PSRAM */

  putreg32(getreg32(RP23XX_XIP_CTRL) | RP23XX_XIP_CTRL_WRITABLE_M1,
           RP23XX_XIP_CTRL);

  return psram_size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psramconfig
 *
 * Description:
 *   Detect and bring up the external QSPI PSRAM on CONFIG_RP23XX_PSRAM_
 *   CS1_GPIO, mapping it into the M1 XIP window.  A silent no-op if no
 *   PSRAM answers the chip select (psram_init() returns 0).
 *
 ****************************************************************************/

void rp23xx_psramconfig(void)
{
  /* Muxing the CS pin to XIP_CS1 does not touch the QMI direct-mode bus,
   * so it is safe to do from ordinary flash-resident code, unlike the rest
   * of PSRAM bring-up (see psram_init()).
   */

  rp23xx_gpio_set_function(CONFIG_RP23XX_PSRAM_CS1_GPIO, PSRAM_CS_FUNCSEL);

  psram_init();
}
