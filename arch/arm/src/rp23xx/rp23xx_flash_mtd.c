/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_flash_mtd.c
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
 * This binds an MTD device over a fixed region of the internal QSPI flash,
 * reserved for that purpose, using the ROM's flash erase/program calls.
 * The region starts at CONFIG_RP23XX_FLASH_MTD_BASE and is
 * CONFIG_RP23XX_FLASH_MTD_SIZE bytes long, both counted from the start of
 * the flash's XIP window.
 *
 * That reservation is enforced by the board's own linker script rather
 * than by anything in this driver: every rp23xx board script declares a
 * FLASH memory region shorter than the physical chip (e.g. 4 MiB out of an
 * 8 MiB part), so a firmware image that grew into the reserved region
 * would fail to *link*, loudly, long before it could ever overwrite this
 * driver's storage at runtime.
 *
 * Erasing/programming requires momentarily taking the flash out of XIP
 * mode - the same physical pins the CPU is normally fetching instructions
 * through - so the code that does it cannot itself be running from flash.
 * do_erase()/do_program() are placed in .data so they run from RAM (the
 * same locate_code(".data") trick rp23xx_psram.c uses for its QMI-timing
 * code, since rp23xx has no dedicated ramfunc section). For the same
 * reason they call out only through the resolved ROM function pointers
 * (an indirect branch, unaffected by Thumb's branch-range limit) and never
 * directly to another flash-resident helper.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/ioctl.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "rp23xx_rom.h"
#include "rp23xx_flash_mtd.h"

#ifdef CONFIG_SMP
#include <nuttx/spinlock.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XIP_BASE                 0x10000000

/* Per the RP2350 datasheet (Table 10, "Address map for XIP bus segment"),
 * the no-cache/no-allocate alias sits at 0x14000000 - not 0x13000000 as on
 * rp2040, where the cached/non-cached and allocating/non-allocating modes
 * were four separate 16 MiB-spaced aliases (0x10/0x11/0x12/0x13000000).
 * RP2350 collapsed that to two: XIP_BASE (cached, allocating) and this one.
 */
#define XIP_NOCACHE_NOALLOC_BASE 0x14000000

#define FLASH_MTD_BASE_ADDR (XIP_BASE + CONFIG_RP23XX_FLASH_MTD_BASE)
#define FLASH_MTD_READ_ADDR \
  (XIP_NOCACHE_NOALLOC_BASE + CONFIG_RP23XX_FLASH_MTD_BASE)

/* Blocks are the smallest unit that can be erased; sectors the smallest
 * unit that can be programmed.  Both match the flash chip's own limits.
 */

#define FLASH_BLOCK_SIZE  (4 * 1024)
#define FLASH_BLOCK_COUNT (CONFIG_RP23XX_FLASH_MTD_SIZE / FLASH_BLOCK_SIZE)

#define FLASH_SECTOR_SIZE  256
#define FLASH_SECTOR_COUNT (CONFIG_RP23XX_FLASH_MTD_SIZE / FLASH_SECTOR_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_flash_dev_s
{
  struct mtd_dev_s mtd;
  mutex_t          lock;
};

typedef void (*connect_internal_flash_f)(void);
typedef void (*flash_exit_xip_f)(void);
typedef void (*flash_range_erase_f)(uint32_t addr, size_t count,
                                    uint32_t block_size, uint8_t block_cmd);
typedef void (*flash_range_program_f)(uint32_t addr, const uint8_t *data,
                                      size_t count);
typedef void (*flash_flush_cache_f)(void);
typedef void (*flash_enter_cmd_xip_f)(void);

#ifdef CONFIG_SMP
/* Coordinates "pause"/"resume" of the other core(s) while flash is out of
 * XIP mode, so they cannot fetch an instruction from it either.  Generic
 * NuttX SMP API - no rp23xx-specific dependency; ported verbatim from the
 * rp2040 flash driver.
 */

struct smp_isolation_data_s
{
  volatile spinlock_t cpu_wait;
  volatile spinlock_t cpu_pause;
  volatile spinlock_t cpu_resume;
  struct smp_call_data_s call_data;
};

struct smp_isolation_s
{
  int isolated_cpuid;
  struct smp_isolation_data_s cpu_data[CONFIG_SMP_NCPUS];
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rp23xx_flash_erase(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks);
static ssize_t rp23xx_flash_bread(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  FAR uint8_t *buffer);
static ssize_t rp23xx_flash_bwrite(FAR struct mtd_dev_s *dev,
                                   off_t startblock, size_t nblocks,
                                   FAR const uint8_t *buffer);
static ssize_t rp23xx_flash_read(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR uint8_t *buffer);
static int     rp23xx_flash_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rp23xx_flash_dev_s g_rp23xx_flash_dev =
{
  .lock = NXMUTEX_INITIALIZER,
};

static bool g_initialized;

static struct
{
  connect_internal_flash_f connect_internal_flash;
  flash_exit_xip_f         flash_exit_xip;
  flash_range_erase_f      flash_range_erase;
  flash_range_program_f    flash_range_program;
  flash_flush_cache_f      flash_flush_cache;
  flash_enter_cmd_xip_f    flash_enter_cmd_xip;
} g_rom_functions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
static int pause_cpu_handler(FAR void *context)
{
  FAR struct smp_isolation_data_s *cpu_data =
    (FAR struct smp_isolation_data_s *)context;

  spin_lock(&cpu_data->cpu_resume);
  spin_unlock(&cpu_data->cpu_pause);
  spin_lock(&cpu_data->cpu_wait);
  spin_unlock(&cpu_data->cpu_wait);
  spin_unlock(&cpu_data->cpu_resume);

  return OK;
}

static void init_smp_isolation(FAR struct smp_isolation_s *data)
{
  int cpuid;

  for (cpuid = 0; cpuid < CONFIG_SMP_NCPUS; cpuid++)
    {
      spin_lock_init(&data->cpu_data[cpuid].cpu_wait);
      spin_lock_init(&data->cpu_data[cpuid].cpu_pause);
      spin_lock_init(&data->cpu_data[cpuid].cpu_resume);
    }
}

static void enter_smp_isolation(FAR struct smp_isolation_s *data)
{
  FAR struct smp_isolation_data_s *cpu_data;
  int other;

  sched_lock();

  data->isolated_cpuid = this_cpu();

  for (other = 0; other < CONFIG_SMP_NCPUS; other++)
    {
      cpu_data = &data->cpu_data[other];

      if (other != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_wait);
          spin_lock(&cpu_data->cpu_pause);
          spin_unlock(&cpu_data->cpu_resume);
        }

      nxsched_smp_call_init(&cpu_data->call_data, pause_cpu_handler,
                            cpu_data);
      nxsched_smp_call_single_async(other, &cpu_data->call_data);
    }

  for (other = 0; other < CONFIG_SMP_NCPUS; other++)
    {
      cpu_data = &data->cpu_data[other];

      if (other != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_pause);
          spin_unlock(&cpu_data->cpu_pause);
        }
    }
}

static void leave_smp_isolation(FAR struct smp_isolation_s *data)
{
  FAR struct smp_isolation_data_s *cpu_data;
  int other;

  for (other = 0; other < CONFIG_SMP_NCPUS; other++)
    {
      if (other != data->isolated_cpuid)
        {
          spin_unlock(&data->cpu_data[other].cpu_wait);
        }
    }

  for (other = 0; other < CONFIG_SMP_NCPUS; other++)
    {
      cpu_data = &data->cpu_data[other];

      if (other != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_resume);
          spin_unlock(&cpu_data->cpu_resume);
        }
    }

  sched_unlock();
}
#endif

/****************************************************************************
 * Name: do_erase
 *
 * Description:
 *   RAM-resident: takes the flash out of XIP mode, erases, and restores
 *   XIP.  See this file's header comment for why it must run from RAM.
 *
 ****************************************************************************/

static void locate_code(".data") noinline_function
do_erase(uint32_t addr, size_t count)
{
  g_rom_functions.connect_internal_flash();
  g_rom_functions.flash_exit_xip();

  /* Tries 65536-byte blocks first (command 0xd8), falling back to
   * 4096-byte blocks as needed for any remainder.
   */

  g_rom_functions.flash_range_erase(addr, count, 65536, 0xd8);

  g_rom_functions.flash_flush_cache();
  g_rom_functions.flash_enter_cmd_xip();
}

/****************************************************************************
 * Name: do_program
 *
 * Description:
 *   RAM-resident counterpart of do_erase() for programming.
 *
 ****************************************************************************/

static void locate_code(".data") noinline_function
do_program(uint32_t addr, FAR const uint8_t *data, size_t count)
{
  g_rom_functions.connect_internal_flash();
  g_rom_functions.flash_exit_xip();

  g_rom_functions.flash_range_program(addr, data, count);

  g_rom_functions.flash_flush_cache();
  g_rom_functions.flash_enter_cmd_xip();
}

/****************************************************************************
 * Name: rp23xx_flash_erase
 ****************************************************************************/

static int rp23xx_flash_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks)
{
  FAR struct rp23xx_flash_dev_s *priv =
    (FAR struct rp23xx_flash_dev_s *)dev;
  irqstate_t flags;
  int ret;
#ifdef CONFIG_SMP
  struct smp_isolation_s smp_isolation;

  init_smp_isolation(&smp_isolation);
#endif

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SMP
  enter_smp_isolation(&smp_isolation);
#endif

  flags = enter_critical_section();

  do_erase(FLASH_MTD_BASE_ADDR + FLASH_BLOCK_SIZE * startblock,
          FLASH_BLOCK_SIZE * nblocks);

  leave_critical_section(flags);

#ifdef CONFIG_SMP
  leave_smp_isolation(&smp_isolation);
#endif

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_bread
 ****************************************************************************/

static ssize_t rp23xx_flash_bread(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  FAR uint8_t *buffer)
{
  FAR struct rp23xx_flash_dev_s *priv =
    (FAR struct rp23xx_flash_dev_s *)dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read through the XIP no-allocate/no-cache alias: programming does not
   * update the XIP cache, so a cached read here could return stale data.
   */

  memcpy(buffer,
        (FAR const void *)(uintptr_t)(FLASH_MTD_READ_ADDR +
                                      FLASH_SECTOR_SIZE * startblock),
        FLASH_SECTOR_SIZE * nblocks);

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_bwrite
 ****************************************************************************/

static ssize_t rp23xx_flash_bwrite(FAR struct mtd_dev_s *dev,
                                   off_t startblock, size_t nblocks,
                                   FAR const uint8_t *buffer)
{
  FAR struct rp23xx_flash_dev_s *priv =
    (FAR struct rp23xx_flash_dev_s *)dev;
  irqstate_t flags;
  int ret;
#ifdef CONFIG_SMP
  struct smp_isolation_s smp_isolation;

  init_smp_isolation(&smp_isolation);
#endif

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SMP
  enter_smp_isolation(&smp_isolation);
#endif

  flags = enter_critical_section();

  do_program(FLASH_MTD_BASE_ADDR + FLASH_SECTOR_SIZE * startblock,
            buffer, FLASH_SECTOR_SIZE * nblocks);

  leave_critical_section(flags);

#ifdef CONFIG_SMP
  leave_smp_isolation(&smp_isolation);
#endif

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_read
 ****************************************************************************/

static ssize_t rp23xx_flash_read(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct rp23xx_flash_dev_s *priv =
    (FAR struct rp23xx_flash_dev_s *)dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer,
        (FAR const void *)(uintptr_t)(FLASH_MTD_READ_ADDR + offset),
        nbytes);

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: rp23xx_flash_ioctl
 ****************************************************************************/

static int rp23xx_flash_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                              unsigned long arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo != NULL)
            {
              memset(geo, 0, sizeof(*geo));
              geo->blocksize    = FLASH_SECTOR_SIZE;
              geo->erasesize    = FLASH_BLOCK_SIZE;
              geo->neraseblocks = FLASH_BLOCK_COUNT;
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = rp23xx_flash_erase(dev, 0, FLASH_BLOCK_COUNT);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct mtd_dev_s *rp23xx_flash_mtd_initialize(void)
{
  if (g_initialized)
    {
      errno = EBUSY;
      return NULL;
    }

  if (FLASH_BLOCK_COUNT < 4)
    {
      errno = ENOMEM;
      return NULL;
    }

  g_rom_functions.connect_internal_flash =
    (connect_internal_flash_f)
    rom_func_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH);
  g_rom_functions.flash_exit_xip =
    (flash_exit_xip_f)rom_func_lookup(ROM_FUNC_FLASH_EXIT_XIP);
  g_rom_functions.flash_range_erase =
    (flash_range_erase_f)rom_func_lookup(ROM_FUNC_FLASH_RANGE_ERASE);
  g_rom_functions.flash_range_program =
    (flash_range_program_f)rom_func_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM);
  g_rom_functions.flash_flush_cache =
    (flash_flush_cache_f)rom_func_lookup(ROM_FUNC_FLASH_FLUSH_CACHE);
  g_rom_functions.flash_enter_cmd_xip =
    (flash_enter_cmd_xip_f)rom_func_lookup(ROM_FUNC_FLASH_ENTER_CMD_XIP);

  if (g_rom_functions.connect_internal_flash == NULL ||
      g_rom_functions.flash_exit_xip == NULL ||
      g_rom_functions.flash_range_erase == NULL ||
      g_rom_functions.flash_range_program == NULL ||
      g_rom_functions.flash_flush_cache == NULL ||
      g_rom_functions.flash_enter_cmd_xip == NULL)
    {
      ferr("ERROR: a required ROM flash function was not found\n");
      errno = ENOSYS;
      return NULL;
    }

  g_rp23xx_flash_dev.mtd.erase  = rp23xx_flash_erase;
  g_rp23xx_flash_dev.mtd.bread  = rp23xx_flash_bread;
  g_rp23xx_flash_dev.mtd.bwrite = rp23xx_flash_bwrite;
  g_rp23xx_flash_dev.mtd.read   = rp23xx_flash_read;
  g_rp23xx_flash_dev.mtd.ioctl  = rp23xx_flash_ioctl;
  g_rp23xx_flash_dev.mtd.name   = "rp23xx_flash";

  g_initialized = true;
  return &g_rp23xx_flash_dev.mtd;
}
