/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_ota_mtd.c
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
 * This binds an MTD device over the image slot the running firmware did
 * *not* boot from, so a firmware update writes through the ordinary MTD
 * interface and cannot reach the image it is running.  The slot is chosen
 * from the BootROM's own boot info and partition table rather than from
 * configuration, so it follows the partition table actually in effect.
 *
 * Reads go through the non-translating XIP alias.  Under an A/B partition
 * table the BootROM translates the ordinary XIP window to whichever slot
 * booted, so a read there would answer with the running image's bytes
 * whatever address was asked for.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "rp23xx_rom.h"
#include "rp23xx_flash_mtd.h"
#include "rp23xx_ota_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000

/* Boot info returns an echoed flags word, then boot_word, a diagnostic and
 * two reboot params.  boot_word's third byte is the booted partition.
 */

#define SYS_INFO_BOOT_INFO      0x0040
#define BOOT_INFO_WORDS         5
#define BOOT_INFO_PARTITION_LSB 16

/* One echoed flags word, then a location and a flags word per partition. */

#define PT_INFO_PARTITION_LOCATION_AND_FLAGS 0x0010
#define PT_WORDS_PER_PARTITION               2
#define PT_QUERY_WORDS                       16

#define PT_LOCATION_FIRST_SECTOR_MASK 0x00001fff
#define PT_LOCATION_LAST_SECTOR_LSB   13
#define PT_LOCATION_LAST_SECTOR_MASK  0x00001fff
#define PT_SECTOR_SIZE                4096

/* Program unit and erase block, matching the flash chip's own limits. */

#define OTA_BLOCK_SIZE 256
#define OTA_ERASE_SIZE RP23XX_FLASH_MTD_BLOCK_SIZE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_ota_mtd_s
{
  struct mtd_dev_s mtd;
  uint32_t         base;   /* Storage offset of the slot */
  uint32_t         size;   /* Slot length in bytes */
};

typedef int (*rom_get_sys_info_f)(uint32_t *out, uint32_t words,
                                  uint32_t flags);
typedef int (*rom_get_partition_table_info_f)(uint32_t *out, uint32_t words,
                                              uint32_t partition_and_flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rp23xx_ota_mtd_s g_ota_mtd;
static bool g_ota_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: booted_partition
 *
 * Description:
 *   The partition index the running image booted from, or negative if the
 *   ROM does not report one.
 *
 ****************************************************************************/

static int booted_partition(void)
{
  uint32_t words[BOOT_INFO_WORDS];
  rom_get_sys_info_f get_sys_info;
  int filled;

  get_sys_info = (rom_get_sys_info_f)rom_func_lookup(ROM_FUNC_GET_SYS_INFO);
  if (get_sys_info == NULL)
    {
      return -ENOSYS;
    }

  filled = get_sys_info(words, BOOT_INFO_WORDS, SYS_INFO_BOOT_INFO);
  if (filled != BOOT_INFO_WORDS || words[0] != SYS_INFO_BOOT_INFO)
    {
      return -EIO;
    }

  return (int)(int8_t)((words[1] >> BOOT_INFO_PARTITION_LSB) & 0xff);
}

/****************************************************************************
 * Name: partition_bounds
 *
 * Description:
 *   Storage offset and length of one partition, and the partition count.
 *
 ****************************************************************************/

static int partition_bounds(int index, uint32_t *base, uint32_t *size,
                            int *count)
{
  uint32_t words[PT_QUERY_WORDS];
  rom_get_partition_table_info_f get_pt;
  uint32_t loc;
  uint32_t first;
  uint32_t last;
  int filled;
  int total;

  get_pt = (rom_get_partition_table_info_f)
    rom_func_lookup(ROM_FUNC_GET_PARTITION_TABLE_INFO);
  if (get_pt == NULL)
    {
      return -ENOSYS;
    }

  filled = get_pt(words, PT_QUERY_WORDS,
                  PT_INFO_PARTITION_LOCATION_AND_FLAGS);
  if (filled < 2 || words[0] != PT_INFO_PARTITION_LOCATION_AND_FLAGS)
    {
      return -EIO;
    }

  total = (filled - 1) / PT_WORDS_PER_PARTITION;
  if (count != NULL)
    {
      *count = total;
    }

  if (index < 0 || index >= total)
    {
      return -ENOENT;
    }

  loc   = words[1 + (index * PT_WORDS_PER_PARTITION)];
  first = loc & PT_LOCATION_FIRST_SECTOR_MASK;
  last  = (loc >> PT_LOCATION_LAST_SECTOR_LSB) & PT_LOCATION_LAST_SECTOR_MASK;

  *base = first * PT_SECTOR_SIZE;
  *size = (last + 1 - first) * PT_SECTOR_SIZE;
  return OK;
}

/****************************************************************************
 * Name: rp23xx_ota_erase
 ****************************************************************************/

static int rp23xx_ota_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks)
{
  FAR struct rp23xx_ota_mtd_s *priv = (FAR struct rp23xx_ota_mtd_s *)dev;
  uint32_t offset = startblock * OTA_ERASE_SIZE;
  size_t len = nblocks * OTA_ERASE_SIZE;
  int ret;

  if (offset + len > priv->size)
    {
      return -EINVAL;
    }

  ret = rp23xx_flash_region_erase(priv->base + offset, len);
  return ret < 0 ? ret : (int)nblocks;
}

/****************************************************************************
 * Name: rp23xx_ota_bread
 ****************************************************************************/

static ssize_t rp23xx_ota_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR uint8_t *buf)
{
  FAR struct rp23xx_ota_mtd_s *priv = (FAR struct rp23xx_ota_mtd_s *)dev;
  uint32_t offset = startblock * OTA_BLOCK_SIZE;
  size_t len = nblocks * OTA_BLOCK_SIZE;

  if (buf == NULL || offset + len > priv->size)
    {
      return -EINVAL;
    }

  memcpy(buf,
         (FAR const void *)(uintptr_t)(XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE +
                                       priv->base + offset),
         len);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_ota_bwrite
 ****************************************************************************/

static ssize_t rp23xx_ota_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct rp23xx_ota_mtd_s *priv = (FAR struct rp23xx_ota_mtd_s *)dev;
  uint32_t offset = startblock * OTA_BLOCK_SIZE;
  size_t len = nblocks * OTA_BLOCK_SIZE;
  int ret;

  if (buf == NULL || offset + len > priv->size)
    {
      return -EINVAL;
    }

  ret = rp23xx_flash_region_program(priv->base + offset, buf, len);
  return ret < 0 ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: rp23xx_ota_read
 ****************************************************************************/

static ssize_t rp23xx_ota_read(FAR struct mtd_dev_s *dev, off_t offset,
                               size_t nbytes, FAR uint8_t *buf)
{
  FAR struct rp23xx_ota_mtd_s *priv = (FAR struct rp23xx_ota_mtd_s *)dev;

  if (buf == NULL || offset < 0 || offset + nbytes > priv->size)
    {
      return -EINVAL;
    }

  memcpy(buf,
         (FAR const void *)(uintptr_t)(XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE +
                                       priv->base + offset),
         nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: rp23xx_ota_ioctl
 ****************************************************************************/

static int rp23xx_ota_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                            unsigned long arg)
{
  FAR struct rp23xx_ota_mtd_s *priv = (FAR struct rp23xx_ota_mtd_s *)dev;
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
              geo->blocksize    = OTA_BLOCK_SIZE;
              geo->erasesize    = OTA_ERASE_SIZE;
              geo->neraseblocks = priv->size / OTA_ERASE_SIZE;
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = rp23xx_flash_region_erase(priv->base, priv->size);
        break;

      case RP23XX_OTA_IOC_CONFIRM:
        {
          /* The ROM rewrites flash to clear the flag and wants 4 KiB of
           * word aligned scratch of its own.
           */

          static uint32_t scratch[OTA_ERASE_SIZE / sizeof(uint32_t)];

          ret = rp23xx_flash_explicit_buy((FAR uint8_t *)scratch,
                                          sizeof(scratch));
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ota_write
 *
 * Description:
 *   Stream image bytes into the slot at the file position.  Writes must be
 *   whole program units at a program-unit offset: the caller owns any
 *   buffering, so a short tail is its to pad rather than ours to hide.
 *
 ****************************************************************************/

static ssize_t ota_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct rp23xx_ota_mtd_s *priv = &g_ota_mtd;
  ssize_t nblocks;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  if ((filep->f_pos % OTA_BLOCK_SIZE) != 0 || (buflen % OTA_BLOCK_SIZE) != 0)
    {
      return -EINVAL;
    }

  nblocks = rp23xx_ota_bwrite(&priv->mtd, filep->f_pos / OTA_BLOCK_SIZE,
                              buflen / OTA_BLOCK_SIZE,
                              (FAR const uint8_t *)buffer);
  if (nblocks < 0)
    {
      return nblocks;
    }

  filep->f_pos += (off_t)buflen;
  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: ota_read
 ****************************************************************************/

static ssize_t ota_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct rp23xx_ota_mtd_s *priv = &g_ota_mtd;
  ssize_t ret;

  ret = rp23xx_ota_read(&priv->mtd, filep->f_pos, buflen,
                        (FAR uint8_t *)buffer);
  if (ret > 0)
    {
      filep->f_pos += (off_t)ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ota_seek
 ****************************************************************************/

static off_t ota_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct rp23xx_ota_mtd_s *priv = &g_ota_mtd;
  off_t pos;

  switch (whence)
    {
      case SEEK_SET:
        pos = offset;
        break;

      case SEEK_CUR:
        pos = filep->f_pos + offset;
        break;

      case SEEK_END:
        pos = (off_t)priv->size + offset;
        break;

      default:
        return -EINVAL;
    }

  if (pos < 0 || pos > (off_t)priv->size)
    {
      return -EINVAL;
    }

  filep->f_pos = pos;
  return pos;
}

/****************************************************************************
 * Name: ota_ioctl
 ****************************************************************************/

static int ota_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct rp23xx_ota_mtd_s *priv = &g_ota_mtd;

  UNUSED(filep);
  return rp23xx_ota_ioctl(&priv->mtd, cmd, arg);
}

static const struct file_operations g_ota_fops =
{
  .read  = ota_read,
  .write = ota_write,
  .seek  = ota_seek,
  .ioctl = ota_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct mtd_dev_s *rp23xx_ota_mtd_initialize(void)
{
  uint32_t base;
  uint32_t size;
  int count = 0;
  int booted;
  int target;
  int ret;

  if (g_ota_initialized)
    {
      errno = EBUSY;
      return NULL;
    }

  booted = booted_partition();
  if (booted < 0)
    {
      ferr("ERROR: no booted partition reported: %d\n", booted);
      errno = ENODEV;
      return NULL;
    }

  /* Two slots pair as A and B; the one not booted is the update target. */

  ret = partition_bounds(booted, &base, &size, &count);
  if (ret < 0 || count != 2)
    {
      ferr("ERROR: partition table not an A/B pair: %d, %d\n", ret, count);
      errno = ENODEV;
      return NULL;
    }

  target = (booted == 0) ? 1 : 0;
  ret = partition_bounds(target, &base, &size, NULL);
  if (ret < 0)
    {
      ferr("ERROR: no bounds for partition %d: %d\n", target, ret);
      errno = ENODEV;
      return NULL;
    }

  if ((base % OTA_ERASE_SIZE) != 0 || (size % OTA_ERASE_SIZE) != 0)
    {
      ferr("ERROR: partition %d is not erase-block aligned\n", target);
      errno = EINVAL;
      return NULL;
    }

  g_ota_mtd.base       = base;
  g_ota_mtd.size       = size;
  g_ota_mtd.mtd.erase  = rp23xx_ota_erase;
  g_ota_mtd.mtd.bread  = rp23xx_ota_bread;
  g_ota_mtd.mtd.bwrite = rp23xx_ota_bwrite;
  g_ota_mtd.mtd.read   = rp23xx_ota_read;
  g_ota_mtd.mtd.ioctl  = rp23xx_ota_ioctl;
  g_ota_mtd.mtd.name   = "rp23xx_ota";

  g_ota_initialized = true;

  syslog(LOG_INFO, "rp23xx_ota: slot %d at 0x%08lx, %lu KiB (booted %d)\n",
         target, (unsigned long)base, (unsigned long)(size / 1024), booted);

  return &g_ota_mtd.mtd;
}

int rp23xx_ota_register(FAR const char *path)
{
  if (rp23xx_ota_mtd_initialize() == NULL)
    {
      return -ENODEV;
    }

  return register_driver(path, &g_ota_fops, 0666, NULL);
}
