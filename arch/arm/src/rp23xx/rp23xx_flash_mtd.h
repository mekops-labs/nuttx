/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_flash_mtd.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_FLASH_MTD_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_FLASH_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Erase-block size of the internal QSPI flash.  Callers carving the region
 * into partitions size them in these units. */

#define RP23XX_FLASH_MTD_BLOCK_SIZE (4 * 1024)

/****************************************************************************
 * Name: rp23xx_flash_mtd_initialize
 *
 * Description:
 *   Bind an MTD device over the CONFIG_RP23XX_FLASH_MTD_BASE/_SIZE region
 *   of the internal QSPI flash, using the ROM flash erase/program calls.
 *   That region must be reserved for this purpose - it is not checked
 *   against the firmware image's actual extent (see the driver's file
 *   header for how the reservation is enforced instead).
 *
 * Returned Value:
 *   A reference to the created MTD device, or NULL with errno set on
 *   failure (e.g. called more than once).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *rp23xx_flash_mtd_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_FLASH_MTD_H */
