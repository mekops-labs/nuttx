/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_ota_mtd.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_OTA_MTD_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_OTA_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_ota_mtd_initialize
 *
 * Description:
 *   Bind an MTD device over the image slot the running firmware did not
 *   boot from, for staging a firmware update.  Both the slot bounds and the
 *   choice of slot come from the BootROM's partition table and boot info,
 *   so the running image is never the one exposed.
 *
 *   The internal flash MTD must be bound first: that is what resolves the
 *   ROM flash entry points this shares.
 *
 * Returned Value:
 *   A reference to the created MTD device, or NULL with errno set - if the
 *   ROM reports no booted partition, if the table is not an A/B pair, or if
 *   called more than once.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *rp23xx_ota_mtd_initialize(void);

#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_OTA_MTD_H */
