/****************************************************************************
 * arch/arm/include/rp23xx/psram.h
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

#ifndef __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H
#define __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H

#include <nuttx/config.h>

#if defined(CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#include <nuttx/mm/mm.h>

/****************************************************************************
 * Name: rp23xx_psram_heap
 *
 * Description:
 *   Return the separate PSRAM heap created by up_extraheaps_init() (see
 *   rp23xx_heaps.c). Only available when CONFIG_RP23XX_PSRAM_HEAP_SEPARATE
 *   is selected, since that is the only mode where PSRAM is its own,
 *   independently addressable mm_heap_s object rather than merged into the
 *   main heap or exposed as the default up_allocate_heap() region.
 *
 ****************************************************************************/

FAR struct mm_heap_s *rp23xx_psram_heap(void);

#endif /* CONFIG_RP23XX_PSRAM_HEAP_SEPARATE */

#endif /* __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H */
