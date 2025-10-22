/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

 #ifndef _FAL_CFG_H_
 #define _FAL_CFG_H_
 
 #include "ch.h"
 #include "w25qxx.h"
 
 #define NOR_FLASH_DEV_NAME             "w25q64"
 #define FAL_PART_HAS_TABLE_CFG
 
 /* ===================== Flash device Configuration ========================= */
 extern const struct fal_flash_dev w25q64_flash;
 
 /* flash device table */
 #define FAL_FLASH_DEV_TABLE                                          \
 {                                                                    \
     &w25q64_flash,                                           \
 }
 /* ====================== Partition Configuration ========================== */
 #ifdef FAL_PART_HAS_TABLE_CFG
 /* partition table */
 #define FAL_PART_TABLE                                                               \
 {                                                                                    \
     {FAL_PART_MAGIC_WORD,        "kvdb1",     "w25q64",         0,   4 * 1024 * 1024, 0} \
 }
 #endif /* FAL_PART_HAS_TABLE_CFG */
 
 #endif /* _FAL_CFG_H_ */