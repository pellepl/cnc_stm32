/*
 * config.h
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "system_config.h"

/***** Run time configuration *****/

#define CONFIG_NVR_MAGIC              0xf00dbeef
#define CONFIG_NVR_MAGIC_A            0
#define CONFIG_NVR_DBG_MASK_A         1
#define CONFIG_NVR_DBG_LEVEL_A        2
#define CONFIG_NVR_CNC_SR_TIM_REC_A   3
#define CONFIG_NVR_CNC_POS_TIM_REC_A  4

/***** NV CNC info *****/

#define CNC_NVR_POS_MAGIC             0xbeadfeed
#define CNC_NVR_POS_MAGIC_A           5
#define CNC_NVR_POS_X_A               6
#define CNC_NVR_POS_Y_A               7
#define CNC_NVR_POS_Z_A               8

#define CNC_NVR_OFFS_MAGIC            0xc0cac01a
#define CNC_NVR_OFFS_MAGIC_A          9
#define CNC_NVR_OFFS_X_A              10
#define CNC_NVR_OFFS_Y_A              11
#define CNC_NVR_OFFS_Z_A              12

s32_t CONFIG_load();
s32_t CONFIG_store();
s32_t CONFIG_CNC_pos_load();
s32_t CONFIG_CNC_offs_load();
s32_t CONFIG_CNC_pos_store(s32_t x, s32_t y, s32_t z);
s32_t CONFIG_CNC_offs_store(s32_t x, s32_t y, s32_t z);

#endif /* CONFIG_H_ */
