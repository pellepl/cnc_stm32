/*
 * config.c
 *
 *  Created on: Aug 2, 2012
 *      Author: petera
 */
#include "config.h"
#include "miniutils.h"
#include "nvstorage.h"
#include "system.h"
#include "cnc_comm.h"
#include "cnc_control.h"

s32_t CONFIG_load() {
  u32_t magic = 0;
  s32_t res;
  res = NVS_read(NV_RAM, CONFIG_NVR_MAGIC_A, &magic);
  if (res == NV_OK && magic == CONFIG_NVR_MAGIC) {
    u32_t d;
    res = NVS_read(NV_RAM, CONFIG_NVR_DBG_MASK_A, &d);
    if (res == NV_OK) {
      SYS_dbg_mask_set(d);
    }
    res = NVS_read(NV_RAM, CONFIG_NVR_DBG_LEVEL_A, &d);
    if (res == NV_OK) {
      SYS_dbg_level(d);
    }
#ifdef CONFIG_CNC
    res = NVS_read(NV_RAM, CONFIG_NVR_CNC_SR_TIM_REC_A, &d);
    if (res == NV_OK) {
      CNC_COMM_set_sr_timer_recurrence(d);
    }
    res = NVS_read(NV_RAM, CONFIG_NVR_CNC_POS_TIM_REC_A, &d);
    if (res == NV_OK) {
      CNC_COMM_set_pos_timer_recurrence(d);
    }
#endif
  } else {
    res = NV_ERR_BAD_MAGIC;
    DBG(D_ANY, D_WARN, "invalid config (magic = %08x)\n", magic);
  }
  return res;
}

s32_t CONFIG_store() {
  s32_t res;
  do {
    res = NVS_protect(NV_RAM, FALSE);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CONFIG_NVR_MAGIC_A, 0);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CONFIG_NVR_DBG_MASK_A, SYS_dbg_get_mask());
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CONFIG_NVR_DBG_LEVEL_A, SYS_dbg_get_level());
    if (res != NV_OK) break;
#ifdef CONFIG_CNC
    res = NVS_write(NV_RAM, CONFIG_NVR_CNC_SR_TIM_REC_A, CNC_COMM_get_sr_timer_recurrence());
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CONFIG_NVR_CNC_POS_TIM_REC_A, CNC_COMM_get_pos_timer_recurrence());
    if (res != NV_OK) break;
#endif

    res = NVS_write(NV_RAM, CONFIG_NVR_MAGIC_A, CONFIG_NVR_MAGIC);
    if (res != NV_OK) break;
  } while (0);

  if (res != NV_OK) {
    DBG(D_ANY, D_WARN, "failed writing config %i\n", res);
  }

  return res;
}

s32_t CONFIG_CNC_pos_store(s32_t x, s32_t y, s32_t z) {
  s32_t res;
  do {
    res = NVS_protect(NV_RAM, FALSE);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_POS_MAGIC_A, 0);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_POS_X_A, x);
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CNC_NVR_POS_Y_A, y);
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CNC_NVR_POS_Z_A, z);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_POS_MAGIC_A, CNC_NVR_POS_MAGIC);
    if (res != NV_OK) break;
  } while (0);

  if (res != NV_OK) {
    DBG(D_ANY, D_WARN, "failed writing nvr pos %i\n", res);
  }

  return res;
}

s32_t CONFIG_CNC_offs_store(s32_t x, s32_t y, s32_t z) {
  s32_t res;
  do {
    res = NVS_protect(NV_RAM, FALSE);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_OFFS_MAGIC_A, 0);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_OFFS_X_A, x);
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CNC_NVR_OFFS_Y_A, y);
    if (res != NV_OK) break;
    res = NVS_write(NV_RAM, CNC_NVR_OFFS_Z_A, z);
    if (res != NV_OK) break;

    res = NVS_write(NV_RAM, CNC_NVR_OFFS_MAGIC_A, CNC_NVR_OFFS_MAGIC);
    if (res != NV_OK) break;
  } while (0);

  if (res != NV_OK) {
    DBG(D_ANY, D_WARN, "failed writing nvr offs %i\n", res);
  }

  return res;
}

#ifdef CONFIG_CNC
s32_t CONFIG_CNC_pos_load() {
  u32_t magic = 0;
  s32_t res;
  res = NVS_read(NV_RAM, CNC_NVR_POS_MAGIC_A, &magic);
  if (res == NV_OK && magic == CNC_NVR_POS_MAGIC) {
    u32_t x, y, z;
    res = NVS_read(NV_RAM, CNC_NVR_POS_X_A, &x);
    if (res != NV_OK) {
      return res;
    }
    res = NVS_read(NV_RAM, CNC_NVR_POS_Y_A, &y);
    if (res != NV_OK) {
      return res;
    }
    res = NVS_read(NV_RAM, CNC_NVR_POS_Z_A, &z);
    if (res != NV_OK) {
      return res;
    }
    CNC_config_pos((s32_t)x, (s32_t)y, (s32_t)z);
  } else {
    res = NV_ERR_BAD_MAGIC;
    DBG(D_ANY, D_WARN, "invalid pos config (magic = %08x)\n", magic);
  }
  return res;
}

s32_t CONFIG_CNC_offs_load() {
  u32_t magic = 0;
  s32_t res;
  res = NVS_read(NV_RAM, CNC_NVR_OFFS_MAGIC_A, &magic);
  if (res == NV_OK && magic == CNC_NVR_OFFS_MAGIC) {
    u32_t x, y, z;
    res = NVS_read(NV_RAM, CNC_NVR_OFFS_X_A, &x);
    if (res != NV_OK) {
      return res;
    }
    res = NVS_read(NV_RAM, CNC_NVR_OFFS_Y_A, &y);
    if (res != NV_OK) {
      return res;
    }
    res = NVS_read(NV_RAM, CNC_NVR_OFFS_Z_A, &z);
    if (res != NV_OK) {
      return res;
    }
    CNC_config_offs_pos((s32_t)x, (s32_t)y, (s32_t)z);
  } else {
    res = NV_ERR_BAD_MAGIC;
    DBG(D_ANY, D_WARN, "invalid offs config (magic = %08x)\n", magic);
  }
  return res;
}
#endif

