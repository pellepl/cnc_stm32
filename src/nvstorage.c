/*
 * nvstorage.c
 *
 *  Created on: Aug 1, 2012
 *      Author: petera
 */

#include "nvstorage.h"
#include "miniutils.h"
#include "spi_flash_m25p16.h"

typedef struct {
  volatile u8_t lock;
  volatile u8_t protected;
  u32_t max_size;
  void (*nv_init)();
  s32_t (*nv_read)(nv_device dev, u32_t a, u32_t *d);
  s32_t (*nv_write)(nv_device dev, u32_t a, u32_t d);
  s32_t (*nv_read_buf)(nv_device dev, u32_t a, u32_t *d, u32_t len);
  s32_t (*nv_write_buf)(nv_device dev, u32_t a, u32_t *d, u32_t len);
  s32_t (*nv_protect)(nv_device dev, char protect);
} nv_def;

static nv_def *_nv_def[_NV_END];

static nv_def NV_RAM_impl;

s32_t NVS_read(nv_device dev, u32_t a, u32_t *d) {
  if (d == NULL) {
    return NV_ERR_ARG;
  }
  nv_def *def = _nv_def[dev];
  if (a >= def->max_size) {
    return NV_ERR_ADDR;
  }
  if (def->lock) {
    return NV_ERR_BUSY;
  }
  def->lock = TRUE;
  s32_t r = def->nv_read(dev, a, d);
  def->lock = FALSE;
  return r;
}

s32_t NVS_write(nv_device dev, u32_t a, u32_t d) {
  nv_def *def = _nv_def[dev];
  if (a >= def->max_size) {
    return NV_ERR_ADDR;
  }
  if (def->protected) {
    return NV_ERR_PROTECTED;
  }
  if (def->lock) {
    return NV_ERR_BUSY;
  }
  def->lock = TRUE;
  s32_t r = def->nv_write(dev, a, d);
  def->lock = FALSE;
  return r;
}

s32_t NVS_protect(nv_device dev, char protect) {
  nv_def *def = _nv_def[dev];
  if (def->lock) {
    return NV_ERR_BUSY;
  }
  def->lock = TRUE;
  s32_t r = def->nv_protect(dev, protect);
  def->lock = FALSE;
  if (r == NV_OK) {
    def->protected = protect;
  }
  return r;
}


// Generic buffer helpers

static s32_t NV_generic_read_buf(nv_device dev, u32_t a, u32_t *d, u32_t len) {
  nv_def *def = _nv_def[dev];
  if (a + len >= def->max_size) {
    return NV_ERR_ADDR;
  }
  if (def->lock) {
    return NV_ERR_BUSY;
  }
  def->lock = TRUE;
  u32_t i;
  s32_t res = NV_OK;
  for (i = 0; res == NV_OK && i < len; i++) {
    res = def->nv_read(dev, i + a, d++);
  }
  def->lock = FALSE;
  return res;
}

static s32_t NV_generic_write_buf(nv_device dev, u32_t a, u32_t *d, u32_t len) {
  nv_def *def = _nv_def[dev];
  if (a + len >= def->max_size) {
    return NV_ERR_ADDR;
  }
  if (def->protected) {
    return NV_ERR_PROTECTED;
  }
  if (def->lock) {
    return NV_ERR_BUSY;
  }
  def->lock = TRUE;
  u32_t i;
  s32_t res = NV_OK;
  for (i = 0; res == NV_OK && i < len; i++) {
    res = def->nv_write(dev, i + a, *d++);
  }
  def->lock = FALSE;
  return res;
}

// NV RAM impl

#define A2BKP(x) ((x) < 9 ? (((x)+1)*4) : (((x)+16)*4))

static s32_t NV_RAM_read(nv_device dev, u32_t a, u32_t *d) {
  PWR_BackupAccessCmd(ENABLE);
  u16_t h = BKP_ReadBackupRegister(A2BKP(a*2));
  u16_t l = BKP_ReadBackupRegister(A2BKP(a*2 + 1));
  *d = (h << 16) | (l);
  PWR_BackupAccessCmd(DISABLE);
  return NV_OK;
}

static s32_t NV_RAM_write(nv_device dev, u32_t a, u32_t d) {
  PWR_BackupAccessCmd(ENABLE);
  BKP_WriteBackupRegister(A2BKP(a*2), (u16_t)(d>>16));
  BKP_WriteBackupRegister(A2BKP(a*2 + 1), (u16_t)(d));
  PWR_BackupAccessCmd(DISABLE);
  return NV_OK;
}

static s32_t NV_RAM_protect(nv_device dev, char protect) {
  PWR_BackupAccessCmd(protect ? DISABLE : ENABLE);
  return NV_OK;
}

static void NV_RAM_init() {
  NV_RAM_impl.max_size = 42;
  NV_RAM_impl.nv_read = NV_RAM_read;
  NV_RAM_impl.nv_write = NV_RAM_write;
  NV_RAM_impl.nv_read_buf = NV_generic_read_buf;
  NV_RAM_impl.nv_write_buf = NV_generic_write_buf;
  NV_RAM_impl.nv_protect = NV_RAM_protect;
  NV_RAM_impl.protected = TRUE;
  //PWR_BackupAccessCmd(ENABLE);
  //NVS_protect(NV_RAM, FALSE);
}

// end NV RAM impl


void NVS_init() {
  memset(&NV_RAM_impl, 0, sizeof(NV_RAM_impl));
  NV_RAM_impl.nv_init = NV_RAM_init;
  _nv_def[NV_RAM] = &NV_RAM_impl;
  int i;
  for (i = 0; i < _NV_END; i++) {
    _nv_def[i]->nv_init();
  }
}
