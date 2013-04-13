/*
 * nvstorage.h
 *
 *  Created on: Aug 1, 2012
 *      Author: petera
 */

#ifndef NVSTORAGE_H_
#define NVSTORAGE_H_

#include "system.h"

#define NV_OK             0
#define NV_ERR_BUSY       -1
#define NV_ERR_ADDR       -2
#define NV_ERR_ARG        -3
#define NV_ERR_PROTECTED  -4
#define NV_ERR_BAD_MAGIC  -5

typedef enum {
  NV_RAM = 0,
  NV_SPIFLASH = 1,
  _NV_END
} nv_device;

s32_t NVS_read(nv_device dev, u32_t a, u32_t *d);
s32_t NVS_write(nv_device dev, u32_t a, u32_t d);
s32_t NVS_read_buf(nv_device dev, u32_t a, u32_t *d, u32_t len);
s32_t NVS_write_buf(nv_device dev, u32_t a, u32_t *d, u32_t len);
s32_t NVS_protect(nv_device dev, char protect);
void NVS_init();

#endif /* NVSTORAGE_H_ */
