/*
 * bootloader.h
 *
 *  Created on: Apr 6, 2013
 *      Author: petera
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include "bootloader_exec.h"

#define BOOTLOADER_TEXT __attribute__ (( section(".bootloader_text"), used, long_call ))
#define BOOTLOADER_DATA __attribute__ (( section(".bootloader_data"), used ))

typedef struct {
  bootloader_operation operation;
  u32_t suboperation;
  USART_TypeDef *uart_hw;
  SPI_TypeDef *spi_hw;
} bootdata;

typedef enum {
  FLASH_OK = 0,
  FLASH_ERR_BUSY,
  FLASH_ERR_WRITE_PROTECTED,
  FLASH_ERR_TIMEOUT,
  FLASH_ERR_OTHER
} FLASH_res;

extern bootdata boot;

void b_put(char c);
void b_putstr(const char *s);
void b_puthex8(u8_t x);
void b_puthex16(u16_t x);
void b_puthex32(u32_t x);
void b_puthex_buf(u8_t *d, u32_t len);
void b_putint(s32_t x);

void b_spif_read(u32_t addr, u8_t *b, u16_t len);
void b_spif_write(u32_t addr, u8_t *b, u16_t len);

void b_flash_open();
void b_flash_close();
FLASH_res b_flash_erase(u32_t addr);
FLASH_res b_flash_write(u32_t addr, u32_t data);
FLASH_res b_flash_unprotect();
FLASH_res b_flash_protect();


void bootloader_init();

#endif /* BOOTLOADER_H_ */
