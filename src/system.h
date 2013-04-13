/*
 * system.h
 *
 *  Created on: Jul 22, 2012
 *      Author: petera
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stm32f10x.h"
#include "system_config.h"
#include "types.h"

enum reboot_reason_e {
  REBOOT_COLD_START = 0,
  REBOOT_UNKONWN,
  REBOOT_USER,
  REBOOT_ASSERT,
  REBOOT_CRASH,
  REBOOT_EXEC_BOOTLOADER,
  REBOOT_BOOTLOADER,
};

#include "shared_mem.h"


#define ASSERT(x) \
do { \
  if (!(x)) {\
    SYS_assert(__FILE__, __LINE__);\
  }\
} while (0);

// want to keep DBG as macro giving compiler opportunity to optimize away all
// occurrences of DBG masks being 0
#if DBG_LEVEL_PREFIX
extern const char* __dbg_level_str[4];
#define DBG_LEVEL_PRINT print("%s ", __dbg_level_str[level])
#else
#define DBG_LEVEL_PRINT
#endif
#ifdef DBG_OFF
#define DBG(mask, level, f, ...)
#else
#define DBG(mask, level, f, ...) do { \
     if (((mask) & __dbg_mask) && level >= __dbg_level) { \
       if (DBG_TIMESTAMP_PREFIX) { \
         u8_t __hh; u8_t __mm; u8_t __ss; u16_t __mil; \
         SYS_get_time(NULL, &__hh, &__mm, &__ss, &__mil); \
         print("%02i:%02i:%02i.%03i ", __hh, __mm, __ss, __mil); \
       } \
       if (DBG_LEVEL_PREFIX) DBG_LEVEL_PRINT; \
       print((f), ## __VA_ARGS__); \
     } \
  } while (0)
#endif
/**
 * Called at startup
 */
void SYS_init();
/**
 * Called from timer irq. Returns TRUE on ms update, FALSE otherwise.
 */
bool SYS_timer();
/**
 * Get milliseconds since system clock start
 */
time SYS_get_time_ms();
/**
 * Get ticks since system clock start
 */
time SYS_get_tick();
/**
 * Get current system time
 */
void SYS_get_time(u16_t *d, u8_t *h, u8_t *m, u8_t *s, u16_t *ms);
/**
 * Sets current system time
 */
void SYS_set_time(u16_t d, u8_t h, u8_t m, u8_t s, u16_t ms);

void SYS_assert(const char* file, int line);
void SYS_hardsleep_ms(u32_t ms);
void SYS_dbg_mask_set(u32_t mask);
void SYS_dbg_mask_enable(u32_t mask);
void SYS_dbg_mask_disable(u32_t mask);
void SYS_dbg_level(u32_t level);
u32_t SYS_dbg_get_mask();
u32_t SYS_dbg_get_level();
u32_t SYS_build_number();
u32_t SYS_build_date();
void SYS_reboot(enum reboot_reason_e);

#define memcpy(d,s,n) __builtin_memcpy((d),(s),(n))
#define memset(s,t,n) __builtin_memset((s),(t),(n))
#define offsetof(st, m) \
     ((u32_t) ( (char *)&((st *)0)->m - (char *)0 ))

extern void *_variadic_call(void *func, int argc, void* args);
extern int _sqrt(int);

#define GPIO_enable(port, pin) (port)->BSRR = (pin)
#define GPIO_disable(port, pin) (port)->BSRR = ((pin)<<16)
#define GPIO_set(port, pin_ena, pin_dis) (port)->BSRR = (pin_ena) | ((pin_dis)<<16)
#define GPIO_read(port, pin) (((port)->IDR & (pin)) != 0)

extern volatile u32_t __dbg_mask;
extern volatile u32_t __dbg_level;

#endif /* SYSTEM_H_ */
