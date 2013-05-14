/*
 * system.c
 *
 *  Created on: Jul 22, 2012
 *      Author: petera
 */

#include "system.h"
#include "miniutils.h"
#include "uart.h"
#include "cnc_control.h"
#include "os.h"

volatile u32_t __dbg_mask = 0xffffffff;
volatile u32_t __dbg_level = D_DEBUG;

#ifdef DBG_TRACE_MON
u16_t _trace_log[TRACE_SIZE];
volatile u32_t _trace_log_ix = 0;
#endif

#ifdef DBG_LEVEL_PREFIX
const char* __dbg_level_str[4] =
{ "DBG", "INF", "WRN", "FTL" };
#endif

static struct {
  volatile time time_tick;
  volatile time time_ms_c;
  volatile time time_sub;
  volatile u16_t time_ms;
  volatile u8_t time_s;
  volatile u8_t time_m;
  volatile u8_t time_h;
  volatile u16_t time_d;
} sys;

bool SYS_timer() {
  bool r = FALSE;
  sys.time_tick++;
  sys.time_sub++;
  if (sys.time_sub >= SYS_MAIN_TIMER_FREQ / SYS_TIMER_TICK_FREQ) {
    sys.time_sub = 0;
    sys.time_ms_c++;
    sys.time_ms++;
    r = TRUE;
    if (sys.time_ms == 1000) {
      sys.time_ms = 0;
      sys.time_s++;
      if (sys.time_s == 60) {
        sys.time_s = 0;
        sys.time_m++;
        if (sys.time_m == 60) {
          sys.time_m = 0;
          sys.time_h++;
          if (sys.time_h == 24) {
            sys.time_h = 0;
            sys.time_d++;
          }
        }
      }
    }
  }
  return r;
}

void SYS_init() {
  memset(&sys, 0, sizeof(sys));
#ifdef DBG_TRACE_MON
  _trace_log_ix = 0;
  memset(_trace_log, 0, sizeof(_trace_log));
#endif
}

time SYS_get_time_ms() {
  return sys.time_ms_c;
}

void SYS_get_time(u16_t *d, u8_t *h, u8_t *m, u8_t *s, u16_t *ms) {
  if (d) *d = sys.time_d;
  if (h) *h = sys.time_h;
  if (m) *m = sys.time_m;
  if (s) *s = sys.time_s;
  if (ms) *ms = sys.time_ms;
}

void SYS_set_time(u16_t d, u8_t h, u8_t m, u8_t s, u16_t ms) {
  if (h < 24 && m < 60 && s < 60 && ms < 1000) {
    sys.time_d = d;
    sys.time_h = h;
    sys.time_m = m;
    sys.time_s = s;
    sys.time_ms = ms;
  }
}

time SYS_get_tick() {
  return sys.time_tick;
}

void SYS_assert(const char* file, int line) {
  __disable_irq();
  SHMEM_set_reboot_reason(REBOOT_ASSERT);
#ifdef CONFIG_CNC
  CNC_enable_error(1<<CNC_ERROR_BIT_EMERGENCY);
#endif
  UART_sync_tx(_UART(STDOUT), TRUE);
  UART_tx_flush(_UART(STDOUT));
  uprint(STDOUT, TEXT_BAD("\nASSERT: %s:%i\n"), file, line);
  UART_tx_flush(_UART(STDOUT));
  OS_DBG_list_all(TRUE);
  UART_tx_flush(_UART(STDOUT));
  SYS_dump_trace();
  UART_tx_flush(_UART(STDOUT));
  const int ASSERT_BLINK = 0x100000;
  volatile int a;
  __asm__ volatile ("bkpt #0\n");
  while (1) {
    a = ASSERT_BLINK;
    while (a--) {
      if (a < ASSERT_BLINK/2) {
        GPIO_set(GPIOC, GPIO_Pin_7, GPIO_Pin_6);
      } else {
        GPIO_set(GPIOC, GPIO_Pin_6, GPIO_Pin_7);
      }
    }
  }
}

void SYS_hardsleep_ms(u32_t ms) {
  time release = SYS_get_time_ms() + ms;
  while (SYS_get_time_ms() < release);
}


void SYS_dbg_mask_set(u32_t mask) {
  __dbg_mask = mask;
}

void SYS_dbg_mask_enable(u32_t mask) {
  __dbg_mask |= mask;
}

void SYS_dbg_mask_disable(u32_t mask) {
  __dbg_mask &= ~mask;
}

void SYS_dbg_level(u32_t level) {
  __dbg_level = level;
}

u32_t SYS_dbg_get_level() {
  return __dbg_level;
}

u32_t SYS_dbg_get_mask() {
  return __dbg_mask;
}

extern char __BUILD_DATE;
extern char __BUILD_NUMBER;

u32_t SYS_build_number() {
  return (u32_t)&__BUILD_NUMBER;
}

u32_t SYS_build_date() {
  return (u32_t)&__BUILD_DATE;
}

__attribute__ (( noreturn )) void SYS_reboot(enum reboot_reason_e r) {
  SHMEM_set_reboot_reason(r);
  NVIC_SystemReset();
  while (1);
}

void SYS_dump_trace() {
#ifdef DBG_TRACE_MON
  const char *msg_text[] = TRACE_NAMES;
  const char *irq_text[] = TRACE_IRQ_NAMES;
  __disable_irq();
  bool old_sync_tx = UART_sync_tx(_UART(STDOUT), TRUE);
  UART_tx_flush(_UART(STDOUT));
  u32_t s_ix = _trace_log_ix;
  if (s_ix == 0) {
    s_ix = TRACE_SIZE - 1;
  } else {
    s_ix--;
  }
  u32_t ix = s_ix;
  u32_t t_len = 0;
  // find trace start
  while (_trace_log[ix] != 0) {
    t_len++;
    if (ix == 0) {
      ix = TRACE_SIZE - 1;
    } else {
      ix--;
    }
    if (ix == s_ix) break;
  }
  // display all trace chronologically
  while (t_len-- > 0)  {
    if (ix == TRACE_SIZE - 1) {
      ix = 0;
    } else {
      ix++;
    }
    u16_t e = _trace_log[ix];
    _trc_types op = e >> 8;
    u8_t arg = e & 0xff;
    if (_trace_log[ix] == 0) {
      break;
    }

    os_thread *t;
    switch (op) {
    case _TRC_OP_OS_CTX_LEAVE:
    case _TRC_OP_OS_CTX_ENTER:
    case _TRC_OP_OS_CREATE:
    case _TRC_OP_OS_YIELD:
    case _TRC_OP_OS_MUTACQ_LOCK:
    case _TRC_OP_OS_MUTWAIT_LOCK:
    case _TRC_OP_OS_SIGWAKED:
    case _TRC_OP_OS_TIMWAKED:
    case _TRC_OP_OS_DEAD:
      t = OS_DBG_get_thread_by_id(arg);
      if (t != NULL) {
        print("%s  %s\n", msg_text[op], t->name);
      } else {
        print(TEXT_BAD("%s  thread id: %02x\n"), msg_text[op], arg);
      }
      break;
    case _TRC_OP_IRQ_ENTER:
    case _TRC_OP_IRQ_EXIT:
      print("%s  %s\n", msg_text[op], irq_text[arg]);
      break;
    default:
      print("%s  %02x\n", msg_text[op], arg);
      break;
    }
  };
  UART_tx_flush(_UART(STDOUT));

  UART_sync_tx(_UART(STDOUT), old_sync_tx);
  __enable_irq();
#endif
}

