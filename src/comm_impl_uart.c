/*
 * comm_impl.c
 *
 *  Created on: Jul 1, 2012
 *      Author: petera
 */

#include "comm_impl.h"
#include "stm32f10x.h"
#include "system.h"
#include "uart.h"
#include "taskq.h"
#include "heap.h"
#include "cnc_comm.h"
#include "comm.h"
#include "comm_file.h"

#define POOL_GUARD          0xee

typedef struct {
  u8_t free;
  u8_t data[COMM_LNK_MAX_DATA];
  u8_t guard;
  comm_arg rx;
} pool_pkt;

typedef struct {
  uart *uart;
} comm_channel;

static struct {
  // communication stack
  comm driver;
  // communication stack uart port
  uart *uart;
  // original communication stack link layer rx to upper layer report function
  comm_rx_fn post_link_comm_rx_up_f;
  // packet report task
  task *task;
  // communication stack ticker
  task_timer timer;
  // rx zero byte counter
  u32_t zcount;
#if COMM_IMPL_USE_POOL
  // current pool packet index
  u32_t pool_ix;
  // current pool use
  volatile u32_t pool_use;
  // high watermark pool use
  volatile u32_t max_pool_use;
  // packet pool, used in irq context
  pool_pkt pool[COMM_PKT_POOL_SIZE];
#endif
  u8_t channel;
  comm_channel channels[COMM_UARTS];
} ucomm;

comm *COMM_UART_get_comm() {
  return &ucomm.driver;
}

//
// comm callback implementations
//

// *** phy

static void COMM_UART_zcount_warn(u32_t a, void* v) {
  DBG(D_COMM, D_WARN, "COMM %08i warn lots of zeros\n", SYS_get_time_ms());
  UART_put_char(ucomm.uart, '?');
}

//typedef int (*comm_phy_tx_char_fn)(unsigned char c);
static int COMM_UART_tx_char(unsigned char c) {
  UART_put_char(ucomm.uart, c);
  return R_COMM_OK;
}

static int COMM_UART_tx_buf(unsigned char *c, unsigned short len) {
  s32_t res = UART_put_buf(ucomm.uart, c, len);
  return (res != len) ? R_COMM_PHY_FAIL : R_COMM_OK;
}

//typedef int (*comm_phy_rx_char_fn)(unsigned char *c);
// NOT USED
// doing callback in uart driver rx callback function instead


// uart receive char callback function, irq context
void COMM_UART_comm_phy_rx(void* arg, u8_t c) {
  comm* com = (comm*)arg;
  if (c)
    ucomm.zcount = 0;
  else
    ucomm.zcount++;
  if (ucomm.zcount > 256) {
    task *pkt_task = TASK_create(COMM_UART_zcount_warn, 0);
    TASK_run(pkt_task, 0, 0);
    ucomm.zcount = 0;
  }
  // directly call communication stack phy layer rx to upper layer report function
  com->phy.up_rx_f(com, c, NULL);
}

// *** memory

//typedef void (*comm_lnk_alloc_rx_fn)(comm *comm, void **data, void **arg, unsigned int data_len, unsigned int arg_len);
static void COMM_UART_alloc(comm *c, void **data, void **arg, unsigned int size_data, unsigned int size_arg) {
#if COMM_IMPL_USE_POOL
  // called from link layer, irq context: allocate a pkt from pool
  ASSERT(ucomm.pool_use < COMM_PKT_POOL_SIZE);
  *data = &ucomm.pool[ucomm.pool_ix].data[0];
  *arg = &ucomm.pool[ucomm.pool_ix].rx;
  ucomm.pool[ucomm.pool_ix].guard = POOL_GUARD;
  ucomm.pool_use++;
  ucomm.pool_ix++;
  ucomm.max_pool_use = MAX(ucomm.max_pool_use, ucomm.pool_use);
  if (ucomm.pool_ix >= COMM_PKT_POOL_SIZE) {
    ucomm.pool_ix = 0;
  }
#else
  *data = HEAP_malloc(size_data);
  *arg = HEAP_malloc(size_arg);
#endif
}

//typedef void (*comm_lnk_free_rx_fn)(comm *comm, void *data, void *arg);
static void COMM_UART_free(comm *c, void *data, void *arg) {
  // freed in app layer when finished
#if COMM_IMPL_USE_POOL
  //_comm.pool_use--;
#else
//  HEAP_free(data);
//  HEAP_free(arg);
#endif
}

// ** stack

//
// comm impl
//

// comm stack lnk layer circumvention, irq -> kernel context

// called from link layer via task queue
// for further comm stack handling
static void COMM_UART_task_on_pkt(u32_t arg, void* arg_p) {
  comm_arg *rx = (comm_arg *)arg_p;
#if COMM_IMPL_USE_POOL
  // copy irq context pooled packet to kernel heap
  // preventing pooled packet to be overwritten by irq while kernel is parsing it
  u8_t *pkt_data = HEAP_malloc(rx->len);
  comm_arg *rx_new = HEAP_malloc(sizeof(comm_arg));
  ASSERT(pkt_data);
  ASSERT(rx_new);
  ASSERT(rx->data[COMM_LNK_MAX_DATA] == POOL_GUARD);
  memcpy(pkt_data, rx->data, rx->len);
  memcpy(rx_new, rx, sizeof(comm_arg));
  rx_new->data = pkt_data;

  ucomm.pool_use--;

  ucomm.post_link_comm_rx_up_f(&ucomm.driver, rx_new);

  HEAP_free(rx_new);
  HEAP_free(pkt_data);
#else
  ucomm.post_link_comm_rx_up_f(&ucomm.driver, rx);
  HEAP_free(rx->data);
  HEAP_free(rx);
#endif
}

// called from link layer when a packet is received, irq context
// creates a task for further comm stack handling instead of keep calling from within irq
static int COMM_UART_comm_lnk_rx(comm *com, comm_arg *rx) {
  task *pkt_task = TASK_create(COMM_UART_task_on_pkt, 0);
  TASK_run(pkt_task, 0, rx);
  return R_COMM_OK;
}

// task timer, communication stack ticker
static void COMM_UART_ticker(u32_t a, void* p) {
  comm_tick(&ucomm.driver, COMM_cb_get_tick_count());
}

//
// app specific
//

void COMM_UART_next_channel() {
  ucomm.channel++;
  if (ucomm.channel >= COMM_UARTS) {
    ucomm.channel = 0;
  }
  COMM_UART_set_uart(ucomm.channels[ucomm.channel].uart);
}

// set communication stacks physical uart port
void COMM_UART_set_uart(uart* u) {
  if (ucomm.uart) {
    UART_set_callback(ucomm.uart, NULL, NULL);
  }
  ucomm.uart = u;
  UART_set_callback(ucomm.uart, COMM_UART_comm_phy_rx, &ucomm.driver);
}

void COMM_UART_init(uart* u) {
  DBG(D_COMM, D_DEBUG, "COMM init\n");
  memset(&ucomm, 0, sizeof(comm));


  // comm stack setup
  comm_init(
      &ucomm.driver,        // comm stack struct
      1,                    // this address
      0,                    // comm_phy_rx_char - called from uart irq
      COMM_UART_tx_char,         // comm_phy_tx_char
      COMM_UART_tx_buf,          // comm_phy_tx_buf
      0,                    // comm_phy_tx_flush
      COMM_cb_get_tick_count,  // comm_app_get_time
      COMM_cb_rx_pkt,          // comm_app_user_rx
      COMM_cb_ack_pkt,         // comm_app_user_ack
      COMM_cb_err,             // comm_app_user_err
      COMM_cb_tra_inf,         // comm_app_user_inf
      COMM_cb_alert            // comm_app_alert
      );
  // using comm stacks allocation callback for pkt buffers
  comm_init_alloc(&ucomm.driver, COMM_UART_alloc, COMM_UART_free);

  // rewire communication stacks link callback to go via taskq
  // save old upcall from link layer
  ucomm.post_link_comm_rx_up_f = ucomm.driver.lnk.up_rx_f;
  // set new upcall to invoke old upcall via task instead
  ucomm.driver.lnk.up_rx_f = COMM_UART_comm_lnk_rx;

  // start comm ticker
  ucomm.task = TASK_create(COMM_UART_ticker, TASK_STATIC);
  TASK_start_timer(ucomm.task, &ucomm.timer, 0, 0, 0, COMM_RESEND_TICK/2, "ucomm_tick");

  {
    const u8_t comm_uart_list[] = COMM_UART_LIST;
    int i;
    for (i = 0; i < COMM_UARTS; i++) {
      ucomm.channels[i].uart = _UART(comm_uart_list[i]);
    }
  }

  // put UART characters into comms phy layer
  COMM_UART_set_uart(u);

  // stack now active
}

