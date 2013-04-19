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

#define COMM_IMPL_STATS     1
#define COMM_IMPL_USE_POOL  1

#define POOL_GUARD          0xee

#if COMM_IMPL_STATS
static u32_t recd = 0;
static time clockpoint;
static u32_t avg_byps = 0;
static u32_t pktcount_rx = 0;
static u32_t pktcount_tx = 0;
static u32_t inseq_count = 0;
static u32_t outofseq_count = 0;
static u32_t resent_count = 0;
#endif

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
  // last received sequence number
  u16_t last_seqno;
  // bit mask of last received sequence number
  u32_t seq_mask;
  // last received packet
  comm_arg *cur_rx;
  u8_t channel;
  comm_channel channels[COMM_UARTS];
} _comm;

comm *COMM_get_comm() {
  return &_comm.driver;
}

int COMM_tx(int dst, u8_t* data, u16_t len, int ack) {
#if COMM_IMPL_STATS
  pktcount_tx++;
#endif
  s32_t res = comm_tx(&_comm.driver, dst, len, data, ack);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMM tx failed %i\n", res);
  return res;
}

int COMM_reply(u8_t *data, u16_t len) {
  s32_t res = comm_reply(&_comm.driver, _comm.cur_rx, len, data);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMM reply failed %i\n", res);
  return res;
}

//
// comm callback implementations
//

//typedef void (*comm_lnk_alloc_rx_fn)(comm *comm, void **data, void **arg, unsigned int data_len, unsigned int arg_len);
static void COMM_alloc(comm *c, void **data, void **arg, unsigned int size_data, unsigned int size_arg) {
#if COMM_IMPL_USE_POOL
  // called from link layer, irq context: allocate a pkt from pool
  ASSERT(_comm.pool_use < COMM_PKT_POOL_SIZE);
  *data = &_comm.pool[_comm.pool_ix].data[0];
  *arg = &_comm.pool[_comm.pool_ix].rx;
  _comm.pool[_comm.pool_ix].guard = POOL_GUARD;
  _comm.pool_use++;
  _comm.pool_ix++;
  _comm.max_pool_use = MAX(_comm.max_pool_use, _comm.pool_use);
  if (_comm.pool_ix >= COMM_PKT_POOL_SIZE) {
    _comm.pool_ix = 0;
  }
#else
  *data = HEAP_malloc(size_data);
  *arg = HEAP_malloc(size_arg);
#endif
}

//typedef void (*comm_lnk_free_rx_fn)(comm *comm, void *data, void *arg);
static void COMM_free(comm *c, void *data, void *arg) {
  // freed in app layer when finished
#if COMM_IMPL_USE_POOL
  //_comm.pool_use--;
#else
//  HEAP_free(data);
//  HEAP_free(arg);
#endif
}

//typedef comm_time (*comm_app_get_time_fn)(void);
static comm_time COMM_get_tick_count() {
  return SYS_get_time_ms();
}

//typedef int (*comm_phy_tx_char_fn)(unsigned char c);
static int COMM_tx_char(unsigned char c) {
  UART_put_char(_comm.uart, c);
  return R_COMM_OK;
}

static int COMM_tx_buf(unsigned char *c, unsigned short len) {
  s32_t res = UART_put_buf(_comm.uart, c, len);
  return (res != len) ? R_COMM_PHY_FAIL : R_COMM_OK;
}

//typedef int (*comm_phy_rx_char_fn)(unsigned char *c);
// NOT USED
// doing callback in uart driver rx callback function instead

static s16_t seq_delta(u16_t seqnoCurrent, u16_t seqnoLastRegistered) {
  if (seqnoCurrent < 0x100 && seqnoLastRegistered > 0xeff) {
    return seqnoCurrent + 0x1000 - seqnoLastRegistered;
  } else if (seqnoCurrent > 0xeff && seqnoLastRegistered < 0x100) {
    return -(seqnoLastRegistered + 0x1000 - seqnoCurrent);
  } else {
    return seqnoCurrent - seqnoLastRegistered;
  }
}

/* received stuff from rx->src, in here one might to call comm_app_reply  */
//typedef int (*comm_app_user_rx_fn)(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data);
static int COMM_rx_pkt(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data) {
  s32_t res = R_COMM_OK;
  _comm.cur_rx = rx;

  // avoid sending up packets that were resent but already received
  // keep track of latest 32 received packet sequence numbers
  s16_t seqd = seq_delta(rx->seqno, _comm.last_seqno);
  u8_t already_received = FALSE;

  // check if this packet already was received
  if (seqd <= 0) {
    already_received = _comm.seq_mask & (1 << (-seqd));
    // register this packet as received
    _comm.seq_mask |= (1 << (-seqd));
  } else {
    // shift received packet mask
    _comm.seq_mask <<= seqd;
    // register this packet as received
    _comm.seq_mask |= 1;
  }

  DBG(D_COMM, D_DEBUG, "COMM PKT: seq:%03x len:%02x src:%01x flg:%02x last:%03x delta:%i %s\n",
      rx->seqno, rx->len, rx->src, rx->flags, _comm.last_seqno, seqd, already_received ? "ALREADY RECEIVED": "FRESH");

  if (seqd > 0) {
    // got a new packet, register latest seqno
    _comm.last_seqno = rx->seqno;
  }


#if COMM_IMPL_STATS
  {
    pktcount_rx++;
    if ((rx->flags & COMM_FLAG_RESENT_BIT)) {
      resent_count++;
    }
    if (seqd > 1 || seqd < 0) {
      outofseq_count++;
    } else if (seqd == 1) {
      inseq_count++;
    }

    time now = SYS_get_time_ms();
    recd += len;
    if (now - clockpoint >= 1000) {
      u32_t d = (u32_t)(now - clockpoint);
      u32_t byps = (recd * 1000) / d;
      avg_byps = 1 * byps / 32 + 31 *avg_byps / 32;
      DBG(D_COMM, D_INFO, "COMM STAT thru %i bytes/s avg:%i\n", byps, avg_byps);
      clockpoint = now;
      recd = 0;
    }
  }
#endif

#if 0
#if D_COMM
  {
    int i;
    char c[256*3+2];
    for (i = 0; i < len; i++) {
      sprint(&c[i*3], "%02x ", data[i]);
    }
    c[len*3] = 0;
    DBG(D_COMM, D_DEBUG, "%s\n", c);
  }
#endif
#endif

#ifdef CONFIG_CNC
  if (!already_received) {
    if (data[0] == COMM_PROTOCOL_FILE_TRANSFER_R) {
      res = COMM_FILE_on_pkt(data, len);
    } else {
      res = CNC_COMM_on_pkt(rx->seqno, data, len);
    }
  }
#endif

  return res;
}

/* received an ack */
//typedef void (*comm_app_user_ack_fn)(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data);
static void COMM_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_DEBUG, "COMM ack seqno:0x%03x\n", seqno);
#ifdef CONFIG_CNC
  CNC_COMM_on_ack(seqno);
#endif
}

/* invoked on error */
//typedef void (*comm_app_user_err_fn)(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data);
static void COMM_err(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_WARN, "COMM ERR %i seqno:0x%03x\n", err, seqno);
#ifdef CONFIG_CNC
  CNC_COMM_on_err(seqno, err);
#endif
}

/* invoked on transport info */
//typedef void (*comm_app_user_inf_fn)(comm *comm, comm_arg *rx);
static void COMM_tra_inf(comm *comm, comm_arg *rx) {
  DBG(D_DEBUG, D_COMM, "COMM inf %02x\n", rx->data[0]);
}

static void COMM_zcount_warn(u32_t a, void* v) {
  DBG(D_COMM, D_WARN, "COMM %08i warn lots of zeros\n", SYS_get_time_ms());
  UART_put_char(_comm.uart, '?');
}

//
// comm impl
//

int COMM_dump() {
  int i,j;
  print("COMM\n----\n");
  print("  version %08x\n", CNC_COMM_VERSION);
#if COMM_IMPL_STATS
  print("  stat rx count:%i sequence:%i miss:%i resent:%i avgthruput:%ibytes/s\n",
      pktcount_rx, inseq_count, outofseq_count, resent_count, avg_byps);
  print("  stat tx count:%i\n",
      pktcount_tx);
#endif
#if COMM_IMPL_USE_POOL
  print("  sys pooluse:%i max:%i capacity:%i\n", _comm.pool_use, _comm.max_pool_use, COMM_PKT_POOL_SIZE);
#endif
  print("  channel: %i of %i\n", _comm.channel, COMM_UARTS);
  print("  lnk state:%i ix:%i len:%i l_crc:%04x buf:%p\n",
      _comm.driver.lnk.state, _comm.driver.lnk.ix, _comm.driver.lnk.len, _comm.driver.lnk.l_crc, _comm.driver.lnk.buf);
  print("  nwk addr:%i of %i\n",
      _comm.driver.nwk.addr, (1 + COMM_TRA_MAX_USERS));
  print("  tra seqno");
  for (i = 0; i < COMM_TRA_MAX_USERS; i++) {
    print(" [u:%i %03x]", i, _comm.driver.tra.seqno[i]);
  }
  print("\n");
  for (j = 0; j < COMM_MAX_PENDING; j++) {
    print("  tra tx_pend[ix:%i bsy:%i seqno:%03x resends:%i]\n", j,
        _comm.driver.tra.acks_tx_pend[j].busy,
        _comm.driver.tra.acks_tx_pend[j].arg.seqno,
        _comm.driver.tra.acks_tx_pend[j].resends);
  }
  for (i = 0; i < COMM_TRA_MAX_USERS; i++) {
    print("  tra rx_pend u:%i", i);
    for (j = 0; j < COMM_MAX_PENDING; j++) {
      print(" [ix:%i seqno:%04x]", j,
          _comm.driver.tra.acks_rx_pend[i][j]);
    }
    print("\n");
  }

  return 0;
}

// uart receive char callback function, irq context
void COMM_comm_phy_rx(void* arg, u8_t c) {
  comm* com = (comm*)arg;
  if (c)
    _comm.zcount = 0;
  else
    _comm.zcount++;
  if (_comm.zcount > 256) {
    task *pkt_task = TASK_create(COMM_zcount_warn, 0);
    TASK_run(pkt_task, 0, 0);
    _comm.zcount = 0;
  }
  // directly call communication stack phy layer rx to upper layer report function
  com->phy.up_rx_f(com, c);
}

// called from link layer via task queue
// for further comm stack handling
static void COMM_TASK_on_pkt(u32_t arg, void* arg_p) {
  comm_arg *rx = (comm_arg *)arg_p;
#if COMM_IMPL_USE_POOL
  // copy irq context pooled packet to task context heap
  u8_t *pkt_data = HEAP_malloc(rx->len);
  comm_arg *rx_new = HEAP_malloc(sizeof(comm_arg));
  ASSERT(pkt_data);
  ASSERT(rx_new);
  ASSERT(rx->data[COMM_LNK_MAX_DATA] == POOL_GUARD);
  memcpy(pkt_data, rx->data, rx->len);
  memcpy(rx_new, rx, sizeof(comm_arg));
  rx_new->data = pkt_data;

  _comm.pool_use--;

  _comm.post_link_comm_rx_up_f(&_comm.driver, rx_new);

  HEAP_free(rx_new);
  HEAP_free(pkt_data);
#else
  _comm.post_link_comm_rx_up_f(&_comm.driver, rx);
  HEAP_free(rx->data);
  HEAP_free(rx);
#endif
}

// called from link layer when a packet is received, irq context
// creates a task for further comm stack handling instead of keep calling from within irq
static int COMM_comm_lnk_rx(comm *com, comm_arg *rx) {
  task *pkt_task = TASK_create(COMM_TASK_on_pkt, 0);
  TASK_run(pkt_task, 0, rx);
  return R_COMM_OK;
}

// task timer, communication stack ticker
static void COMM_ticker(u32_t a, void* p) {
  comm_tick(&_comm.driver, COMM_get_tick_count());
}

void COMM_next_channel() {
  _comm.channel++;
  if (_comm.channel >= COMM_UARTS) {
    _comm.channel = 0;
  }
  COMM_set_uart(_comm.channels[_comm.channel].uart);
}

// set communication stacks physical uart port
void COMM_set_uart(uart* u) {
  if (_comm.uart) {
    UART_set_callback(_comm.uart, NULL, NULL);
  }
  _comm.uart = u;
  UART_set_callback(_comm.uart, COMM_comm_phy_rx, &_comm.driver);
}

void COMM_init(uart* u) {
  DBG(D_COMM, D_DEBUG, "COMM init\n");
  memset(&_comm, 0, sizeof(comm));


  // comm stack setup
  comm_init(
      &_comm.driver,        // comm stack struct
      1,                    // this address
      0,                    // comm_phy_rx_char - called from uart irq
      COMM_tx_char,         // comm_phy_tx_char
      COMM_tx_buf,          // comm_phy_tx_buf
      COMM_get_tick_count,  // comm_app_get_time
      COMM_rx_pkt,          // comm_app_user_rx
      COMM_ack_pkt,         // comm_app_user_ack
      COMM_err,             // comm_app_user_err
      COMM_tra_inf          // comm_app_user_inf
      );
  // using comm stacks allocation callback for pkt buffers
  comm_init_alloc(&_comm.driver, COMM_alloc, COMM_free);

  // rewire communication stacks link callback to go via taskq
  // save old upcall from link layer
  _comm.post_link_comm_rx_up_f = _comm.driver.lnk.up_rx_f;
  // set new upcall to invoke old upcall via task instead
  _comm.driver.lnk.up_rx_f = COMM_comm_lnk_rx;

  // start comm ticker
  _comm.task = TASK_create(COMM_ticker, TASK_STATIC);
  TASK_start_timer(_comm.task, &_comm.timer, 0, 0, 0, 100, "comm_tick");

#if COMM_IMPL_STATS
  clockpoint = SYS_get_time_ms();
#endif

  {
    const u8_t comm_uart_list[] = COMM_UART_LIST;
    int i;
    for (i = 0; i < COMM_UARTS; i++) {
      _comm.channels[i].uart = _UART(comm_uart_list[i]);
    }
  }

  // put UART characters into comms phy layer
  COMM_set_uart(u);

  // stack now active
}


