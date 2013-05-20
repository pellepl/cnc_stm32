/*
 * comm_impl_udp.c
 *
 *  Created on: May 11, 2013
 *      Author: petera
 */

#include "comm_impl.h"
#include "stm32f10x.h"
#include "system.h"
#include "heap.h"
#include "cnc_comm.h"
#include "comm.h"
#include "comm_file.h"
#include "os.h"
#include "enc28j60_spi_eth.h"

#include "net.h"
#include "ip_arp_udp_tcp.h"

static struct {
  // communication stack
  comm driver;
  os_thread thread;
  u8_t rx_frame[400];
  u8_t tx_frame[400];
  u16_t tx_ix;
} ecomm;

comm *COMM_UDP_get_comm() {
  return &ecomm.driver;
}

//
// comm callback implementations
//

// *** phy

//typedef int (*comm_phy_tx_char_fn)(unsigned char c);
static int COMM_UDP_tx_char(unsigned char c) {
  ecomm.tx_frame[ecomm.tx_ix++] = c;
  return R_COMM_OK;
}

static int COMM_UDP_tx_buf(unsigned char *c, unsigned short len) {
  memcpy(&ecomm.tx_frame[ecomm.tx_ix], c, len);
  ecomm.tx_ix += len;
  return R_COMM_OK;
}

//typedef int (*comm_phy_tx_flush_fn)(comm *comm, comm_arg* tx);
static int COMM_UDP_tx_flush(comm *comm, comm_arg* tx) {
  ecomm.tx_ix = UDP_DATA_P;
  if (ETH_SPI_tx_free()) {
    // TODO PETER
    u16_t comm_len = tx->len + 4;
    u8_t dip[4] = {192,168,0,102};
    u8_t dmac[6] = {0xa4,0xba,0xdb,0xfb,0xd4,0x8d};
    client_set_gwmac(dmac);
    send_udp_prepare(ecomm.tx_frame, 0xcafe, dip, 0xcafe);
    send_udp_finalize(ecomm.tx_frame, comm_len);
    if (ETH_SPI_send(ecomm.tx_frame, UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN + comm_len, 0)) {
      DBG(D_COMM, D_DEBUG, "COMM UDP sent frame %i bytes\n", UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN+comm_len);
      return R_COMM_OK;
    } else {
      DBG(D_COMM, D_DEBUG, "COMM UDP send frame %i bytes FAIL\n", UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN+comm_len);
      return R_COMM_PHY_FAIL;
    }
  } else {
    return R_COMM_PHY_FAIL;
  }
}

//typedef int (*comm_phy_rx_char_fn)(unsigned char *c);
// NOT USED
// doing callback in eth driver rx callback function instead

//
// comm impl
//

static void *COMM_UDP_thread_func(void *arg) {
  DBG(D_COMM, D_DEBUG, "COMM UDP eth rx thread started\n");
  time last_tick = SYS_get_time_ms();
  while (TRUE) {
    u16_t len;
    while (ETH_SPI_state() != ETH_UP) {
      DBG(D_COMM, D_DEBUG, "COMM UDP eth not up, sleep\n");
      OS_thread_sleep(1000);
    }
    if (ETH_SPI_state() == ETH_UP) {
      bool pkt = ETH_SPI_read(ecomm.rx_frame, &len, 100);
      if (ETH_SPI_state() != ETH_UP) {
        DBG(D_COMM, D_DEBUG, "COMM UDP eth down while waiting for frame\n");
        continue;
      } else {
        if (pkt) {
          // got packet, handle it
          bool fin = FALSE;
          int res = R_COMM_OK;
          int i = 0;
          DBG(D_COMM, D_DEBUG, "COMM UDP rx frame, len %i\n", len);
          while (i < len - UDP_DATA_P && !fin && res == R_COMM_OK) {
            res = ecomm.driver.phy.up_rx_f(&ecomm.driver, ecomm.rx_frame[i + UDP_DATA_P],
                &fin);
            i++;
          }
          DBG(D_COMM, D_DEBUG, "COMM UDP rx frame comm stack res:%i fin:%i\n", res, fin);
        }
        // check if tick is needed
        if (SYS_get_time_ms() - last_tick >= COMM_RESEND_TICK / 2) {
          last_tick = SYS_get_time_ms();
          //DBG(D_COMM, D_DEBUG, "COMM UDP eth rx tmo, tick\n");
          comm_tick(&ecomm.driver, COMM_cb_get_tick_count());
        }
      } // if eth still is up
    } // if eth is up
  } // while true
  return NULL ;
}

static struct {
  u32_t canary1;
  u8_t buf[COMM_LNK_MAX_DATA];
  u32_t canary2;
  comm_arg arg;
  u32_t canary3;
} ucomm_data;

static void COMM_UDP_alloc(comm *c, void **data, void **arg, unsigned int size_data, unsigned int size_arg) {
  // TODO PETER udp stack is synchronous, use static buffers directly
  *data = &ucomm_data.buf[0];
  *arg = &ucomm_data.arg;
  ucomm_data.canary1 = 0x123456fe;
  ucomm_data.canary2 = 0xfedc1234;
  ucomm_data.canary3 = 0x5678abcd;
}

//typedef void (*comm_lnk_free_rx_fn)(comm *comm, void *data, void *arg);
static void COMM_UDP_free(comm *c, void *data, void *arg) {
  ASSERT(ucomm_data.canary1 == 0x123456fe);
  ASSERT(ucomm_data.canary2 == 0xfedc1234);
  ASSERT(ucomm_data.canary3 == 0x5678abcd);
}

static u32_t comm_udp_thr_stack[0x101];

void COMM_UDP_init() {
  DBG(D_COMM, D_DEBUG, "COMM UDP init\n");
  memset(&ecomm, 0, sizeof(comm));

  ecomm.tx_ix = UDP_DATA_P;

  OS_thread_create(&ecomm.thread, OS_THREAD_FLAG_PRIVILEGED,
      COMM_UDP_thread_func, NULL, comm_udp_thr_stack,
      sizeof(comm_udp_thr_stack) - 4, "eth_comm");

  // comm stack setup
  comm_init(&ecomm.driver,        // comm stack struct
      1,                    // this address
      0,                    // comm_phy_rx_char - called from eth callback
      COMM_UDP_tx_char,         // comm_phy_tx_char
      COMM_UDP_tx_buf,          // comm_phy_tx_buf
      COMM_UDP_tx_flush,        // comm_phy_tx_flush
      COMM_cb_get_tick_count,  // comm_app_get_time
      COMM_cb_rx_pkt,          // comm_app_user_rx
      COMM_cb_ack_pkt,         // comm_app_user_ack
      COMM_cb_err,             // comm_app_user_err
      COMM_cb_tra_inf,         // comm_app_user_inf
      COMM_cb_alert            // comm_app_alert
      );
  // using comm stacks allocation callback for pkt buffers
  comm_init_alloc(&ecomm.driver, COMM_UDP_alloc, COMM_UDP_free);

  // stack now active
}
