#include "enc28j60_spi_eth.h"
#include "stm32f10x.h"
#include "os.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"
#include "net.h"
#include "spi_dev_os_generic.h"
#include "miniutils.h"
#include "heap.h"
#include "dhcp.h"

#ifdef CONFIG_ETHSPI
u8_t mac_address[6] = ETH_MAC;
u8_t ip_address[4] = ETH_IP;

spi_dev_gen _enc28j60_spi_dev;

#define ETHSPI_MAX_PKT_SIZE       400
#define ETHSPI_TX_QUEUE_SIZE      3
#define ETHSPI_RX_QUEUE_SIZE      3

typedef struct {
  u8_t data[ETHSPI_MAX_PKT_SIZE];
  u16_t len;
} ethernet_frame;

static struct {
  volatile bool irq_pending;
  volatile bool active;

  os_cond irq_cond;
  os_mutex irq_mutex;
  os_cond tx_cond;
  os_mutex tx_mutex;
  os_cond rx_cond;
  os_mutex rx_mutex;

  u8_t rxbuf[ETHSPI_MAX_PKT_SIZE];
  u8_t txbuf[ETHSPI_MAX_PKT_SIZE];

  volatile void *rx_stack;
  os_thread irq_thread;

  bool rx_error;
  bool tx_error;

  volatile bool tx_ready;

  struct {
    ethernet_frame frames[ETHSPI_TX_QUEUE_SIZE];
    u8_t wix;
    u8_t rix;
    u8_t len;
  } tx_queue;

  struct {
    ethernet_frame frames[ETHSPI_RX_QUEUE_SIZE];
    u8_t wix;
    u8_t rix;
    volatile u8_t len;
  } rx_queue;

  struct {
    bool query;
    bool active;
    u8_t ipaddr[4];
    u8_t mask[4];
    u8_t gwip[4];
    u8_t dhcp_server[4];
    u8_t dns_server[4];
  } dhcp;

#ifdef ETH_STATS
  u32_t rx_packets;
  u32_t rx_data;
  u32_t tx_packets;
  u32_t tx_data;
#endif
} ethspi;

///////////////////// Helper functions /////////////////////

static void _eth_spi_store_and_signal_rx(u8_t *data, u16_t len) {
  OS_mutex_lock(&ethspi.rx_mutex);
  if (ethspi.rx_queue.len < ETHSPI_RX_QUEUE_SIZE) {
    ethernet_frame *f = &ethspi.rx_queue.frames[ethspi.rx_queue.wix];
    memcpy(f, data, len);
    ethspi.rx_queue.wix = (ethspi.rx_queue.wix+1) % ETHSPI_RX_QUEUE_SIZE;
    ethspi.rx_queue.len++;
    OS_cond_broadcast(&ethspi.rx_cond);
  } else {
    // overflow
    DBG(D_ETH, D_WARN, "ethspi RX user frame overflow\n");
  }

  OS_mutex_unlock(&ethspi.rx_mutex);
}

static void _eth_spi_handle_pkt() {
  u16_t rx_stat;
  int plen = enc28j60PacketReceive(ETHSPI_MAX_PKT_SIZE, ethspi.rxbuf, &rx_stat);
  if (plen == 0) {
    DBG(D_ETH, D_DEBUG, "ethspi no packet, rx_stat:%04x\n", rx_stat);
    return;
  }
#ifdef ETH_STATS
  ethspi.rx_packets++;
  ethspi.rx_data += plen;
#endif

  DBG(D_ETH, D_DEBUG, "ethspi got packet, len %i, rx_stat:%04x\n", plen, rx_stat);
  //printbuf(ethspi.rxbuf, MIN(64, plen));

  // doing dhcp, do not allow anything else right now
  if (ethspi.dhcp.query && ethspi.dhcp.active) {
    int dhcp_res;
    dhcp_res = check_for_dhcp_answer(ethspi.rxbuf, plen);
    DBG(D_ETH, D_DEBUG, "ethspi DHCP:%i state:%i\n", dhcp_res, dhcp_state());
    if (dhcp_state() == DHCP_STATE_OK) {
      ethspi.dhcp.query = FALSE;
      DBG(D_ETH, D_DEBUG, "ethspi DHCP OK\n");
      DBG(D_ETH, D_INFO, "ethspi DHCP ip   %i.%i.%i.%i\n", ethspi.dhcp.ipaddr[0], ethspi.dhcp.ipaddr[1], ethspi.dhcp.ipaddr[2], ethspi.dhcp.ipaddr[3]);
      DBG(D_ETH, D_DEBUG, "ethspi DHCP gwip %i.%i.%i.%i\n", ethspi.dhcp.gwip[0], ethspi.dhcp.gwip[1], ethspi.dhcp.gwip[2], ethspi.dhcp.gwip[3]);
      DBG(D_ETH, D_DEBUG, "ethspi DHCP mask %i.%i.%i.%i\n", ethspi.dhcp.mask[0], ethspi.dhcp.mask[1], ethspi.dhcp.mask[2], ethspi.dhcp.mask[3]);
      DBG(D_ETH, D_DEBUG, "ethspi DHCP dhcp %i.%i.%i.%i\n", ethspi.dhcp.dhcp_server[0], ethspi.dhcp.dhcp_server[1], ethspi.dhcp.dhcp_server[2], ethspi.dhcp.dhcp_server[3]);
      DBG(D_ETH, D_DEBUG, "ethspi DHCP dns  %i.%i.%i.%i\n", ethspi.dhcp.dns_server[0], ethspi.dhcp.dns_server[1], ethspi.dhcp.dns_server[2], ethspi.dhcp.dns_server[3]);
      memcpy(ip_address, ethspi.dhcp.ipaddr, 4);
      set_ip(ethspi.dhcp.ipaddr);
    }
    return;
  }

  // arp is broadcast if unknown but a host may also
  // verify the mac address by sending it to
  // a unicast address.
  if (eth_type_is_arp_and_my_ip(ethspi.rxbuf, plen)) {
    make_arp_answer_from_request(ethspi.rxbuf);
    return;
  }

  // check if ip packets (icmp or udp) are for us or broadcast:
  if (eth_type_is_ip_and_broadcast(ethspi.rxbuf, plen) == 0) {
    if (eth_type_is_ip_and_my_ip(ethspi.rxbuf, plen) == 0) {
      return;
    }
    if (ethspi.rxbuf[IP_PROTO_P]==IP_PROTO_ICMP_V &&
        ethspi.rxbuf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
      // a ping packet, let's send pong
      make_echo_reply_from_request(ethspi.rxbuf, plen);
      return;
    }
  }

/*
  // we listen on port 0xcafe
  if (ethspi.rxbuf[IP_PROTO_P] == IP_PROTO_UDP_V
      && ethspi.rxbuf[UDP_DST_PORT_H_P] == 0xca  && ethspi.rxbuf[UDP_DST_PORT_L_P] == 0xf) {
    int payloadlen = ethspi.rxbuf[UDP_LEN_L_P];//plen - 34 - UDP_HEADER_LEN;
    DBG(D_ETH, D_DEBUG, "ethspi UDP len:%i\n", payloadlen);
    //char *nisse = "hello wurlde";
    //make_udp_reply_from_request(ethbuf, nisse, strlen(nisse), 1200);
    DBG(D_ETH, D_DEBUG, "ethspi eth mac dst: %02x.%02x.%02x.%02x.%02x.%02x\n",
        ethspi.rxbuf[0], ethspi.rxbuf[1], ethspi.rxbuf[2], ethspi.rxbuf[3], ethspi.rxbuf[4], ethspi.rxbuf[5]); // ETH_DST_MAC
    DBG(D_ETH, D_DEBUG, "ethspi eth mac src: %02x.%02x.%02x.%02x.%02x.%02x\n",
        ethspi.rxbuf[6], ethspi.rxbuf[7], ethspi.rxbuf[8], ethspi.rxbuf[9], ethspi.rxbuf[10], ethspi.rxbuf[11]); // ETH_SRC_MAC
    DBG(D_ETH, D_DEBUG, "ethspi eth type:    %02x %02x\n",
        ethspi.rxbuf[12], ethspi.rxbuf[13]); // ETH_TYPE_H_P, ETH_TYPE_L_P
    DBG(D_ETH, D_DEBUG, "ethspi ip src:  %i.%i.%i.%i\n",
        ethspi.rxbuf[26], ethspi.rxbuf[27], ethspi.rxbuf[28], ethspi.rxbuf[29]); // IP_SRC_P
    DBG(D_ETH, D_DEBUG, "ethspi ip dst:  %i.%i.%i.%i\n",
        ethspi.rxbuf[30], ethspi.rxbuf[31], ethspi.rxbuf[32], ethspi.rxbuf[33]); // IP_DST_P
    DBG(D_ETH, D_DEBUG, "ethspi udp src port:  %i\n",
        (ethspi.rxbuf[34] << 8) | ethspi.rxbuf[35]); // UDP_SRC_PORT_H_P
    DBG(D_ETH, D_DEBUG, "ethspi udp dst port:  %i\n",
        (ethspi.rxbuf[36] << 8) | ethspi.rxbuf[37]); // UDP_DST_PORT_H_P
    DBG(D_ETH, D_DEBUG, "ethspi udp len:       %04x\n",
        (ethspi.rxbuf[38] << 8) | ethspi.rxbuf[39]); // UDP_LEN_H_P
    DBG(D_ETH, D_DEBUG, "ethspi udp checksum:  %04x\n",
        (ethspi.rxbuf[40] << 8) | ethspi.rxbuf[41]); // UDP_CHECKSUM_H_P
  }
  IF_DBG(D_ETH, D_DEBUG) {
    printbuf(&ethspi.rxbuf[0], plen);
  }
  */
  _eth_spi_store_and_signal_rx(ethspi.rxbuf, plen);
}

static void _eth_spi_send_ethernet_frame(u8_t *packet, u16_t len) {
  // Set the write pointer to start of transmit buffer area
  enc28j60Write(EWRPTL, TXSTART_INIT & 0xFF);
  enc28j60Write(EWRPTH, TXSTART_INIT >> 8);
  // Set the TXND pointer to correspond to the packet size given
  enc28j60Write(ETXNDL, (TXSTART_INIT + len) & 0xFF);
  enc28j60Write(ETXNDH, (TXSTART_INIT + len) >> 8);
  // write per-packet control byte (0x00 means use macon3 settings)
  enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
  // copy the packet into the transmit buffer
  enc28j60WriteBuffer(len, packet);
  // send the contents of the transmit buffer onto the network
  enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
#ifdef ETH_STATS
  ethspi.tx_packets++;
  ethspi.tx_data += len;
#endif
}

static void *_eth_spi_irq_thread_handler(void *a) {
  DBG(D_ETH, D_INFO, "ethspi irq thread started\n");
#ifdef ETH_INIT_DHCP
  ETH_SPI_dhcp();
#endif

  bool can_send = TRUE;
  while (ethspi.active) {
    // await interrupt
    if (!enc28j60hasRxPkt()) {
      OS_mutex_lock(&ethspi.irq_mutex);
      while (ethspi.active && !ethspi.irq_pending) {
        u32_t timeout = OS_cond_timed_wait(&ethspi.irq_cond, &ethspi.irq_mutex, 2000);
        if (timeout && !ethspi.irq_pending && ethspi.dhcp.active && dhcp_state() != DHCP_STATE_OK) {
          // getting dhcp, retry
          OS_mutex_unlock(&ethspi.irq_mutex);
          DBG(D_ETH, D_INFO, "ethspi request dhcp\n");
          ETH_SPI_dhcp();
          OS_mutex_lock(&ethspi.irq_mutex);
          continue;
        }
      }
      ethspi.irq_pending = FALSE;
      OS_mutex_unlock(&ethspi.irq_mutex);
    }

    // disable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIE, EIE_INTIE);

    // check & clear interrupts, handle incoming packets
    u8_t eir = 0;
    bool rx_pkt;
    while (ethspi.active && (
        ((eir = enc28j60Read(EIR)) &
            (EIR_PKTIF | EIR_RXERIF | EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF))
            | (rx_pkt = enc28j60hasRxPkt())
            )) {
      if (eir & EIR_RXERIF) {
        DBG(D_ETH, D_WARN, "ethspi RXERR interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_RXERIF);
        ethspi.rx_error = TRUE;
      }
      if (eir & EIR_TXERIF) {
        DBG(D_ETH, D_WARN, "ethspi TXERR interrupt\n");
        // reset transmit logic problem acc to errata
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
        ethspi.tx_error = TRUE;
        can_send = TRUE;
      }
      if (eir & EIR_TXIF) {
        DBG(D_ETH, D_DEBUG, "ethspi TX interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXIF);
        can_send = TRUE;
      }
      if (eir & EIR_DMAIF) {
        DBG(D_ETH, D_DEBUG, "ethspi DMA interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_DMAIF);
      }
      if (eir & EIR_LINKIF) {
        DBG(D_ETH, D_DEBUG, "ethspi LINK interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_LINKIF);
      }
      //if (eir & EIR_PKTIF) { // not reliable
      if (rx_pkt) {
        DBG(D_ETH, D_DEBUG, "ethspi PKT interrupt\n");
        // handle incoming packet directly
        _eth_spi_handle_pkt();
      }
    }

    // enc28j60 is free to send frames
    if (can_send) {
      OS_mutex_lock(&ethspi.tx_mutex);
      // check if there are any queued packets
      if (ethspi.tx_queue.len> 0) {
        DBG(D_ETH, D_DEBUG, "ethspi TX transmit frame %i, pending: %i\n", ethspi.tx_queue.rix, ethspi.tx_queue.len);
        can_send = FALSE;
        ethspi.tx_ready = FALSE;
        _eth_spi_send_ethernet_frame(
            &ethspi.tx_queue.frames[ethspi.tx_queue.rix].data[0],
            ethspi.tx_queue.frames[ethspi.tx_queue.rix].len);
        ethspi.tx_queue.rix = (ethspi.tx_queue.rix+1) % ETHSPI_TX_QUEUE_SIZE;
        if (ethspi.tx_queue.len == ETHSPI_TX_QUEUE_SIZE) {
          // notify threads waiting that more frames
          // now may be queued as queue is not full anymore
          OS_cond_broadcast(&ethspi.tx_cond);
        }
        ethspi.tx_queue.len--;
      } else {
        // flag that we are free to send frames
        ethspi.tx_ready = TRUE;
      }
      OS_mutex_unlock(&ethspi.tx_mutex);
    }

    // enable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE);

    // error handling
    if (ethspi.rx_error) {
      ethspi.rx_error = FALSE;
    }
    if (ethspi.tx_error) {
      ethspi.tx_error = FALSE;
    }
  } // active
  DBG(D_ETH, D_INFO, "ethspi thread ended\n");
  HEAP_free((void *)ethspi.rx_stack);
  ethspi.rx_stack = NULL;
  return NULL;
}

///////////////////// Interface functions /////////////////////

void ETH_SPI_start() {
  if (ethspi.rx_stack) {
    DBG(D_ETH, D_WARN, "ethspi thread already running\n");
    return;
  }
  ethspi.active = TRUE;
  ethspi.rx_stack = HEAP_malloc(0x404);
  OS_thread_create(
      &ethspi.irq_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      _eth_spi_irq_thread_handler,
      NULL,
      (void *)ethspi.rx_stack, 0x400,
      "ethspi");
}

void ETH_SPI_dhcp() {
  ethspi.dhcp.active = TRUE;
  ethspi.dhcp.query = TRUE;
  dhcp_start(&ethspi.txbuf[0],
      ETHSPI_MAX_PKT_SIZE,
      mac_address,
      ethspi.dhcp.ipaddr,
      ethspi.dhcp.mask,
      ethspi.dhcp.gwip,
      ethspi.dhcp.dhcp_server,
      ethspi.dhcp.dns_server
      );
}

bool ETH_SPI_send(u8_t *data, u16_t len, time timeout) {
  // queue a packet for tx
  OS_mutex_lock(&ethspi.tx_mutex);
  bool timed_out = FALSE;
  if (ethspi.active && ethspi.tx_ready) {
    // send directly
    ethspi.tx_ready = FALSE;
    _eth_spi_send_ethernet_frame(data, len);
  } else {
    // queue frame
    time alarm = SYS_get_time_ms() + timeout;
    while (ethspi.active && ethspi.tx_queue.len >= ETHSPI_TX_QUEUE_SIZE &&
        !(timed_out = SYS_get_time_ms() > alarm)) {
      DBG(D_ETH, D_DEBUG, "ethspi TX stalled, pending: %i\n", ethspi.tx_queue.len);
      OS_cond_timed_wait(&ethspi.tx_cond, &ethspi.tx_mutex, timeout);
    }
    if (ethspi.active && !timed_out) {
      ethernet_frame *pkt = &ethspi.tx_queue.frames[ethspi.tx_queue.wix];
      memcpy(pkt->data, data, len);
      pkt->len = len;
      DBG(D_ETH, D_DEBUG, "ethspi TX queued @ %i, pending: %i\n",ethspi.tx_queue.wix, ethspi.tx_queue.len+1);
      ethspi.tx_queue.wix = (ethspi.tx_queue.wix+1) % ETHSPI_TX_QUEUE_SIZE;
      ethspi.tx_queue.len++;
    } else if (!ethspi.active) {
      timed_out = TRUE;
    }
  }
  OS_mutex_unlock(&ethspi.tx_mutex);
  return !timed_out;
}

bool ETH_SPI_read(u8_t *data, u16_t *len, time timeout) {
  OS_mutex_lock(&ethspi.rx_mutex);
  bool timed_out = FALSE;
  time alarm = SYS_get_time_ms() + timeout;
  while (ethspi.active && ethspi.rx_queue.len == 0 &&
      !(timed_out = SYS_get_time_ms() > alarm)) {
    OS_cond_timed_wait(&ethspi.rx_cond, &ethspi.rx_mutex, timeout);
  }
  if (ethspi.active && !timed_out) {
    ethernet_frame *pkt = &ethspi.rx_queue.frames[ethspi.rx_queue.rix];
    memcpy(data, pkt->data, pkt->len);
    *len = pkt->len;
    ethspi.rx_queue.rix = (ethspi.rx_queue.rix+1) % ETHSPI_RX_QUEUE_SIZE;
    ethspi.rx_queue.len--;
  } else if (!ethspi.active) {
    timed_out = TRUE;
  }
  OS_mutex_unlock(&ethspi.rx_mutex);
  return !timed_out;
}

bool ETH_SPI_tx_free() {
  return ethspi.tx_ready;
}

int ETH_SPI_available() {
  return ethspi.rx_queue.len;
}


void ETH_SPI_stop() {
  ethspi.active = FALSE;
  OS_mutex_lock(&ethspi.irq_mutex);
  OS_cond_signal(&ethspi.irq_cond);
  OS_mutex_unlock(&ethspi.irq_mutex);
  OS_mutex_lock(&ethspi.tx_mutex);
  OS_cond_broadcast(&ethspi.tx_cond);
  OS_mutex_unlock(&ethspi.tx_mutex);
}


void ETH_SPI_irq() {
  if(EXTI_GetITStatus(SPI_ETH_INT_EXTI_LINE) != RESET) {
    EXTI_ClearITPendingBit(SPI_ETH_INT_EXTI_LINE);
    //DBG(D_ETH, D_DEBUG, "ethspi irq\n");
    if (ethspi.active) {
      ethspi.irq_pending = TRUE;
      OS_cond_signal(&ethspi.irq_cond);
    }
  }
}

void ETH_SPI_init() {
  memset(&ethspi, 0, sizeof(ethspi));
  ethspi.tx_ready = TRUE;

  OS_mutex_init(&ethspi.irq_mutex, OS_MUTEX_ATTR_CRITICAL_IRQ);
  OS_cond_init(&ethspi.irq_cond);
  OS_mutex_init(&ethspi.tx_mutex, 0);
  OS_cond_init(&ethspi.tx_cond);
  OS_mutex_init(&ethspi.rx_mutex, 0);
  OS_cond_init(&ethspi.rx_cond);

  DBG(D_ETH, D_DEBUG, "ethspi spigen init\n");
  SPI_DEV_GEN_init(
      &_enc28j60_spi_dev,
      SPIDEV_CONFIG_CPHA_1E |
      SPIDEV_CONFIG_CPOL_LO |
      SPIDEV_CONFIG_FBIT_MSB |
      SPIDEV_CONFIG_SPEED_9M,
      _SPI_BUS(1),
      SPI_ETH_GPIO_PORT, SPI_ETH_GPIO_PIN);
  DBG(D_ETH, D_DEBUG, "ethspi spigen open\n");
  SPI_DEV_GEN_open(&_enc28j60_spi_dev);
  DBG(D_ETH, D_DEBUG, "ethspi enc28j60 init\n");
  enc28j60Init(mac_address, EIE_PKTIE | EIE_TXIE | EIE_RXERIE | EIE_TXERIE);

  DBG(D_ETH, D_DEBUG, "ethspi enc28j60 init done, chip rev %02x\n", enc28j60getrev());
  DBG(D_ETH, D_DEBUG, "ethspi mac readback: %02x:%02x:%02x:%02x:%02x:%02x\n",
      enc28j60Read(MAADR5), enc28j60Read(MAADR4), enc28j60Read(MAADR3),
      enc28j60Read(MAADR2), enc28j60Read(MAADR1), enc28j60Read(MAADR0)
      );

  SYS_hardsleep_ms(20);
  enc28j60PhyWrite(PHLCON,0x476);
  SYS_hardsleep_ms(20);

  // init eth/ip layer
  init_ip_arp_udp_tcp(mac_address, ip_address, 80);

  DBG(D_ETH, D_INFO, "ethspi setup finished, ip %i.%i.%i.%i @ mac %02x.%02x.%02x.%02x.%02x.%02x\n",
      ip_address[0], ip_address[1], ip_address[2], ip_address[3], mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
}

void ETH_SPI_dump() {
  print("ETH SPI\n-------\n");
  print("State\n");
  print("  active:         %s\n", ethspi.active ? "ENABLED" : "DISABLED");
  print("  dhcp active:    %s %s\n", ethspi.dhcp.active ? "ACTIVE" : "INACTIVE", ethspi.dhcp.query ? "QUERY" : "IDLE");
  if (ethspi.dhcp.active && !ethspi.dhcp.query) {
    print("  dhcp leasetime: %08x (%i minutes left)\n", dhcp_lease_timeout(), (dhcp_lease_timeout() - SYS_get_time_ms()) / (1000*60));

  }
  print("  local ip:       %i.%i.%i.%i\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
  print("  gateway ip:     %i.%i.%i.%i\n", ethspi.dhcp.gwip[0], ethspi.dhcp.gwip[1], ethspi.dhcp.gwip[2], ethspi.dhcp.gwip[3]);
  print("  mask:           %i.%i.%i.%i\n", ethspi.dhcp.mask[0], ethspi.dhcp.mask[1], ethspi.dhcp.mask[2], ethspi.dhcp.mask[3]);
  print("  link up:        %s\n", enc28j60linkup() ? "YES": "NO");
  print("  tx pending:     %i/%i", ethspi.tx_queue.len, ETHSPI_TX_QUEUE_SIZE);
  if (ethspi.tx_queue.len == ETHSPI_TX_QUEUE_SIZE) {
    print(TEXT_NOTE(" FULL"));
  }
  print("\n");
  print("  rx pending:     %i/%i", ethspi.rx_queue.len, ETHSPI_RX_QUEUE_SIZE);
  if (ethspi.rx_queue.len == ETHSPI_RX_QUEUE_SIZE) {
    print(TEXT_NOTE(" FULL"));
  }
  print("\n");
#ifdef ETH_STATS
  print("  tx pkts:%i  sent:%i bytes\n", ethspi.tx_packets, ethspi.tx_data);
  print("  rx pkts:%i  recd:%i bytes\n", ethspi.rx_packets, ethspi.rx_data);
#endif

#if OS_DBG_MON
  print("OS\n");
  print("  IRQ THREAD, MUTEX & COND\n");
  OS_DBG_print_thread(&ethspi.irq_thread, TRUE, 2);
  OS_DBG_print_mutex(&ethspi.irq_mutex, TRUE, 2);
  OS_DBG_print_cond(&ethspi.irq_cond, TRUE, 2);
  print("  TX MUTEX & COND\n");
  OS_DBG_print_mutex(&ethspi.tx_mutex, TRUE, 2);
  OS_DBG_print_cond(&ethspi.tx_cond, TRUE, 2);
#endif
  print("HW\n");
  print("  chip rev:       %02x\n", enc28j60getrev());
  print("  mac readback:   %02x:%02x:%02x:%02x:%02x:%02x\n",
      enc28j60Read(MAADR5), enc28j60Read(MAADR4), enc28j60Read(MAADR3),
      enc28j60Read(MAADR2), enc28j60Read(MAADR1), enc28j60Read(MAADR0)
  );
  print("  irq flag:       %i\n", ethspi.irq_pending);
  print("  irq pin:        %i\n", GPIO_read(SPI_ETH_INT_GPIO_PORT, SPI_ETH_INT_GPIO_PIN) ? 1 : 0);
  u8_t eie = enc28j60Read(EIE);
  u8_t eir = enc28j60Read(EIR);
  print("  EPKTCNT:        %02x\n", enc28j60Read(EPKTCNT));
  print("  EIR:            %02x\n", eir);
  print("    PKTIF:         %i\n", (eir & EIR_PKTIF) ? 1 : 0);
  print("    DMAIF:         %i\n", (eir & EIR_DMAIF) ? 1 : 0);
  print("    LINKIF:        %i\n", (eir & EIR_LINKIF) ? 1 : 0);
  print("    TXIF:          %i\n", (eir & EIR_TXIF) ? 1 : 0);
  print("    WOLIF:         %i\n", (eir & EIR_WOLIF) ? 1 : 0);
  print("    TXERIF:        %i\n", (eir & EIR_TXERIF) ? 1 : 0);
  print("    RXERIF:        %i\n", (eir & EIR_RXERIF) ? 1 : 0);
  print("  EIE:            %02x\n", eie);
  print("    INTIE:         %i\n", (eie & EIE_INTIE) ? 1 : 0);
  print("    PKTIE:         %i\n", (eie & EIE_PKTIE) ? 1 : 0);
  print("    DMAIE:         %i\n", (eie & EIE_DMAIE) ? 1 : 0);
  print("    LINKIE:        %i\n", (eie & EIE_LINKIE) ? 1 : 0);
  print("    TXIE:          %i\n", (eie & EIE_TXIE) ? 1 : 0);
  print("    WOLIE:         %i\n", (eie & EIE_WOLIE) ? 1 : 0);
  print("    TXERIE:        %i\n", (eie & EIE_TXERIE) ? 1 : 0);
  print("    RXERIE:        %i\n", (eie & EIE_RXERIE) ? 1 : 0);
  print("  EIE & EIR:      %02x\n", eie & eir);
}
#endif
