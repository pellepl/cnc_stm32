#include "enc28j60_spi_eth.h"
#include "stm32f10x.h"
#include "os.h"
#include "enc28j60.h"
#include "ip_arp_udp.h"
#include "net.h"
#include "spi_dev_os_generic.h"
#include "miniutils.h"
#include "heap.h"

#ifdef CONFIG_ETHSPI
u8_t mac[6] = {0xcc,0xaa,0xff,0xee,0x00,0x11};
u8_t ip[4] = {192,168,0,150};

spi_dev_gen _enc28j60_spi_dev;

static struct {
  volatile bool irq_pending;
  volatile bool active;

  os_cond irq_cond;
  os_mutex irq_mutex;
  os_cond tx_cond;
  os_mutex tx_mutex;
  u8_t rxbuf[512];
  u8_t txbuf[512];
  volatile void *rx_stack;
  os_thread irq_thread;

  bool rx_pkt_available;
  bool tx_pkt_free;
  bool rx_error;
  bool tx_error;
} ethspi;

static void _eth_spi_handle_pkt() {
  u16_t rx_stat;
  int plen = enc28j60PacketReceive(500, ethspi.rxbuf, &rx_stat);
  if (plen == 0) {
    DBG(D_ETH, D_DEBUG, "ethspi no packet, rx_stat:%04x\n", rx_stat);
    return;
  }
  DBG(D_ETH, D_DEBUG, "ethspi got packet, len %i, rx_stat:%04x\n", plen, rx_stat);
  //printbuf(ethspi.rxbuf, MIN(64, plen));

  // arp is broadcast if unknown but a host may also
  // verify the mac address by sending it to
  // a unicast address.
  if (eth_type_is_arp_and_my_ip(ethspi.rxbuf, plen)) {
    memcpy(ethspi.txbuf, ethspi.rxbuf, plen);
    make_arp_answer_from_request(ethspi.txbuf, plen);
    return;
  }

  // check if ip packets (icmp or udp) are for us:
  if (eth_type_is_ip_and_my_ip(ethspi.rxbuf, plen) == 0) {
    return;
  }

  if (ethspi.rxbuf[IP_PROTO_P]==IP_PROTO_ICMP_V &&
      ethspi.rxbuf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
    // a ping packet, let's send pong
    memcpy(ethspi.txbuf, ethspi.rxbuf, plen);
    make_echo_reply_from_request(ethspi.txbuf, plen);
    return;
  }

  // we listen on port 1200=0x4B0
  if (ethspi.rxbuf[IP_PROTO_P] == IP_PROTO_UDP_V
      /*&& ethspi.rxbuf[UDP_DST_PORT_H_P] ==4  && ethspi.rxbuf[UDP_DST_PORT_L_P] == 0xb0*/) {
    int payloadlen = /*ethspi.rxbuf[UDP_LEN_L_P]*/plen - 34 - UDP_HEADER_LEN;
    DBG(D_ETH, D_DEBUG, "ethspi UDP len:%i\n", payloadlen);
    IF_DBG(D_ETH, D_DEBUG) {
      printbuf(&ethspi.rxbuf[UDP_DATA_P], MIN(128, payloadlen));
    }
    //char *nisse = "hello wurlde";
    //make_udp_reply_from_request(ethbuf, nisse, strlen(nisse), 1200);

    return;
  }
}

void *ETH_SPI_irq_thread_handler(void *a) {
  DBG(D_ETH, D_INFO, "ethspi irq thread started\n");

  while (ethspi.active) {
    // await interrupt
    if (!enc28j60hasRxPkt()) {
      OS_mutex_lock(&ethspi.irq_mutex);
      while (ethspi.active && !ethspi.irq_pending) {
        OS_cond_wait(&ethspi.irq_cond, &ethspi.irq_mutex);
      }
      ethspi.irq_pending = FALSE;
      OS_mutex_unlock(&ethspi.irq_mutex);
    }

    asm volatile ("nop\n"); // compile barrier

    // disable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIE, EIE_INTIE);

    // check & clear interrupts
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
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
        ethspi.tx_error = TRUE;
      }
      if (eir & EIR_TXIF) {
        DBG(D_ETH, D_DEBUG, "ethspi TX interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXIF);
        ethspi.tx_pkt_free = TRUE;
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
        // handle directly
        _eth_spi_handle_pkt();
      }
    }

    // enable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE);

    if (ethspi.rx_error) {
      ethspi.rx_error = FALSE;
    }
    if (ethspi.tx_error) {
      ethspi.tx_error = FALSE;
    }
    if (ethspi.tx_pkt_free) {
      OS_mutex_lock(&ethspi.tx_mutex);
      OS_cond_signal(&ethspi.tx_cond);
      OS_mutex_unlock(&ethspi.tx_mutex);
    }
  } // active
  DBG(D_ETH, D_INFO, "ethspi thread ended\n");
  HEAP_free((void *)ethspi.rx_stack);
  ethspi.rx_stack = NULL;
  return NULL;
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
      ETH_SPI_irq_thread_handler,
      NULL,
      (void *)ethspi.rx_stack, 0x400,
      "ethspi");
}

void ETH_SPI_stop() {
  OS_mutex_lock(&ethspi.irq_mutex);
  ethspi.active = FALSE;
  OS_cond_signal(&ethspi.irq_cond);
  OS_mutex_unlock(&ethspi.irq_mutex);
}

void ETH_SPI_init() {
  memset(&ethspi, 0, sizeof(ethspi));

  ethspi.tx_pkt_free = TRUE;
  OS_mutex_init(&ethspi.irq_mutex, OS_MUTEX_ATTR_CRITICAL_IRQ);
  OS_cond_init(&ethspi.irq_cond);
  OS_mutex_init(&ethspi.tx_mutex, 0);
  OS_cond_init(&ethspi.tx_cond);

  DBG(D_ETH, D_DEBUG, "ethspi spigen init\n");
  SPI_DEV_GEN_init(
      &_enc28j60_spi_dev,
      SPIDEV_CONFIG_CPHA_1E |
      SPIDEV_CONFIG_CPOL_LO |
      SPIDEV_CONFIG_FBIT_MSB |
      SPIDEV_CONFIG_SPEED_9M,
      _SPI_BUS(0),
      SPI_ETH_GPIO_PORT, SPI_ETH_GPIO_PIN);
  DBG(D_ETH, D_DEBUG, "ethspi spigen open\n");
  SPI_DEV_GEN_open(&_enc28j60_spi_dev);
  DBG(D_ETH, D_DEBUG, "ethspi enc28j60 init\n");
  enc28j60Init(mac, EIE_PKTIE | EIE_TXIE | EIE_RXERIE | EIE_TXERIE);

  DBG(D_ETH, D_DEBUG, "ethspi enc28j60 init done, chip rev %02x\n", enc28j60getrev());
  DBG(D_ETH, D_DEBUG, "ethspi mac readback: %02x:%02x:%02x:%02x:%02x:%02x\n",
      enc28j60Read(MAADR5), enc28j60Read(MAADR4), enc28j60Read(MAADR3),
      enc28j60Read(MAADR2), enc28j60Read(MAADR1), enc28j60Read(MAADR0)
      );

  SYS_hardsleep_ms(20);
  enc28j60PhyWrite(PHLCON,0x476);
  SYS_hardsleep_ms(20);
  init_ip_arp_udp(mac, ip);
  DBG(D_ETH, D_INFO, "ethspi setup finished, ip %i.%i.%i.%i @ mac %02x.%02x.%02x.%02x.%02x.%02x\n",
      ip[0], ip[1], ip[2], ip[3], mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void ETH_SPI_dump() {
  print("ETH SPI\n-------\n");
  print("State\n");
  print("  active:   %i\n", ethspi.active);
  print("  irq flag: %i\n", ethspi.irq_pending);
#if OS_DBG_MON
  print("OS\n");
  OS_DBG_print_thread(&ethspi.irq_thread, TRUE, 2);
  OS_DBG_print_mutex(&ethspi.irq_mutex, TRUE, 2);
  OS_DBG_print_cond(&ethspi.irq_cond, TRUE, 2);
  OS_DBG_print_mutex(&ethspi.tx_mutex, TRUE, 2);
  OS_DBG_print_cond(&ethspi.tx_cond, TRUE, 2);
#endif
  print("HW\n");
  print("  irq pin:        %i\n", GPIO_read(SPI_ETH_INT_GPIO_PORT, SPI_ETH_INT_GPIO_PIN) ? 1 : 0);
  print("  chip rev:       %02x\n", enc28j60getrev());
  print("  mac readback:   %02x:%02x:%02x:%02x:%02x:%02x\n",
      enc28j60Read(MAADR5), enc28j60Read(MAADR4), enc28j60Read(MAADR3),
      enc28j60Read(MAADR2), enc28j60Read(MAADR1), enc28j60Read(MAADR0)
  );
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
