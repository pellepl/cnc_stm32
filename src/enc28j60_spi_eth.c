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
  volatile bool rx_packet;
  os_cond rx_cond;
  os_mutex rx_mutex;
  u8_t rxbuf[512];
  u8_t txbuf[512];
  volatile bool active;
  volatile void *stack;
  os_thread thread;
} ethspi;

static void printbuf(u8_t *buf, u16_t len) {
  int i = 0, ix = 0;
  while (i < len) {
    for (i = ix; i < MIN(ix+32, len); i++) {
      print("%02x ", buf[i]);
    }
    print ("  ");
    for (i = ix; i < MIN(ix+32, len); i++) {
      print("%c", buf[i] < 32 ? '.' : buf[i]);
    }
    ix += 32;
    print("\n");
  }
}

static void __eth_spi_handle_pkt() {
  // decrement the packet counter indicate we are done with this packet
  //enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
  int plen = enc28j60PacketReceive(250, ethspi.rxbuf);
  if (plen == 0) {
    print("ETH: no packet\n");
    return;
  }
  print("ETH: got packet, len %i\n", plen);
  printbuf(ethspi.rxbuf, plen);

  // arp is broadcast if unknown but a host may also
  // verify the mac address by sending it to
  // a unicast address.
  if (eth_type_is_arp_and_my_ip(ethspi.rxbuf, plen)) {
    // TODO PETER
    memcpy(ethspi.txbuf, ethspi.rxbuf, plen);
    make_arp_answer_from_request(ethspi.txbuf, plen);
    return;
  }
  return;
  // check if ip packets (icmp or udp) are for us:
  if (eth_type_is_ip_and_my_ip(ethspi.rxbuf,plen)==0){
    return;
  }

  if (ethspi.rxbuf[IP_PROTO_P]==IP_PROTO_ICMP_V &&
      ethspi.rxbuf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
    // a ping packet, let's send pong
    // TODO PETER
    memcpy(ethspi.txbuf, ethspi.rxbuf, plen);
    make_echo_reply_from_request(ethspi.txbuf, plen);
    print("ETH: pong\n");
    return;
  }

  // we listen on port 1200=0x4B0
  if (ethspi.rxbuf[IP_PROTO_P] == IP_PROTO_UDP_V &&
      ethspi.rxbuf[UDP_DST_PORT_H_P] ==4  && ethspi.rxbuf[UDP_DST_PORT_L_P] == 0xb0) {
    int payloadlen = ethspi.rxbuf[UDP_LEN_L_P] - UDP_HEADER_LEN;
    print("ETH: UDP len:%i\n", payloadlen);
    printbuf(&ethspi.rxbuf[UDP_DATA_P], payloadlen);
    //char *nisse = "hello wurlde";
    //make_udp_reply_from_request(ethbuf, nisse, strlen(nisse), 1200);

    return;
  }
}

void *ETH_SPI_thread_func(void *a) {
  DBG(D_ETH, D_INFO, "ethspi thread started\n");
  while (ethspi.active) {
    if (enc28j60hasRxPkt()) {
      ethspi.rx_packet = TRUE;
    } else {
      OS_mutex_lock(&ethspi.rx_mutex);
      DBG(D_ETH, D_DEBUG, "ethspi await rx\n");
//      while (ethspi.active && !ethspi.rx_packet) {
        OS_cond_wait(&ethspi.rx_cond, &ethspi.rx_mutex);
//      }
      ethspi.rx_packet = FALSE;
      OS_mutex_unlock(&ethspi.rx_mutex);
      DBG(D_ETH, D_DEBUG, "ethspi release rx\n");
    }

    // disable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIE, EIE_INTIE);

    u8_t eir = 0;
    while (ethspi.active &&
        ((eir = enc28j60Read(EIR)) &
            (EIR_PKTIF | EIR_RXERIF | EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF))
//|| TRUE
            ) {
      if (eir & EIR_RXERIF) {
        DBG(D_ETH, D_DEBUG, "ethspi clear RXERR interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_RXERIF);
      }
      if (eir & EIR_TXERIF) {
        DBG(D_ETH, D_DEBUG, "ethspi clear TXERR interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
      }
      if (eir & EIR_TXIF) {
        DBG(D_ETH, D_DEBUG, "ethspi clear TX interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXIF);
      }
      if (eir & EIR_DMAIF) {
        DBG(D_ETH, D_DEBUG, "ethspi clear DMA interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_DMAIF);
      }
      if (eir & EIR_LINKIF) {
        DBG(D_ETH, D_DEBUG, "ethspi clear LINK interrupt\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_LINKIF);
      }
      //if (eir & EIR_PKTIF) { // not reliable
      if (enc28j60hasRxPkt()) {
        DBG(D_ETH, D_DEBUG, "ethspi packet\n");
        __eth_spi_handle_pkt();
      }
    }

    // disable eth interrupts
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE);

  } // active
  DBG(D_ETH, D_INFO, "ethspi thread ended\n");
  HEAP_free((void *)ethspi.stack);
  ethspi.stack = NULL;
  return NULL;
}

void ETH_SPI_irq() {
  if(EXTI_GetITStatus(SPI_ETH_INT_EXTI_LINE) != RESET) {
    EXTI_ClearITPendingBit(SPI_ETH_INT_EXTI_LINE);
    //DBG(D_ETH, D_DEBUG, "ethspi irq\n");
    if (ethspi.active) {
      ethspi.rx_packet = TRUE;
      OS_cond_signal(&ethspi.rx_cond);
    }
  }
}

void ETH_SPI_start() {
  if (ethspi.stack) {
    DBG(D_ETH, D_WARN, "ethspi thread already running\n");
    return;
  }
  ethspi.active = TRUE;
  ethspi.stack = HEAP_malloc(0x404);
  OS_thread_create(
      &ethspi.thread,
      0,
      ETH_SPI_thread_func,
      NULL,
      (void *)ethspi.stack, 0x400,
      "ethspi");
}

void ETH_SPI_stop() {
  OS_mutex_lock(&ethspi.rx_mutex);
  ethspi.active = FALSE;
  OS_cond_signal(&ethspi.rx_cond);
  OS_mutex_unlock(&ethspi.rx_mutex);
}

void ETH_SPI_init() {
  memset(&ethspi, 0, sizeof(ethspi));
  OS_mutex_init(&ethspi.rx_mutex, OS_MUTEX_ATTR_CRITICAL_IRQ);
  OS_cond_init(&ethspi.rx_cond);

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
  enc28j60Init(mac);

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
  print("  active:         %i\n", ethspi.active);
  print("  rx_packet flag: %i\n", ethspi.rx_packet);
#if OS_DBG_MON
  print("OS\n");
  OS_DBG_print_thread(&ethspi.thread, TRUE, 2);
  OS_DBG_print_mutex(&ethspi.rx_mutex, TRUE, 2);
  OS_DBG_print_cond(&ethspi.rx_cond, TRUE, 2);
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
