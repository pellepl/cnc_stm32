/*
 * enc28j60_spi_eth.h
 *
 *  Created on: Mar 31, 2013
 *      Author: petera
 */

#ifndef ENC28J60_SPI_ETH_H_
#define ENC28J60_SPI_ETH_H_

#include "system.h"

typedef enum {
  ETH_DOWN = 0,
  ETH_QUERY_IP,
  ETH_UP,
} eth_state;

void ETH_SPI_init();
void ETH_SPI_start();
void ETH_SPI_dhcp();
bool ETH_SPI_send(u8_t *data, u16_t len, time timeout);
bool ETH_SPI_read(u8_t *data, u16_t *len, time timeout);
bool ETH_SPI_tx_free();
int ETH_SPI_available();
eth_state ETH_SPI_state();
void ETH_SPI_stop();
void ETH_SPI_irq();
void ETH_SPI_dump();

#endif /* ENC28J60_SPI_ETH_H_ */
