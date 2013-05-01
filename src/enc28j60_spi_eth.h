/*
 * enc28j60_spi_eth.h
 *
 *  Created on: Mar 31, 2013
 *      Author: petera
 */

#ifndef ENC28J60_SPI_ETH_H_
#define ENC28J60_SPI_ETH_H_

#include "system.h"

void ETH_SPI_init();
void ETH_SPI_start();
void ETH_SPI_dhcp();
void ETH_SPI_stop();
void ETH_SPI_irq();
void ETH_SPI_dump();

#endif /* ENC28J60_SPI_ETH_H_ */
