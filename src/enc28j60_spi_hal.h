/*
 * enc28j60_spi_hal.h
 *
 *  Created on: Mar 27, 2013
 *      Author: petera
 */

#ifndef ENC28J60_SPI_HAL_H_
#define ENC28J60_SPI_HAL_H_

#include "enc28j60_spi_eth.h"
#include "spi_dev_os_generic.h"
#include "system.h"

extern spi_dev_gen _enc28j60_spi_dev;

#define ENC28J60_HAL_TXRX(tx, tx_len, rx, rx_len) \
  SPI_DEV_GEN_txrx(&_enc28j60_spi_dev, (u8_t *)(tx), (u16_t)(tx_len), (u8_t *)(rx), (u16_t)(rx_len));

#define ENC28J60_HAL_SEQ(seq, len) \
  SPI_DEV_GEN_sequence(&_enc28j60_spi_dev, (spi_dev_sequence *)(seq), (u8_t)(len));

#define ENC28J60_IFC_TX_ETH_FRAME(data, len) \
  ETH_SPI_send((data), (len), 0)

// TODO
#define ENC28J60_HAL_WAIT_US(t) \
    SYS_hardsleep_ms(1);

// TODO
#define ENC28J60_HAL_WAIT_MS(t) \
    SYS_hardsleep_ms((t));


#endif /* ENC28J60_SPI_HAL_H_ */
