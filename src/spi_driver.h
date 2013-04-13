/*
 * spi_driver.h
 *
 *  Created on: Aug 15, 2012
 *      Author: petera
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include "stm32f10x.h"
#include "system.h"

#define SPI_BUFFER    256

#define SPI_OK                    0
#define SPI_ERR_BUS_BUSY          -1
#define SPI_ERR_BUS_LEN_EXCEEDED  -2

/*
 * SPI bus
 */
typedef struct spi_bus_s {
  SPI_TypeDef *hw;
  u32_t dma_rx_irq;
  u32_t dma_tx_irq;
  DMA_Channel_TypeDef *dma_rx_channel;
  DMA_Channel_TypeDef *dma_tx_channel;
  u8_t nvic_irq;
  u8_t attached_devices;
  volatile bool busy;
  bool keep_alive;
  u8_t buf[SPI_BUFFER];
  u16_t max_buf_len;
  u8_t *rx_buf;
  u16_t rx_len;
  void (*spi_bus_callback)(struct spi_bus_s *s);
  void *user_p;
  volatile u32_t user_arg;
} spi_bus;

#define SPI_MAX_ID   1

extern spi_bus __spi_bus_vec[SPI_MAX_ID];

#define _SPI_BUS(x) (&__spi_bus_vec[(x)])

/* Receives data from spi bus. The length here is up to the user.
 * @param keep_alive wether bus should be closed after completed
 *        operation or not
 * @return SPI_OK or error
 */
int SPI_rx(spi_bus *s, u8_t *rx, u16_t rx_len, bool keep_alive);

/* Sends data on spi bus
 * @param tx_len length to tx, must not exceed internal buffer length of driver
 * @param keep_alive wether bus should be closed after completed
 *        operation or not
 * @return SPI_OK or error
 */
int SPI_tx(spi_bus *s, u8_t *tx, u16_t tx_len, bool keep_alive);

/* Sends and receives data on spi bus, full duplex
 * @param tx_len length to tx, must not exceed internal buffer length of driver
 * @param rx_len length to rx, must not exceed internal buffer length of driver
 * @param keep_alive wether bus should be closed after completed
 *        operation or not
 * @return SPI_OK or error
 */
int SPI_rxtx(spi_bus *s, u8_t *tx, u16_t tx_len, u8_t *rx, u16_t rx_len, bool keep_alive);

/* Closes indefinitely.
 * @return SPI_ERR_BUSY if bus was busy, but still it is closed.
 */
int SPI_close(spi_bus *spi);

/* Sets a callback which is invoked upon following rx/tx completions.
 * Called directly after bus is freed and finalized in irq context.
 * @return SPI_OK or SPI_ERR_BUSY if bus is currently busy
 */
int SPI_set_callback(spi_bus *spi, void (*spi_bus_callback)(spi_bus *s));

int SPI_config(spi_bus *s, u16_t config);

bool SPI_is_busy(spi_bus *spi);


void SPI_enable_irq(spi_bus *spi);
void SPI_disable_irq(spi_bus *spi);


void SPI_register(spi_bus *spi);
void SPI_release(spi_bus *spi);

void SPI_irq(spi_bus *spi);

void SPI_enter_critical();
void SPI_exit_critical();

void SPI_init();

#endif /* SPI_DRIVER_H_ */
