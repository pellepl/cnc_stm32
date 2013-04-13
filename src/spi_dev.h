/*
 * spi_dev.h
 *
 *  Created on: Sep 21, 2012
 *      Author: petera
 */

#ifndef SPI_DEV_H_
#define SPI_DEV_H_

#include "spi_driver.h"
#include "taskq.h"

#define SPI_CONF_IRQ_DRIVEN         (1<<0)
#define SPI_CONF_IRQ_CALLBACK       (1<<1)

#define SPI_ERR_DEV_BUSY            -100


#define SPIDEV_CONFIG_CPOL_MASK     (1<<0)
#define SPIDEV_CONFIG_CPHA_MASK     (1<<1)
#define SPIDEV_CONFIG_FBIT_MASK     (1<<2)
#define SPIDEV_CONFIG_SPEED_MASK    ((0x7)<<3)

#define SPIDEV_CONFIG_CPOL_LO       (1<<0)
#define SPIDEV_CONFIG_CPOL_HI       (0<<0)
#define SPIDEV_CONFIG_CPHA_2E       (1<<1)
#define SPIDEV_CONFIG_CPHA_1E       (0<<1)
#define SPIDEV_CONFIG_FBIT_LSB      (1<<2)
#define SPIDEV_CONFIG_FBIT_MSB      (0<<2)
#define SPIDEV_CONFIG_SPEED_36M     ((0x0)<<3)
#define SPIDEV_CONFIG_SPEED_18M     ((0x1)<<3)
#define SPIDEV_CONFIG_SPEED_9M      ((0x2)<<3)
#define SPIDEV_CONFIG_SPEED_4_5M    ((0x3)<<3)
#define SPIDEV_CONFIG_SPEED_2_3M    ((0x4)<<3)
#define SPIDEV_CONFIG_SPEED_1_1M    ((0x5)<<3)
#define SPIDEV_CONFIG_SPEED_562_5K  ((0x6)<<3)
#define SPIDEV_CONFIG_SPEED_SLOWEST  ((0x7)<<3)


typedef struct  {
  u8_t *tx;
  u8_t cs_release : 1;
  u16_t tx_len : 15;
  u8_t *rx;
  u16_t rx_len;
} spi_dev_sequence;

/*
 * SPI device
 */
typedef struct spi_dev_s {
  u16_t configuration;
  spi_bus *bus;
  hw_io_port cs_port;
  hw_io_pin cs_pin;
  bool opened;
  u8_t irq_conf;
  spi_dev_sequence cur_seq;
  spi_dev_sequence *seq_list;
  u8_t seq_len;
  void (*spi_dev_callback)(struct spi_dev_s *s, int result);
} spi_dev;

typedef void (*spi_dev_callback)(spi_dev *dev, int result);

void SPI_DEV_init(spi_dev *dev, u16_t configuration, spi_bus *bus,
    hw_io_port cs_port, hw_io_pin cs_pin, u8_t irq_conf);

void SPI_DEV_open(spi_dev *dev);

int SPI_DEV_set_callback(spi_dev *dev, spi_dev_callback cb);

int SPI_DEV_sequence(spi_dev *dev, spi_dev_sequence *seq, u8_t seq_len);

int SPI_DEV_txrx(spi_dev *dev, u8_t *tx, u16_t tx_len, u8_t *rx, u16_t rx_len);

void SPI_DEV_close(spi_dev *dev);

bool SPI_DEV_is_busy(spi_dev *dev);


#endif /* SPI_DEV_H_ */
