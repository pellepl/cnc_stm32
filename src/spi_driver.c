/*
 * spi_driver.c
 * Handles low level parts of spi bus
 *
 *  Created on: Aug 15, 2012
 *      Author: petera
 */

#include "spi_driver.h"
#include "spi_dev.h"
#include "miniutils.h"
#ifdef CONFIG_SPI

/*
 * Regarding TX:
 * To avoid busy-waiting on TX flag after DMA TX finish interrupt, we
 * always also receive the same amount of data. This will give us a convenient
 * DMA RX finish interrupt when all data is shifted out (as same amount
 * has been shifted in). The received data is put on the same address as
 * transmitted data, but transmitted data is put into shift register
 * before received data is put on the same address. Some energy is
 * wasted also but for the sake of simplicity...
 */

spi_bus __spi_bus_vec[SPI_MAX_ID];

// Finalizes hw blocks of a spi operation
static void SPI_finalize(spi_bus *s, char keep_alive) {
#ifndef CONFIG_SPI_POLL
  DMA_Cmd(s->dma_tx_channel, DISABLE);
  DMA_Cmd(s->dma_rx_channel, DISABLE);
  if (!keep_alive) {
    ////DBG(D_SPI, D_FATAL, "SPI end await rxne\n");
    ////while (SPI_I2S_GetITStatus(s->hw, SPI_I2S_IT_RXNE) != SET);
    ////DBG(D_SPI, D_FATAL, "SPI end await rxe\n");
    ////while (SPI_I2S_GetITStatus(s->hw, SPI_I2S_IT_TXE) != SET);
    //SPI_Cmd(s->hw, DISABLE);
  }
#endif
}

// IRQ: Finishes off an rx/tx operation, copies data and callbacks
static void SPI_finish(spi_bus *s) {
  if (s->busy) { // check should not be needed, but..
    if (s->rx_buf) {
      memcpy(s->rx_buf, s->buf, s->rx_len);
    }
    SPI_finalize(s, s->keep_alive);
    s->busy = FALSE;
    if (s->spi_bus_callback) {
      s->spi_bus_callback(s);
    }
  }
}

// Initiates a spi rx/tx opertaion
static void SPI_begin(spi_bus *s, u16_t tx_len, u16_t rx_len, u8_t *actual_target) {
#ifdef CONFIG_SPI_POLL
  u16_t tx_ix = 0;
  u16_t rx_ix = 0;
  if (actual_target == NULL) {
    rx_len = 0;
  }
  // clear rx buffer would we have txed something
  (void)SPI_I2S_ReceiveData(s->hw);

  while (tx_ix < tx_len || rx_ix < rx_len) {
    bool clock_data = TRUE;
    if (tx_ix < tx_len) {
      while (SPI_I2S_GetFlagStatus(s->hw, SPI_I2S_FLAG_TXE) == RESET);
      //print("T%02x ", s->buf[tx_ix]);
      SPI_I2S_SendData(s->hw, s->buf[tx_ix++]);
      clock_data = FALSE;
    }
    if (rx_ix < rx_len) {
      if (clock_data) {
        while (SPI_I2S_GetFlagStatus(s->hw, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(s->hw, 0xff);
      }
      while (SPI_I2S_GetFlagStatus(s->hw, SPI_I2S_FLAG_RXNE) == RESET);
      if (actual_target) {
        u8_t r = SPI_I2S_ReceiveData(s->hw);;
        actual_target[rx_ix++] = r;
        //print("R%02x ", r);
      } else {
        rx_ix++;
      }
    }
  }
  // wait until last tx char is entered into shift buffer
  while (SPI_I2S_GetFlagStatus(s->hw, SPI_I2S_FLAG_TXE) == RESET);
  // wait until last tx char is shifted out from buffer
  while (SPI_I2S_GetFlagStatus(s->hw, SPI_I2S_FLAG_RXNE) == SET) {
    (void)SPI_I2S_ReceiveData(s->hw);
  }
  DBG(D_SPI, D_DEBUG, " -- SPI tx:%04x rx:%04x\n", tx_ix, rx_ix);
  SPI_finish(s);
#else
  // if no actual target: put rxed data into temp buffer, overwriting tx data
  // but this is put into shift register before overwrite
  s->dma_rx_channel->CMAR = (u32_t)(actual_target == 0 ? s->buf : actual_target);
  s->dma_rx_channel->CNDTR = rx_len;
  s->dma_tx_channel->CMAR = (u32_t)s->buf;
  s->dma_tx_channel->CNDTR = tx_len == 0 ? rx_len : tx_len;

#ifndef CONFIG_SPI_CHUNK_RX
  if (tx_len == 0) {
    // only rx, so tx same byte
    s->buf[0] = 0xff;
    s->dma_tx_channel->CCR &= ~DMA_MemoryInc_Enable;
  } else {
    // (rx)/tx, so tx buffer
    s->dma_tx_channel->CCR |= DMA_MemoryInc_Enable;
  }
#endif

  DMA_Cmd(s->dma_rx_channel, ENABLE);
  DMA_Cmd(s->dma_tx_channel, ENABLE);
  //SPI_Cmd(s->hw, ENABLE);
#endif // CONFIG_SPI_POLL
}

int SPI_close(spi_bus *s) {
  SPI_finalize(s, FALSE);
  SPI_Cmd(s->hw, DISABLE);
  s->user_p = 0;
  s->user_arg = 0;
  if (s->busy) {
    s->busy = FALSE;
    return SPI_ERR_BUS_BUSY;
  }
  return SPI_OK;
}

int SPI_config(spi_bus *s, u16_t config) {
  SPI_InitTypeDef SPI_InitStructure;
  (void)SPI_close(s);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL =
      (config & SPIDEV_CONFIG_CPOL_MASK) == SPIDEV_CONFIG_CPOL_HI ? SPI_CPOL_High : SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA =
      (config & SPIDEV_CONFIG_CPHA_MASK) == SPIDEV_CONFIG_CPHA_1E ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_FirstBit =
      (config & SPIDEV_CONFIG_FBIT_MASK) == SPIDEV_CONFIG_FBIT_MSB ? SPI_FirstBit_MSB : SPI_FirstBit_LSB;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  switch (config & SPIDEV_CONFIG_SPEED_MASK) {
  case SPIDEV_CONFIG_SPEED_36M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    break;
  case SPIDEV_CONFIG_SPEED_18M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    break;
  case SPIDEV_CONFIG_SPEED_9M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    break;
  case SPIDEV_CONFIG_SPEED_4_5M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    break;
  case SPIDEV_CONFIG_SPEED_2_3M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    break;
  case SPIDEV_CONFIG_SPEED_1_1M:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    break;
  case SPIDEV_CONFIG_SPEED_562_5K:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    break;
  case SPIDEV_CONFIG_SPEED_SLOWEST:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    break;
  }
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(s->hw, &SPI_InitStructure);
  SPI_Cmd(s->hw, ENABLE);

  return SPI_OK;
}

int SPI_tx(spi_bus *s, u8_t *tx, u16_t len, bool keep_alive) {
  if (s->busy) {
    return SPI_ERR_BUS_BUSY;
  }
  s->busy = TRUE;
  if (len > s->max_buf_len) {
    s->busy = FALSE;
    return SPI_ERR_BUS_LEN_EXCEEDED;
  }
  // get tx data into temp buffer
  // receive phony data into temp buffer
  memcpy(s->buf, tx, len);
  s->keep_alive = keep_alive;
  s->rx_buf = 0; // no memcpy at DMA irq
  s->rx_len = 0;
  SPI_begin(s, len, len, 0);
  return SPI_OK;
}

int SPI_rx(spi_bus *s, u8_t *rx, u16_t len, bool keep_alive)  {
  if (s->busy) {
    return SPI_ERR_BUS_BUSY;
  }
  s->busy = TRUE;
#ifdef CONFIG_SPI_CHUNK_RX
  if (len > s->max_buf_len) {
    s->busy = FALSE;
    return SPI_ERR_BUS_LEN_EXCEEDED;
  }
#endif

  // read phony tx data from temp buffer
  // write rx data directly to destination, no memcpy afterwards
  s->keep_alive = keep_alive;
  s->rx_buf = 0; // no memcpy at DMA irq
  s->rx_len = 0;
  SPI_begin(s, 0, len, rx);
  return SPI_OK;
}

int SPI_rxtx(spi_bus *s, u8_t *tx, u16_t tx_len, u8_t *rx, u16_t rx_len, bool keep_alive) {
  if (s->busy) {
    return SPI_ERR_BUS_BUSY;
  }
  s->busy = TRUE;
  u16_t maxlen = MAX(tx_len, rx_len);
  if (maxlen > s->max_buf_len) {
    s->busy = FALSE;
    return SPI_ERR_BUS_LEN_EXCEEDED;
  }

  // get tx data into temp buffer
  // rec rx data into temp buffer, memcpy after DMA is finished to actual
  // destination
  // this is due to tx_len might be bigger than rx len so DMA may not write
  // directly to rx buf
  memcpy(s->buf, tx, tx_len);
  s->keep_alive = keep_alive;
  s->rx_buf = rx; // memcpy at DMA irq
  s->rx_len = rx_len;
  SPI_begin(s, maxlen, maxlen, 0);
  return SPI_OK;
}

int SPI_set_callback(spi_bus *spi, void (*spi_bus_callback)(spi_bus *s)) {
  if (spi->busy) {
    return SPI_ERR_BUS_BUSY;
  }
  spi->spi_bus_callback = spi_bus_callback;
  return SPI_OK;
}

void SPI_init() {
  // setup spi bus descriptor
  memset(__spi_bus_vec, 0, sizeof(__uart_vec));

  _SPI_BUS(0)->max_buf_len = SPI_BUFFER;
  _SPI_BUS(0)->hw = SPI_MASTER;
  _SPI_BUS(0)->dma_rx_irq = DMA1_IT_TC2;
  _SPI_BUS(0)->dma_tx_irq = DMA1_IT_TC3;
  _SPI_BUS(0)->dma_rx_channel = SPI_MASTER_Rx_DMA_Channel;
  _SPI_BUS(0)->dma_tx_channel = SPI_MASTER_Tx_DMA_Channel;
  _SPI_BUS(0)->nvic_irq = SPI_MASTER_Rx_IRQ_Channel;
}

bool SPI_is_busy(spi_bus *spi) {
  return spi->busy;
}

void SPI_register(spi_bus *spi) {
  spi->attached_devices++;
}

void SPI_release(spi_bus *spi) {
  spi->attached_devices--;
  if (spi->attached_devices == 0) {
    SPI_close(spi);
  }
}

void SPI_enable_irq(spi_bus *spi) {
  NVIC->ISER[spi->nvic_irq >> 0x05] =
    (u32_t)0x01 << (spi->nvic_irq  & (u8_t)0x1F);
}

void SPI_disable_irq(spi_bus *spi) {
  NVIC->ICER[spi->nvic_irq >> 0x05] =
    (u32_t)0x01 << (spi->nvic_irq & (u8_t)0x1F);
}

void SPI_irq(spi_bus *s) {
#if 0
  DBG(D_SPI, D_FATAL, "SPI DMA IRQ\n");
  if (DMA_GetITStatus(DMA1_IT_TE2))
  {
    DBG(D_SPI, D_FATAL, "SPI DMA IRQ IT TE2\n");
    DMA_ClearITPendingBit(s->dma_tx_irq);
  }
  if (DMA_GetITStatus(DMA1_IT_TE3))
  {
    DBG(D_SPI, D_FATAL, "SPI DMA IRQ IT TE3\n");
    DMA_ClearITPendingBit(s->dma_tx_irq);
  }

  if (DMA_GetITStatus(s->dma_tx_irq))
  {
    DBG(D_SPI, D_FATAL, "SPI DMA IRQ IT TC3 TX\n");
    DMA_ClearITPendingBit(s->dma_tx_irq);
  }
#endif

  if (DMA_GetITStatus(s->dma_rx_irq)) {
    // RX
    DMA_ClearITPendingBit(s->dma_rx_irq);
    SPI_finish(s);
  }
}

#endif
