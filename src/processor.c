/*
 * processor.c
 *
 *  Created on: Aug 1, 2012
 *      Author: petera
 */

#include "processor.h"
#include "system.h"

static void RCC_config() {
#ifdef CONFIG_UART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#ifdef CONFIG_UART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
#ifdef CONFIG_UART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#ifdef CONFIG_UART4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif

#ifdef CONFIG_LED
#ifdef HW_DEBUG
  RCC_APB2PeriphClockCmd(LED12_APBPeriph_GPIO, ENABLE);
  RCC_APB2PeriphClockCmd(LED34_APBPeriph_GPIO, ENABLE);
#endif
#endif

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
#ifdef CONFIG_CNC
  RCC_APB2PeriphClockCmd(CNC_APBPeriph_GPIO, ENABLE);
#endif
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* PCLK1 = HCLK/1 */
  RCC_PCLK1Config(RCC_HCLK_Div1);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

#ifdef CONFIG_SPI
  /* Enable SPI1_MASTER clock and GPIO clock for SPI1_MASTER */
  RCC_APB2PeriphClockCmd(SPI1_MASTER_GPIO_CLK | SPI1_MASTER_CLK, ENABLE);

  /* Enable SPI1_MASTER DMA clock */
  RCC_AHBPeriphClockCmd(SPI1_MASTER_DMA_CLK, ENABLE);

  /* Enable SPI2_MASTER clock and GPIO clock for SPI2_MASTER */
  RCC_APB2PeriphClockCmd(SPI2_MASTER_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(SPI2_MASTER_CLK, ENABLE);

  /* Enable SPI2_MASTER DMA clock */
  RCC_AHBPeriphClockCmd(SPI2_MASTER_DMA_CLK, ENABLE);

#ifdef CONFIG_ETHSPI
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif
#endif

#ifdef CONFIG_I2C
  RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(I2C1_GPIO_CLK, ENABLE);
#endif

#ifdef CONFIG_ADC
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
#endif

#ifdef CONFIG_USB_CDC
  RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif
}

static void NVIC_config(void)
{
  // Configure the NVIC Preemption Priority Bits
  // use 3 bits for preemption and 1 bit for  subgroup
  u8_t prioGrp = 8 - __NVIC_PRIO_BITS;
  // use 4 bits for preemption and 0 subgroups
  //u8_t prioGrp = 8 - __NVIC_PRIO_BITS - 1;
  NVIC_SetPriorityGrouping(prioGrp);


  // Config systick interrupt, highest
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));

  // Config pendsv interrupt, lowest
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(prioGrp, 7, 1));

  // Config & enable TIM2 interrupt
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(prioGrp, 0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  // Config & enable uarts interrupt
#ifdef CONFIG_UART1
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(prioGrp, 2, 0));
  NVIC_EnableIRQ(USART1_IRQn);
#endif
#ifdef CONFIG_UART2
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioGrp, 2, 0));
  NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef CONFIG_UART3
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(prioGrp, 2, 1));
  NVIC_EnableIRQ(USART3_IRQn);
#endif
#ifdef CONFIG_UART4
  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(prioGrp, 2, 1));
  NVIC_EnableIRQ(UART4_IRQn);
#endif

#ifdef CONFIG_SPI
  // Config & enable the SPI-DMA interrupt
  NVIC_SetPriority(SPI1_MASTER_Rx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 0));
  NVIC_EnableIRQ(SPI1_MASTER_Rx_IRQ_Channel);
  NVIC_SetPriority(SPI1_MASTER_Tx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 1));
  NVIC_EnableIRQ(SPI1_MASTER_Tx_IRQ_Channel);

  NVIC_SetPriority(SPI2_MASTER_Rx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 0));
  NVIC_EnableIRQ(SPI2_MASTER_Rx_IRQ_Channel);
  NVIC_SetPriority(SPI2_MASTER_Tx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 1));
  NVIC_EnableIRQ(SPI2_MASTER_Tx_IRQ_Channel);

#ifdef CONFIG_ETHSPI
  // Config & enable the enc28j60 rx interrupt
  NVIC_SetPriority(SPI_ETH_INT_EXTI_IRQn, NVIC_EncodePriority(prioGrp, 4, 0));
  NVIC_EnableIRQ(SPI_ETH_INT_EXTI_IRQn);
#endif
#endif

#ifdef CONFIG_I2C
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));
  NVIC_EnableIRQ(I2C1_ER_IRQn);
#endif

#if OS_DBG_MON
  // Config & enable the dump button interrupt
  NVIC_SetPriority(OS_DUMP_IRQ_EXTI_IRQn, NVIC_EncodePriority(prioGrp, 6, 0));
  NVIC_EnableIRQ(OS_DUMP_IRQ_EXTI_IRQn);
#endif

#ifdef CONFIG_USB_CDC
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(prioGrp, 3, 0));
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

  NVIC_SetPriority(USBWakeUp_IRQn, NVIC_EncodePriority(prioGrp, 2, 0));
  NVIC_EnableIRQ(USBWakeUp_IRQn);
#endif
}

static void UART1_config() {
#ifdef CONFIG_UART1
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = UART1_SPEED;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART1 interrupt */
  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
#endif
}

static void UART2_config() {
#ifdef CONFIG_UART2
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = UART2_SPEED;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART */
  USART_Init(USART2, &USART_InitStructure);

  /* Enable USART interrupt */
  USART_ITConfig(USART2, USART_IT_TC, DISABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Enable the USART */
  USART_Cmd(USART2, ENABLE);
#endif
}

static void UART3_config() {
#ifdef CONFIG_UART3
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART3_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART3_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = UART3_SPEED;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART */
  USART_Init(USART3, &USART_InitStructure);

  /* Enable USART interrupt */
  USART_ITConfig(USART3, USART_IT_TC, DISABLE);
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  /* Enable the USART */
  USART_Cmd(USART3, ENABLE);
#endif
}

static void UART4_config() {
#ifdef CONFIG_UART4
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = UART4_GPIO_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART4_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART4_GPIO_PORT, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = UART4_SPEED;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART */
  USART_Init(UART4, &USART_InitStructure);

  /* Enable USART interrupt */
  USART_ITConfig(UART4, USART_IT_TC, DISABLE);
  USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

  /* Enable the USART */
  USART_Cmd(UART4, ENABLE);
#endif
}

static void SPI_config() {
#ifdef CONFIG_SPI
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;

  // SPI1

  /* Configure SPI1_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI1_MASTER_PIN_SCK | SPI1_MASTER_PIN_MOSI | SPI1_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI1_MASTER_GPIO, &GPIO_InitStructure);

  /* Configure SPI1_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI_FLASH_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_FLASH_GPIO_PORT, &GPIO_InitStructure);

  /* SPI1_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI DMA rx */
  DMA_DeInit(SPI1_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI1_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI1_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI1_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI1_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL


  // SPI2

  /* Configure SPI2_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI2_MASTER_PIN_SCK | SPI2_MASTER_PIN_MOSI | SPI2_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI2_MASTER_GPIO, &GPIO_InitStructure);

  /* SPI2_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI2 DMA rx */
  DMA_DeInit(SPI2_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI2_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI2_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI2_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI2_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL

#endif // CONFIG_SPI
}

static void ETH_SPI_config() {
#ifdef CONFIG_ETHSPI
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SPI_ETH_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_ETH_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SPI_ETH_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPI_ETH_INT_GPIO_PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(SPI_ETH_INT_GPIO_PORT_SOURCE, SPI_ETH_INT_GPIO_PIN_SOURCE);

  EXTI_InitStructure.EXTI_Line = SPI_ETH_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

#endif
}

static void OS_DUMP_IRQ_config() {
#ifdef OS_DUMP_IRQ
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = OS_DUMP_IRQ_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(OS_DUMP_IRQ_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = OS_DUMP_IRQ_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(OS_DUMP_IRQ_GPIO_PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(OS_DUMP_IRQ_GPIO_PORT_SOURCE, OS_DUMP_IRQ_GPIO_PIN_SOURCE);

  EXTI_InitStructure.EXTI_Line = OS_DUMP_IRQ_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
#endif
}

static void BUZZER_config() {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void LED_config() {
#ifdef CONFIG_LED
  GPIO_InitTypeDef GPIO_InitStructure;
#ifdef HW_DEBUG

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LED1_GPIO;
  GPIO_Init(LED12_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = LED2_GPIO;
  GPIO_Init(LED12_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = LED3_GPIO;
  GPIO_Init(LED34_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = LED4_GPIO;
  GPIO_Init(LED34_GPIO_PORT, &GPIO_InitStructure);
#endif
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LED_SHIFT_CLK;
  GPIO_Init(LED_SHIFT_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = LED_SHIFT_DAT;
  GPIO_Init(LED_SHIFT_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = LED_SHIFT_STR;
  GPIO_Init(LED_SHIFT_PORT, &GPIO_InitStructure);
#endif
}

static void CNC_config() {
#ifdef CONFIG_CNC
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =
      CNC_GPIO_DIR_A | CNC_GPIO_STEP_A |
      CNC_GPIO_DIR_X | CNC_GPIO_STEP_X |
      CNC_GPIO_DIR_Y | CNC_GPIO_STEP_Y |
      CNC_GPIO_DIR_Z | CNC_GPIO_STEP_Z |
      CNC_GPIO_SENSE
      ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(CNC_GPIO_PORT, &GPIO_InitStructure);
#endif
}

static void TIM_config() {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  u16_t prescaler = 0;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/SYS_MAIN_TIMER_FREQ;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, prescaler, TIM_PSCReloadMode_Immediate);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

static void ADC_config() {
#ifdef CONFIG_ADC
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure C05 (ADC Channel15) as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
}

static void I2C_config() {
#ifdef CONFIG_I2C
  GPIO_InitTypeDef GPIO_InitStruct;


  GPIO_InitStruct.GPIO_Pin = I2C1_SCL_GPIO_PIN | I2C1_SDA_GPIO_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(I2C1_GPIO_PORT, &GPIO_InitStruct);
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

#endif
}

static void USB_CDC_config() {
#ifdef CONFIG_USB_CDC
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
#endif
}

// bootloader settings

static void SPI_config_bootloader() {
#ifdef CONFIG_SPI
  // Abort all DMA transfers
  /* Disable DMA SPI1 RX channel transfer complete interrupt */
  DMA_ITConfig(SPI1_MASTER_Rx_DMA_Channel, DMA_IT_TC, DISABLE);

  /* Disable SPI1_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI1_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , DISABLE);

  /* Disable DMA SPI2 RX channel transfer complete interrupt */
  DMA_ITConfig(SPI2_MASTER_Rx_DMA_Channel, DMA_IT_TC, DISABLE);

  /* Disable SPI1_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI2_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , DISABLE);
#endif // CONFIG_SPI
}


// ifc


void PROC_base_init() {
  RCC_config();
  NVIC_config();
}

void PROC_periph_init() {
  UART1_config();
  UART2_config();
  UART3_config();
  UART4_config();
  BUZZER_config();
  LED_config();
  SPI_config();
  ETH_SPI_config();
  TIM_config();
  CNC_config();
  ADC_config();
  I2C_config();
  USB_CDC_config();
  OS_DUMP_IRQ_config();
}

void PROC_periph_init_bootloader() {
  SPI_config_bootloader();
}
