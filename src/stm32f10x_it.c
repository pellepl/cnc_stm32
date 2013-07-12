#include "stm32f10x_it.h"
#include "uart.h"
#include "spi_driver.h"
#include "timer.h"
#include "os.h"
#include "enc28j60_spi_eth.h"
#include "i2c_driver.h"

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
#if USER_HARDFAULT == 0
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#endif

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

#ifdef CONFIG_UART1
void USART1_IRQHandler(void)
{
  TRACE_IRQ_ENTER(1);
  UART_irq(&__uart_vec[0]);
  TRACE_IRQ_EXIT(1);
}
#endif

#ifdef CONFIG_UART2
void USART2_IRQHandler(void)
{
  //TRACE_IRQ_ENTER(2);
  UART_irq(&__uart_vec[1]);
  //TRACE_IRQ_EXIT(2);
}
#endif

#ifdef CONFIG_UART3
void USART3_IRQHandler(void)
{
  TRACE_IRQ_ENTER(3);
  UART_irq(&__uart_vec[2]);
  TRACE_IRQ_EXIT(3);
}
#endif

#ifdef CONFIG_UART4
void UART4_IRQHandler(void)
{
  TRACE_IRQ_ENTER(4);
  UART_irq(&__uart_vec[3]);
  TRACE_IRQ_EXIT(4);
}
#endif

void TIM2_IRQHandler(void)
{
  //TRACE_IRQ_ENTER(5);
  TIMER_irq();
  //TRACE_IRQ_EXIT(5);
}

#include "miniutils.h"

#ifdef CONFIG_SPI
void DMA1_Channel2_IRQHandler() {
  // DMA1 Channel 2 SPI1 RX
  TRACE_IRQ_ENTER(6);
  SPI_irq(&__spi_bus_vec[0]);
  TRACE_IRQ_EXIT(6);
}
void DMA1_Channel3_IRQHandler() {
  // DMA1 Channel 3 SPI1 TX
  TRACE_IRQ_ENTER(7);
  SPI_irq(&__spi_bus_vec[0]);
  TRACE_IRQ_EXIT(7);
}
void DMA1_Channel4_IRQHandler() {
  // DMA1 Channel 4 SPI2 RX
  TRACE_IRQ_ENTER(8);
  SPI_irq(&__spi_bus_vec[1]);
  TRACE_IRQ_EXIT(8);
}
void DMA1_Channel5_IRQHandler() {
  // DMA1 Channel 5 SPI2 TX
  TRACE_IRQ_ENTER(9);
  SPI_irq(&__spi_bus_vec[1]);
  TRACE_IRQ_EXIT(9);
}
#endif

#ifdef CONFIG_ETHSPI
void EXTI4_IRQHandler(void)
{
  TRACE_IRQ_ENTER(10);
  ETH_SPI_irq();
  TRACE_IRQ_EXIT(10);
}
#endif

#ifdef OS_DUMP_IRQ
void EXTI2_IRQHandler(void)
{
  TRACE_IRQ_ENTER(11);
  OS_DBG_dump_irq();
  SYS_dump_trace();
  TRACE_IRQ_EXIT(11);
}
#endif


void PendSV_Handler(void)
{
  TRACE_IRQ_ENTER(12);
  __os_pendsv();
  // cannot have trace after pendsv, this will corrupt ctx switcher
  //TRACE_IRQ_EXIT(12);
}

void SysTick_Handler(void)
{
  TRACE_IRQ_ENTER(13);
  __os_systick();
  TRACE_IRQ_EXIT(13);
}

#ifdef CONFIG_I2C
void I2C1_ER_IRQHandler(void)
{
  TRACE_IRQ_ENTER(14);
  I2C_IRQ_err(&__i2c_bus_vec[0]);
  TRACE_IRQ_EXIT(14);
}

void I2C1_EV_IRQHandler(void)
{
  TRACE_IRQ_ENTER(15);
  I2C_IRQ_ev(&__i2c_bus_vec[0]);
  TRACE_IRQ_EXIT(15);
}
#endif


/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
