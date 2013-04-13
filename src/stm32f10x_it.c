#include "stm32f10x_it.h"
#include "uart.h"
#include "spi_driver.h"
#include "timer.h"
#include "os.h"
#include "enc28j60_spi_eth.h"

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
  UART_irq(&__uart_vec[0]);
}
#endif

#ifdef CONFIG_UART2
void USART2_IRQHandler(void)
{
  UART_irq(&__uart_vec[1]);
}
#endif

#ifdef CONFIG_UART3
void USART3_IRQHandler(void)
{
  UART_irq(&__uart_vec[2]);
}
#endif

#ifdef CONFIG_UART4
void UART4_IRQHandler(void)
{
  UART_irq(&__uart_vec[3]);
}
#endif

void TIM2_IRQHandler(void)
{
  TIMER_irq();
}

#include "miniutils.h"

#ifdef CONFIG_SPI
void DMA1_Channel2_IRQHandler() {
  // DMA1 Channel 2 SPI RX
  SPI_irq(&__spi_bus_vec[0]);
}
void DMA1_Channel3_IRQHandler() {
  // DMA1 Channel 3 SPI TX
  SPI_irq(&__spi_bus_vec[0]);
}
#endif

#ifdef CONFIG_ETHSPI
void EXTI4_IRQHandler(void)
{
  ETH_SPI_irq();
}
#endif

#ifdef OS_DUMP_IRQ
void EXTI2_IRQHandler(void)
{
  OS_DBG_dump_irq();
}
#endif


void PendSV_Handler(void)
{
  __os_pendsv();
}

void SysTick_Handler(void)
{
  __os_systick();
}


/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
