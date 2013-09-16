#include "stm32f10x_it.h"
#include "uart_driver.h"
#include "spi_driver.h"
#include "timer.h"
#include "os.h"
#include "enc28j60_spi_eth.h"
#include "i2c_driver.h"
#include "usb_istr.h"

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
  TRACE_IRQ_ENTER(USART1_IRQn);
  UART_irq(&__uart_vec[0]);
  TRACE_IRQ_EXIT(USART1_IRQn);
}
#endif

#ifdef CONFIG_UART2
void USART2_IRQHandler(void)
{
  //TRACE_IRQ_ENTER(USART2_IRQn);
  UART_irq(&__uart_vec[1]);
  //TRACE_IRQ_EXIT(USART2_IRQn);
}
#endif

#ifdef CONFIG_UART3
void USART3_IRQHandler(void)
{
  TRACE_IRQ_ENTER(USART3_IRQn);
  UART_irq(&__uart_vec[2]);
  TRACE_IRQ_EXIT(USART3_IRQn);
}
#endif

#ifdef CONFIG_UART4
void UART4_IRQHandler(void)
{
  TRACE_IRQ_ENTER(UART4_IRQn);
  UART_irq(&__uart_vec[3]);
  TRACE_IRQ_EXIT(UART4_IRQn);
}
#endif

void TIM2_IRQHandler(void)
{
  //TRACE_IRQ_ENTER(TIM2_IRQn);
  TIMER_irq();
  //TRACE_IRQ_EXIT(TIM2_IRQn);
}

#include "miniutils.h"

#ifdef CONFIG_SPI
void DMA1_Channel2_IRQHandler() {
  // DMA1 Channel 2 SPI1 RX
  TRACE_IRQ_ENTER(DMA1_Channel2_IRQn);
  SPI_irq(&__spi_bus_vec[0]);
  TRACE_IRQ_EXIT(DMA1_Channel2_IRQn);
}
void DMA1_Channel3_IRQHandler() {
  // DMA1 Channel 3 SPI1 TX
  TRACE_IRQ_ENTER(DMA1_Channel3_IRQn);
  SPI_irq(&__spi_bus_vec[0]);
  TRACE_IRQ_EXIT(DMA1_Channel3_IRQn);
}
void DMA1_Channel4_IRQHandler() {
  // DMA1 Channel 4 SPI2 RX
  TRACE_IRQ_ENTER(DMA1_Channel4_IRQn);
  SPI_irq(&__spi_bus_vec[1]);
  TRACE_IRQ_EXIT(DMA1_Channel4_IRQn);
}
void DMA1_Channel5_IRQHandler() {
  // DMA1 Channel 5 SPI2 TX
  TRACE_IRQ_ENTER(DMA1_Channel5_IRQn);
  SPI_irq(&__spi_bus_vec[1]);
  TRACE_IRQ_EXIT(DMA1_Channel5_IRQn);
}
#endif

#ifdef CONFIG_ETHSPI
void EXTI4_IRQHandler(void)
{
  TRACE_IRQ_ENTER(EXTI4_IRQn);
  ETH_SPI_irq();
  TRACE_IRQ_EXIT(EXTI4_IRQn);
}
#endif

#ifdef OS_DUMP_IRQ
void EXTI2_IRQHandler(void)
{
  TRACE_IRQ_ENTER(EXTI2_IRQn);
  OS_DBG_dump_irq(IODBG);
  SYS_dump_trace(IODBG);
  TRACE_IRQ_EXIT(EXTI2_IRQn);
}
#endif

void PendSV_Handler(void)
{
  TRACE_IRQ_ENTER(PendSV_IRQn);
  __os_pendsv();
  // cannot have trace after pendsv, this will corrupt ctx switcher
  //TRACE_IRQ_EXIT(PendSV_IRQn);
}

void SysTick_Handler(void)
{
  TRACE_IRQ_ENTER(SysTick_IRQn);
  __os_systick();
  TRACE_IRQ_EXIT(SysTick_IRQn);
}

#ifdef CONFIG_I2C
void I2C1_ER_IRQHandler(void)
{
  TRACE_IRQ_ENTER(I2C1_ER_IRQn);
  I2C_IRQ_err(&__i2c_bus_vec[0]);
  TRACE_IRQ_EXIT(I2C1_ER_IRQn);
}

void I2C1_EV_IRQHandler(void)
{
  TRACE_IRQ_ENTER(I2C1_EV_IRQn);
  I2C_IRQ_ev(&__i2c_bus_vec[0]);
  TRACE_IRQ_EXIT(I2C1_EV_IRQn);
}
#endif

#ifdef CONFIG_USB_CDC
void USBWakeUp_IRQHandler(void)
{
  TRACE_IRQ_ENTER(USBWakeUp_IRQn);
  print("usbwku\n");
  EXTI_ClearITPendingBit(EXTI_Line18);
  TRACE_IRQ_EXIT(USBWakeUp_IRQn);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
// Called once every ms
//  TRACE_IRQ_ENTER(USB_LP_CAN1_RX0_IRQn);
  USB_Istr();
//  TRACE_IRQ_EXIT(USB_LP_CAN1_RX0_IRQn);
}
#endif



/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
