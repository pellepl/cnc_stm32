/*
 * system_config.h
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#include "types.h"
#include "stm32f10x.h"


#define APP_NAME "CNC control"


/****************************************/
/***** Functionality block settings *****/
/****************************************/


// enable uart1
#define CONFIG_UART1
// enable uart2
#define CONFIG_UART2
// enable uart3
#define CONFIG_UART3
// enable uart4
#define CONFIG_UART4

#define CONFIG_UART_CNT   4 // update according to enabled uarts

// enable SPI driver
#define CONFIG_SPI

// Enable ENC28J60 ethernet spi driver
#define CONFIG_ETHSPI
#define UDP_client
#define TCP_client

// enable led driver
#define CONFIG_LED

// enable CNC app
#define CONFIG_CNC


/*********************************************/
/***** Hardware build time configuration *****/
/*********************************************/

/** Processor specifics **/

#ifndef USER_HARDFAULT
// enable user hardfault handler
#define USER_HARDFAULT 1
#endif

// hardware debug (blinking leds etc)
#define HW_DEBUG


/** General **/

// internal flash start address
#define FLASH_START       FLASH_BASE
// internal flash page erase size
#define FLASH_PAGE_SIZE   0x800 // hd
// internal flash protection/unprotection for firmware
#define FLASH_PROTECT     FLASH_WRProt_AllPages
// internal flash total size in bytes
#define FLASH_TOTAL_SIZE  (512*1024) // hd

// firmware upgrade placement on spi flash
#define FIRMWARE_SPIF_ADDRESS_BASE   \
  (FLASH_M25P16_SIZE_TOTAL - \
      ((FLASH_TOTAL_SIZE+sizeof(fw_upgrade_info))/FLASH_M25P16_SIZE_SECTOR_ERASE)*FLASH_M25P16_SIZE_SECTOR_ERASE - \
      FLASH_M25P16_SIZE_SECTOR_ERASE)

#define FIRMWARE_SPIF_ADDRESS_META (FIRMWARE_SPIF_ADDRESS_BASE)
#define FIRMWARE_SPIF_ADDRESS_DATA (FIRMWARE_SPIF_ADDRESS_BASE + sizeof(fw_upgrade_info))


typedef GPIO_TypeDef * hw_io_port;
typedef uint16_t hw_io_pin;

/** CNC **/

#ifdef CONFIG_CNC

// cnc control port
#define CNC_GPIO_PORT         GPIOE
// cnc control port clock
#define CNC_APBPeriph_GPIO    RCC_APB2Periph_GPIOE

// cnc A step pin
#define CNC_GPIO_STEP_A       GPIO_Pin_10
// cnc A dir pin
#define CNC_GPIO_DIR_A        GPIO_Pin_9
// cnc X step pin
#define CNC_GPIO_STEP_X       GPIO_Pin_8
// cnc X dir pin
#define CNC_GPIO_DIR_X        GPIO_Pin_11
// cnc Y step pin
#define CNC_GPIO_STEP_Y       GPIO_Pin_12
// cnc Y dir pin
#define CNC_GPIO_DIR_Y        GPIO_Pin_13
// cnc Z step pin
#define CNC_GPIO_STEP_Z       GPIO_Pin_14
// cnc Z dir pin
#define CNC_GPIO_DIR_Z        GPIO_Pin_15
// cnc sense pin
#define CNC_GPIO_SENSE        GPIO_Pin_1

#define CNC_GPIO_DEF(set, reset) \
  CNC_GPIO_PORT->BSRR = ((set)) | ((reset)<<16)

#define CNC_GPIO_DEF_READ() \
  (CNC_GPIO_PORT->IDR)

#endif // CONFIG_CNC

/** UART **/

#ifdef CONFIG_UART1
#define UART1_GPIO_PORT       GPIOA
#define UART1_GPIO_RX         GPIO_Pin_10
#define UART1_GPIO_TX         GPIO_Pin_9
#endif

#ifdef CONFIG_UART2
#define UART2_GPIO_PORT       GPIOA
#define UART2_GPIO_RX         GPIO_Pin_3
#define UART2_GPIO_TX         GPIO_Pin_2
#endif

#ifdef CONFIG_UART3
#define UART3_GPIO_PORT       GPIOD
#define UART3_GPIO_RX         GPIO_Pin_9
#define UART3_GPIO_TX         GPIO_Pin_8
#endif

#ifdef CONFIG_UART4
#define UART4_GPIO_PORT       GPIOC
#define UART4_GPIO_RX         GPIO_Pin_11
#define UART4_GPIO_TX         GPIO_Pin_10
#endif

/** SPI **/

#ifdef CONFIG_SPI

// enable/disable chunking when rxing over spi
//#define CONFIG_SPI_CHUNK_RX

// make SPI driver use polling method, otherwise DMA requests are used
// warning - polling method should only be used for debugging and may be
// unstable. Do not sent multitudes of data using this flag
#define CONFIG_SPI_POLL

#define SPI1_MASTER_GPIO              GPIOA
#define SPI1_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPI1_MASTER_PIN_SCK           GPIO_Pin_5
#define SPI1_MASTER_PIN_MISO          GPIO_Pin_6
#define SPI1_MASTER_PIN_MOSI          GPIO_Pin_7

#define SPI1_MASTER                   SPI1
#define SPI1_MASTER_BASE              SPI1_BASE
#define SPI1_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI1_MASTER_DMA               DMA1
#define SPI1_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI1_MASTER_Rx_DMA_Channel    DMA1_Channel2
#define SPI1_MASTER_Tx_DMA_Channel    DMA1_Channel3
#define SPI1_MASTER_Rx_IRQ_Channel    DMA1_Channel2_IRQn
#define SPI1_MASTER_Tx_IRQ_Channel    DMA1_Channel3_IRQn

#define SPI2_MASTER_GPIO              GPIOB
#define SPI2_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOB
#define SPI2_MASTER_PIN_SCK           GPIO_Pin_13
#define SPI2_MASTER_PIN_MISO          GPIO_Pin_14
#define SPI2_MASTER_PIN_MOSI          GPIO_Pin_15

#define SPI2_MASTER                   SPI2
#define SPI2_MASTER_BASE              SPI2_BASE
#define SPI2_MASTER_CLK               RCC_APB1Periph_SPI2
#define SPI2_MASTER_DMA               DMA1
#define SPI2_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI2_MASTER_Rx_DMA_Channel    DMA1_Channel4
#define SPI2_MASTER_Tx_DMA_Channel    DMA1_Channel5
#define SPI2_MASTER_Rx_IRQ_Channel    DMA1_Channel4_IRQn
#define SPI2_MASTER_Tx_IRQ_Channel    DMA1_Channel5_IRQn

/** SPI FLASH **/

// spi flash chip select port and pin
#define SPI_FLASH_GPIO_PORT          GPIOA
#define SPI_FLASH_GPIO_PIN           GPIO_Pin_4

/** SPI ETH ENC28J60 **/

#ifdef CONFIG_ETHSPI

// eth/spi chip select port and pin
#define SPI_ETH_GPIO_PORT             GPIOD
#define SPI_ETH_GPIO_PIN              GPIO_Pin_6
// eth/spi rx frame interrupt port and pin
#define SPI_ETH_INT_GPIO_PORT         GPIOC
#define SPI_ETH_INT_GPIO_PIN          GPIO_Pin_4
#define SPI_ETH_INT_GPIO_PORT_SOURCE  GPIO_PortSourceGPIOC
#define SPI_ETH_INT_GPIO_PIN_SOURCE   GPIO_PinSource4
#define SPI_ETH_INT_EXTI_LINE         EXTI_Line4
#define SPI_ETH_INT_EXTI_IRQn         EXTI4_IRQn
#endif

#endif // CONFIG_SPI

/** LED **/

#ifdef CONFIG_LED
#ifdef HW_DEBUG

#define LED12_GPIO_PORT       GPIOD
#define LED12_APBPeriph_GPIO  RCC_APB2Periph_GPIOD
#define LED34_GPIO_PORT       GPIOC
#define LED34_APBPeriph_GPIO  RCC_APB2Periph_GPIOC
#define LED1_GPIO             GPIO_Pin_6
#define LED2_GPIO             GPIO_Pin_13
#define LED3_GPIO             GPIO_Pin_7
#define LED4_GPIO             GPIO_Pin_6

#endif

/** LED SHIFTER **/

// led tick timer divisor
#define LED_TIMER_DIVISOR     2048
// number of leds in total
#define LED_COUNT             14
// number of leds in sihft register
#define LED_SHIFT_REG_SIZE    12
// led shifter control port
#define LED_SHIFT_PORT        GPIOD
// led shifter clock pin
#define LED_SHIFT_CLK         GPIO_Pin_11
// led shifter data pin
#define LED_SHIFT_DAT         GPIO_Pin_5
// led shifter strobe pin
#define LED_SHIFT_STR         GPIO_Pin_4

#endif


/****************************************************/
/******** Application build time configuration ******/
/****************************************************/

/** TICKER **/

// timer frequency
#define SYS_MAIN_TIMER_FREQ   40000
// system tick frequency
#define SYS_TIMER_TICK_FREQ   1000

/** COMMUNICATION **/

// other communication id
#define COMM_CONTROLLER_ADDRESS 2
// store communication statistics
#define COMM_IMPL_STATS     1
// use packet pool
#define COMM_IMPL_USE_POOL  1


/** UART **/

#define COMMIN      0
#define COMMOUT     0
#define STDIN       1
#define STDOUT      1
#define SPLIN       2
#define SPLOUT      2
#define BTIN        3
#define BTOUT       3
#define UART1_SPEED 115200
#define UART2_SPEED 460800
#define UART3_SPEED 115200
#define UART4_SPEED 115200

#define COMM_UART_LIST {COMMIN, BTIN}
#define COMM_UARTS  2

/** ETH **/
#define ETH_ENC28J60_MII_POLL_GUARD   (0x10000)
#define ETH_MAC {0xc0,0xff,0xee,0xca,0xfe,0x01}
#define ETH_IP  {192,168,0,150}
#define ETH_INIT_DHCP

/** LED SHIFTER **/

#define LED_CNC_WORK_BIT      11
#define LED_CNC_WORK          (1<<LED_CNC_WORK_BIT)
#define LED_CNC_COMM_BIT      10
#define LED_CNC_COMM          (1<<LED_CNC_COMM_BIT)
#define LED_CNC_DISABLE_BIT   8
#define LED_CNC_DISABLE       (1<<LED_CNC_DISABLE_BIT)
#define LED_ERROR1_BIT        7
#define LED_ERROR1            (1<<LED_ERROR1_BIT)
#define LED_ERROR2_BIT        6
#define LED_ERROR2            (1<<LED_ERROR2_BIT)
#define LED_ERROR3_BIT        3
#define LED_ERROR3            (1<<LED_ERROR3_BIT)
#define LED_SPI_FLASH_BIT     1
#define LED_SPI_FLASH         (1<<LED_SPI_FLASH_BIT)

/** DEBUG **/

// disable all asserts
//#define ASSERT_OFF

// disable all debug output
//#define DBG_OFF

// enable debug monitor for os
#define OS_DBG_MON            1

// enable stack boundary checks
#define OS_STACK_CHECK        1

// enable stack usage checks
#define OS_STACK_USAGE_CHECK  1


#endif /* SYSTEM_CONFIG_H_ */
