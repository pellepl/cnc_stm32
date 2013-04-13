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

// enable led driver
#define CONFIG_LED

// enable CNC app
#define CONFIG_CNC


/*********************************************/
/***** Hardware build time configuration *****/
/*********************************************/

/** General **/

// internal flash start address
#define FLASH_START       FLASH_BASE
// internal flash page erase size
#define FLASH_PAGE_SIZE   0x400 // md
// internal flash protection/unprotection for firmware
#define FLASH_PROTECT     FLASH_WRProt_AllPages
// internal flash total size in bytes
#define FLASH_TOTAL_SIZE  (512*1024) // md

// firmware upgrade placement on spi flash
#define FIRMWARE_SPIF_ADDRESS   \
  (FLASH_M25P16_SIZE_TOTAL - \
      ((FLASH_TOTAL_SIZE+sizeof(fw_upgrade_info))/FLASH_M25P16_SIZE_SECTOR_ERASE)*FLASH_M25P16_SIZE_SECTOR_ERASE - \
      FLASH_M25P16_SIZE_SECTOR_ERASE)

#ifndef USER_HARDFAULT
// enable user hardfault handler
#define USER_HARDFAULT 1
#endif

// hardware debug (blinking leds etc)
#define HW_DEBUG

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
//#define CONFIG_SPI_POLL

#define SPI_MASTER_GPIO              GPIOA
#define SPI_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPI_MASTER_PIN_SCK           GPIO_Pin_5
#define SPI_MASTER_PIN_MISO          GPIO_Pin_6
#define SPI_MASTER_PIN_MOSI          GPIO_Pin_7

#define SPI_MASTER                   SPI1
#define SPI_MASTER_BASE              SPI1_BASE
#define SPI_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI_MASTER_DMA               DMA1
#define SPI_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI_MASTER_Rx_DMA_Channel    DMA1_Channel2
#define SPI_MASTER_Tx_DMA_Channel    DMA1_Channel3
#define SPI_MASTER_Rx_IRQ_Channel    DMA1_Channel2_IRQn
#define SPI_MASTER_Tx_IRQ_Channel    DMA1_Channel3_IRQn

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

/** OS **/

// Enable stack boundary checks
#define OS_STACK_CHECK        1
// Enable stack usage checks
#define OS_STACK_USAGE_CHECK  1
// Enable os debug functionality
#define OS_DBG_MON            1
#if OS_DBG_MON
// Max thread peer elements
#define OS_THREAD_PEERS       8
// Max mutex peer elements
#define OS_MUTEX_PEERS        32
// Max cond peer elements
#define OS_COND_PEERS         16

// Enable os dump by external interrupt
#define OS_DUMP_IRQ

#ifdef OS_DUMP_IRQ
#define OS_DUMP_IRQ_GPIO_PORT         GPIOE
#define OS_DUMP_IRQ_GPIO_PIN          GPIO_Pin_2
#define OS_DUMP_IRQ_GPIO_PORT_SOURCE  GPIO_PortSourceGPIOE
#define OS_DUMP_IRQ_GPIO_PIN_SOURCE   GPIO_PinSource2
#define OS_DUMP_IRQ_EXTI_LINE         EXTI_Line2
#define OS_DUMP_IRQ_EXTI_IRQn         EXTI2_IRQn
#endif

#endif

/** TICKER **/

// timer frequency
#define SYS_MAIN_TIMER_FREQ   40000
// system tick frequency
#define SYS_TIMER_TICK_FREQ   1000

/** COMMUNICATION **/

// other communication id
#define COMM_CONTROLLER_ADDRESS 2

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

/** DEBUG OUTPUT **/

// disable all debug output
//#define DBG_OFF

#ifndef DBG_OFF

#define DBG_TIMESTAMP_PREFIX   1
#define DBG_LEVEL_PREFIX       0

#define _DBG_BIT_NAMES { \
  "sys",\
  "app",\
  "task",\
  "os",\
  "heap",\
  "comm",\
  "console",\
  "nvs", \
  "spi", \
  "eth", \
}

#ifdef DBG_SYS_OFF
#define D_SYS     0
#else
#define D_SYS     (1<<0)
#endif
#ifdef DBG_APP_OFF
#define D_APP     0
#else
#define D_APP     (1<<1)
#endif
#ifdef DBG_TASK_OFF
#define D_TASK    0
#else
#define D_TASK    (1<<2)
#endif
#ifdef DBG_OS_OFF
#define D_OS      0
#else
#define D_OS      (1<<3)
#endif
#ifdef DBG_HEAP_OFF
#define D_HEAP    0
#else
#define D_HEAP    (1<<4)
#endif
#ifdef DBG_COMM_OFF
#define D_COMM    0
#else
#define D_COMM    (1<<5)
#endif
#ifdef DBG_CONSOLE_OFF
#define D_CONSOLE 0
#else
#define D_CONSOLE (1<<6)
#endif
#ifdef DBG_NVS_OFF
#define D_NVS     0
#else
#define D_NVS     (1<<7)
#endif
#ifdef DBG_SPI_OFF
#define D_SPI     0
#else
#define D_SPI     (1<<8)
#endif
#ifdef DBG_ETH_OFF
#define D_ETH     0
#else
#define D_ETH     (1<<9)
#endif
#ifdef DBG_ANY_OFF
#define D_ANY     0
#else
#define D_ANY     (0xffffffff)
#endif

#else // DBG_OFF

#define D_SYS    0
#define D_APP     0
#define D_TASK    0
#define D_OS      0
#define D_HEAP    0
#define D_COMM    0
#define D_CONSOLE 0
#define D_NVS     0
#define D_SPI     0
#define D_ETH     0
#define D_ANY     0

#endif  // DBG_OFF

#define D_DEBUG   0
#define D_INFO    1
#define D_WARN    2
#define D_FATAL   3

#endif /* SYSTEM_CONFIG_H_ */
