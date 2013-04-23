/*
 * console.c
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#include "uart.h"
#include "taskq.h"
#include "miniutils.h"
#include "system.h"
#include "cnc_control.h"
#include "comm_impl.h"
#include "cnc_comm.h"
#include "heap.h"
#include "spi_dev.h"
#include "led.h"
#include "spi_flash_m25p16.h"
#include "spi_dev_os_generic.h"
#include "spi_driver.h"
#include "nvstorage.h"
#include "os.h"
#ifdef CONFIG_ETHSPI
#include "enc28j60.h"
#include "enc28j60_spi_eth.h"
#endif
#include "bl_exec.h"

static u8_t in[256];

#define CONSOLE_PROMPT "> "
#define IS_STRING(s) ((u8_t*)(s) >= in && (u8_t*)(s) < in + sizeof(in))

typedef int(*func)(int a, ...);

typedef struct cmd_s {
  const char* name;
  const func fn;
  const char* help;
} cmd;

struct {
  uart_rx_callback prev_uart_rx_f;
  void *prev_uart_arg;
  uart *uart_pipe;
  u8_t uart_pipe_plusses;
  char uart_pipe_via;
} console_state;


#ifdef CONFIG_CNC
static int f_cnc_status();
static int f_cnc_on();
static int f_cnc_off();
static int f_cnc_feed(int feed);
static int f_cnc_move(int x, int y, int z);
static int f_cnc_srmask(u32_t sr_mask);
static int f_cnc_xyz(int sx, int fx, int sy, int fy, int sz, int fz, int r);
static int f_cnc_xyz_imm(int sx, int fx, int sy, int fy, int sz, int fz);
static int f_cnc_pause(int pause);
static int f_cnc_pon();
static int f_cnc_poff();
static int f_cnc_pflush();
static int f_cnc_sr_recurrence(int delta);
static int f_cnc_pos_recurrence(int delta);
static int f_cnc_io();
static int f_cnc_err_on(int);
static int f_cnc_err_off(int);
#endif

static int f_comm_send(int dst, char* data, int ack);
static int f_comm_uart(int uart);

static int f_uwrite(int uart, char* data);
static int f_uread(int uart, int numchars);
static int f_uconnect(int uart);
static int f_uconf(int uart, int speed);

static int f_reset();
static int f_reset_boot();
static int f_reset_fw_upgrade();
static int f_time(int d, int h, int m, int s, int ms);
static int f_help(char *s);
static int f_dump();
static int f_assert();
static int f_dbg();

#ifdef CONFIG_SPI
static int f_spifinit();
static int f_spifpro();
static int f_spifrd(int, int);
static int f_spifrdb(void *, int);
static int f_spifer(int, int);
static int f_spifwr(int, int);
static int f_spifcl(int);
static int f_spifbusy();
static int f_spifmaer();
static int f_spifdump();
#ifdef CONFIG_ETHSPI
static int f_spiginit();
static int f_spigrx(int num);
static int f_spigtx(char *str);
static int f_spigtxrx(char *str, int num);
static int f_spigclose();



static int f_ethregr(int reg);
static int f_ethregw(int reg, int data);

#endif
#endif

static int f_read_nvram();
static int f_wr_nvram(int a, int d);

static int f_hardfault(int a) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiv-by-zero"
  SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
  volatile int q = 3;
  volatile int x = 0;
  return q/x;
#pragma GCC diagnostic pop
}

#define USE_FSMC

#ifndef USE_FSMC
static void sleep() {
  volatile int i = 100;
  while(i-- > 0);
}


#define LCD_CS_on  GPIOD->BRR = GPIO_Pin_7; sleep()
#define LCD_CS_off GPIOD->BSRR = GPIO_Pin_7; sleep()
#define LCD_RS_on  GPIOD->BRR = GPIO_Pin_11; sleep()
#define LCD_RS_off GPIOD->BSRR = GPIO_Pin_11; sleep()
#define LCD_RD_on  GPIOD->BRR = GPIO_Pin_4; sleep()
#define LCD_RD_off GPIOD->BSRR = GPIO_Pin_4; sleep()
#define LCD_WR_on  GPIOD->BRR = GPIO_Pin_5; sleep()
#define LCD_WR_off GPIOD->BRR = GPIO_Pin_5; sleep()

static void output(u16_t x) {
  u16_t set_D = 0;
  u16_t res_D = 0;
  u16_t set_E = 0;
  u16_t res_E = 0;
  if (x & (1<<15))
    set_D |= GPIO_Pin_10;
  else
    res_D |= GPIO_Pin_10;
  if (x & (1<<14))
    set_D |= GPIO_Pin_9;
  else
    res_D |= GPIO_Pin_9;
  if (x & (1<<13))
    set_D |= GPIO_Pin_8;
  else
    res_D |= GPIO_Pin_8;

  if (x & (1<<12))
    set_E |= GPIO_Pin_15;
  else
    res_E |= GPIO_Pin_15;
  if (x & (1<<11))
    set_E |= GPIO_Pin_14;
  else
    res_E |= GPIO_Pin_14;
  if (x & (1<<10))
    set_E |= GPIO_Pin_13;
  else
    res_E |= GPIO_Pin_13;
  if (x & (1<<9))
    set_E |= GPIO_Pin_12;
  else
    res_E |= GPIO_Pin_12;
  if (x & (1<<8))
    set_E |= GPIO_Pin_11;
  else
    res_E |= GPIO_Pin_11;
  if (x & (1<<7))
    set_E |= GPIO_Pin_10;
  else
    res_E |= GPIO_Pin_10;
  if (x & (1<<6))
    set_E |= GPIO_Pin_9;
  else
    res_E |= GPIO_Pin_9;
  if (x & (1<<5))
    set_E |= GPIO_Pin_8;
  else
    res_E |= GPIO_Pin_8;
  if (x & (1<<4))
    set_E |= GPIO_Pin_7;
  else
    res_E |= GPIO_Pin_7;

  if (x & (1<<3))
    set_D |= GPIO_Pin_1;
  else
    res_D |= GPIO_Pin_1;
  if (x & (1<<2))
    set_D |= GPIO_Pin_0;
  else
    res_D |= GPIO_Pin_0;
  if (x & (1<<1))
    set_D |= GPIO_Pin_15;
  else
    res_D |= GPIO_Pin_15;
  if (x & (1<<0))
    set_D |= GPIO_Pin_14;
  else
    res_D |= GPIO_Pin_14;
  GPIOD->BSRR = set_D | (res_D << 16);
  GPIOE->BSRR = set_E | (res_E << 16);
}

static void LCD_WR_REG(u16_t reg, u16_t data){
  LCD_CS_on;
  // register
  output(reg);
  LCD_RS_on;

  LCD_WR_on;
  LCD_WR_off;

  LCD_RS_off;

  // data
  output(data);
  LCD_WR_on;
  LCD_WR_off;

  LCD_CS_off;
}

static void LCD_WR(u16_t data) {
  LCD_CS_on;
  // data
  output(data);

  LCD_WR_on;
  LCD_WR_off;

  LCD_CS_off;
}

static u16_t LCD_RD_REG(u16_t reg) {
  GPIO_InitTypeDef GPIO_InitStructure;
  LCD_CS_on;

  // register
  output(reg);
  LCD_RS_on;

  LCD_WR_on;
  LCD_WR_off;

  LCD_RS_off;

  // set as input
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_10 | // DB17 FSMC_D15
      GPIO_Pin_9 |  // DB16 FSMC_D14
      GPIO_Pin_8 |  // DB15 FSMC_D13
      GPIO_Pin_1 |  // DB3  FSMC_D3
      GPIO_Pin_0 |  // DB2  FSMC_D2
      GPIO_Pin_15 | // DB1  FSMC_D1
      GPIO_Pin_14 | // DB0  FSMC_D0
      0;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_15 | // DB14 FSMC_D12
      GPIO_Pin_14 | // DB13 FSMC_D11
      GPIO_Pin_13 | // DB12 FSMC_D10
      GPIO_Pin_12 | // DB11 FSMC_D9
      GPIO_Pin_11 | // DB10 FSMC_D8
      GPIO_Pin_10 | // DB7 FSMC_D7
      GPIO_Pin_9  | // DB6 FSMC_D6
      GPIO_Pin_8  | // DB5 FSMC_D5
      GPIO_Pin_7  | // DB4 FSMC_D4
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  // data
  LCD_RD_on;
  u16_t i_d = GPIOD->IDR;
  u16_t i_e = GPIOE->IDR;
  LCD_RD_off;


  LCD_CS_off;

  // set as output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_10 | // DB17 FSMC_D15
      GPIO_Pin_9 |  // DB16 FSMC_D14
      GPIO_Pin_8 |  // DB15 FSMC_D13
      GPIO_Pin_1 |  // DB3  FSMC_D3
      GPIO_Pin_0 |  // DB2  FSMC_D2
      GPIO_Pin_15 | // DB1  FSMC_D1
      GPIO_Pin_14 | // DB0  FSMC_D0
      0;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_15 | // DB14 FSMC_D12
      GPIO_Pin_14 | // DB13 FSMC_D11
      GPIO_Pin_13 | // DB12 FSMC_D10
      GPIO_Pin_12 | // DB11 FSMC_D9
      GPIO_Pin_11 | // DB10 FSMC_D8
      GPIO_Pin_10 | // DB7 FSMC_D7
      GPIO_Pin_9  | // DB6 FSMC_D6
      GPIO_Pin_8  | // DB5 FSMC_D5
      GPIO_Pin_7  | // DB4 FSMC_D4
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  u16_t x = 0;
  if (i_d & GPIO_Pin_10) x |= (1<<15);
  if (i_d & GPIO_Pin_9) x |= (1<<14);
  if (i_d & GPIO_Pin_8) x |= (1<<13);

  if (i_e & GPIO_Pin_15) x |= (1<<12);
  if (i_e & GPIO_Pin_14) x |= (1<<11);
  if (i_e & GPIO_Pin_13) x |= (1<<10);
  if (i_e & GPIO_Pin_12) x |= (1<<9);
  if (i_e & GPIO_Pin_11) x |= (1<<8);
  if (i_e & GPIO_Pin_10) x |= (1<<7);
  if (i_e & GPIO_Pin_9) x |= (1<<6);
  if (i_e & GPIO_Pin_8) x |= (1<<5);
  if (i_e & GPIO_Pin_7) x |= (1<<4);

  if (i_d & GPIO_Pin_1) x |= (1<<3);
  if (i_d & GPIO_Pin_0) x |= (1<<2);
  if (i_d & GPIO_Pin_15) x |= (1<<1);
  if (i_d & GPIO_Pin_14) x |= (1<<0);

  return x;
}

static void setup_LCD_ifc() {
  GPIO_InitTypeDef GPIO_InitStructure;
  // RCC block clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

  // GPIO config
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_10 | // DB17 FSMC_D15
      GPIO_Pin_9 |  // DB16 FSMC_D14
      GPIO_Pin_8 |  // DB15 FSMC_D13
      GPIO_Pin_7 |  // CS   FSMC_NE1
      GPIO_Pin_11 | // RS   FSMC_A16
      GPIO_Pin_5 |  // WR   FSMC_NWE
      GPIO_Pin_4 |  // RD   FSMC_NOE
      GPIO_Pin_1 |  // DB3  FSMC_D3
      GPIO_Pin_0 |  // DB2  FSMC_D2
      GPIO_Pin_15 | // DB1  FSMC_D1
      GPIO_Pin_14 | // DB0  FSMC_D0
      0;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_15 | // DB14 FSMC_D12
      GPIO_Pin_14 | // DB13 FSMC_D11
      GPIO_Pin_13 | // DB12 FSMC_D10
      GPIO_Pin_12 | // DB11 FSMC_D9
      GPIO_Pin_11 | // DB10 FSMC_D8
      GPIO_Pin_10 | // DB7 FSMC_D7
      GPIO_Pin_9  | // DB6 FSMC_D6
      GPIO_Pin_8  | // DB5 FSMC_D5
      GPIO_Pin_7  | // DB4 FSMC_D4
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);


  // lcd ctrl
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_1  | // RESET
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  LCD_CS_off;
  LCD_WR_off;
  LCD_RD_off;
  LCD_RS_off;
}

#else


#define BASE_ADDR 0x60000000
#define REG_OFFS  0x00000000
#define DAT_OFFS  0x00020000

#define LCD_WR_REG(reg, data) do {\
    *((volatile u16_t*)((u32_t)BASE_ADDR + REG_OFFS)) = (u16_t)(reg);\
    *((volatile u16_t*)((u32_t)BASE_ADDR + DAT_OFFS)) = (u16_t)(data);\
} while (0);
#define LCD_WR(data) do {\
    *((volatile u16_t*)((u32_t)BASE_ADDR + DAT_OFFS)) = (u16_t)(data);\
} while (0);
static u16_t LCD_RD_REG(u16_t reg) {
    *((volatile u16_t*)((u32_t)BASE_ADDR + REG_OFFS)) = (u16_t)(reg);
    return *((volatile u16_t*)((u32_t)BASE_ADDR + DAT_OFFS));
}

static void setup_LCD_ifc() {
  GPIO_InitTypeDef GPIO_InitStructure;
  FSMC_NORSRAMInitTypeDef fsmc_conf;
  FSMC_NORSRAMTimingInitTypeDef fsmc_rw_timing;
  FSMC_NORSRAMTimingInitTypeDef fsmc_w_timing;
  fsmc_conf.FSMC_ReadWriteTimingStruct = &fsmc_rw_timing;
  fsmc_conf.FSMC_WriteTimingStruct = &fsmc_w_timing;

  // RCC block clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  // GPIO config
  // SRAM / LED ifc
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_10 | // DB17 FSMC_D15
      GPIO_Pin_9 |  // DB16 FSMC_D14
      GPIO_Pin_8 |  // DB15 FSMC_D13
      GPIO_Pin_7 |  // CS   FSMC_NE1
      GPIO_Pin_11 | // RS   FSMC_A16
      GPIO_Pin_5 |  // RW   FSMC_NWE
      GPIO_Pin_4 |  // RD   FSMC_NOE
      GPIO_Pin_1 |  // DB3  FSMC_D3
      GPIO_Pin_0 |  // DB2  FSMC_D2
      GPIO_Pin_15 | // DB1  FSMC_D1
      GPIO_Pin_14 | // DB0  FSMC_D0
      0;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_15 | // DB14 FSMC_D12
      GPIO_Pin_14 | // DB13 FSMC_D11
      GPIO_Pin_13 | // DB12 FSMC_D10
      GPIO_Pin_12 | // DB11 FSMC_D9
      GPIO_Pin_11 | // DB10 FSMC_D8
      GPIO_Pin_10 | // DB7 FSMC_D7
      GPIO_Pin_9  | // DB6 FSMC_D6
      GPIO_Pin_8  | // DB5 FSMC_D5
      GPIO_Pin_7  | // DB4 FSMC_D4
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);


  // keys
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_5 |
      GPIO_Pin_4 |
      GPIO_Pin_3 |
      GPIO_Pin_2 |
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);


  // led ctrl
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_1  | // RESET
      0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  // FSMC block
  FSMC_NORSRAMStructInit(&fsmc_conf);
  fsmc_conf.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  fsmc_conf.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  fsmc_conf.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  fsmc_conf.FSMC_WaitSignal= FSMC_WaitSignal_Disable;
#if 1
  fsmc_rw_timing.FSMC_AddressSetupTime = 0; //0xF;
  fsmc_rw_timing.FSMC_AddressHoldTime = 0; //0xF;
  fsmc_rw_timing.FSMC_DataSetupTime = 1; //0xFF;
  fsmc_rw_timing.FSMC_BusTurnAroundDuration = 0; //0xF;
  fsmc_rw_timing.FSMC_CLKDivision = 0; //0xF;
  fsmc_rw_timing.FSMC_DataLatency = 0; //0xF;
  fsmc_rw_timing.FSMC_AccessMode = FSMC_AccessMode_A;
#endif

  FSMC_NORSRAMInit(&fsmc_conf);

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);

}



#endif


#if 0
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

/* Write to FSMC -----------------------------------------------------------*/
 /* DMA2 channel5 configuration */
 DMA_DeInit(DMA2_Channel5);
 DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SRC_Const_Buffer;
 DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Bank1_SRAM3_ADDR;
 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
 DMA_InitStructure.DMA_BufferSize = 32;
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
 DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
 DMA_InitStructure.DMA_Priority = DMA_Priority_High;
 DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
 DMA_Init(DMA2_Channel5, &DMA_InitStructure);

 /* Enable DMA2 channel5 */
 DMA_Cmd(DMA2_Channel5, ENABLE);

 /* Check if DMA2 channel5 transfer is finished */
 while(!DMA_GetFlagStatus(DMA2_FLAG_TC5));

 /* Clear DMA2 channel5 transfer complete flag bit */
 DMA_ClearFlag(DMA2_FLAG_TC5);
#endif

#define Delay(x) SYS_hardsleep_ms(x)

static int f_test() {
  setup_LCD_ifc();
  print("pulling reset\n");
  volatile int i;
  GPIOE->BRR = GPIO_Pin_1;
  for(i=200000;i>0;i--);
  GPIOE->BSRR = GPIO_Pin_1;
  for(i=200000;i>0;i--);
  print("reset pulled\n");
  for(i=200000;i>0;i--);
  for(i=0;i<256;i++) {
    print("reg 0x%02x = %04x\n",i, LCD_RD_REG(i));
  }

  //############# void Power_Set(void) ################//
  LCD_WR_REG(0x0000,0x0001);
  Delay(10);

  LCD_WR_REG(0x0015,0x0030);
  LCD_WR_REG(0x0011,0x0040);
  LCD_WR_REG(0x0010,0x1628);
  LCD_WR_REG(0x0012,0x0000);
  LCD_WR_REG(0x0013,0x104d);
  Delay(10);
  LCD_WR_REG(0x0012,0x0010);
  Delay(10);
  LCD_WR_REG(0x0010,0x2620);
  LCD_WR_REG(0x0013,0x344d); //304d
  Delay(10);

  LCD_WR_REG(0x0001,0x0100);
  LCD_WR_REG(0x0002,0x0300);
  LCD_WR_REG(0x0003,0x1030);
  LCD_WR_REG(0x0008,0x0604);
  LCD_WR_REG(0x0009,0x0000);
  LCD_WR_REG(0x000A,0x0008);

  LCD_WR_REG(0x0041,0x0002);
  LCD_WR_REG(0x0060,0x2700);
  LCD_WR_REG(0x0061,0x0001);
  LCD_WR_REG(0x0090,0x0182);
  LCD_WR_REG(0x0093,0x0001);
  LCD_WR_REG(0x00a3,0x0010);
  Delay(10);

  //################# void Gamma_Set(void) ####################//
  LCD_WR_REG(0x30,0x0000);
  LCD_WR_REG(0x31,0x0502);
  LCD_WR_REG(0x32,0x0307);
  LCD_WR_REG(0x33,0x0305);
  LCD_WR_REG(0x34,0x0004);
  LCD_WR_REG(0x35,0x0402);
  LCD_WR_REG(0x36,0x0707);
  LCD_WR_REG(0x37,0x0503);
  LCD_WR_REG(0x38,0x1505);
  LCD_WR_REG(0x39,0x1505);
  Delay(10);

  //################## void Display_ON(void) ####################//
  LCD_WR_REG(0x0007,0x0001);
  Delay(10);
  LCD_WR_REG(0x0007,0x0021);
  LCD_WR_REG(0x0007,0x0023);
  Delay(10);
  LCD_WR_REG(0x0007,0x0033);
  Delay(10);
  LCD_WR_REG(0x0007,0x0133);

#if 1
    int c = 0;
    int _c1 = 0x8888;
    int _c2 = 0x0000;
    while (TRUE) {
      u16_t inp = GPIOE->IDR;
      if ((inp & GPIO_Pin_5) == 0) {
        c += 0x3f;
      }
      if ((inp & GPIO_Pin_4) == 0) {
        c += 0x1f<<6;
      }
      if ((inp & GPIO_Pin_3) == 0) {
        c += 0x3f<<(5+6);
      }
      if ((inp & GPIO_Pin_2) == 0) {
        c -= 2;
      }
      c++;
      LCD_WR_REG(0x0020,0x0000);
      LCD_WR_REG(0x0021,0x0000);
      LCD_WR_REG(0x0022,0x0000);
      int x = 0;
      int y = 0;
      for (x = 0; x < 320; x++) {
        int c1 = ((((c>>2) + x)&0x1f)<0x10?_c1:_c2);
        int c2 = ((((c>>2) + x)&0x1f)<0x10?_c2:_c1);
        for (y = 0; y < 240; y++) {
          int q = c + ((x>>3) * (y>>3));
          if ((((c>>0) + y) & 0x1f)<0x10) {
            LCD_WR(q^c1);
          } else {
            LCD_WR(q^c2);
          }
        }
      }
    }
#endif
  for(i=0;i<256;i++) {
    print("reg 0x%02x = %04x\n",i, LCD_RD_REG(i));
  }
#if 0
  int r = 0x80;
  for(i=0;i<0x100;i++) {
    LCD_WR_REG(r, i);
    int x = LCD_RD_REG(r);
    if (i != x) {
     print("writing 0x%04x to 0x%02x => giving %04x\n",i, r, x);
    }
  }
#endif

  return 0;
}

static int _argc;
static void *_args[16];

void DBG_uart_pipe_irq(void *a, u8_t c);

#define HELP_UART_DEFS "uart - 0,1,2,3 - 0 is COMM, 1 is DBG, 2 is SPL, 3 is BT\n"

static cmd c_tbl[] = {
#ifdef CONFIG_CNC

    {.name = "cnc_dump",  .fn = (func)CNC_dump,
        .help = "Dumps state of cnc\n"},
    {.name = "cnc_on",  .fn = (func)f_cnc_on,
        .help = "Enable cnc\n"
    },
    {.name = "cnc_off",  .fn = (func)f_cnc_off,
        .help = "Disable cnc\n"
    },
    {.name = "cnc_feed",  .fn = (func)f_cnc_feed,
        .help = "Set cnc feedrate in mm per minute\n"\
        "ex: cnc_feed 140\n"
    },
    {.name = "cnc_move",  .fn = (func)f_cnc_move,
        .help = "Moves cnc position linearly in given feedrate\n"
            "cnc_move <mmX> <mmY> <mmZ>"\
            "ex: cnc_move -10 30 0\n"
    },
    {.name = "cnc_xyz",  .fn = (func)f_cnc_xyz,
        .help = "Puts a movement into cnc latch register\n"\
        "cnc_xyz <stepsX> <freqX> <stepsY> <freqY> <stepsZ> <freqZ> <rapid>\n"\
        "step - number of stepper motor steps - integer, sign gives direction\n"\
        "freq - stepper frequency - hexadecimal, Q18.14\n"
    },
    {.name = "cnc_xyz_imm",  .fn = (func)f_cnc_xyz_imm,
        .help = "Puts a movement into cnc working register\n"\
        "cnc_xyz_imm <stepsX> <freqX> <stepsY> <freqY> <stepsZ> <freqZ>\n"\
        "step - number of stepper motor steps - integer, sign gives direction\n"\
        "freq - stepper frequency - hexadecimal, Q18.14\n"\
        "NOTE: this will overwrite current motion and has effect directly\n"
    },
    {.name = "cnc_pause",  .fn = (func)f_cnc_pause,
        .help = "Puts a pause into cnc latch register\n"\
        "cnc_pause <pause in ms>\n"
    },
    {.name = "cnc_pon",  .fn = (func)f_cnc_pon,
        .help = "Enable cnc pipeline\n"
    },
    {.name = "cnc_poff",  .fn = (func)f_cnc_poff,
        .help = "Disable cnc pipeline\n"
    },
    {.name = "cnc_pflush",  .fn = (func)f_cnc_pflush,
        .help = "Flush cnc pipeline\n" \
        "NOTE: cnc_pflush erases all commands in the cnc execution pipeline\n"
    },
    {.name = "cnc_sr",  .fn = (func)f_cnc_status,
        .help = "Prints status of cnc\n"
    },
    {.name = "cnc_srmask",  .fn = (func)f_cnc_srmask,
        .help = "Set status report mask\n"\
        "Sets report mask - if bits in cnc status reg changes, callback is made\n"\
        "cnc_srmask <mask> where\n"\
        "b0 - control enable/disable\n"\
        "b1 - movement still\n"\
        "b2 - movement paused\n"\
        "b3 - movement rapid\n"\
        "b4 - pipeline active\n"\
        "b5 - pipeline empty\n"\
        "b6 - pipeline full\n"\
        "b7 - latch reg full\n"\
        "ex: cnc_srmask 0b10100011\n"
    },
    {.name = "cnc_sr_recurrence", .fn = (func)f_cnc_sr_recurrence,
        .help = "Sets delta time of sr report recurrence in ms\n" \
        "cnc_sr_recurrence <time_in_ms> - if <time_in_ms> is zero, reporting is disabled\n" \
        "ex: cnc_sr_recurrence 1000\n"
    },
    {.name = "cnc_pos_recurrence", .fn = (func)f_cnc_pos_recurrence,
        .help = "Sets delta time of position report recurrence in ms\n" \
        "cnc_pos_recurrence <time_in_ms> - if <time_in_ms> is zero, reporting is disabled\n" \
        "ex: cnc_pos_recurrence 1000\n"
    },
    {.name = "cnc_io",  .fn = (func)f_cnc_io,
        .help = "Set individual pin state of cnc\n"\
        "cnc_io ((<pin>)* <1|0|flip <times> <hz>>)*\n" \
        "pin - dx,sx,dy,sy,dz,sz,da,sa\n" \
        "ex1: cnc_io dx sx 0 dy dz 1\n" \
        "ex2: cnc_io dx 1 sx flip 400 100\n"
    },
    {.name = "cnc_err_off",  .fn = (func)f_cnc_err_off,
        .help = "Clear cnc error state\n"\
        "cnc_err_off <error>\n" \
        "ex: cnc_err_off 0xff\n"
    },
    {.name = "cnc_err_on",  .fn = (func)f_cnc_err_on,
        .help = "Set cnc error state\n"\
        "cnc_err_on <error>\n" \
        "ex: cnc_err_on 0xff\n"
    },
#endif // CONFIG_CNC

    {.name = "comm_send",  .fn = (func)f_comm_send,
            .help = "Sends packet of data to dest\n"\
            "comm_send <dest> <data> <acked>\n"\
            "ex: comm_send 2 \"hello world\" 1\n"
    },
    {.name = "comm_uart",  .fn = (func)f_comm_uart,
        .help = "Set CNC communication uart\n"\
        "comm_uart <uart>\n"\
        HELP_UART_DEFS \
        "ex: comm_uart 3\n"
    },
    {.name = "comm_dump",  .fn = (func)COMM_dump,
            .help = "Dumps state of communication\n"
    },
    {.name = "heap_dump",  .fn = (func)HEAP_dump,
            .help = "Dumps state of heap\n"
    },
    {.name = "task_dump",  .fn = (func)TASK_dump,
            .help = "Dumps state of task queue\n"
    },
    {.name = "dump",  .fn = (func)f_dump,
            .help = "Dumps state of all system\n"
    },
    {.name = "time",  .fn = (func)f_time,
            .help = "Prints or sets time\n"\
                "time or time <day> <hour> <minute> <second> <millisecond>\n"
    },
    {.name = "uwrite",  .fn = (func)f_uwrite,
        .help = "Writes to uart\n"\
        "uwrite <uart> <string>\n"\
        HELP_UART_DEFS \
        "ex: uwrite 2 \"foo\"\n"
    },
    {.name = "uread",  .fn = (func)f_uread,
        .help = "Reads from uart\n"\
        "uread <uart> (<numchars>)\n"\
        HELP_UART_DEFS \
        "numchars - number of chars to read, if omitted uart is drained\n"\
        "ex: uread 2 10\n"
    },
    {.name = "uconnect",  .fn = (func)f_uconnect,
        .help = "Connects to another uart\n"\
        "uconnect <uart> (-via)\n"\
        HELP_UART_DEFS \
        "-via - vias data to original channel\n"\
        "Once connected, enter '+++' to disconnect\n"\
        "ex: uconnect 2\n"
    },
    {.name = "uconf",  .fn = (func)f_uconf,
        .help = "Configure uart\n"\
        "uconf <uart> <speed>\n"\
        HELP_UART_DEFS \
        "ex: uconf 2 9600\n"
    },

#ifdef CONFIG_LED
    {.name = "led",   .fn = (func)LED_set,
        .help = "Enable/disable leds\n"\
        "led <enable> <disable>\n"\
        "ex: led 0xffff 0x0000\n"
    },
    {.name = "ledb",   .fn = (func)LED_blink,
        .help = "blink leds\n"\
        "ledb <led> <cycle> <duty> <blinks>\n"\
        "ex: ledb 0xffff 16 8 100\n"
    },
#endif

#ifdef CONFIG_SPI
    {.name = "spifinit",     .fn = (func)f_spifinit,
        .help = "Init & open spi flash dev\n"
    },
    {.name = "spifpro",     .fn = (func)f_spifpro,
        .help = "(Un)protect spi flash dev\n" \
        "spifpro <0|~0> where zero unprotects and nonzero protects\n"
    },
    {.name = "spifrd",     .fn = (func)f_spifrd,
        .help = "Read spi flash dev\n"\
        "spifrd <addr> <len>\n"
    },
    {.name = "spifrdb",     .fn = (func)f_spifrdb,
        .help = "Read spi flash dev, output binary\n"\
        "spifrdb <addr> <len>\n"
    },
    {.name = "spifer",     .fn = (func)f_spifer,
        .help = "Erase spi flash dev\n"\
            "spifer <addr> <len>\n" \
            "NOTE: all sectors covered by address and length are erased\n"
    },
    {.name = "spifwr",     .fn = (func)f_spifwr,
        .help = "Write spi flash dev\n"\
        "spifwr <addr> <len>\n"
    },
    {.name = "spifcl",     .fn = (func)f_spifcl,
        .help = "Close spi flash dev\n" \
            "spifcl <0|~0> where zero closes gracefully and nonzero forces a close\n"
    },
    {.name = "spifbusy",     .fn = (func)f_spifbusy,
        .help = "Check spiflash busy flag\n"
    },
    {.name = "spifmaer",     .fn = (func)f_spifmaer,
        .help = "Spi flash dev mass erase\n"
    },
    {.name = "spifdump",     .fn = (func)f_spifdump,
        .help = "Dump spi flash dev stats\n"
    },
#ifdef CONFIG_ETHSPI
    {.name = "spiginit",     .fn = (func)f_spiginit,
        .help = "Init & open generic spi device\n"
    },
    {.name = "spigrx",     .fn = (func)f_spigrx,
        .help = "Read spi data\nex: spigrx 5\n"
    },
    {.name = "spigtx",     .fn = (func)f_spigtx,
        .help = "Send spi data\nex: spigtx \"0012aacc\"\n"
    },
    {.name = "spigtxrx",     .fn = (func)f_spigtxrx,
        .help = "Send and receive spi data\nex: spigtxrx \"0012aacc\" 5\n"
    },
    {.name = "spigcl",     .fn = (func)f_spigclose,
        .help = "Close generic spi device\n"
    },

    {.name = "ethinit",     .fn = (func)ETH_SPI_init,
        .help = "Initialize ENC28J60 spi eth device\n"
    },
    {.name = "ethup",     .fn = (func)ETH_SPI_start,
        .help = "Start eth thread\n"
    },
    {.name = "ethdown",     .fn = (func)ETH_SPI_stop,
        .help = "Stop eth thread\n"
    },
    {.name = "ethdump",     .fn = (func)ETH_SPI_dump,
        .help = "Dump ETH SPI state\n"
    },

    {.name = "ethregr",     .fn = (func)f_ethregr,
        .help = "Read ENC28J60 register\n"
    },
    {.name = "ethregw",     .fn = (func)f_ethregw,
        .help = "Write ENC28J60 register\n"
    },

#endif
#endif

    {.name = "rdnv",     .fn = (func)f_read_nvram,
        .help = "Read nv ram\n"
    },
    {.name = "wrnv",     .fn = (func)f_wr_nvram,
        .help = "Write nv ram\n"
    },

    {.name = "test",     .fn = (func)f_test,
        .help = "Test func\n"
    },

    {.name = "dbg",   .fn = (func)f_dbg,
        .help = "Set debug filter and level\n"\
        "dbg (level <dbg|info|warn|fatal>) (enable [x]*) (disable [x]*)\n"\
        "x - <task|heap|comm|cnc|console|nvs|spi|all>\n"\
        "ex: dbg level info disable all enable cnc comm\n"
    },
    {.name = "assert",  .fn = (func)f_assert,
        .help = "Asserts system\n"\
        "NOTE system will need to be rebooted\n"
    },
#if OS_DBG_MON
    {.name = "os_dump",  .fn = (func)OS_DBG_dump,
        .help = "Dumps state of os\n"
    },
#endif
    {.name = "reset",  .fn = (func)f_reset,
        .help = "Resets system\n"
    },
    {.name = "reset_boot",  .fn = (func)f_reset_boot,
        .help = "Resets system into bootloader\n"
    },
    {.name = "reset_fwupgrade",  .fn = (func)f_reset_fw_upgrade,
        .help = "Resets system into bootloader for upgrade\n"
    },
    {.name = "hardfault",  .fn = (func)f_hardfault,
        .help = "Generate hardfault, div by zero\n"
    },
    {.name = "help",  .fn = (func)f_help,
        .help = "Prints help\n"\
        "help or help <command>\n"
    },
    {.name = "?",     .fn = (func)f_help,
        .help = "Prints help\n"\
        "help or help <command>\n"
    },

    // menu end marker
    {.name = NULL,    .fn = (func)0,        .help = NULL},
};

#ifdef CONFIG_CNC

static int f_cnc_on() {
  CNC_set_enabled(TRUE);
  return 0;
}

static int f_cnc_off() {
  CNC_set_enabled(FALSE);
  return 0;
}

static int cur_feed = 140;
static int f_cnc_feed(int feed) {
  if (_argc == 0) {
    print("current feed: %i mm/min\n", cur_feed);
  } else if (_argc == 1) {
    cur_feed = feed;
  } else {
    return -1;
  }
  return 0;
}

static int f_cnc_move(int x, int y, int z) {
  if (_argc != 3) {
    return -1;
  }
  u32_t d2 = x*x + y*y + z*z;
  u32_t d = _sqrt(d2<<8);

  s32_t sx = x * CNC_STEPS_PER_MM_X;
  s32_t sy = y * CNC_STEPS_PER_MM_Y;
  s32_t sz = z * CNC_STEPS_PER_MM_Z;
  u32_t fx = ((cur_feed * ABS(x)) << (4+CNC_FP_DECIMALS)) / d;
  u32_t fy = ((cur_feed * ABS(y)) << (4+CNC_FP_DECIMALS)) / d;
  u32_t fz = ((cur_feed * ABS(z)) << (4+CNC_FP_DECIMALS)) / d;
  print ("d:%i.%02i x:%i y:%i z:%i fx:%08x fy:%08x fz:%08x tot_speed:%i\n", (d>>4),(100*(d&0xf))/16,x,y,z,fx,fy,fz,
      _sqrt((fx>>CNC_FP_DECIMALS)*(fx>>CNC_FP_DECIMALS)+
          (fy>>CNC_FP_DECIMALS)*(fy>>CNC_FP_DECIMALS)+
          (fz>>CNC_FP_DECIMALS)*(fz>>CNC_FP_DECIMALS)));
  u32_t res = CNC_latch_xyz(sx, fx, sy, fy, sz, fz, 0);
  if (!res) {
    print("CNC_latch failed, latch busy\n");
  }

  return 0;
}

static int f_cnc_xyz(int sx, int fx, int sy, int fy, int sz, int fz, int r) {
  if (_argc != 7) {
    return -1;
  }
  u32_t res = CNC_latch_xyz(sx, fx, sy, fy, sz, fz, r);
  if (!res) {
    print("CNC_latch failed, latch busy\n");
  }
  return 0;
}

static int f_cnc_xyz_imm(int sx, int fx, int sy, int fy, int sz, int fz) {
  if (_argc != 6) {
    return -1;
  }
  CNC_set_regs_imm(sx, fx, sy, fy, sz, fz);
  return 0;
}

static int f_cnc_pause(int pause) {
  if (_argc != 1) {
    return -1;
  }
  u32_t res = CNC_latch_pause(pause);
  if (!res) {
    print("CNC_latch failed, latch busy\n");
  }
  return 0;
}

static int f_cnc_pon() {
  CNC_pipeline_enable(TRUE);
  return 0;
}

static int f_cnc_poff() {
  CNC_pipeline_enable(FALSE);
  return 0;
}

static int f_cnc_pflush() {
  CNC_pipeline_flush();
  return 0;
}

static int f_cnc_srmask(u32_t sr_mask) {
  if (_argc != 1) {
    return -1;
  }
  CNC_set_status_mask(sr_mask);
  return 0;
}


static int f_cnc_status() {
  u32_t sr = CNC_get_status();
  print("b0 control enabled: %s\n", (sr & (1<<CNC_STATUS_BIT_CONTROL_ENABLED)) ? "on" : "off");
  print("b1 movement still : %s\n", (sr & (1<<CNC_STATUS_BIT_MOVEMENT_STILL)) ? "on" : "off");
  print("b2 movement paused: %s\n", (sr & (1<<CNC_STATUS_BIT_MOVEMENT_PAUSE)) ? "on" : "off");
  print("b3 movement rapid : %s\n", (sr & (1<<CNC_STATUS_BIT_MOVEMENT_RAPID)) ? "on" : "off");
  print("b4 pipeline active: %s\n", (sr & (1<<CNC_STATUS_BIT_PIPE_ACTIVE)) ? "on" : "off");
  print("b5 pipeline empty : %s\n", (sr & (1<<CNC_STATUS_BIT_PIPE_EMPTY)) ? "on" : "off");
  print("b6 pipeline full  : %s\n", (sr & (1<<CNC_STATUS_BIT_PIPE_FULL)) ? "on" : "off");
  print("b7 latch reg full : %s\n", (sr & (1<<CNC_STATUS_BIT_LATCH_FULL)) ? "on" : "off");
  return 0;
}

static int f_cnc_sr_recurrence(int delta) {
  if (_argc != 1) {
    return -1;
  } else {
    CNC_COMM_set_sr_timer_recurrence(delta);
    CNC_COMM_apply_sr_timer_recurrence();
  }
  return 0;
}

static int f_cnc_pos_recurrence(int delta) {
  if (_argc != 1) {
    return -1;
  } else {
    CNC_COMM_set_pos_timer_recurrence(delta);
    CNC_COMM_apply_pos_timer_recurrence();
  }
  return 0;
}

static int f_cnc_io() {
  if (_argc == 0) {
    u32_t v = CNC_GPIO_DEF_READ();
    print("dsdsdsds\nAAXXYYZZ\n");
    print("%c%c%c%c%c%c%c%c\n", v & CNC_GPIO_DIR_A ? '1':'0', v & CNC_GPIO_STEP_A ? '1':'0', v & CNC_GPIO_DIR_X ? '1':'0', v & CNC_GPIO_STEP_X ? '1':'0'
        , v & CNC_GPIO_DIR_Y ? '1':'0', v & CNC_GPIO_STEP_Y ? '1':'0', v & CNC_GPIO_DIR_Z ? '1':'0', v & CNC_GPIO_STEP_Z ? '1':'0');
    return 0;
  }
  if (CNC_get_status() & (1<<CNC_STATUS_BIT_CONTROL_ENABLED)) {
    print("WARNING: disable cnc control first\n");
    return 0;
  }
  int i;
  u32_t pinmask = 0;
  for (i = 0; i < _argc; i++) {
    char *s = (char*)_args[i];
    if (IS_STRING(s)) {
      if (strcmp("da", s) == 0) {
        pinmask |= CNC_GPIO_DIR_A;
      }
      else if (strcmp("sa", s) == 0) {
        pinmask |= CNC_GPIO_STEP_A;
      }
      else if (strcmp("dx", s) == 0) {
        pinmask |= CNC_GPIO_DIR_X;
      }
      else if (strcmp("sx", s) == 0) {
        pinmask |= CNC_GPIO_STEP_X;
      }
      else if (strcmp("dy", s) == 0) {
        pinmask |= CNC_GPIO_DIR_Y;
      }
      else if (strcmp("sy", s) == 0) {
        pinmask |= CNC_GPIO_STEP_Y;
      }
      else if (strcmp("dz", s) == 0) {
        pinmask |= CNC_GPIO_DIR_Z;
      }
      else if (strcmp("sz", s) == 0) {
        pinmask |= CNC_GPIO_STEP_Z;
      } else if (strcmp("flip", s) == 0) {
        if (_argc < i + 2) {
          return -1;
        }
        u32_t flips = (u32_t)_args[++i];
        u32_t hz = (u32_t)_args[++i];
        u32_t delta = 1000 / hz;
        print("flipping pins %i times, deltatime %i ms...", flips, delta);
        while (flips-- > 0) {
          CNC_GPIO_DEF(0, pinmask);
          SYS_hardsleep_ms(delta/2);
          CNC_GPIO_DEF(pinmask, 0);
          SYS_hardsleep_ms(delta/2);
        }
        print (" done\n");
      } else {
        return -1;
      }
    } else {
      if ((int)_args[i] == 0) {
        CNC_GPIO_DEF(0, pinmask);
        pinmask = 0;
      } else if ((int)_args[i] == 1) {
        CNC_GPIO_DEF(pinmask, 0);
        pinmask = 0;
      } else {
        return -1;
      }
    }
  }
  return 0;
}

static int f_cnc_err_off(int err) {
  CNC_disable_error(err);
  return 0;
}

static int f_cnc_err_on(int err) {
  CNC_enable_error(err);
  return 0;
}

#endif // CONFIG_CNC

static int f_comm_send(int dst, char* data, int ack) {
  if (_argc != 3 || !IS_STRING(data)) {
    return -1;
  }
  s32_t res = COMM_tx(dst, (u8_t*)data, strlen(data), ack);
  if (res < R_COMM_OK) {
    print("COMM ERROR: %i\n", res);
  } else {
    print("Sent, seqno:0x%03x\n", res);
  }
  return 0;
}

static int f_comm_uart(int uart) {
  if (IS_STRING(uart) || uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  COMM_set_uart(_UART(uart));
  return 0;
}

static int f_reset() {
  SYS_reboot(REBOOT_USER);
  return 0;
}

static int f_reset_boot() {
  SYS_reboot(REBOOT_EXEC_BOOTLOADER);
  return 0;
}

static int f_reset_fw_upgrade() {
  bootloader_update_fw();
  SYS_reboot(REBOOT_EXEC_BOOTLOADER);
  return 0;
}

static int f_time(int ad, int ah, int am, int as, int ams) {
  if (_argc == 0) {
    u16_t d, ms; u8_t h, m, s;
    SYS_get_time(&d, &h, &m, &s, &ms);
    print("day:%i time:%02i:%02i:%02i.%03i\n", d,h,m,s,ms);
  } else if (_argc == 5) {
    SYS_set_time(ad, ah, am, as, ams);
  } else {
    return -1;
  }
  return 0;
}

#ifdef DBG_OFF
static int f_dbg() {
  print("Debug disabled compile-time\n");
  return 0;
}
#else
const char* DBG_BIT_NAME[] = _DBG_BIT_NAMES;

static void print_debug_setting() {
  print("DBG level: %i\n", SYS_dbg_get_level());
  int d;
  for (d = 0; d < sizeof(DBG_BIT_NAME)/sizeof(const char*); d++) {
    print("DBG mask %s: %s\n", DBG_BIT_NAME[d], SYS_dbg_get_mask() & (1<<d) ? "ON":"OFF");
  }
}

static int f_dbg() {
  enum state {NONE, LEVEL, ENABLE, DISABLE} st = NONE;
  int a;
  if (_argc == 0) {
    print_debug_setting();
    return 0;
  }
  for (a = 0; a < _argc; a++) {
    u32_t f = 0;
    char *s = (char*)_args[a];
    if (!IS_STRING(s)) {
      return -1;
    }
    if (strcmp("level", s) == 0) {
      st = LEVEL;
    } else if (strcmp("enable", s) == 0) {
      st = ENABLE;
    } else if (strcmp("disable", s) == 0) {
      st = DISABLE;
    } else {
      switch (st) {
      case LEVEL:
        if (strcmp("dbg", s) == 0) {
          SYS_dbg_level(D_DEBUG);
        }
        else if (strcmp("info", s) == 0) {
          SYS_dbg_level(D_INFO);
        }
        else if (strcmp("warn", s) == 0) {
          SYS_dbg_level(D_WARN);
        }
        else if (strcmp("fatal", s) == 0) {
          SYS_dbg_level(D_FATAL);
        } else {
          return -1;
        }
        break;
      case ENABLE:
      case DISABLE: {
        int d;
        for (d = 0; f == 0 && d < sizeof(DBG_BIT_NAME)/sizeof(const char*); d++) {
          if (strcmp(DBG_BIT_NAME[d], s) == 0) {
            f = (1<<d);
          }
        }
        if (strcmp("all", s) == 0) {
          f = D_ANY;
        }
        if (f == 0) {
          return -1;
        }
        if (st == ENABLE) {
          SYS_dbg_mask_enable(f);
        } else {
          SYS_dbg_mask_disable(f);
        }
        break;
      }
      default:
        return -1;
      }
    }
  }
  CONFIG_store();
  print_debug_setting();
  return 0;
}
#endif

static int f_assert() {
  ASSERT(FALSE);
  return 0;
}

static int f_uwrite(int uart, char* data) {
  if (_argc != 2 || !IS_STRING(data)) {
    return -1;
  }
  if (uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  char c;
  while ((c = *data++) != 0) {
    UART_put_char(_UART(uart), c);
  }
  return 0;
}

static int f_uread(int uart, int numchars) {
  if (_argc < 1  || _argc > 2) {
    return -1;
  }
  if (uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  if (_argc == 1) {
    numchars = 0x7fffffff;
  }
  int l = UART_rx_available(_UART(uart));
  l = MIN(l, numchars);
  int ix = 0;
  while (ix++ < l) {
    print("%c", UART_get_char(_UART(uart)));
  }
  print("\n%i bytes read\n", l);
  return 0;
}

static int f_uconnect(int uart) {
  if (_argc < 1) {
    return -1;
  }
  if (IS_STRING(uart) || uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }

  if (uart == STDIN) {
    print("Cannot pipe STDIN channel\n");
    return 0;
  }

  int i;
  char via = FALSE;
  for (i = 1; i < _argc; i++) {
    if (strcmp("-via", _args[i]) == 0) {
      via = TRUE;
      break;
    }
  }

  console_state.uart_pipe_via = via;

  print("\nPiping uart %i, bash '+++' to exit\n", uart);

  console_state.uart_pipe = _UART(uart);
  UART_get_callback(_UART(uart),
      &console_state.prev_uart_rx_f, &console_state.prev_uart_arg);
  UART_set_callback(_UART(uart),DBG_uart_pipe_irq,NULL);

  return 0;
}

static int f_uconf(int uart, int speed) {
  if (_argc != 2) {
    return -1;
  }
  if (IS_STRING(uart) || IS_STRING(speed) || uart < 0 || uart >= CONFIG_UART_CNT) {
    return -1;
  }
  USART_TypeDef *uart_hw = _UART(uart)->hw;

  USART_Cmd(uart_hw, DISABLE);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = speed;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART */
  USART_Init(uart_hw, &USART_InitStructure);

  USART_Cmd(uart_hw, ENABLE);

  return 0;
}

static int f_udisconnect() {
  if (console_state.uart_pipe == 0) {
    return -1;
  }
  UART_set_callback(console_state.uart_pipe,
      console_state.prev_uart_rx_f, console_state.prev_uart_arg);
  console_state.uart_pipe = 0;
  print("\nUART pipe disconnected\n");
  print(CONSOLE_PROMPT);
  return 0;
}

#ifdef CONFIG_SPI
static u8_t *spi_buf = 0;
static u16_t spi_buflen;
static bool binary = FALSE;

static void test_spi_finished(spi_flash_dev *dev, int res) {
  print("spif callback, res %i\n", res);
  if (spi_buf) {
    if (binary) {
      binary = FALSE;
      print("\nZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
      int i;
      for (i = 0; i < spi_buflen; i++) {
        print("%c", spi_buf[i]);
      }
      print("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ\n");
    } else {
      int i;
      for (i = 0; i < spi_buflen; i++) {
        print("%02x", spi_buf[i]);
      }
      print("\n");
    }
    HEAP_free(spi_buf);
    spi_buf = 0;
  }
}

static int f_spifinit() {
  int res;
  print("spif init\n");
  SPI_FLASH_M25P16_app_init();
  print("spif open..");
  res = SPI_FLASH_open(SPI_FLASH, test_spi_finished);
  print (" res %i\n", res);
  return 0;
}

static int f_spifpro(int arg) {
  if (_argc != 1) {
    return -1;
  }
  int res;
  if (arg) {
    print("spif protect..");
    res = SPI_FLASH_protect(SPI_FLASH, test_spi_finished);
  } else {
    print("spif unprotect..");
    res = SPI_FLASH_unprotect(SPI_FLASH, test_spi_finished);
  }
  print (" res %i\n", res);
  return 0;
}

static int f_spifrd(int addr, int len) {
  if (_argc != 2) {
    return -1;
  }
  spi_buflen = len;
  spi_buf = HEAP_malloc(spi_buflen);
  memset(spi_buf, 0xee, spi_buflen);
  print("spif read..");
  int res = SPI_FLASH_read(SPI_FLASH, test_spi_finished, addr, len, spi_buf);
  print (" res %i\n", res);
  return 0;
}

static int f_spifrdb(void *addr_v, int len) {
  if (_argc != 2) {
    return -1;
  }
  u32_t addr;
  if (IS_STRING(addr_v) && strcmp("FW_META", addr_v) == 0) {
    addr = FIRMWARE_SPIF_ADDRESS_META;
  } else if (IS_STRING(addr_v) && strcmp("FW_DATA", addr_v) == 0) {
      addr = FIRMWARE_SPIF_ADDRESS_DATA;
  } else {
    addr = (u32_t)addr_v;
  }
  binary = TRUE;
  spi_buflen = len;
  spi_buf = HEAP_malloc(spi_buflen);
  memset(spi_buf, 0xee, spi_buflen);
  print("spif read binary %08x..", addr);
  int res = SPI_FLASH_read(SPI_FLASH, test_spi_finished, addr, len, spi_buf);
  print (" res %i\n", res);
  return 0;
}

static int f_spifer(int addr, int len) {
  if (_argc != 2) {
    return -1;
  }
  print("spif erase..");
  int res = SPI_FLASH_erase(SPI_FLASH, test_spi_finished, addr, len);
  print (" res %i\n", res);
  return 0;
}

static int f_spifmaer() {
  if (_argc != 0) {
    return -1;
  }
  print("spif mass erase..");
  int res = SPI_FLASH_mass_erase(SPI_FLASH, test_spi_finished);
  print (" res %i\n", res);
  return 0;
}

static int f_spifwr(int addr, int len) {
  if (_argc != 2) {
    return -1;
  }
  int res;
  spi_buflen = len;
  spi_buf = HEAP_malloc(spi_buflen);
  int i;
  for (i = 0; i < len; i++) {
    if (i == 0) {
      spi_buf[i] = 1;
    } else if (i == 1) {
      spi_buf[i] = 1;
    } else {
      spi_buf[i] = spi_buf[i-1] + spi_buf[i-2];
    }
  }
  print("spif write..");
  res = SPI_FLASH_write(SPI_FLASH, test_spi_finished, addr, len, spi_buf);
  print (" res %i\n", res);
  return 0;

}

static int f_spifcl(int forced) {
  if (_argc != 1) {
    return -1;
  }
  int res;
  print("spif close %s..", forced ? "forced" : "gracefully");
  if(forced) {
    res = SPI_FLASH_close_force(SPI_FLASH);
  } else {
    res = SPI_FLASH_close(SPI_FLASH, test_spi_finished);
  }
  print (" res %i\n", res);
  return 0;
}

static int f_spifbusy() {
  if (_argc != 0) {
    return -1;
  }
  int res;
  spi_buflen = 1;
  spi_buf = HEAP_malloc(spi_buflen);
  print("spif busy..");
  res = SPI_FLASH_read_busy(SPI_FLASH, test_spi_finished, spi_buf);
  print (" res %i\n", res);
  return 0;
}

static int f_spifdump() {
  SPI_FLASH_dump(SPI_FLASH);
  return 0;
}

#ifdef CONFIG_ETHSPI

static spi_dev_gen spi_gdev;

static int f_spiginit() {
  print("spigen init & open\n");
  SPI_DEV_GEN_init(
      &spi_gdev,
      SPIDEV_CONFIG_CPHA_2E | SPIDEV_CONFIG_CPOL_HI | SPIDEV_CONFIG_FBIT_MSB | SPIDEV_CONFIG_SPEED_36M,
      _SPI_BUS(0),
      SPI_ETH_GPIO_PORT, SPI_ETH_GPIO_PIN);
      //SPI_FLASH_GPIO_PORT, SPI_FLASH_GPIO_PIN);
  print("spigen open\n");
  SPI_DEV_GEN_open(&spi_gdev);
  return 0;
}

static int f_spigrx(int num) {
  int res;
  int i;
  u8_t * buf = HEAP_malloc(num);
  res = SPI_DEV_GEN_txrx(&spi_gdev, NULL, 0, buf, num);
  print("spigen read %i bytes, res %i\nrx: ", res);
  for (i = 0; i < num; i++) {
    print("%02x ", buf[i]);
  }
  print("\n");
  HEAP_free(buf);
  return 0;
}

static void parse_hex(char *s, u8_t* buf, int *len) {
  int i = 0;
  u8_t d = 0;
  u8_t c;
  while ((c = s[i])) {
    if (c >= '0' && c <= '9') {
      d |= c - '0';
    } else if (c >= 'a' && c <= 'f') {
      d |= c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      d |= c - 'A' + 10;
    } else {
      *len = -1;
      return;
    }
    if (i & 1) {
      buf[i/2] = d;
      d = 0;
    } else {
      d <<= 4;
    }
    i++;
  }
  *len = i / 2;
}

static int f_spigtx(char *tx_str) {
  if (!IS_STRING(tx_str)) {
    print("tx data ex: \"cafeb00b\"\n");
    return -1;
  }

  u8_t * buf = HEAP_malloc(32);
  int tx_len;
  parse_hex(tx_str, buf, &tx_len);
  if (tx_len < 0) {
    HEAP_free(buf);
    return -1;
  }
  int res = SPI_DEV_GEN_txrx(&spi_gdev, buf, tx_len, NULL, 0);
  print("spigen write %i bytes, res %i\n", tx_len, res);
  HEAP_free(buf);
  return 0;

}

static int f_spigtxrx(char *tx_str, int rx_len) {
  int i;
  if (!IS_STRING(tx_str)) {
    print("tx data ex: \"cafeb00b\"\n");
    return -1;
  }

  u8_t * tx_buf = HEAP_malloc(32);
  int tx_len;
  parse_hex(tx_str, tx_buf, &tx_len);
  if (tx_len < 0) {
    HEAP_free(tx_buf);
    return -1;
  }
  u8_t * rx_buf = HEAP_malloc(32);
  int res = SPI_DEV_GEN_txrx(&spi_gdev, tx_buf, tx_len, rx_buf, rx_len);
  print("spigen txrx tx:%i bytes, res %i\nrx: ", tx_len, res);
  HEAP_free(tx_buf);
  for (i = 0; i < rx_len; i++) {
    print("%02x ", rx_buf[i]);
  }
  print("\n");

  HEAP_free(rx_buf);
  return 0;
}

static int f_spigclose() {
  print("spigen close\n");
  SPI_DEV_GEN_close(&spi_gdev);
  return 0;
}

static int f_ethregr(int reg) {
  int data =  enc28j60Read(reg);
  print("reg %02x = %02x\n", reg, data);
  return 0;
}
static int f_ethregw(int reg, int data) {
  enc28j60Write(reg, data);
  int rdata =  enc28j60Read(reg);
  print("reg %02x = %02x : %02x\n", reg, data, rdata);
  return 0;
}


#endif // CONFIG_ETHSPI

#endif // CONFIG_SPI

static int f_help(char *s) {
  if (IS_STRING(s)) {
    int i = 0;
    while (c_tbl[i].name != NULL) {
      if (strcmp(s, c_tbl[i].name) == 0) {
        print("%s\t%s", c_tbl[i].name, c_tbl[i].help);
        return 0;
      }
      i++;
    }
    print("%s\tno such command\n",s);
  } else {
    print("  CNC CONTROL\n");
    print("  ===========\n");
    int i = 0;
    while (c_tbl[i].name != NULL) {
      int len = strpbrk(c_tbl[i].help, "\n") - c_tbl[i].help;
      char tmp[64];
      strncpy(tmp, c_tbl[i].help, len+1);
      tmp[len+1] = 0;
      char fill[24];
      int fill_len = sizeof(fill)-strlen(c_tbl[i].name);
      memset(fill, ' ', sizeof(fill));
      fill[fill_len] = 0;
      print("  %s%s%s", c_tbl[i].name, fill, tmp);
      i++;
    }
  }
  return 0;
}

static int f_read_nvram() {
  int a;
  u32_t d;
  for (a = 0; a < 42; a++) {
    s32_t res = NVS_read(NV_RAM, a, &d);
    print("addr %02x data:%08x res:%i\n", a, d, res);
  }
  return 0;
}

static int f_wr_nvram(int a, int d) {
  s32_t res;
  res = NVS_write(NV_RAM, a, d);
  print("wr nvram addr %02x data:%08x res:%i\n", a, d, res);
  return 0;
}

static int f_dump() {
  print("FULL DUMP\n=========\n");
#ifdef CONFIG_CNC
  CNC_dump();
  print("\n");
  f_cnc_status();
  print("\n");
#endif
  COMM_dump();
  print("\n");
  HEAP_dump();
  print("\n");
  TASK_dump();
  print("\n");
#ifdef CONFIG_ETHSPI
  ETH_SPI_dump();
  print("\n");
#endif
#if OS_DBG_MON
  OS_DBG_dump();
  print("\n");
#endif
  print("=========\n");
  return 0;
}

void DBG_TASK_on_piped_output(u32_t len, void *p) {
  while (UART_rx_available(console_state.uart_pipe) > 0) {
    UART_put_char(_UART(STDOUT), UART_get_char(console_state.uart_pipe));
  }
}

static void DBG_TASK_on_piped_input(u32_t len, void *p) {
  u32_t rlen = UART_get_buf(_UART(STDIN), in, MIN(len, sizeof(in)));
  int i = 0;
  for (i = 0; i < rlen; i++) {
    if (in[i] == '+') {
      console_state.uart_pipe_plusses++;
      if (console_state.uart_pipe_plusses > 2) {
        f_udisconnect();
        return;
      }
    } else {
      console_state.uart_pipe_plusses = 0;
    }
  }
  UART_put_buf(console_state.uart_pipe, in, rlen);
}

void DBG_TASK_on_input(u32_t len, void *p) {
  if (console_state.uart_pipe != 0) {
    DBG_TASK_on_piped_input(len, p);
    return;
  }
  if (len > sizeof(in)) {
    DBG(D_CONSOLE, D_WARN, "CONS input overflow\n");
    print(CONSOLE_PROMPT);
    return;
  }
  u32_t rlen = UART_get_buf(_UART(STDIN), in, MIN(len, sizeof(in)));
  if (rlen != len) {
    DBG(D_CONSOLE, D_WARN, "CONS uart mismatch\n");
    print(CONSOLE_PROMPT);
    return;
  }
  cursor cursor;
  strarg_init(&cursor, (char*)in, rlen);
  strarg arg;
  _argc = 0;
  func fn = NULL;
  int ix = 0;

  // parse command and args
  while (strarg_next(&cursor, &arg)) {
    if (arg.type == INT) {
      //DBG(D_CONSOLE, D_DEBUG, "CONS arg %i:\tlen:%i\tint:%i\n",arg_c, arg.len, arg.val);
    } else if (arg.type == STR) {
      //DBG(D_CONSOLE, D_DEBUG, "CONS arg %i:\tlen:%i\tstr:\"%s\"\n", arg_c, arg.len, arg.str);
    }
    if (_argc == 0) {
      // first argument, look for command function
      if (arg.type != STR) {
        break;
      } else {
        while (c_tbl[ix].name != NULL) {
          if (strcmp(arg.str, c_tbl[ix].name) == 0) {
            fn = c_tbl[ix].fn;
            break;
          }
          ix++;
        }
        if (fn == NULL) {
          break;
        }
      }
    } else {
      // succeeding arguments¸ store them in global vector
      if (_argc-1 >= 16) {
        DBG(D_CONSOLE, D_WARN, "CONS too many args\n");
        fn = NULL;
        break;
      }
      _args[_argc-1] = (void*)arg.val;
    }
    _argc++;
  }

  // execute command
  if (fn) {
    _argc--;
    DBG(D_CONSOLE, D_DEBUG, "CONS calling [%p] with %i args\n", fn, _argc);
    int res = (int)_variadic_call(fn, _argc, _args);
    if (res == -1) {
      print("%s", c_tbl[ix].help);
    } else {
      print("OK\n");
    }
  } else {
    print("? - try help\n");
  }
  print(CONSOLE_PROMPT);
}


void DBG_timer() {
}

void DBG_uart_check_char(void *a, u8_t c) {
  if (c == '\n') {
    task *t = TASK_create(DBG_TASK_on_input, 0);
    TASK_run(t, UART_rx_available(_UART(STDIN)), NULL);
  }
}

void DBG_uart_pipe_irq(void *a, u8_t c) {
  task *t = TASK_create(DBG_TASK_on_piped_output, 0);
  TASK_run(t, UART_rx_available(console_state.uart_pipe), NULL);
  if (console_state.uart_pipe_via && console_state.prev_uart_rx_f) {
    console_state.prev_uart_rx_f(console_state.prev_uart_arg, c);
  }
}

void DBG_init() {
  memset(&console_state, 0, sizeof(console_state));
  DBG(D_CONSOLE, D_DEBUG, "CONSOLE init\n");
  UART_set_callback(_UART(STDIN), DBG_uart_check_char, NULL);
  print("\n"APP_NAME"\n");
  print("version   : %02x.%02x.%04x\n", (CNC_COMM_VERSION>>24), (CNC_COMM_VERSION>>16), CNC_COMM_VERSION & 0xffff);
  print("build     : %i\n", SYS_build_number());
  print("build date: %i\n", SYS_build_date());
  print("\ntype '?' or 'help' for list of commands\n\n");
  print(CONSOLE_PROMPT);
}



