#include "stm32f10x.h"
#include "system.h"
#include "uart.h"
#include "timer.h"
#include "comm_proto_sys.h"
#include "comm_proto_cnc.h"
#include "cnc_control.h"
#include "comm_impl.h"
#include "miniutils.h"
#include "taskq.h"
#include "heap.h"
#include "cli.h"
#include "processor.h"
#include "nvstorage.h"
#include "spi_driver.h"
#include "adc.h"
#include "led.h"
#include "os.h"
#include "linker_symaccess.h"
#include "bl_exec.h"
#include "spi_flash_m25p16.h"
#include "spi_flash_os.h"
#include "comm_proto_file.h"
#include "enc28j60_spi_eth.h"
#include "spiffs_wrapper.h"
#include "i2c_driver.h"
#include "eval.h"

os_thread kernel_thread;

//#define SPIFFS_TEST_THR

#ifdef CONFIG_SPIFFS
#ifdef SPIFFS_TEST_THR
os_thread spiffs_test_thr;
u32_t spiffs_test_stack[0x200];
static void *spiffs_test_thr_func(void *a) {
  while (TRUE) {
    OS_thread_sleep(1*1000);
    int c = 0;
    char b[64];
    spiffs_file fd = SPIFFS_open(FS_get_filesystem(), "count", SPIFFS_RDWR, 0);
    if (fd > 0) {
      SPIFFS_read(
          FS_get_filesystem(),
          fd,
          b,
          64);
      c = atoin(b, 10, strlen(b));
      SPIFFS_fremove(FS_get_filesystem(), fd);
      SPIFFS_close(FS_get_filesystem(), fd);
      //print("spiffs: read %i\n", c);
    }
    sprint(b, "%i%c", c+1,0);
    fd = SPIFFS_open( FS_get_filesystem(), "count", SPIFFS_RDWR | SPIFFS_CREAT | SPIFFS_TRUNC, 0);
    if (fd > 0) {
      SPIFFS_write(FS_get_filesystem(),
          fd,
          b,
          strlen(b)+1);
    }
    SPIFFS_close(FS_get_filesystem(), fd);
    //print("spiffs: wrote %i\n", c+1);
  }
  return NULL;
}
#endif
#endif

#ifdef DBG_OS_THREAD_BLINKY
os_thread dbg_blinky_thread;
u32_t dbg_blinky_stack[0x1f];
static void *dbg_blinky_thread_func(void *a) {
  while (TRUE) {
    GPIO_disable(GPIOC, GPIO_Pin_7);
    OS_thread_sleep(990);
    GPIO_enable(GPIOC, GPIO_Pin_7);
    OS_thread_sleep(10);
  }
  return NULL;
}
#endif

#ifdef DBG_KERNEL_TASK_BLINKY
task_timer dbg_blinky_task_timer;
task *dbg_blinky_task;
static bool _dbg_bl_state = TRUE;
static void dbg_blinky_task_func(u32_t i, void *p) {
  if (_dbg_bl_state) {
    GPIO_disable(GPIOC, GPIO_Pin_6);
    TASK_set_timer_recurrence(&dbg_blinky_task_timer, 10);
    _dbg_bl_state = FALSE;
  } else {
    GPIO_enable(GPIOC, GPIO_Pin_6);
    TASK_set_timer_recurrence(&dbg_blinky_task_timer, 990);
    _dbg_bl_state = TRUE;
  }
}
#endif

// main thread loop

static void *kernel_func(void *a) {
  print(TEXT_NOTE("Kernel running...\n"));

  // init comm stack and connect to phy
  COMM_UART_init(_UART(COMMIN));
  COMM_UDP_init();
  COMM_init();
  // TODO PETER COMM_set_stack(COMM_UART_get_comm(), 0);
  COMM_set_stack(COMM_UDP_get_comm(), COMM_UDP_beacon_handler);
  COMM_SYS_init();
  COMM_FILE_init();
#ifdef CONFIG_CNC
  COMM_CNC_init();
#endif

#ifdef CONFIG_ETHSPI
  ETH_SPI_init();
  ETH_SPI_start();
#endif

  SFOS_init();

#ifdef CONFIG_SPIFFS
  FS_sys_init();
#endif

#ifdef DBG_KERNEL_TASK_BLINKY
  dbg_blinky_task = TASK_create(dbg_blinky_task_func, TASK_STATIC);
  TASK_start_timer(dbg_blinky_task, &dbg_blinky_task_timer, 0,0,0,950,"dbg_blink");
#endif

  // start blinky thread
#ifdef DBG_OS_THREAD_BLINKY
  OS_thread_create(
      &dbg_blinky_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      dbg_blinky_thread_func,
      0,
      dbg_blinky_stack,
      sizeof(dbg_blinky_stack)-4,
      "dbg_blink");
#endif

#ifdef CONFIG_SPIFFS
#ifdef SPIFFS_TEST_THR
  OS_thread_create(
      &spiffs_test_thr,
      OS_THREAD_FLAG_PRIVILEGED,
      spiffs_test_thr_func,
      0,
      spiffs_test_stack,
      sizeof(spiffs_test_stack)-4,
      "test_spiffs");
#endif
#endif

  while (1) {
    while (TASK_tick());
    TASK_wait();
  }
  return 0;
}

static void main_spi_cb(spi_flash_dev *dev, int result) {
  print("spi flash open cb res:%i\n", result);
}

// main entry from bootstrap

int main(void) {
  s32_t res;
  __disable_irq();
  PROC_base_init();
  SYS_init();
  UART_init();
  UART_assure_tx(_UART(STDOUT), TRUE);
#ifdef CONFIG_SPI
  SPI_init();
#endif
#ifdef CONFIG_LED
  LED_SHIFT_init(LED_SHIFT_REG_SIZE);
  LED_init(LED_TIMER_DIVISOR);
#endif
  PROC_periph_init();
#ifdef CONFIG_ADC
  ADC_init();
#endif
#ifdef CONFIG_I2C
  I2C_init();
#endif
#ifdef CONFIG_USB_CDC
  USB_Init();
#endif

  __enable_irq();
  print("\n\n\nHardware initialization done\n");

  print("Shared memory on 0x%08x\n", SHARED_MEMORY_ADDRESS);
  bool shmem_resetted = SHMEM_validate();
  if (!shmem_resetted) {
    print("Shared memory reset\n");
  }
  enum reboot_reason_e rr = SHMEM_get()->reboot_reason;
  SHMEM_set_reboot_reason(REBOOT_UNKONWN);
  print("Reboot reason: %i\n", rr);
  if (rr == REBOOT_EXEC_BOOTLOADER) {
    print("Peripheral init for bootloader\n");
    PROC_periph_init_bootloader();
    print("Bootloader execute\n");
    bootloader_execute();
  }

  print("Non-volatile settings initialization...\n");
  NVS_init();
  res = CONFIG_load();
#ifdef CONFIG_CNC
  if (res != NV_OK) {
    CNC_enable_error(1<<CNC_ERROR_BIT_SETTINGS_CORRUPT);
  }
#endif
  print("Non-volatile settings read, res %i\n", res);

  print("Subsystem initialization...\n");

#ifdef CONFIG_LED
  LED_blink(0xffffffff, 1, 0, 0);
#endif
  HEAP_init();
  print("Stack 0x%08x -- 0x%08x\n", STACK_START, STACK_END);

  print("Subsystem initialization done\n");

  OS_init();
  TASK_init();

#ifdef CONFIG_SPI
  print("spif init\n");
  SPI_FLASH_M25P16_app_init();
  res = SPI_FLASH_open(SPI_FLASH, main_spi_cb);
  print("spif open res %i\n", res);

#endif

  #define KERNEL_STACK_EXTRA 0x800

  print("Main thread stack size: %i bytes\n", __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA);

  eval_init();

  CLI_init();

#ifdef CONFIG_ADC
  {
    int i = 16;
    uint32_t s = 0;
    while (i--) {
      s ^= (ADC_sample() << (i*2));
    }
    rand_seed(s);
  }
#else
  rand_seed(0xd0decaed ^ SYS_get_tick());
#endif

  OS_thread_create(
      &kernel_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      kernel_func,
      0,
      (void *)(STACK_START+4), __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA,
      "kernel");


  while(1) {
    __WFI();
  }

  return 0;
}

// assert failed handler from stmlib? TODO

void assert_failed(uint8_t* file, uint32_t line) {
  SYS_assert((char*)file, (s32_t)line);
}

// user hardfault handler

#if USER_HARDFAULT

void **HARDFAULT_PSP;
//register void *stack_pointer asm("sp");
volatile void *stack_pointer;
volatile unsigned int stacked_r0;
volatile unsigned int stacked_r1;
volatile unsigned int stacked_r2;
volatile unsigned int stacked_r3;
volatile unsigned int stacked_r12;
volatile unsigned int stacked_lr;
volatile unsigned int stacked_pc;
volatile unsigned int stacked_psr;

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  SHMEM_set_reboot_reason(REBOOT_CRASH);

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  u32_t bfar = SCB->BFAR;
  u32_t cfsr = SCB->CFSR;
  u32_t hfsr = SCB->HFSR;
  u32_t dfsr = SCB->DFSR;
  u32_t afsr = SCB->AFSR;

  // Hijack the process stack pointer to make backtrace work
  asm("mrs %0, psp" : "=r"(HARDFAULT_PSP) : :);
  stack_pointer = HARDFAULT_PSP;

  UART_sync_tx(_UART(STDOUT), TRUE);

  UART_tx_flush(_UART(STDOUT));

  print(TEXT_BAD("\n!!! HARDFAULT !!!\n\n"));
  print("Stacked registers:\n");
  print("  pc:   0x%08x\n", stacked_pc);
  print("  lr:   0x%08x\n", stacked_lr);
  print("  psr:  0x%08x\n", stacked_psr);
  print("  sp:   0x%08x\n", stack_pointer);
  print("  r0:   0x%08x\n", stacked_r0);
  print("  r1:   0x%08x\n", stacked_r1);
  print("  r2:   0x%08x\n", stacked_r2);
  print("  r3:   0x%08x\n", stacked_r3);
  print("  r12:  0x%08x\n", stacked_r12);
  print("\nFault status registers:\n");
  print("  BFAR: 0x%08x\n", bfar);
  print("  CFSR: 0x%08x\n", cfsr);
  print("  HFSR: 0x%08x\n", hfsr);
  print("  DFSR: 0x%08x\n", dfsr);
  print("  AFSR: 0x%08x\n", afsr);
  print("\n");
  if (cfsr & (1<<(7+0))) {
    print("MMARVALID: MemMan 0x%08x\n", SCB->MMFAR);
  }
  if (cfsr & (1<<(4+0))) {
    print("MSTKERR: MemMan error during stacking\n");
  }
  if (cfsr & (1<<(3+0))) {
    print("MUNSTKERR: MemMan error during unstacking\n");
  }
  if (cfsr & (1<<(1+0))) {
    print("DACCVIOL: MemMan memory access violation, data\n");
  }
  if (cfsr & (1<<(0+0))) {
    print("IACCVIOL: MemMan memory access violation, instr\n");
  }

  if (cfsr & (1<<(7+8))) {
    print("BFARVALID: BusFlt 0x%08x\n", SCB->BFAR);
  }
  if (cfsr & (1<<(4+8))) {
    print("STKERR: BusFlt error during stacking\n");
  }
  if (cfsr & (1<<(3+8))) {
    print("UNSTKERR: BusFlt error during unstacking\n");
  }
  if (cfsr & (1<<(2+8))) {
    print("IMPRECISERR: BusFlt error during data access\n");
  }
  if (cfsr & (1<<(1+8))) {
    print("PRECISERR: BusFlt error during data access\n");
  }
  if (cfsr & (1<<(0+8))) {
    print("IBUSERR: BusFlt bus error\n");
  }

  if (cfsr & (1<<(9+16))) {
    print("DIVBYZERO: UsaFlt division by zero\n");
  }
  if (cfsr & (1<<(8+16))) {
    print("UNALIGNED: UsaFlt unaligned access\n");
  }
  if (cfsr & (1<<(3+16))) {
    print("NOCP: UsaFlt execute coprocessor instr\n");
  }
  if (cfsr & (1<<(2+16))) {
    print("INVPC: UsaFlt general\n");
  }
  if (cfsr & (1<<(1+16))) {
    print("INVSTATE: UsaFlt execute ARM instr\n");
  }
  if (cfsr & (1<<(0+16))) {
    print("UNDEFINSTR: UsaFlt execute bad instr\n");
  }

  if (hfsr & (1<<31)) {
    print("DEBUGEVF: HardFlt debug event\n");
  }
  if (hfsr & (1<<30)) {
    print("FORCED: HardFlt SVC/BKPT within SVC\n");
  }
  if (hfsr & (1<<1)) {
    print("VECTBL: HardFlt vector fetch failed\n");
  }

  SYS_dump_trace();

  while(1);
}
#endif
