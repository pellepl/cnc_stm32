#include "stm32f10x.h"
#include "system.h"
#include "uart.h"
#include "timer.h"
#include "cnc_comm.h"
#include "cnc_control.h"
#include "comm_impl.h"
#include "miniutils.h"
#include "taskq.h"
#include "heap.h"
#include "console.h"
#include "processor.h"
#include "nvstorage.h"
#include "spi_driver.h"
#include "led.h"
#include "os.h"
#include "linker_symaccess.h"
#include "bl_exec.h"
#include "spi_flash_m25p16.h"
#include "comm_file.h"

os_thread main_thread;

static void *main_thread_func(void *a) {
  while (1) {
    if (!TASK_tick()) {
      __WFI();
    }
  }
  return 0;
}

static void main_spi_cb(spi_flash_dev *dev, int result) {
  print("spi flash open cb res:%i\n", result);
}

int main(void) {
  s32_t res;
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
  TASK_init();
#ifdef CONFIG_SPI
  print("spif init\n");
  SPI_FLASH_M25P16_app_init();
  res = SPI_FLASH_open(SPI_FLASH, main_spi_cb);
  print("spif open res %i\n", res);
  COMM_FILE_init();

#endif
  COMM_init(_UART(COMMIN));
#ifdef CONFIG_CNC
  CNC_COMM_init();
#endif

  print("Stack 0x%08x -- 0x%08x\n", STACK_START, STACK_END);

  print("Subsystem initialization done\n");

  OS_init();

#define KERNEL_STACK_EXTRA 0x400

  print("Main thread stack size: %i bytes\n", __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA);

  DBG_init();

  OS_thread_create(&main_thread, OS_THREAD_FLAG_PRIVILEGED, main_thread_func, 0,
      (void *)(STACK_START+4), __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA, "main_kernel");

  while(1) {
    print("z");
    SYS_hardsleep_ms(500);
#if OS_DBG_MON
    while (UART_rx_available(_UART(STDIN))) {
      if (UART_get_char(_UART(STDIN)) == ' ') {
        print("DEBUG MONITOR\n");
        OS_DBG_list_all();
      }
    }
#endif
  }

  return 0;
}

void assert_failed(uint8_t* file, uint32_t line) {
  SYS_assert((char*)file, (s32_t)line);
}
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

  // Hijack the process stack pointer to make backtrace work
  asm("mrs %0, psp" : "=r"(HARDFAULT_PSP) : :);
  stack_pointer = HARDFAULT_PSP;

  UART_tx_flush(_UART(STDOUT));
  while(1);
}
#endif
