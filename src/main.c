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
  print(TEXT_NOTE("Kernel running...\n"));
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

#define KERNEL_STACK_EXTRA 0x800

  print("Main thread stack size: %i bytes\n", __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA);

  DBG_init();

  OS_thread_create(
      &main_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      main_thread_func,
      0,
      (void *)(STACK_START+4), __get_MSP() - (u32_t)(STACK_START) - KERNEL_STACK_EXTRA,
      "main_kernel");

  while(1) {
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


  while(1);
}
#endif
