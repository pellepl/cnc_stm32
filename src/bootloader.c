#include "system.h"
#include "bootloader.h"
#include "bootloader_exec.h"
#include "linker_symaccess.h"

#define SHMEM ((shmem *)SHARED_MEMORY_ADDRESS)

BOOTLOADER_DATA bootdata boot;

BOOTLOADER_DATA char boot_msg_welcome[] = "**** Bootloader ram domain entered\n";
BOOTLOADER_DATA char boot_msg_bye[] = "**** Bootloader returning, bye\n\n";
BOOTLOADER_DATA char boot_msg_stack[] = "  stack pointer: ";
BOOTLOADER_DATA char boot_msg_operation[] = "  operation: ";
BOOTLOADER_DATA char boot_msg_suboperation[] = ", subop: ";
BOOTLOADER_DATA char boot_msg_flash_successful[] = "  flashing procedure complete\n";
BOOTLOADER_DATA char boot_msg_err_bad_fw_magic[] = "  bad fw magic: ";
BOOTLOADER_DATA char boot_msg_err_flash_fail[] = "  flash operation failed:";

BOOTLOADER_TEXT static bool _bootloader_check_spi_flash();

BOOTLOADER_TEXT static void _bootloader_reset(enum reboot_reason_e rr, u32_t subop) {
  b_putstr(boot_msg_bye);
  SHMEM->reboot_reason = rr;
  SHMEM->user[BOOTLOADER_SHMEM_SUBOPERATION_UIX] = subop;
  SHMEM_CALC_CHK(SHMEM, SHMEM->chk);
  NVIC_SystemReset();
  while(1);
}

BOOTLOADER_TEXT void bootloader_init() {
  boot.uart_hw = (USART_TypeDef*)(SHMEM->user[BOOTLOADER_SHMEM_UART_UIX]);
  boot.spi_hw = (SPI_TypeDef*)(SHMEM->user[BOOTLOADER_SHMEM_SPI_FLASH_UIX]);
  boot.operation = SHMEM->user[BOOTLOADER_SHMEM_OPERATION_UIX];
  boot.suboperation = SHMEM->user[BOOTLOADER_SHMEM_SUBOPERATION_UIX];
  b_putstr(boot_msg_welcome);

  b_putstr(boot_msg_stack);
  b_puthex32(__get_MSP());
  b_put('\n');

  b_putstr(boot_msg_operation);
  b_putint(boot.operation);
  b_putstr(boot_msg_suboperation);
  b_putint(boot.suboperation);
  b_put('\n');

  FLASH_res res;
  switch (boot.operation) {
  case BOOTLOADER_FLASH_FIRMWARE:
    switch (boot.suboperation) {
    case 0:
      // check spi flash contents
      if (_bootloader_check_spi_flash()) {
        _bootloader_reset(REBOOT_EXEC_BOOTLOADER, 1);
      } else {
        _bootloader_reset(REBOOT_BOOTLOADER, 0);
      }
      break;
    case 1:
      // unprotect
      res = b_flash_unprotect();
      if (res == FLASH_OK) {
        _bootloader_reset(REBOOT_EXEC_BOOTLOADER, 2);
      } else {
        b_putstr(boot_msg_err_flash_fail);
        b_putint(res);
        b_put('\n');
        _bootloader_reset(REBOOT_BOOTLOADER, 0);
      }
      break;
    case 2:
      // flash
      _bootloader_reset(REBOOT_EXEC_BOOTLOADER, 3);
      break;
    case 3:
      // protect
      res = b_flash_protect();
      if (res == FLASH_OK) {
        b_putstr(boot_msg_flash_successful);
        _bootloader_reset(REBOOT_EXEC_BOOTLOADER, 4);
      } else {
        b_putstr(boot_msg_err_flash_fail);
        b_putint(res);
        b_put('\n');
        _bootloader_reset(REBOOT_BOOTLOADER, 0);
      }
      break;
    default:
      _bootloader_reset(REBOOT_BOOTLOADER, 0);
      break;
    }
    break;
  case BOOTLOADER_EXECUTE:
  default:
    _bootloader_reset(REBOOT_BOOTLOADER, 0);
    break;
  }
}

BOOTLOADER_TEXT static bool _bootloader_check_spi_flash() {
  fw_upgrade_info fui;
  u32_t addr = SHMEM->user[BOOTLOADER_SHMEM_SPIF_FW_ADDR_UIX];
  b_spif_read(addr, (u8_t*)&fui, sizeof(fui));
  if (fui.magic != FW_MAGIC) {
    b_putstr(boot_msg_err_bad_fw_magic);
    b_puthex32(fui.magic);
    b_put('\n');
    return FALSE;
  }
  // todo calc crc
  return TRUE;
}
