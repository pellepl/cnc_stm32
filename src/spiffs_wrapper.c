#include "system.h"
#include "spiffs.h"
#include "os.h"
#include "spi_flash_os.h"
#include "spiffs_wrapper.h"
#include "heap.h"

#ifdef CONFIG_SPIFFS
os_mutex fs_mutex;

static spiffs fs;
static u8_t spiffs_work[256*2];
static u8_t spiffs_fds[256];
static u8_t spiffs_cache[256*5];

#define SPIFFS_MOUNT_STACK  0x300

typedef struct {
  os_thread spiffs_mounter;
  void *spiffs_mount_stack;
} spiffs_mounter_init;

static void *spiffs_mounter_f(void *a) {
  spiffs_mounter_init *smi = (spiffs_mounter_init *)a;
  print("mounting spiffs..\n");
  FS_mount();
  print("mounted spiffs..\n");
  HEAP_free(smi->spiffs_mount_stack);
  HEAP_free(smi);
  return NULL;
}

void FS_sys_init() {
  OS_mutex_init(&fs_mutex, 0); //OS_MUTEX_ATTR_REENTRANT);
#if 1
  spiffs_mounter_init *smi = HEAP_malloc(sizeof(spiffs_mounter_init)+2);
  smi->spiffs_mount_stack = HEAP_malloc(SPIFFS_MOUNT_STACK);
  OS_thread_create(
      &smi->spiffs_mounter,
      OS_THREAD_FLAG_PRIVILEGED,
      spiffs_mounter_f,
      smi,
      smi->spiffs_mount_stack,
      SPIFFS_MOUNT_STACK-4,
      "spiffs_mounter");
#endif
}

static u32_t old_perc = 999;
static void fs_check_cb_f(spiffs_check_type type, spiffs_check_report report,
    u32_t arg1, u32_t arg2) {
  if (report == SPIFFS_CHECK_PROGRESS) {
    if (old_perc != arg1) {
      old_perc = arg1;
      print("CHECK REPORT: ");
      switch(type) {
      case SPIFFS_CHECK_LOOKUP:
        print("LU "); break;
      case SPIFFS_CHECK_INDEX:
        print("IX "); break;
      case SPIFFS_CHECK_PAGE:
        print("PA "); break;
      }
      print("%i%%\n", (arg1 * 100) / 256);
    }
  } else {
    print("   check: ");
    switch (type) {
    case SPIFFS_CHECK_INDEX:
      print("INDEX  "); break;
    case SPIFFS_CHECK_LOOKUP:
      print("LOOKUP "); break;
    case SPIFFS_CHECK_PAGE:
      print("PAGE   "); break;
    default:
      print("????   "); break;
    }
    if (report == SPIFFS_CHECK_ERROR) {
      print("ERROR %i", arg1);
    } else if (report == SPIFFS_CHECK_DELETE_BAD_FILE) {
      print("DELETE BAD FILE %04x", arg1);
    } else if (report == SPIFFS_CHECK_DELETE_ORPHANED_INDEX) {
      print("DELETE ORPHANED INDEX %04x", arg1);
    } else if (report == SPIFFS_CHECK_DELETE_PAGE) {
      print("DELETE PAGE %04x", arg1);
    } else if (report == SPIFFS_CHECK_FIX_INDEX) {
      print("FIX INDEX %04x:%04x", arg1, arg2);
    } else if (report == SPIFFS_CHECK_FIX_LOOKUP) {
      print("FIX INDEX %04x:%04x", arg1, arg2);
    } else {
      print("??");
    }
    print("\n");
  }
}

void FS_mount() {
  spiffs_config cfg;
  #ifndef SPIFFS_SINGLETON
  cfg.phys_addr = 0;
  cfg.phys_erase_block = 65536;
  cfg.phys_size = 1024*1024;
  cfg.log_block_size = 65536;
  cfg.log_page_size = 256;
  #endif
  cfg.hal_read_f = SFOS_read;
  cfg.hal_write_f = SFOS_write;
  cfg.hal_erase_f = SFOS_erase;

  SPIFFS_mount(
      &fs,
      &cfg,
      spiffs_work,
      spiffs_fds,
      sizeof(spiffs_fds),
      spiffs_cache,
      sizeof(spiffs_cache),
      fs_check_cb_f);
}

spiffs *FS_get_filesystem() {
  return &fs;
}

#endif
