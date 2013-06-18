/*
 * spiffs.h
 *
 *  Created on: May 26, 2013
 *      Author: petera
 */


#ifndef SPIFFS_H_
#define SPIFFS_H_

#define SPIFFS_OK                       0
#define SPIFFS_ERR_FULL                 -10000
#define SPIFFS_ERR_NOT_FOUND            -10001
#define SPIFFS_ERR_END_OF_OBJECT        -10002
#define SPIFFS_ERR_DELETED              -10003
#define SPIFFS_ERR_NOT_FINALIZED        -10004
#define SPIFFS_ERR_NOT_INDEX            -10005
#define SPIFFS_ERR_INDEX_SPAN_MISMATCH  -10006
#define SPIFFS_ERR_OUT_OF_FILE_DESCS    -10007
#define SPIFFS_ERR_FILE_CLOSED          -10008
#define SPIFFS_ERR_FILE_DELETED         -10009
#define SPIFFS_ERR_BAD_DESCRIPTOR       -10010
#define SPIFFS_COUNTINUE                -11000

#include "system.h"
#include "miniutils.h"

// types and constants

// spiffs file descriptor index type
typedef s16_t spiffs_file;
// spiffs file descriptor attributes
typedef u32_t spiffs_attr;
// spiffs file descriptor mode
typedef u32_t spiffs_mode;

// block index type
typedef u16_t spiffs_block_ix; // (address-phys_addr) / block_size
// page index type
typedef u16_t spiffs_page_ix;  // (address-phys_addr) / page_size
// object id type - most significant bit is reserved for index flag
typedef u16_t spiffs_obj_id;
// object span index type
typedef u16_t spiffs_span_ix;
// object type
typedef u8_t spiffs_obj_type;

// object name length
#define SPIFFS_OBJ_NAME_LEN (32-sizeof(spiffs_obj_type))

// spi read call type
typedef s32_t (*spiffs_read)(u32_t addr, u32_t size, u8_t *dst);
// spi write call type
typedef s32_t (*spiffs_write)(u32_t addr, u32_t size, u8_t *src);
// spi erase call type
typedef s32_t (*spiffs_erase)(u32_t addr, u32_t size);

// size of buffer on stack used when copying data
#define SPIFFS_COPY_BUFFER_STACK        (64)

// Garbage collecting examines all pages in a block which and sums up
// to a block score. Deleted pages normal gives positive score and
// used pages normally gives a negative score (as these must be moved).
// The larger the score, the more likely it is that the block will
// be garbage collected.

// garbage collecting heuristics - weight used for deleted pages
#define SPIFFS_GC_HEUR_W_DELET          (10)
// garbage collecting heuristics - weight used for used pages
#define SPIFFS_GC_HEUR_W_USED           (-1)

#define SPIFFS_DBG(...) DBG(D_FS, D_DEBUG, __VA_ARGS__)

#ifndef SPIFFS_DBG
#define SPIFFS_DBG(...) \
    print(__VA_ARGS__)
#endif

#define SPIFFS_APPEND                   (1<<0)
#define SPIFFS_TRUNC                    (1<<1)
#define SPIFFS_CREAT                    (1<<2)

#define SPIFFS_SEEK_SET                 (0)
#define SPIFFS_SEEK_CUR                 (1)
#define SPIFFS_SEEK_END                 (2)

#define SPIFFS_TYPE_FILE                (1)
#define SPIFFS_TYPE_DIR                 (2)
#define SPIFFS_TYPE_HARD_LINK           (3)
#define SPIFFS_TYPE_SOFT_LINK           (4)

// phys structs

// spiffs spi configuration struct
typedef struct {
  // physical size of the spi flash
  u32_t phys_size;
  // physical offset in spi flash used for spiffs,
  // must be on block boundary
  u32_t phys_addr;
  // physical size when erasing a block
  u32_t phys_erase_block;

  // physical read function
  spiffs_read hal_read_f;
  // physical write function
  spiffs_write hal_write_f;
  // physical erase function
  spiffs_erase hal_erase_f;

  // logical size of a block, must be on physical
  // block size boundary and must never be less than
  // a physical block
  u32_t log_block_size;
  // logical size of a page, must be at least
  // log_block_size / 8
  u32_t log_page_size;
} spiffs_config;

typedef struct {
  // file system configuration
  spiffs_config cfg;
  // number of logical blocks
  u32_t block_count;

  // cursor for free blocks, block index
  spiffs_block_ix free_cursor_block_ix;
  // cursor for free blocks, entry index
  int free_cursor_obj_lu_entry;
  // cursor when searching, block index
  spiffs_block_ix cursor_block_ix;
  // cursor when searching, entry index
  int cursor_obj_lu_entry;

  // primary work buffer, size of a logical page
  u8_t *lu_work;
  // secondary work buffer, size of a logical page
  u8_t *work;
  // file descriptor memory area
  u8_t *fd_space;
  // available file descriptors
  u32_t fd_count;

  s32_t errno;

  u32_t free_blocks;
} spiffs;

typedef struct {
  spiffs_obj_id obj_id;
  u32_t size;
  spiffs_obj_type type;
  u8_t name[SPIFFS_OBJ_NAME_LEN];
} spiffs_stat;

// functions

/**
 * Initializes the file system dynamic parameters
 * @param fs      the file system struct
 * @param config  the physical and logical configuration of the file system
 * @param work    a memory work buffer comprising 2*config->log_page_size bytes used throughout
 *                all file system operations
 * @param fd_space      memory for file descriptors
 * @param fd_space_size memory size for file descriptors
 */
s32_t SPIFFS_init(spiffs *fs, spiffs_config *config, u8_t *work, u8_t *fd_space, u32_t fd_space_size);

s32_t SPIFFS_creat(spiffs *fs, const char *path, spiffs_mode mode);
spiffs_file SPIFFS_open(spiffs *fs, const char *path, spiffs_attr attr, spiffs_mode mode);
s32_t SPIFFS_read(spiffs *fs, spiffs_file fh, void *buf, s32_t len);
s32_t SPIFFS_write(spiffs *fs, spiffs_file fh, void *buf, s32_t len);
s32_t SPIFFS_lseek(spiffs *fs, spiffs_file fh, s32_t offs, int whence);
s32_t SPIFFS_remove(spiffs *fs, const char *path);
s32_t SPIFFS_fremove(spiffs *fs, spiffs_file fh);
s32_t SPIFFS_fstat(spiffs *fs, spiffs_file fh, spiffs_stat *s);
void SPIFFS_close(spiffs *fs, spiffs_file fh);
s32_t SPIFFS_errno(spiffs *fs);

#endif /* SPIFFS_H_ */
