/*
 * spiffs_nucleus.h
 *
 *  Created on: Jun 15, 2013
 *      Author: petera
 */

/* SPIFFS layout
 *
 * spiffs is designed for following spi flash characteristics:
 *   - only big areas of data (blocks) can be erased
 *   - erasing resets all bits in a block to ones
 *   - writing pulls ones to zeroes
 *   - zeroes cannot be pulled to ones, without erase
 *   - wear leveling
 *
 * spiffs is also meant to be run on embedded, memory constraint devices.
 *
 * Entire area is divided in blocks. Entire area is also divided in pages.
 * Each block contains same number of pages. A page cannot be erased, but a
 * block can be erased.
 *
 * Entire area must be block_size * x
 * page_size must be block_size / (2^y) where y > 2
 *
 * ex: area = 1024*1024 bytes, block size = 65536 bytes, page size = 256 bytes
 *
 * BLOCK 0  PAGE 0       object lookup 1
 *          PAGE 1       object lookup 2
 *          ...
 *          PAGE n-1     object lookup n
 *          PAGE n       object data 1
 *          PAGE n+1     object data 2
 *          ...
 *          PAGE n+m-1   object data m
 *
 * BLOCK 1  PAGE n+m     object lookup 1
 *          PAGE n+m+1   object lookup 2
 *          ...
 *          PAGE 2n+m-1  object lookup n
 *          PAGE 2n+m    object data 1
 *          PAGE 2n+m    object data 2
 *          ...
 *          PAGE 2n+2m-1 object data m
 * ...
 *
 * n is number of object lookup pages, which is number of pages needed to index all pages
 * in a block by object id
 *   : block_size / page_size * sizeof(obj_id) / page_size
 * m is number data pages, which is number of pages in block minus number of lookup pages
 *   : block_size / page_size - block_size / page_size * sizeof(obj_id) / page_size
 * thus, n+m is total number of pages in a block
 *   : block_size / page_size
 *
 * ex: n = 65536/256*2/256 = 2, m = 65536/256 - 2 = 254 => n+m = 65536/256 = 256
 *
 * Object lookup pages contain object id entries. Each entry represent the corresponding
 * data page.
 * Assuming a 16 bit object id, an object id being 0xffff represents a free page.
 * An object id being 0x0000 represents a deleted page.
 *
 * ex: page 0 : lookup : 0008 0001 0aaa ffff ffff ffff ffff ffff ..
 *     page 1 : lookup : ffff ffff ffff ffff ffff ffff ffff ffff ..
 *     page 2 : data   : data for object id 0008
 *     page 3 : data   : data for object id 0001
 *     page 4 : data   : data for object id 0aaa
 *     ...
 *
 *
 * Object data pages can be either object index pages or object content.
 * All object data pages contains a data page header, containing object id and span index.
 * The span index denotes the object page ordering amongst data pages with same object id.
 * This applies to both object index pages (when index spans more than one page of entries),
 * and object data pages.
 * An object index page contains page entries pointing to object content page. The entry index
 * in a object index page correlates to the span index in the actual object data page.
 * The first object index page (span index 0) is called object index header page, and also
 * contains object flags (directory/file), size, object name etc.
 *
 * ex:
 *  BLOCK 1
 *    PAGE 256: objectl lookup page 1
 *      [*123] [ 123] [ 123] [ 123]
 *      [ 123] [*123] [ 123] [ 123]
 *      [free] [free] [free] [free] ...
 *    PAGE 257: objectl lookup page 2
 *      [free] [free] [free] [free] ...
 *    PAGE 258: object index page (header)
 *      obj.id:0123 span.ix:0000 flags:INDEX
 *      size:1600 name:ex.txt type:file
 *      [259] [260] [261] [262]
 *    PAGE 259: object data page
 *      obj.id:0123 span.ix:0000 flags:DATA
 *    PAGE 260: object data page
 *      obj.id:0123 span.ix:0001 flags:DATA
 *    PAGE 261: object data page
 *      obj.id:0123 span.ix:0002 flags:DATA
 *    PAGE 262: object data page
 *      obj.id:0123 span.ix:0003 flags:DATA
 *    PAGE 263: object index page
 *      obj.id:0123 span.ix:0001 flags:INDEX
 *      [264] [265] [fre] [fre]
 *      [fre] [fre] [fre] [fre]
 *    PAGE 264: object data page
 *      obj.id:0123 span.ix:0004 flags:DATA
 *    PAGE 265: object data page
 *      obj.id:0123 span.ix:0005 flags:DATA
 *
 */
#ifndef SPIFFS_NUCLEUS_H_
#define SPIFFS_NUCLEUS_H_

#define SPIFFS_EV_IX_UPD                0
#define SPIFFS_EV_IX_NEW                1
#define SPIFFS_EV_IX_DEL                2

#define SPIFFS_OBJ_ID_IX_FLAG           (1<<(8*sizeof(spiffs_obj_id)-1))

#define SPIFFS_UNDEFINED_LEN            (-1)

#define SPIFFS_OBJ_ID_ERASED            ((spiffs_obj_id)0)
#define SPIFFS_OBJ_ID_FREE              ((spiffs_obj_id)-1)

// total number of pages per block, including object lookup pages
#define SPIFFS_PAGES_PER_BLOCK(fs) \
  (((fs)->cfg.log_block_size)/(fs)->cfg.log_page_size)
// number of object lookup pages per block
#define SPIFFS_OBJ_LOOKUP_PAGES(fs)     \
  (MAX(1, (SPIFFS_PAGES_PER_BLOCK(fs) * sizeof(spiffs_obj_id)) / (fs)->cfg.log_page_size))
// number of object lookup entries in all object lookup pages
#define SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) \
  (SPIFFS_PAGES_PER_BLOCK(fs)-SPIFFS_OBJ_LOOKUP_PAGES(fs))
// converts a block to physical address
#define SPIFFS_BLOCK_TO_PADDR(fs, block) \
  ((fs)->cfg.phys_addr + (block)*(fs)->cfg.log_block_size)
// converts a object lookup entry to page index
#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, block, entry) \
  ((block)*SPIFFS_PAGES_PER_BLOCK(fs) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry))
// converts a object lookup entry to physical address of corresponding page
#define SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, block, entry) \
  (SPIFFS_BLOCK_TO_PADDR(fs, block) + (SPIFFS_OBJ_LOOKUP_PAGES(fs) + entry) * (fs)->cfg.log_page_size)
// converts a page to physical address
#define SPIFFS_PAGE_TO_PADDR(fs, page) \
  ((fs)->cfg.phys_addr + (page) * (fs)->cfg.log_page_size)
// returns containing block for given page
#define SPIFFS_BLOCK_FOR_PAGE(fs, page) \
  ((page) / SPIFFS_PAGES_PER_BLOCK(fs))
// converts page to entry in object lookup page
#define SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, page) \
  ((page) % SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs))
// returns data size in a data page
#define SPIFFS_DATA_PAGE_SIZE(fs) \
    ((fs)->cfg.log_page_size - sizeof(spiffs_page_header))

// define helpers object

// entries in an object header page index
#define SPIFFS_OBJ_HDR_IX_LEN(fs) \
  (((fs)->cfg.log_page_size - sizeof(spiffs_page_object_ix_header))/sizeof(spiffs_page_ix))
// entries in an object page index
#define SPIFFS_OBJ_IX_LEN(fs) \
  (((fs)->cfg.log_page_size - sizeof(spiffs_page_object_ix))/sizeof(spiffs_page_ix))
// object index entry for given data span index
#define SPIFFS_OBJ_IX_ENTRY(fs, spix) \
  ((spix) < SPIFFS_OBJ_HDR_IX_LEN(fs) ? (spix) : (((spix)-SPIFFS_OBJ_HDR_IX_LEN(fs))%SPIFFS_OBJ_IX_LEN(fs)))
// object index span index number for given data span index or entry
#define SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, spix) \
  ((spix) < SPIFFS_OBJ_HDR_IX_LEN(fs) ? 0 : (1+((spix)-SPIFFS_OBJ_HDR_IX_LEN(fs))/SPIFFS_OBJ_IX_LEN(fs)))

// spiffs nucleus file descriptor
typedef struct {
  // the filesystem of this descriptor
  spiffs *fs;
  // number of file descriptor - if 0, the file descriptor is closed
  spiffs_file file_nbr;
  // object id - if SPIFFS_OBJ_ID_ERASED, the file was deleted
  spiffs_obj_id obj_id;
  // size of the file
  u32_t size;
  // cached object index header page index
  spiffs_page_ix objix_hdr_pix;
  // cached offset object index page index
  spiffs_page_ix cursor_objix_pix;
  // cached offset object index span index
  spiffs_span_ix cursor_objix_spix;
  // current offset
  u32_t offset;
  // mode
  spiffs_mode mode;
} spiffs_fd;

// if 0, writing is finalized, else under modification
#define SPIFFS_PH_FLAG_FINAL  (1<<0)
// if 0, page is deleted, else valid
#define SPIFFS_PH_FLAG_DELET  (1<<1)
// if 0, page is corrupt, else ok
#define SPIFFS_PH_FLAG_CORRU  (1<<2)
// if 0, this is an index page, else a data page
#define SPIFFS_PH_FLAG_INDEX  (1<<3)


#define SPIFFS_API_CHECK_RES(fs, res) \
  if ((res) != SPIFFS_OK) { \
    (fs)->errno = (res); \
    return -1; \
  }


// object structs

// page header, part of each page except object lookup pages
typedef struct __attribute(( packed )) {
  // object id
  spiffs_obj_id obj_id;
  // object span index
  spiffs_span_ix span_ix;
  // flags
  u8_t flags;
} spiffs_page_header;

// object index header page header
typedef struct __attribute(( packed )) {
  // common page header
  spiffs_page_header p_hdr;
  // alignment
  u8_t _align[4 - (sizeof(spiffs_page_header)&3)==0 ? 4 : (sizeof(spiffs_page_header)&3)];
  // size of object
  u32_t size;
  // type of object
  spiffs_obj_type type;
  // name of object
  u8_t name[SPIFFS_OBJ_NAME_LEN];
} spiffs_page_object_ix_header;

// object index page header
typedef struct __attribute(( packed )) {
 spiffs_page_header p_hdr;
 u8_t _align[4 - (sizeof(spiffs_page_header)&3)==0 ? 4 : (sizeof(spiffs_page_header)&3)];
} spiffs_page_object_ix;

// callback func for object lookup visitor

typedef s32_t (*spiffs_visitor_f)(spiffs *fs, spiffs_obj_id id, spiffs_block_ix bix, int ix_entry,
    u32_t user_data, void *user_p);


// ---------------

s32_t spiffs_phys_cpy(
    spiffs *fs,
    u32_t dst,
    u32_t src,
    u32_t len);

s32_t spiffs_phys_count_free_blocks(
    spiffs *fs);

// ---------------

s32_t spiffs_obj_lu_find_free_obj_id(
    spiffs *fs,
    spiffs_obj_id *obj_id);

s32_t spiffs_obj_lu_find_free(
    spiffs *fs,
    spiffs_block_ix starting_block,
    int starting_index_entry,
    spiffs_block_ix *block_ix,
    int *index_entry);

s32_t spiffs_obj_lu_find_id(
    spiffs *fs,
    spiffs_block_ix starting_block,
    int starting_index_entry,
    spiffs_obj_id obj_id,
    spiffs_block_ix *block_ix,
    int *index_entry);

s32_t spiffs_obj_lu_find_id_and_index(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_span_ix spix,
    spiffs_page_ix *pix);

// ---------------

s32_t spiffs_page_allocate_data(
    spiffs *fs,
    spiffs_page_header *ph,
    u8_t *data,
    u32_t len,
    u32_t page_offs,
    u8_t finalize,
    spiffs_page_ix *pix);

s32_t spiffs_page_move(
    spiffs *fs,
    u8_t *page_data,
    spiffs_page_header *page_hdr,
    spiffs_page_ix src_pix,
    spiffs_page_ix *dst_pix);

s32_t spiffs_page_delete(
    spiffs *fs,
    spiffs_page_ix pix);

// ---------------

s32_t spiffs_object_create(
    spiffs *fs,
    spiffs_obj_id obj_id,
    u8_t name[SPIFFS_OBJ_NAME_LEN],
    spiffs_obj_type type,
    spiffs_page_ix *objix_hdr_pix);

s32_t spiffs_object_update_index_hdr(
    spiffs *fs,
    spiffs_fd *fd,
    spiffs_page_ix objix_hdr_pix,
    u8_t *new_objix_hdr_data,
    u8_t name[SPIFFS_OBJ_NAME_LEN],
    u32_t size,
    spiffs_page_ix *new_pix);

void spiffs_cb_object_event(
    spiffs *fs,
    spiffs_fd *fd,
    int ev,
    spiffs_obj_id obj_id,
    spiffs_span_ix spix,
    spiffs_page_ix new_pix,
    u32_t new_size);

s32_t spiffs_object_open_by_id(
    spiffs *fs,
    spiffs_obj_id obj_id,
    spiffs_fd *f,
    spiffs_attr attr,
    spiffs_mode mode);

s32_t spiffs_object_open_by_page(
    spiffs *fs,
    spiffs_page_ix pix,
    spiffs_fd *f,
    spiffs_attr attr,
    spiffs_mode mode);

s32_t spiffs_object_append(
    spiffs_fd *fd,
    u32_t offset,
    u8_t *data,
    u32_t len);

s32_t spiffs_object_modify(
    spiffs_fd *fd,
    u32_t offset,
    u8_t *data,
    u32_t len);

s32_t spiffs_object_read(
    spiffs_fd *fd,
    u32_t offset,
    u32_t len,
    u8_t *dst);

s32_t spiffs_object_truncate(
    spiffs_fd *fd,
    u32_t new_len,
    u8_t remove_object);

s32_t spiffs_object_find_object_index_header_by_name(
    spiffs *fs,
    u8_t name[SPIFFS_OBJ_NAME_LEN],
    spiffs_page_ix *pix);

// ---------------

s32_t spiffs_gc_check(
    spiffs *fs);

s32_t spiffs_gc_find_candidate(
    spiffs *fs,
    spiffs_block_ix **block_candidate,
    int *candidate_count);

s32_t spiffs_gc_clean(
    spiffs *fs,
    spiffs_block_ix bix);

// ---------------

s32_t spiffs_fd_find_new(
    spiffs *fs,
    spiffs_fd **fd);

s32_t spiffs_fd_return(
    spiffs *fs,
    spiffs_file f);

s32_t spiffs_fd_get(
    spiffs *fs,
    spiffs_file f,
    spiffs_fd **fd);

void spiffs_test_list_objects(spiffs *fs);

#endif /* SPIFFS_NUCLEUS_H_ */
