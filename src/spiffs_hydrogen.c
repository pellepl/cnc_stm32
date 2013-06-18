/*
 * spiffs_hydrogen.c
 *
 *  Created on: Jun 16, 2013
 *      Author: petera
 */

#include "spiffs.h"
#include "spiffs_nucleus.h"


s32_t SPIFFS_creat(spiffs *fs, const char *path, spiffs_mode mode) {
  spiffs_obj_id obj_id;
  s32_t res;

  res = spiffs_obj_lu_find_free_obj_id(fs, &obj_id);
  SPIFFS_API_CHECK_RES(fs, res);
  res = spiffs_object_create(fs, obj_id, (u8_t*)path, SPIFFS_TYPE_FILE, 0);
  SPIFFS_API_CHECK_RES(fs, res);
  return 0;
}

spiffs_file SPIFFS_open(spiffs *fs, const char *path, spiffs_attr attr, spiffs_mode mode) {
  spiffs_fd *fd;
  spiffs_page_ix pix;

  s32_t res = spiffs_fd_find_new(fs, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  res = spiffs_object_find_object_index_header_by_name(fs, (u8_t*)path, &pix);
  SPIFFS_API_CHECK_RES(fs, res);

  if ((mode & SPIFFS_CREAT) && res == SPIFFS_ERR_NOT_FOUND) {
    spiffs_obj_id obj_id;
    res = spiffs_obj_lu_find_free_obj_id(fs, &obj_id);
    SPIFFS_API_CHECK_RES(fs, res);
    res = spiffs_object_create(fs, obj_id, (u8_t*)path, SPIFFS_TYPE_FILE, &pix);
    SPIFFS_API_CHECK_RES(fs, res);
    mode &= ~SPIFFS_TRUNC;
  } else {
    SPIFFS_API_CHECK_RES(fs, res);
  }
  res = spiffs_object_open_by_page(fs, pix, fd, attr, mode);
  SPIFFS_API_CHECK_RES(fs, res);
  if (mode & SPIFFS_TRUNC) {
    res = spiffs_object_truncate(fd, 0, 0);
    SPIFFS_API_CHECK_RES(fs, res);
  }
  if (mode & SPIFFS_APPEND) {
    fd->offset = fd->size == SPIFFS_UNDEFINED_LEN ? 0 : fd->size;
    fd->cursor_objix_pix = 0;
    fd->cursor_objix_spix = -1;
  }

  return fd->file_nbr;
}

s32_t SPIFFS_read(spiffs *fs, spiffs_file fh, void *buf, s32_t len) {
  spiffs_fd *fd;
  s32_t res;

  res = spiffs_fd_get(fs, fh, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  if (fd->offset + len >= fd->size) {
    s32_t avail = fd->size - fd->offset;
    if (avail <= 0) {
      SPIFFS_API_CHECK_RES(fs, SPIFFS_ERR_END_OF_OBJECT);
    }
    res = spiffs_object_read(fd, fd->offset, len, (u8_t*)buf);
    if (res == SPIFFS_ERR_END_OF_OBJECT) {
      return avail;
    } else {
      SPIFFS_API_CHECK_RES(fs, res);
    }
  } else {
    res = spiffs_object_read(fd, fd->offset, len, (u8_t*)buf);
    SPIFFS_API_CHECK_RES(fs, res);
  }
  return len;
}

s32_t SPIFFS_write(spiffs *fs, spiffs_file fh, void *buf, s32_t len) {
  spiffs_fd *fd;
  s32_t res;
  s32_t remaining = len;
  u32_t offset;

  res = spiffs_fd_get(fs, fh, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  offset = fd->offset;

  if (fd->mode & SPIFFS_APPEND) {
    if (fd->size == SPIFFS_UNDEFINED_LEN) {
      offset = 0;
    } else {
      offset = fd->size;
    }
  }

  SPIFFS_DBG("SPIFFS_write %i %04x offs:%i len %i\n", fh, fd->obj_id, offset, fd->size);
  if (fd->size != SPIFFS_UNDEFINED_LEN && offset < fd->size) {
    s32_t m_len = MIN(fd->size - offset, len);
    res = spiffs_object_modify(fd, offset, (u8_t *)buf, m_len);
    SPIFFS_API_CHECK_RES(fs, res);
    remaining -= m_len;
    buf += m_len;
    offset += m_len;
  }
  if (remaining > 0) {
    res = spiffs_object_append(fd, offset, (u8_t *)buf, remaining);
    SPIFFS_API_CHECK_RES(fs, res);
  }
  return len;
}

s32_t SPIFFS_lseek(spiffs *fs, spiffs_file fh, s32_t offs, int whence) {
  spiffs_fd *fd;
  s32_t res;
  res = spiffs_fd_get(fs, fh, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  switch (whence) {
  case SPIFFS_SEEK_CUR:
    offs = fd->offset+offs;
    break;
  case SPIFFS_SEEK_END:
    offs = (fd->size == SPIFFS_UNDEFINED_LEN ? 0 : fd->size) + offs;
    break;
  }

  if (offs > fd->size) {
    res = SPIFFS_ERR_END_OF_OBJECT;
  }
  SPIFFS_API_CHECK_RES(fs, res);

  spiffs_span_ix data_spix = offs / SPIFFS_DATA_PAGE_SIZE(fs);
  spiffs_span_ix objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);
  if (fd->cursor_objix_spix != objix_spix) {
    spiffs_page_ix pix;
    res = spiffs_obj_lu_find_id_and_index(
        fs, fd->obj_id | SPIFFS_OBJ_ID_IX_FLAG, objix_spix, &pix);
    SPIFFS_API_CHECK_RES(fs, res);
    fd->cursor_objix_spix = objix_spix;
    fd->cursor_objix_pix = pix;
  }
  fd->offset = offs;

  return 0;
}

s32_t SPIFFS_remove(spiffs *fs, const char *path) {
  spiffs_fd fd;
  spiffs_page_ix pix;
  s32_t res;
  fd.file_nbr = 0;
  fd.fs = fs;
  res = spiffs_object_find_object_index_header_by_name(fs, (u8_t*)path, &pix);
  SPIFFS_API_CHECK_RES(fs, res);
  res = spiffs_object_open_by_page(fs, pix, &fd, 0,0);
  SPIFFS_API_CHECK_RES(fs, res);
  res = spiffs_object_truncate(&fd, 0, 1);
  SPIFFS_API_CHECK_RES(fs, res);

  return 0;
}

s32_t SPIFFS_fremove(spiffs *fs, spiffs_file fh) {
  spiffs_fd *fd;
  s32_t res;
  res = spiffs_fd_get(fs, fh, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  res = spiffs_object_truncate(fd, 0, 1);

  SPIFFS_API_CHECK_RES(fs, res);
  return 0;
}

s32_t SPIFFS_fstat(spiffs *fs, spiffs_file fh, spiffs_stat *s) {
  spiffs_fd *fd;
  s32_t res;
  spiffs_page_object_ix_header objix_hdr;

  res = spiffs_fd_get(fs, fh, &fd);
  SPIFFS_API_CHECK_RES(fs, res);

  res = fs->cfg.hal_read_f(SPIFFS_PAGE_TO_PADDR(fs, fd->objix_hdr_pix), sizeof(spiffs_page_object_ix_header), (u8_t *)&objix_hdr);
  SPIFFS_API_CHECK_RES(fs, res);

  s->obj_id = objix_hdr.p_hdr.obj_id;
  s->type = objix_hdr.type;
  s->size = objix_hdr.size == SPIFFS_UNDEFINED_LEN ? 0 : objix_hdr.size;
  strncpy((char *)s->name, (char *)objix_hdr.name, SPIFFS_OBJ_NAME_LEN);

  return res;
}


void SPIFFS_close(spiffs *fs, spiffs_file fh) {
  spiffs_fd_return(fs, fh);
}
