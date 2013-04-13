#include "comm_file.h"
#include "comm_impl.h"
#include "cnc_comm.h"
#include "miniutils.h"
#include "spi_flash.h"
#include "spi_flash_m25p16.h"
#include "bootloader_exec.h"

typedef struct  __attribute__ (( packed )) {
  u8_t cmd; // always 0x80
  u32_t sequence;
  u32_t length;
  u8_t name[8];
} comm_file_hdr ;

typedef struct  __attribute__ (( packed ))  {
  u8_t ack; // always 0x81
  u8_t res;
  u32_t last_stored_pkt_in_sequence;
} comm_file_ack;

static struct {
  bool active;
  u32_t length;
  u32_t index;
  u32_t req_sequence;
  u8_t last_pkt_len;
  u8_t errors;
} state;

#define TEST 1

#if TEST
task *test;
#endif

#define COMM_FILE_PKT_OVERHEAD      (offsetof(comm_file_hdr, length))
#define COMM_FILE_MAX_DATA_PKT      (COMM_APP_MAX_DATA - COMM_FILE_PKT_OVERHEAD)

static void _comm_file_spif_cb(spi_flash_dev *dev, int result) {
  //SYS_hardsleep_ms(100);
  DBG(D_SYS, D_DEBUG, "comm file pkt spif cb res:%i\n", result);
  comm_file_ack ack;
  ack.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
  if (result != SPI_OK) {
    DBG(D_SYS, D_WARN, "comm file pkt spif error\n");
    ack.res = COMM_FILE_ERR_ABORT;
    ack.last_stored_pkt_in_sequence = 0xffffffff;
    state.active = FALSE;
    COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&ack, sizeof(comm_file_ack), TRUE);
  } else {
    if (state.last_pkt_len > 0) {
      state.index += state.last_pkt_len;
      state.req_sequence++;
      DBG(D_SYS, D_DEBUG, "comm file pkt spif stored %i of %i\n", state.index, state.length);
    } else {
      // if callback from erase, last_pkt_len is 0
      DBG(D_SYS, D_DEBUG, "comm file pkt spif area erased\n");
    }

    ack.res = COMM_FILE_REPLY_OK;
    ack.last_stored_pkt_in_sequence = state.req_sequence;

    if (state.index == state.length) {
      // finished
      state.active = FALSE;
      DBG(D_SYS, D_INFO, "comm file pkt spif all ok, finished!\n");
    } else {
      // request next packet
      DBG(D_SYS, D_DEBUG, "comm file pkt spif ack & req next %i\n", state.req_sequence);
      s32_t res = COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&ack, sizeof(comm_file_ack), TRUE);
      if (res < R_COMM_OK) {
        DBG(D_SYS, D_WARN, "comm file pkt spif nxt ack tx error %i\n", res);
        state.active = FALSE;
      }
    } // finished?
  } // spi cb result
}

#if TEST
static void test_f(u32_t data, void *arg) {
  _comm_file_spif_cb(NULL, SPI_OK);
}
#endif

s32_t COMM_FILE_on_pkt(u8_t *data, u8_t len) {
  s32_t res = R_COMM_OK;
  u8_t reply;
  comm_file_hdr *h = (comm_file_hdr *)data;
  bool abort = FALSE;

  DBG(D_SYS, D_INFO, "comm file got request, seq: %08x\n", h->sequence);
  if (h->sequence == 0xffffffff) {
    // request to store file
    DBG(D_SYS, D_DEBUG, "comm file header, len: %i, name: %s\n", h->length, h->name);
    if (state.active) {
      DBG(D_SYS, D_WARN, "comm file header when already active\n");
      reply = COMM_FILE_ERR_UNEXPECTED;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }
    if (h->length > FLASH_TOTAL_SIZE) {
      DBG(D_SYS, D_WARN, "comm file header oversized\n");
      reply = COMM_FILE_ERR_NO_SPACE;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }
    reply = COMM_FILE_REPLY_OK;
    COMM_reply(&reply, 1);
    state.active = TRUE;
    state.req_sequence = 0;
    state.last_pkt_len = 0;
    state.index = 0;
    state.length = h->length;
    state.errors = 0;
#if TEST
    s32_t res = SPI_OK;
    TASK_run(test, 0, 0);
#else
    s32_t res = SPI_FLASH_erase(SPI_FLASH, _comm_file_spif_cb, FIRMWARE_SPIF_ADDRESS, h->length);
#endif
    if (res != SPI_OK) {
      DBG(D_SYS, D_WARN, "comm file pkt spif erase request failed %i\n", res);
      abort = TRUE;
    }
  } else {
    // request to store packet
    DBG(D_SYS, D_DEBUG, "comm file got pkt, seq:%i len: %i\n", h->sequence, len - COMM_FILE_PKT_OVERHEAD);
    if (!state.active) {
      DBG(D_SYS, D_WARN, "comm file pkt when inactive\n");
      reply = COMM_FILE_ERR_UNEXPECTED;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    } else if (len <= COMM_FILE_PKT_OVERHEAD) {
      DBG(D_SYS, D_WARN, "comm file pkt too small\n");
      reply = COMM_FILE_ERR_ABORT;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }
    reply = COMM_FILE_REPLY_OK;
    COMM_reply(&reply, 1);

    if (h->sequence == state.req_sequence) {
      // correct sequence
      state.last_pkt_len = len - COMM_FILE_PKT_OVERHEAD;
      state.errors = 0;

#if TEST
      s32_t res = SPI_OK;
      TASK_run(test, 0, 0);
#else
      s32_t res = SPI_FLASH_write(
          SPI_FLASH,
          _comm_file_spif_cb,
          FIRMWARE_SPIF_ADDRESS + h->sequence * COMM_FILE_MAX_DATA_PKT,
          len - COMM_FILE_PKT_OVERHEAD,
          &data[COMM_FILE_PKT_OVERHEAD]);
#endif
      if (res != SPI_OK) {
        DBG(D_SYS, D_WARN, "comm file pkt spif write request failed %i\n", res);
        abort = TRUE;
      }
    } else {
      // bad sequence
      DBG(D_SYS, D_WARN, "comm file pkt out of sequence: got %i, want %i\n", h->sequence, state.req_sequence);
      state.errors++;
      if (state.errors > COMM_FILE_MAX_ERRORS) {
        DBG(D_SYS, D_WARN, "comm file pkt oos error stop trying\n");
        abort = TRUE;
      } else {
        /*
        comm_file_ack a;
        a.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
        a.res = COMM_FILE_REPLY_OK;
        a.last_stored_pkt_in_sequence = state.req_sequence;
        // request next packet
        s32_t res = COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&a, sizeof(comm_file_ack), TRUE);
        if (res < R_COMM_OK) {
          DBG(D_SYS, D_WARN, "comm file pkt oos ack tx error %i\n", res);
          abort = TRUE;
        }
        */
      }
    }
  } // header or packet
  if (abort) {
    comm_file_ack a;
    DBG(D_SYS, D_WARN, "comm file pkt ABORT\n");
    a.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
    a.res = COMM_FILE_ERR_ABORT;
    a.last_stored_pkt_in_sequence = 0xffffffff;
    state.active = FALSE;
    COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&a, sizeof(comm_file_ack), TRUE);
  }
  return res;
}

void COMM_FILE_init() {
  memset(&state, 0, sizeof(state));
#if TEST
  test = TASK_create(test_f, TASK_STATIC);
#endif
}
