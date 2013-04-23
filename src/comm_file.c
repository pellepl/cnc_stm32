#include "comm_file.h"
#include "comm_impl.h"
#include "cnc_comm.h"
#include "miniutils.h"
#include "spi_flash.h"
#include "spi_flash_m25p16.h"
#include "bl_exec.h"

#define COMM_FILE_PKT_OVERHEAD      (offsetof(comm_file_hdr, length))
#define COMM_FILE_MAX_DATA_PKT      (COMM_APP_MAX_DATA - COMM_FILE_PKT_OVERHEAD)

enum op {
  // erase flash
  OP_ERASE = 0,
  // store packet to flash
  OP_STORP,
  // read flash
  OP_READB,
  // wait for packet
  OP_WAITP,
  // write header to flash
  OP_HEADR,
};

typedef struct  __attribute__ (( packed )) {
  u8_t cmd; // always COMM_PROTOCOL_FILE_TRANSFER_R
  u32_t sequence;
  u32_t length;
  u16_t crc;
  u8_t name[8];
} comm_file_hdr ;

typedef struct  __attribute__ (( packed ))  {
  u8_t ack; // always COMM_PROTOCOL_FILE_TRANSFER_A
  u8_t res;
  u32_t last_stored_pkt_in_sequence;
} comm_file_ack;

static struct {
  // flagging if file tranceiving is active
  volatile bool active;
  // current operation
  volatile enum op op;

  // length of current file
  u32_t length;
  // index of r/w operation
  u32_t index;
  // required sequence during rx
  u32_t wanted_sequence;
  // length of last received packet
  u8_t last_pkt_len;
  // error counter
  u8_t errors;
  // rx crc
  u16_t rx_crc;
  // media crc
  u16_t media_crc;
  // server crc
  u16_t server_crc;
  // readback storage
  u8_t buf[COMM_APP_MAX_DATA]; //COMM_FILE_MAX_DATA_PKT preferred, but compiler thinks offsetof is variable
  // file meta data struct
  fw_upgrade_info fw_info;

  u8_t watchdog;
} state;

static void _comm_file_spif_cb_abort(u8_t err) {
  comm_file_ack ack;
  ack.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
  ack.res = (err);
  ack.last_stored_pkt_in_sequence = 0xffffffff;
  state.active = FALSE;
  COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&ack, sizeof(comm_file_ack), TRUE);
}

static void _comm_file_spif_cb_req_next();

/**
 * SPI flash callback
 */
static void _comm_file_spif_cb(spi_flash_dev *dev, int result) {
  s32_t res;
  int i;
  if (!state.active) {
    return;
  }

  if (result != SPI_OK) {
    // spi error
    DBG(D_SYS, D_WARN, "COMMFILE spi ERR %i\n", result);
    _comm_file_spif_cb_abort(COMM_FILE_ERR_ABORT);
    return;
  }

  // spi op succeeded

  // check state
  switch (state.op) {

  // everything erased
  case OP_ERASE: {
    DBG(D_SYS, D_DEBUG, "COMMFILE spi area erased\n");
    _comm_file_spif_cb_req_next();
    return;
  }

  // stored packet, read back and check
  case OP_STORP: {
    state.op = OP_READB;
    res = SPI_FLASH_read(
            SPI_FLASH,
            _comm_file_spif_cb,
            FIRMWARE_SPIF_ADDRESS_DATA + state.wanted_sequence * COMM_FILE_MAX_DATA_PKT,
            state.last_pkt_len,
            &state.buf[0]);
    if (res != SPI_OK) {
      DBG(D_SYS, D_WARN, "COMMFILE spi ERR read request failed %i\n", res);
      _comm_file_spif_cb_abort(COMM_FILE_ERR_ABORT);
    }
    return;
  }

  // read buffer finished
  case OP_READB: {
    // calc read buffer crc
    for (i = 0; i < state.last_pkt_len; i++) {
      state.media_crc = crc_ccitt_16(state.media_crc, state.buf[i]);
    }
    if (state.rx_crc != state.media_crc) {
      DBG(D_SYS, D_WARN, "COMMFILE spi ERR CRC differ rx: %04x rd: %04x ix: %08x\n", state.rx_crc, state.media_crc, state.index);
      _comm_file_spif_cb_abort(COMM_FILE_ERR_CRC_ERR);
      return;
    }

    state.index += state.last_pkt_len;
    state.wanted_sequence++;
    DBG(D_SYS, D_DEBUG, "COMMFILE spi %i%%: %i of %i\n", (state.index*100)/state.length, state.index, state.length);

    _comm_file_spif_cb_req_next();
    return;
  }

  // stored header
  case OP_HEADR: {
    state.active = FALSE;
    DBG(D_SYS, D_INFO, "COMMFILE FINISHED header and file stored\n");
    return;
  }

  // spi callback in bad state
  default: {
    DBG(D_SYS, D_WARN, "COMMFILE spi ERR callback in bad state: %i\n", state.op);
    _comm_file_spif_cb_abort(COMM_FILE_ERR_ABORT);
    return;
  }
  }
}

/**
 * Request next packet (from spi flash callback)
 */
static void _comm_file_spif_cb_req_next() {
  if (state.index < state.length) {
    // more to download, request next packet
    comm_file_ack ack;

    ack.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
    ack.res = COMM_FILE_REPLY_OK;
    ack.last_stored_pkt_in_sequence = state.wanted_sequence;
    DBG(D_SYS, D_DEBUG, "COMMFILE spi ack & req next %i\n", state.wanted_sequence);
    state.op = OP_WAITP;
    s32_t res = COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&ack, sizeof(comm_file_ack), TRUE);
    if (res < R_COMM_OK) {
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR spi nxt ack tx error %i\n", res);
      state.active = FALSE;
    }
  } else {
    // finished downloading
    if (state.rx_crc != state.server_crc || state.media_crc != state.server_crc) {
      // crc error
      DBG(D_SYS, D_WARN, "COMMFILE spi CRC ERR rx: %04x, rd: %04x, remote: %04x\n", state.rx_crc, state.media_crc, state.server_crc);
      _comm_file_spif_cb_abort(COMM_FILE_ERR_CRC_ERR);
      return;
    }
    // crc ok, store header
    DBG(D_SYS, D_DEBUG, "COMMFILE spi all received and verified, store header..\n");
    state.op = OP_HEADR;
    s32_t res = SPI_FLASH_write(
        SPI_FLASH,
        _comm_file_spif_cb,
        FIRMWARE_SPIF_ADDRESS_META,
        sizeof(fw_upgrade_info),
        (u8_t*)&state.fw_info);
    if (res != SPI_OK) {
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR spi write header failed %i\n", res);
      _comm_file_spif_cb_abort(COMM_FILE_ERR_ABORT);
    }
  }
}

/**
 * Got packet callback from comm stack
 */
s32_t COMM_FILE_on_pkt(u8_t *data, u8_t len) {
  s32_t res = R_COMM_OK;
  u8_t reply;
  comm_file_hdr *h = (comm_file_hdr *)data;
  bool abort_and_tx_stop = FALSE;

  DBG(D_SYS, D_DEBUG, "COMMFILE got request, seq: %08x\n", h->sequence);
  if (h->sequence == 0xffffffff) {
    // request to store file, accepted only when inactive
    DBG(D_SYS, D_INFO, "COMMFILE header, len: %i, name: %s, crc: %04x\n", h->length, h->name, h->crc);
    if (state.active) {
      // reply error and ignore packet
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR header when already active\n");
      reply = COMM_FILE_ERR_UNEXPECTED;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }
    if (h->length > FLASH_TOTAL_SIZE) {
      // reply error and ignore packet
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR header oversized\n");
      reply = COMM_FILE_ERR_NO_SPACE;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }
    reply = COMM_FILE_REPLY_OK;
    COMM_reply(&reply, 1);
    state.active = TRUE;
    state.wanted_sequence = 0;
    state.index = 0;
    state.length = h->length;
    state.server_crc = h->crc;
    state.errors = 0;
    state.rx_crc = 0xffff;
    state.media_crc = 0xffff;
    state.watchdog = 0;

    state.fw_info.crc = h->crc;
    state.fw_info.len = h->length;
    state.fw_info.magic = FW_MAGIC;
    strncpy((char*)&state.fw_info.fname[0], (char*)&h->name[0], sizeof(state.fw_info.fname)-1);

    DBG(D_SYS, D_DEBUG, "COMMFILE spi erase request @ %08x %i bytes\n", FIRMWARE_SPIF_ADDRESS_META, h->length);
    state.op = OP_ERASE;
    s32_t res = SPI_FLASH_erase(
        SPI_FLASH,
        _comm_file_spif_cb,
        FIRMWARE_SPIF_ADDRESS_META,
        h->length + sizeof(fw_upgrade_info));
    if (res != SPI_OK) {
      DBG(D_SYS, D_WARN, "COMMFILE spi ERR erase request failed %i\n", res);
      abort_and_tx_stop = TRUE;
    }
  } else {
    // request to store packet, accepted only in WAITP state
    DBG(D_SYS, D_DEBUG, "COMMFILE pkt, seq:%i len: %i\n", h->sequence, len - COMM_FILE_PKT_OVERHEAD);
    if (!state.active) {
      // reply error
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR inactive\n");
      reply = COMM_FILE_ERR_UNEXPECTED;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    } else if (state.op != OP_WAITP) {
      // reply ok and ignore packet
      DBG(D_SYS, D_INFO, "COMMFILE pkt ERR in wrong state\n");
      reply = COMM_FILE_REPLY_OK;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    } else if (len <= COMM_FILE_PKT_OVERHEAD) {
      // reply error and bail out
      DBG(D_SYS, D_WARN, "COMMFILE pkt ERR too small\n");
      state.active = FALSE;
      reply = COMM_FILE_ERR_ABORT;
      COMM_reply(&reply, 1);
      return R_COMM_OK;
    }

    state.watchdog = 0;

    reply = COMM_FILE_REPLY_OK;
    COMM_reply(&reply, 1);

    if (h->sequence == state.wanted_sequence) {
      // correct sequence
      int i;

      state.last_pkt_len = len - COMM_FILE_PKT_OVERHEAD;
      state.errors = 0;

      // save packet data in ram
      memcpy(&state.buf[0], &data[COMM_FILE_PKT_OVERHEAD], state.last_pkt_len);

      // calc received packet crc
      for (i = 0; i < state.last_pkt_len; i++) {
        state.rx_crc = crc_ccitt_16(state.rx_crc, state.buf[i]);
      }

      // write packet to flash
      state.op = OP_STORP;
      s32_t res = SPI_FLASH_write(
          SPI_FLASH,
          _comm_file_spif_cb,
          FIRMWARE_SPIF_ADDRESS_DATA + state.wanted_sequence * COMM_FILE_MAX_DATA_PKT,
          state.last_pkt_len,
          &state.buf[0]);
      if (res != SPI_OK) {
        DBG(D_SYS, D_WARN, "COMMFILE spi ERR write request failed %i\n", res);
        abort_and_tx_stop = TRUE;
      }
    } else {
      // bad sequence
      DBG(D_SYS, D_INFO, "COMMFILE pkt ERR out of sequence: got %i, want %i\n", h->sequence, state.wanted_sequence);
      state.errors++;
      if (state.errors > COMM_FILE_MAX_ERRORS) {
        DBG(D_SYS, D_WARN, "COMMFILE pkt ERR oos error stop trying\n");
        abort_and_tx_stop = TRUE;
      } else if (h->sequence > state.wanted_sequence) {
        // resend request if got future packet comparing to wanted
/*        comm_file_ack ack;
        ack.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
        ack.res = COMM_FILE_REPLY_OK;
        ack.last_stored_pkt_in_sequence = state.wanted_sequence;
        // request wanted packet
        state.op = OP_WAITP;
        s32_t res = COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&ack, sizeof(comm_file_ack), TRUE);
        if (res < R_COMM_OK) {
          DBG(D_SYS, D_WARN, "COMMFILE pkt ERR oos ack tx error %i\n", res);
          abort_and_tx_stop = TRUE;
        }
        */
      }
    }
  } // if header or packet

  if (abort_and_tx_stop) {
    comm_file_ack a;
    DBG(D_SYS, D_WARN, "COMMFILE pkt ERR -> ABORT\n");
    a.ack = COMM_PROTOCOL_FILE_TRANSFER_A;
    a.res = COMM_FILE_ERR_ABORT;
    a.last_stored_pkt_in_sequence = 0xffffffff;
    state.active = FALSE;
    COMM_tx(COMM_CONTROLLER_ADDRESS, (u8_t*)&a, sizeof(comm_file_ack), TRUE);
  }
  return res;
}

void COMM_FILE_watchdog() {
  if (state.active && state.watchdog++ > 5) {
    DBG(D_SYS, D_WARN, "COMMFILE ERR WATCHDOG reset\n");
    COMM_FILE_init();
  }
}

void COMM_FILE_init() {
  memset(&state, 0, sizeof(state));
}
