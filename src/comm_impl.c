#include "comm_impl.h"
#include "system.h"
#include "comm_proto_sys.h"
#include "comm_proto_cnc.h"
#include "comm_proto_file.h"

#define COMM_IMPL_PROTOCOL_HANDLER_REG  16

#if COMM_IMPL_STATS
static u32_t recd = 0;
static time clockpoint;
static u32_t avg_byps = 0;
static u32_t pktcount_rx = 0;
static u32_t pktcount_tx = 0;
static u32_t inseq_count = 0;
static u32_t outofseq_count = 0;
static u32_t resent_count = 0;
#endif

typedef struct {
  u16_t seqno;
  u8_t protocol_handler_id;
} prothand_reg;

static struct {
  // communication stack
  comm *driver;
  // last received sequence number
  u16_t last_seqno;
  // bit mask of last received sequence number
  u32_t seq_mask;
  // last received packet
  comm_arg *cur_rx;
  // beacon handler
  void (*comm_beacon_handler)(comm_addr addr, u8_t type, u16_t len, u8_t *data);
  // tx protocol handler registrations
  prothand_reg prothand_regs[COMM_IMPL_PROTOCOL_HANDLER_REG];
  u8_t prothand_ix;
} comm_state;

void COMM_set_stack(comm *driver,
    void (*comm_beacon_handler)(comm_addr addr, u8_t type, u16_t len, u8_t *data)) {
  comm_state.driver = driver;
  comm_state.comm_beacon_handler = comm_beacon_handler;
}

int COMM_tx(int dst, u8_t* data, u16_t len, int ack) {
  // TODO register protcol for ack
#if COMM_IMPL_STATS
  pktcount_tx++;
#endif
  s32_t res = comm_tx(comm_state.driver, dst, len, data, ack);
  if (res < R_COMM_OK) {
    DBG(D_COMM, D_WARN, "COMM tx failed %i\n", res);
  } else {
    if (ack) {
      // save this seqno for protocol handler sending this pkt when ack arrives
      if (comm_state.prothand_ix == 0) {
        comm_state.prothand_ix = COMM_IMPL_PROTOCOL_HANDLER_REG - 1;
      } else {
        comm_state.prothand_ix--;
      }
      comm_state.prothand_regs[comm_state.prothand_ix].seqno = res;
      comm_state.prothand_regs[comm_state.prothand_ix].protocol_handler_id = data[0];
    }
  }
  return res;
}

int COMM_send_alert() {
#if COMM_IMPL_STATS
  pktcount_tx++;
#endif
  s32_t res = comm_alert(comm_state.driver, 0xbb, 0, 0);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMM alert failed %i\n", res);
  return res;
}

int COMM_reply(u8_t *data, u16_t len) {
  s32_t res = comm_reply(comm_state.driver, comm_state.cur_rx, len, data);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMM reply failed %i\n", res);
  return res;
}


//typedef comm_time (*comm_app_get_time_fn)(void);
comm_time COMM_cb_get_tick_count() {
  return SYS_get_time_ms();
}

// ** stack

static s16_t seq_delta(u16_t seqnoCurrent, u16_t seqnoLastRegistered) {
  if (seqnoCurrent < 0x100 && seqnoLastRegistered > 0xeff) {
    return seqnoCurrent + 0x1000 - seqnoLastRegistered;
  } else if (seqnoCurrent > 0xeff && seqnoLastRegistered < 0x100) {
    return -(seqnoLastRegistered + 0x1000 - seqnoCurrent);
  } else {
    return seqnoCurrent - seqnoLastRegistered;
  }
}

/* received stuff from rx->src, in here one might to call comm_app_reply  */
//typedef int (*comm_app_user_rx_fn)(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data);
int COMM_cb_rx_pkt(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data) {
  s32_t res = R_COMM_OK;
  comm_state.cur_rx = rx;

  // detect packets that were resent but already received
  // keep track of latest 32 received packet sequence numbers
  s16_t seqd = seq_delta(rx->seqno, comm_state.last_seqno);
  u8_t already_received = FALSE;

  // check if this packet already was received
  if (seqd <= 0) {
    already_received = comm_state.seq_mask & (1 << (-seqd));
    // register this packet as received
    comm_state.seq_mask |= (1 << (-seqd));
  } else {
    // shift received packet mask
    comm_state.seq_mask <<= seqd;
    // register this packet as received
    comm_state.seq_mask |= 1;
  }

  DBG(D_COMM, D_DEBUG, "COMM PKT: seq:%03x len:%02x src:%01x flg:%02x last:%03x delta:%i %s\n",
      rx->seqno, rx->len, rx->src, rx->flags, comm_state.last_seqno, seqd, already_received ? "ALREADY RECEIVED": "FRESH");

  if (seqd > 0) {
    // got a new packet, register latest seqno
    comm_state.last_seqno = rx->seqno;
  }


#if COMM_IMPL_STATS
  {
    pktcount_rx++;
    if ((rx->flags & COMM_FLAG_RESENT_BIT)) {
      resent_count++;
    }
    if (seqd > 1 || seqd < 0) {
      outofseq_count++;
    } else if (seqd == 1) {
      inseq_count++;
    }

    time now = SYS_get_time_ms();
    recd += len;
    if (now - clockpoint >= 1000) {
      u32_t d = (u32_t)(now - clockpoint);
      u32_t byps = (recd * 1000) / d;
      avg_byps = 1 * byps / 32 + 31 *avg_byps / 32;
      DBG(D_COMM, D_INFO, "COMM STAT thru %i bytes/s avg:%i\n", byps, avg_byps);
      clockpoint = now;
      recd = 0;
    }
  }
#endif

#if 0
#if D_COMM
  {
    int i;
    char comm_state[256*3+2];
    for (i = 0; i < len; i++) {
      sprint(&comm_state[i*3], "%02x ", data[i]);
    }
    comm_state[len*3] = 0;
    DBG(D_COMM, D_DEBUG, "%s\n", comm_state);
  }
#endif
#endif

#ifdef CONFIG_CNC
  switch (data[0]) {
  case COMM_PROTOCOL_SYS_ID:
    res = COMM_SYS_on_pkt(rx->seqno, &data[1], len-1, already_received);
    break;
  case COMM_PROTOCOL_CNC_ID:
    res = COMM_CNC_on_pkt(rx->seqno, &data[1], len-1, already_received);
    break;
  case COMM_PROTOCOL_FILE_ID:
    res = COMM_FILE_on_pkt(&data[0], len, already_received);
    break;
  default:
    DBG(D_COMM, D_WARN, "COMM unrecognized protocol id %02x!\n", data[0]);
    break;
  }
#endif

  return res;
}

static u8_t comm_get_protocol_id_for_ack(u16_t seqno) {
  int i;
  for (i = 0; i < COMM_IMPL_PROTOCOL_HANDLER_REG; i++) {
    int ix = (comm_state.prothand_ix + i) % COMM_IMPL_PROTOCOL_HANDLER_REG;
    if (comm_state.prothand_regs[ix].seqno == seqno) {
      return comm_state.prothand_regs[ix].protocol_handler_id;
    }
  }
  return 0;
}

/* received an ack */
//typedef void (*comm_app_user_ack_fn)(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data);
void COMM_cb_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_DEBUG, "COMM ack seqno:0x%03x\n", seqno);
#ifdef CONFIG_CNC
  u8_t prot_id = comm_get_protocol_id_for_ack(seqno);
  switch (prot_id) {
  case COMM_PROTOCOL_SYS_ID:
    COMM_SYS_on_ack(seqno);
    break;
  case COMM_PROTOCOL_CNC_ID:
    COMM_CNC_on_ack(seqno);
    break;
  case COMM_PROTOCOL_FILE_ID:
    COMM_FILE_on_ack(seqno);
    break;
  }
#endif
}

/* invoked on error */
//typedef void (*comm_app_user_err_fn)(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data);
void COMM_cb_err(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_WARN, "COMM ERR %i seqno:0x%03x\n", err, seqno);
#ifdef CONFIG_CNC
  u8_t prot_id = comm_get_protocol_id_for_ack(seqno);
  switch (prot_id) {
  case COMM_PROTOCOL_SYS_ID:
    COMM_SYS_on_err(seqno, err);
    break;
  case COMM_PROTOCOL_CNC_ID:
    COMM_CNC_on_err(seqno, err);
    break;
  case COMM_PROTOCOL_FILE_ID:
    COMM_FILE_on_err(seqno, err);
    break;
  }
#endif
}

/* invoked on transport info */
//typedef void (*comm_app_user_inf_fn)(comm *comm, comm_arg *rx);
void COMM_cb_tra_inf(comm *comm, comm_arg *rx) {
  DBG(D_DEBUG, D_COMM, "COMM inf %02x\n", rx->data[0]);
}

void COMM_cb_alert(comm *comm, comm_addr addr, unsigned char type, unsigned short len, unsigned char *data) {
  DBG(D_DEBUG, D_COMM, "COMM node alert addr:%02x type:%02x\n", addr, type);
  if (comm_state.comm_beacon_handler) {
    comm_state.comm_beacon_handler(addr, type, len, data);
  }
}

void COMM_init() {
  memset(&comm_state, 0, sizeof(comm_state));
#if COMM_IMPL_STATS
  clockpoint = SYS_get_time_ms();
#endif
}

int COMM_dump() {
  int i,j;
  print("COMM\n----\n");
  print("  version %08x\n", COMM_CNC_VERSION);
#if COMM_IMPL_STATS
  print("  stat rx count:%i sequence:%i miss:%i resent:%i avgthruput:%ibytes/s\n",
      pktcount_rx, inseq_count, outofseq_count, resent_count, avg_byps);
  print("  stat tx count:%i\n",
      pktcount_tx);
#endif
  print("  lnk state:%i ix:%i len:%i l_crc:%04x buf:%p\n",
      comm_state.driver->lnk.state, comm_state.driver->lnk.ix, comm_state.driver->lnk.len, comm_state.driver->lnk.l_crc, comm_state.driver->lnk.buf);
  print("  nwk addr:%i of %i\n",
      comm_state.driver->nwk.addr, (1 + COMM_TRA_MAX_USERS));
  print("  tra seqno");
  for (i = 0; i < COMM_TRA_MAX_USERS; i++) {
    print(" [u:%i %03x]", i, comm_state.driver->tra.seqno[i]);
  }
  print("\n");
  for (j = 0; j < COMM_MAX_PENDING; j++) {
    print("  tra tx_pend[ix:%i bsy:%i seqno:%03x resends:%i]\n", j,
        comm_state.driver->tra.acks_tx_pend[j].busy,
        comm_state.driver->tra.acks_tx_pend[j].arg.seqno,
        comm_state.driver->tra.acks_tx_pend[j].resends);
  }
  for (i = 0; i < COMM_TRA_MAX_USERS; i++) {
    print("  tra rx_pend u:%i", i);
    for (j = 0; j < COMM_MAX_PENDING; j++) {
      print(" [ix:%i seqno:%04x]", j,
          comm_state.driver->tra.acks_rx_pend[i][j]);
    }
    print("\n");
  }

  return 0;
}


