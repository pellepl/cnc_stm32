/*
 * cnc_comm.c
 *
 *  Created on: Aug 10, 2012
 *      Author: petera
 */


#include "comm_proto_cnc.h"
#include "comm_impl.h"
#include "comm_proto_sys.h"
#include "cnc_control.h"
#include "taskq.h"
#include "miniutils.h"
#include "heap.h"
#include "led.h"
#include "comm.h"
#include "nvstorage.h"
#include "comm_proto_file.h"


#ifdef CONFIG_CNC

#define COMM_CNC_MAX_STORED_LATCH_IDS 4

static struct {
  u16_t seqno[COMM_CNC_MAX_STORED_LATCH_IDS];
  u32_t latch_id[COMM_CNC_MAX_STORED_LATCH_IDS];
  u8_t ix;
} stored_latch_ids;

static task *task_sr;
static task *task_pos;
static task_timer task_sr_timer;
static task_timer task_pos_timer;
static u32_t sr_timer_recurrence = 1000;
static u32_t pos_timer_recurrence = 1000;
static comm_sys_cb event_cb;

#define itomem(i, b) \
  (b)[0] = (((i) & 0x000000ff)>>0); \
  (b)[1] = (((i) & 0x0000ff00)>>8); \
  (b)[2] = (((i) & 0x00ff0000)>>16); \
  (b)[3] = (((i) & 0xff000000)>>24);
#define memtoi(b) \
    ((((b)[0])) & 0x000000ff) | \
    (((b)[1] << 8) & 0x0000ff00) | \
    (((b)[2] << 16) & 0x00ff0000) | \
    (((b)[3] << 24) & 0xff000000)

static s32_t comm_cnc_handle_already_received_latch_cmd(u16_t seqno);

static void comm_cnc_event_cb(enum comm_sys_cb_event event) {
  if (event == DISCONNECTED) {
    u32_t sr = CNC_get_status();
    if ((sr & (1<<CNC_STATUS_BIT_PIPE_EMPTY)) == 0 ||
        (sr & (1<<CNC_STATUS_BIT_LATCH_FULL)) != 0) {
      // lost communication when having stuff in latch or pipe
      DBG(D_APP, D_WARN, "communication lost, stuff in latch or pipe\n");
      CNC_enable_error(1<<CNC_ERROR_BIT_COMM_LOST);
    }
    COMM_UART_next_channel();
  } else if (event == CONNECTED){
    LED_blink_single(LED_CNC_COMM_BIT, 2,1,0);
  }
}

void COMM_CNC_set_sr_timer_recurrence(u32_t delta) {
  if (delta > 10 || delta == 0) {
    sr_timer_recurrence = delta;
  } else {
    sr_timer_recurrence = 1000;
  }
}

void COMM_CNC_apply_sr_timer_recurrence() {
  TASK_set_timer_recurrence(&task_sr_timer,
      COMM_CNC_get_sr_timer_recurrence() == 0 ? 1000 : COMM_CNC_get_sr_timer_recurrence());
  CONFIG_store();
}

static void comm_cnc_set_and_apply_sr_timer_recurrence(u32_t delta) {
  COMM_CNC_set_sr_timer_recurrence(delta);
  COMM_CNC_apply_sr_timer_recurrence();
}

u32_t COMM_CNC_get_sr_timer_recurrence() {
  return sr_timer_recurrence;
}

void COMM_CNC_set_pos_timer_recurrence(u32_t delta) {
  if (delta > 10 || delta == 0) {
    pos_timer_recurrence = delta;
  } else {
    pos_timer_recurrence = 1000;
  }
}

void COMM_CNC_apply_pos_timer_recurrence() {
  TASK_set_timer_recurrence(&task_pos_timer,
      COMM_CNC_get_pos_timer_recurrence() == 0 ? 1000 : COMM_CNC_get_pos_timer_recurrence());
  CONFIG_store();
}

static void comm_cnc_set_and_apply_pos_timer_recurrence(u32_t delta) {
  COMM_CNC_set_pos_timer_recurrence(delta);
  COMM_CNC_apply_pos_timer_recurrence();
}

u32_t COMM_CNC_get_pos_timer_recurrence() {
  return pos_timer_recurrence;
}

u32_t COMM_CNC_get_version() {
  return COMM_CNC_VERSION;
}

s32_t COMM_CNC_on_pkt(u16_t seq, u8_t *data, u16_t len, bool already_received) {
  bool latch_cmd = FALSE;
  s32_t res = R_COMM_OK;
  u8_t cmd = *data++;
  len--;
  u32_t argc = len/4;
  void *f = NULL;
  u8_t gcall = TRUE;
  switch (cmd) {
  case COMM_PROTOCOL_INFO:
    f = COMM_CNC_get_version;
    break;
  case COMM_PROTOCOL_CNC_ENABLE:
    if (argc == 1) {
      f = CNC_set_enabled;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_enabled, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_GET_STATUS:
    if (argc == 0) {
      f = CNC_get_status;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_get_status, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SET_SR_MASK:
    if (argc == 1) {
      f = CNC_set_status_mask;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_status_mask, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_IS_LATCH_FREE:
    if (argc == 0) {
      f = CNC_is_latch_free;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_is_latch_free, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_CUR_MOTION_ID:
    if (argc == 0) {
      f = CNC_get_current_motion_id;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_get_current_motion_id, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SET_LATCH_ID:
    if (argc == 1) {
      f = CNC_set_latch_id;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_latch_id, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_PIPE_ENABLE:
    if (argc == 1) {
      f = CNC_pipeline_enable;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_pipeline_enable, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_PIPE_FLUSH:
    if (argc == 0) {
      f = CNC_pipeline_flush;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_pipeline_flush, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_LATCH_XYZ:
    if (argc == 7) {
      if (already_received) {
        return comm_cnc_handle_already_received_latch_cmd(seq);
      } else {
        latch_cmd = TRUE;
        f = CNC_latch_xyz;
      }
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_latch_xyz, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_LATCH_PAUSE:
    if (argc == 1) {
      if (already_received) {
        return comm_cnc_handle_already_received_latch_cmd(seq);
      } else {
        latch_cmd = TRUE;
        f = CNC_latch_pause;
      }
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_latch_pause, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SET_POS:
    if (argc == 3) {
      f = CNC_set_pos;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_pos, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SET_OFFS_POS:
    if (argc == 3) {
      f = CNC_set_offs_pos;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_offs_pos, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SET_IMM_XYZ:
    if (argc == 6) {
      f = CNC_set_regs_imm;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_set_regs_imm, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_SR_TIMER_DELTA:
    if (argc == 1) {
      f = comm_cnc_set_and_apply_sr_timer_recurrence;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on COMM_CNC_set_and_apply_sr_timer_recurrence, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_POS_TIMER_DELTA:
    if (argc == 1) {
      f = comm_cnc_set_and_apply_pos_timer_recurrence;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on COMM_CNC_set_and_apply_pos_timer_recurrence, %i\n", argc);
    }
    break;
  case COMM_PROTOCOL_RESET:
    if (argc == 0) {
      f = CNC_reset;
    } else {
      DBG(D_APP, D_WARN, "COMM_CNC: bad argc on COMM_CNC_reset, %i\n", argc);
    }
    break;
  default:
    gcall = FALSE;
    break;
  } // switch cmd generic handling

  if (gcall && f) {
    // generic call
    DBG(D_APP, D_DEBUG, "COMM_CNC: cmd %02x => func %08x\n", cmd, f);
    LED_blink_single(LED_CNC_COMM_BIT, 2,1,2);
    IF_DBG(D_APP, D_DEBUG) {
      int argcc;
      for (argcc = 0; argcc < argc; argcc++) {
        u8_t *p = &data[argcc*4];
        u32_t a = memtoi(p);
        DBG(D_APP, D_DEBUG, "COMM_CNC:   arg %i = %08x\n", argcc, a);
      }
    }
    void * fres = _variadic_call(f, argc, data);
    DBG(D_APP, D_DEBUG, "COMM_CNC: cmd %02x returned %08x\n", cmd, fres);
    u8_t buf[sizeof(u32_t)];
    itomem((u32_t)fres, buf);
    if (latch_cmd && (u32_t)fres != CNC_ERR_LATCH_BUSY) {
      // store latch_id for this latch command if we get a resend
      stored_latch_ids.seqno[stored_latch_ids.ix] = seq;
      stored_latch_ids.latch_id[stored_latch_ids.ix] = (u32_t)fres;
      if (stored_latch_ids.ix >= COMM_CNC_MAX_STORED_LATCH_IDS - 1) {
        stored_latch_ids.ix = 0;
      } else {
        stored_latch_ids.ix++;
      }
    }
    res = COMM_reply(buf, sizeof(buf));
  } else {
    switch (cmd) {
    case COMM_PROTOCOL_GET_POS:
    {
      if (argc == 0) {
        LED_blink_single(LED_CNC_COMM_BIT, 2,1,2);
        s32_t px;
        s32_t py;
        s32_t pz;
        CNC_get_pos(&px, &py, &pz);
        u8_t buf[sizeof(s32_t)*3];
        itomem(px, &buf[0]);
        itomem(py, &buf[4]);
        itomem(pz, &buf[8]);
        res = COMM_reply(buf, sizeof(buf));
      } else {
        DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_get_pos, %i\n", argc);
      }
      break;
    }
    case COMM_PROTOCOL_GET_OFFS_POS:
    {
      if (argc == 0) {
        LED_blink_single(LED_CNC_COMM_BIT, 2,1,2);
        s32_t px;
        s32_t py;
        s32_t pz;
        CNC_get_offs_pos(&px, &py, &pz);
        u8_t buf[sizeof(s32_t)*3];
        itomem(px, &buf[0]);
        itomem(py, &buf[4]);
        itomem(pz, &buf[8]);
        res = COMM_reply(buf, sizeof(buf));
      } else {
        DBG(D_APP, D_WARN, "COMM_CNC: bad argc on CNC_get_offs_pos, %i\n", argc);
      }
      break;
    }
    case COMM_PROTOCOL_CONFIG:
    {
      LED_blink_single(LED_CNC_COMM_BIT, 2,1,2);
      while (len >= 5) {
        u8_t conf = *data++;
        u32_t conf_val = memtoi(data);
        data += 4;
        CNC_set_config_specific(conf, conf_val);
        len -= 5;
      }
      u8_t buf[sizeof(u32_t)];
      itomem((u32_t)1, buf);
      res = COMM_reply(buf, sizeof(buf));
      break;
    }
    default:
      DBG(D_APP, D_WARN, "COMM_CNC: unknown command 0x%02x\n", cmd);
      break;
    } // switch cmd specific handling
  }
  return res;
}

void COMM_CNC_on_ack(u16_t seq) {
}

void COMM_CNC_on_err(u16_t seq, s32_t err) {
}

static s32_t comm_cnc_handle_already_received_latch_cmd(u16_t seqno) {
  s32_t res;
  u8_t buf[4];
  int i;
  for (i = 0; i < COMM_CNC_MAX_STORED_LATCH_IDS; i++) {
    if (stored_latch_ids.seqno[i] == seqno) {
      itomem(stored_latch_ids.latch_id[i], buf);
      break;
    }
  }
  if (i == COMM_CNC_MAX_STORED_LATCH_IDS) {
    DBG(D_APP, D_WARN, "got a resent latch command whose seq isn't registered %04x\n", seqno);
    itomem(CNC_ERR_LATCH_BUSY, buf);
  }
  res = COMM_reply(buf, sizeof(buf));

  return res;
}

static void cnc_sr_timer_task(u32_t ignore, void *ignore_more) {
  if (!COMM_SYS_is_connected()) return;
  if (sr_timer_recurrence && pos_timer_recurrence != sr_timer_recurrence) {
    u8_t buf[2 + sizeof(u32_t)];
    buf[0] = COMM_PROTOCOL_CNC_ID;
    buf[1] = COMM_PROTOCOL_EVENT_SR_TIMER;
    u32_t sr = CNC_get_status();
    itomem(sr, &buf[2]);
    COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), FALSE);
  }
}

static void cnc_pos_timer_task(u32_t ignore, void *ignore_more) {
  s32 x; s32 y; s32 z;
  if (!COMM_SYS_is_connected()) return;
  if (pos_timer_recurrence && pos_timer_recurrence != sr_timer_recurrence) {
    u8_t buf[2 + sizeof(u32_t)*3];
    buf[0] = COMM_PROTOCOL_CNC_ID;
    buf[1] = COMM_PROTOCOL_EVENT_POS_TIMER;
    CNC_get_pos(&x, &y, &z);
    itomem(x, &buf[2]);
    itomem(y, &buf[6]);
    itomem(z, &buf[10]);
    COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), FALSE);
  } else if (pos_timer_recurrence && pos_timer_recurrence == sr_timer_recurrence) {
    u8_t buf[2 + sizeof(u32_t)*4];
    buf[0] = COMM_PROTOCOL_CNC_ID;
    buf[1] = COMM_PROTOCOL_EVENT_SR_POS_TIMER;
    CNC_get_pos(&x, &y, &z);
    u32_t sr = CNC_get_status();
    itomem(sr, &buf[2]);
    itomem(x, &buf[6]);
    itomem(y, &buf[10]);
    itomem(z, &buf[14]);
    COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), FALSE);
  }
}

static void cnc_sr_cb_task(u32_t sr, void *ignore) {
  DBG(D_APP, D_DEBUG, "CNC callb: sr 0b%08b\n", sr);
  u8_t buf[2 + sizeof(u32_t)];
  buf[0] = COMM_PROTOCOL_CNC_ID;
  buf[1] = COMM_PROTOCOL_EVENT_SR;
  itomem(sr, &buf[2]);
  COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), FALSE);
}

static void cnc_sr_irq_cb(u32_t sr) {
  task *t = TASK_create(cnc_sr_cb_task, 0);
  TASK_run(t, sr, 0);
}

static void cnc_pos_irq_cb(s32_t x, s32_t y, s32_t z) {
  CONFIG_CNC_pos_store(x,y,z);
}

static void cnc_offs_irq_cb(s32_t x, s32_t y, s32_t z) {
  CONFIG_CNC_offs_store(x,y,z);
}

static void cnc_pipe_cb_task(u32_t id, void *ignore) {
  DBG(D_APP, D_DEBUG, "CNC callb: pipe id 0x%08x\n", id);
  u8_t buf[2 + sizeof(u32_t)];
  buf[0] = COMM_PROTOCOL_CNC_ID;
  buf[1] = COMM_PROTOCOL_EVENT_ID;
  itomem(id, &buf[2]);
  COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), FALSE);
}

static void cnc_pipe_irq_cb(u32_t id) {
  task *t = TASK_create(cnc_pipe_cb_task, 0);
  TASK_run(t, id, 0);
}

void COMM_CNC_init() {
  s32_t res;
  CNC_init(cnc_sr_irq_cb, cnc_pipe_irq_cb, cnc_pos_irq_cb, cnc_offs_irq_cb);
  res = CONFIG_CNC_pos_load();
  if (res != NV_OK) {
    DBG(D_APP, D_WARN, "cnc settings corrupt\n");
    CNC_enable_error(1<<CNC_ERROR_BIT_SETTINGS_CORRUPT);
  }
  print("Non-volatile position read, res %i\n", res);
  res = CONFIG_CNC_offs_load();
  if (res != NV_OK) {
    CNC_enable_error(1<<CNC_ERROR_BIT_SETTINGS_CORRUPT);
  }
  print("Non-volatile offset read, res %i\n", res);

  memset(&stored_latch_ids, 0xff, sizeof(stored_latch_ids));

  COMM_SYS_register_event_cb(&event_cb, comm_cnc_event_cb);

  task_sr = TASK_create(cnc_sr_timer_task, TASK_STATIC);
  TASK_start_timer(task_sr, &task_sr_timer, 0, NULL, 500, 0, "cnc_sr");
  COMM_CNC_apply_sr_timer_recurrence();
  task_pos = TASK_create(cnc_pos_timer_task, TASK_STATIC);
  TASK_start_timer(task_pos, &task_pos_timer, 0, NULL, 500, 0, "cnc_pos");
  COMM_CNC_apply_pos_timer_recurrence();
}

#endif // CONFIG_CNC
