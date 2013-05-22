#include "comm_proto_sys.h"
#include "taskq.h"
#include "comm.h"
#include "comm_impl.h"

static struct {
  volatile u32_t alive_msg_seqno;
  volatile bool connected;
  task *task_alive;
  task_timer task_alive_timer;
  comm_sys_cb *connection_cb;
} comm_sys;

static void comm_sys_fire_callback(enum comm_sys_cb_event event) {
  comm_sys_cb *cur_cb = comm_sys.connection_cb;
  while (cur_cb != NULL) {
    if (cur_cb->cb) {
      cur_cb->cb(event);
    }
    cur_cb = cur_cb->_next;
  }
}

static void comm_sys_handle_comm_dead(s32_t err) {
  if (comm_sys.connected) {
    comm_sys.connected = FALSE;
#ifdef CONFIG_COMM_STATUS
    print("COMM disconnect\n");
#endif
    comm_sys_fire_callback(DISCONNECTED);
  }
}

static void comm_sys_alive_timer_task(u32_t ignore, void *ignore_more) {
#ifdef CONFIG_COMM_ALIVE_TICK
  u8_t buf[2];
  buf[0] = COMM_PROTOCOL_SYS_ID;
  buf[1] = COMM_PROTOCOL_SYS_ALIVE;
  s32_t res = COMM_tx(COMM_CONTROLLER_ADDRESS, &buf[0], sizeof(buf), TRUE);
  if (res >= R_COMM_OK) {
    comm_sys.alive_msg_seqno = res;
  } else {
    comm_sys_handle_comm_dead(res);
  }
#endif
  comm_sys_fire_callback(TICK);
}

void COMM_SYS_register_event_cb(comm_sys_cb *s, comm_sys_cb_fn f) {
  s->cb = f;
  s->_next = comm_sys.connection_cb;
  comm_sys.connection_cb = s;
}

bool COMM_SYS_is_connected() {
  return comm_sys.connected;
}

s32_t COMM_SYS_on_pkt(u16_t seqno, u8_t *data, u8_t len, bool already_received) {
  return R_COMM_OK;
}

void COMM_SYS_on_ack(u16_t seqno) {
  if (comm_sys.alive_msg_seqno != -1 && seqno == comm_sys.alive_msg_seqno) {
    if (!comm_sys.connected) {
      comm_sys_fire_callback(CONNECTED);
#ifdef CONFIG_COMM_STATUS
      print("COMM connect\n");
#endif
    }
    comm_sys.connected = TRUE;
  }
}

void COMM_SYS_on_err(u16_t seqno, s32_t err) {
  if (comm_sys.alive_msg_seqno != -1 && seqno == comm_sys.alive_msg_seqno) {
    comm_sys_handle_comm_dead(err);
  }
}

void COMM_SYS_init() {
  memset(&comm_sys, 0, sizeof(comm_sys));
  comm_sys.alive_msg_seqno = -1;
  comm_sys.task_alive = TASK_create(comm_sys_alive_timer_task, TASK_STATIC);
  TASK_start_timer(comm_sys.task_alive, &comm_sys.task_alive_timer, 0, NULL, 500, 2000, "comm_alive");
}
