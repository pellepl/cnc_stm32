/*
 * comm_proto_sys.h
 *
 *  Created on: May 22, 2013
 *      Author: petera
 */

#include "system.h"
#include "comm.h"

#ifndef COMM_PROTO_SYS_H_
#define COMM_PROTO_SYS_H_

#define COMM_PROTOCOL_SYS_ID             0xff

#define COMM_PROTOCOL_SYS_ALIVE          0x00

enum comm_sys_cb_event {
  TICK = 0,
  CONNECTED,
  DISCONNECTED
};

typedef void (*comm_sys_cb_fn)(enum comm_sys_cb_event event);

typedef struct comm_sys_cb_s {
  comm_sys_cb_fn cb;
  struct comm_sys_cb_s *_next;
} comm_sys_cb;

void COMM_SYS_init();
bool COMM_SYS_is_connected();
s32_t COMM_SYS_send_alive_packet();
void COMM_SYS_register_event_cb(comm_sys_cb *s, comm_sys_cb_fn f);
s32_t COMM_SYS_on_pkt(u16_t seqno, u8_t *data, u8_t len, bool already_received);
void COMM_SYS_on_ack(u16_t seqno);
void COMM_SYS_on_err(u16_t seqno, s32_t err);


#endif /* COMM_PROTO_SYS_H_ */
