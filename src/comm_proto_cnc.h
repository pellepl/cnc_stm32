/*
 * comm_cnc.h
 *
 *  Created on: Aug 10, 2012
 *      Author: petera
 */

#ifndef COMM_CNC_H_
#define COMM_CNC_H_

#include "system.h"
#include "comm.h"

#define COMM_PROTOCOL_CNC_ID              0x01

#define COMM_CNC_VERSION                  0x00010000

#define COMM_PROTOCOL_INFO                0x00
#define COMM_PROTOCOL_CNC_ENABLE          0x01
#define COMM_PROTOCOL_GET_STATUS          0x02
#define COMM_PROTOCOL_SET_SR_MASK         0x03
#define COMM_PROTOCOL_IS_LATCH_FREE       0x04
#define COMM_PROTOCOL_CUR_MOTION_ID       0x05
#define COMM_PROTOCOL_SET_LATCH_ID        0x06
#define COMM_PROTOCOL_PIPE_ENABLE         0x07
#define COMM_PROTOCOL_PIPE_FLUSH          0x08
#define COMM_PROTOCOL_LATCH_XYZ           0x09
#define COMM_PROTOCOL_LATCH_PAUSE         0x0a
#define COMM_PROTOCOL_SET_POS             0x0b
#define COMM_PROTOCOL_SET_IMM_XYZ         0x0c
#define COMM_PROTOCOL_SR_TIMER_DELTA      0x0d
#define COMM_PROTOCOL_POS_TIMER_DELTA     0x0e

#define COMM_PROTOCOL_CONFIG              0x10
#define COMM_PROTOCOL_CONFIG_MAX_X_FREQ   0x01
#define COMM_PROTOCOL_CONFIG_MAX_Y_FREQ   0x02
#define COMM_PROTOCOL_CONFIG_MAX_Z_FREQ   0x03
#define COMM_PROTOCOL_CONFIG_RAPID_X_D    0x11
#define COMM_PROTOCOL_CONFIG_RAPID_Y_D    0x12
#define COMM_PROTOCOL_CONFIG_RAPID_Z_D    0x13

#define COMM_PROTOCOL_GET_POS             0x20
#define COMM_PROTOCOL_SET_OFFS_POS        0x21
#define COMM_PROTOCOL_GET_OFFS_POS        0x22

#define COMM_PROTOCOL_EVENT_SR_TIMER      0xe1
#define COMM_PROTOCOL_EVENT_POS_TIMER     0xe2
#define COMM_PROTOCOL_EVENT_SR_POS_TIMER  0xe3
#define COMM_PROTOCOL_EVENT_SR            0xe4
#define COMM_PROTOCOL_EVENT_ID            0xe5

#define COMM_PROTOCOL_RESET               0xfe

#define COMM_PROTOCOL_ALIVE               0xff

void COMM_CNC_init();
u32_t COMM_CNC_get_version();
void COMM_CNC_set_sr_timer_recurrence(u32_t delta);
u32_t COMM_CNC_get_sr_timer_recurrence();
void COMM_CNC_apply_sr_timer_recurrence();
void COMM_CNC_set_pos_timer_recurrence(u32_t delta);
u32_t COMM_CNC_get_pos_timer_recurrence();
void COMM_CNC_apply_pos_timer_recurrence();
s32_t COMM_CNC_on_pkt(u16_t seq, u8_t *data, u16_t len, bool already_received);
void COMM_CNC_on_ack(u16_t seq);
void COMM_CNC_on_err(u16_t seq, s32_t err);

#endif /* COMM_CNC_H_ */
