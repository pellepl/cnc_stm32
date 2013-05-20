/*
 * comm_impl.h
 *
 *  Created on: Jul 1, 2012
 *      Author: petera
 */

#ifndef COMM_IMPL_H_
#define COMM_IMPL_H_

#include "config.h"
#include "comm.h"

#define COMM_PKT_POOL_SIZE      10

comm *COMM_UART_get_comm();
void COMM_UART_init(uart *u);
void COMM_UART_next_channel();
void COMM_UART_set_uart(uart *u);

comm * COMM_UDP_get_comm();
void COMM_UDP_init();

void COMM_UDP_beacon_handler(comm_addr a, u8_t type, u16_t len, u8_t *data);

void COMM_init();
void COMM_set_stack(comm *driver, void (*comm_beacon_handler)(comm_addr addr, u8_t type, u16_t len, u8_t *data));
int COMM_dump();
int COMM_tx(int dst, u8_t* data, u16_t len, int ack);
int COMM_send_alert();
int COMM_reply(u8_t *data, u16_t len);

comm_time COMM_cb_get_tick_count();
int COMM_cb_rx_pkt(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data);
void COMM_cb_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data);
void COMM_cb_err(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data);
void COMM_cb_tra_inf(comm *comm, comm_arg *rx);
void COMM_cb_alert(comm *comm, comm_addr addr, unsigned char type, unsigned short len, unsigned char *data);

#endif /* COMM_IMPL_H_ */
