/*
 * comm_test.h
 *
 *  Created on: Jul 1, 2012
 *      Author: petera
 */

#ifndef COMM_TEST_H_
#define COMM_TEST_H_

#include "config.h"
#include "comm.h"

#define COMM_PKT_POOL_SIZE      10

comm *COMM_get_comm();
void COMM_init(uart *u);
void COMM_next_channel();
void COMM_set_uart(uart *u);
int COMM_dump();
int COMM_tx(int dst, u8_t* data, u16_t len, int ack);
int COMM_reply(u8_t *data, u16_t len);

#endif /* COMM_TEST_H_ */
