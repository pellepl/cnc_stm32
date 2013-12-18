/*
 * comm_config.h
 *
 *  Created on: Jun 8, 2012
 *      Author: petera
 */

#ifndef COMM_CONFIG_H_
#define COMM_CONFIG_H_

#include "miniutils.h"
#include "system.h"

/* Number of consecutive phy timeouts before reporting to user, set to 0 to avoid tmo reports */
#define COMM_MAX_CONSEQ_TMO         0
/* Ignored when no user differentiation, otherwise affects ram usage */
#define COMM_MAX_USERS              2
/* Maximum pending acks to keep track of (both for rx and tx) */
#define COMM_MAX_PENDING            4
/* maximum number of times to resend an unacked pkt before reporting error to user */
#define COMM_MAX_RESENDS            5
/* comm_time value to wait before resending an unacked pkt */
#define COMM_RESEND_TICK(resends)   200
/* Disable to let everyone listen to everyone, enable to have directed communication */
#define COMM_USER_DIFFERENTIATION   1
/* If 1 acks are sent directly after rxing, if 0 acks are sent in tick call */
#define COMM_ACK_DIRECTLY           1
/* Set to zero to dispatch all pending acks in tick, non-zero to throttle acks per tick */
#define COMM_ACK_THROTTLE           0
/* Enable to use two alternating rx buffers in link layer, otherwise one is used */
#define COMM_LNK_DOUBLE_RX_BUFFER   0
/* Enable to get callbacks when an rx buffer is needed, handle it yourself */
#define COMM_LNK_ALLOCATE_RX_BUFFER    1

#define COMM_LOCK(comm)
#define COMM_UNLOCK(comm)

typedef time comm_time;

#define COMM_MEMSET(d, n, l) memset((d), (n), (l))
#define COMM_MEMCPY(d, s, l) memcpy((d), (s), (l))

#define COMM_DBG 0
#if COMM_DBG
#define COMM_PHY_DBG(x, ...)   print("COMM_PHY:"x"\n", ##__VA_ARGS__)
#define COMM_LNK_DBG(x, ...)   /*do {print("%i ", comm->nwk.addr);print("COMM_LNK:"x"\n", ##__VA_ARGS__);}while(0)*/
#define COMM_NWK_DBG(x, ...)   print("COMM_NWK:"x"\n", ##__VA_ARGS__)
#define COMM_TRA_DBG(x, ...)   print("COMM_TRA:"x"\n", ##__VA_ARGS__)
#else
#define COMM_PHY_DBG(x, ...)
#define COMM_LNK_DBG(x, ...)
#define COMM_NWK_DBG(x, ...)
#define COMM_TRA_DBG(x, ...)
//#define COMM_TRA_DBG(x, ...)   print("COMM_TRA:"x"\n", ##__VA_ARGS__)
#endif

#endif /* COMM_CONFIG_H_ */
