/*
 * heap.h
 *
 *  Created on: Jul 20, 2012
 *      Author: petera
 */

#ifndef HEAP_H_
#define HEAP_H_

void *HEAP_malloc(unsigned int size);
void HEAP_free(void* p);
void HEAP_init();
void HEAP_dump();

#endif /* HEAP_H_ */
