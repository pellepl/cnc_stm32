/*
 * heap.c
 *
 *  Created on: Jul 20, 2012
 *      Author: petera
 */

#include "heap.h"
#include "tinyheap.h"
#include "system.h"

static u8_t _heap_data[TH_BLOCKSIZE * (127)];
static tinyheap _heap;

void *HEAP_malloc(unsigned int size) {
  void *p = th_malloc(&_heap, size);;
  if (p == NULL) {
    DBG(D_HEAP, D_FATAL, "HEAP alloc %i return 0\n", size);
  }
  return p;
}
void HEAP_free(void* p) {
  th_free(&_heap, p);
}
void HEAP_init() {
  DBG(D_HEAP, D_DEBUG, "HEAP init\n");
  memset(&_heap_data[0], 0xee, sizeof(_heap_data));
  th_init(&_heap, (void*)&_heap_data[0], sizeof(_heap_data));
#if TH_CALC_FREE
  DBG(D_HEAP, D_DEBUG, "     %i bytes free\n", th_freecount(&_heap));
#endif
}

void HEAP_dump() {
  th_dump(&_heap);
}
