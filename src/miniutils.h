/*
 * miniutils.h
 *
 *  Created on: Jul 7, 2012
 *      Author: petera
 */

#ifndef MINIUTILS_H_
#define MINIUTILS_H_

#include <stdarg.h>
#include "uart.h"
#include "system.h"

#define __ITOA_NEGATE_B         0
#define __ITOA_NO_ZERO_END_B    1
#define __ITOA_FILL_SPACE_B     2
#define __ITOA_FORCE_SIGN_B     3
#define __ITOA_BASE_SIG_B       4
#define __ITOA_CAPITALS_B       5

#define ITOA_NEGATE             (1<<__ITOA_NEGATE_B)
#define ITOA_NO_ZERO_END        (1<<__ITOA_NO_ZERO_END_B)
#define ITOA_FILL_SPACE         (1<<__ITOA_FILL_SPACE_B)
#define ITOA_FORCE_SIGN         (1<<__ITOA_FORCE_SIGN_B)
#define ITOA_BASE_SIG           (1<<__ITOA_BASE_SIG_B)
#define ITOA_CAPITALS           (1<<__ITOA_CAPITALS_B)

#ifdef USE_COLOR_CODING
#define TEXT_GOOD(s) "\033[1;32m"s"\033[m"
#define TEXT_BAD(s) "\033[1;35m"s"\033[m"
#define TEXT_NOTE(s) "\033[1;36m"s"\033[m"
#else
#define TEXT_GOOD(s) s
#define TEXT_BAD(s) s
#define TEXT_NOTE(s) s
#endif

typedef struct {
  char* s;
  char* wrk;
  int len;
} cursor;

typedef enum {
  INT,
  STR
} strarg_type;

typedef struct {
  strarg_type type;
  union {
    int val;
    char* str;
  };
  int len;
} strarg;


void print(const char* f, ...);
void uprint(int uart, const char* f, ...);
void sprint(char *s, const char* f, ...);
void vprint(const char* f, va_list arg_p);
void vuprint(int uart, const char* f, va_list arg_p);

void v_printf(long p, const char* f, va_list arg_p);

void printbuf(u8_t *buf, u16_t len);

void u_itoa(unsigned int v, char* dst, int base, int num, int flags);
void itoa(int v, char* dst, int base);
void itoan(int v, char* dst, int base, int num);
int atoin(const char* s, int base, int len);
int strlen(const char* c);
int strnlen(const char *c, int size);
int strcmp(const char* s1, const char* s2);
int strncmp(const char* s1, const char* s2, int len);
char* strncpy(char* d, const char* s, int num);
char* strcpy(char* d, const char* s);
const char* strchr(const char* str, int ch);
char* strpbrk(const char* str, const char* key);
char* strstr(const char* str1, const char* str2);
unsigned short crc_ccitt_16(unsigned short crc, unsigned char data);
unsigned int rand(unsigned int seed);
void rand_seed(unsigned int seed);
unsigned int rand_next();
void quicksort(int* orders, void** pp, int elements);
void quicksortCmp(int* orders, void** pp, int elements,
    int(*orderfunc)(void* p));
void strarg_init(cursor *c, char* s, int len);
int strarg_next(cursor *c, strarg* type);

#endif /* MINIUTILS_H_ */
