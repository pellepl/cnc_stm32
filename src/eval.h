#ifndef _EVAL_H_
#define _EVAL_H_

typedef enum {
  STATE_INI = 0,
  STATE_NBR,
  STATE_OP,
  STATE_OP2a,
  STATE_OP2b,
  STATE_VAR,
  STATE_FUNC
} e_state;

#define COMMA         -3
#define LEFT_PH       -2
#define RIGHT_PH      -1

#define STACK_SIZE    20

typedef struct {
  unsigned int is_var:8;
  union {
    int integer;
    unsigned int var_hash;
  };
} stackelem;

typedef struct {
  stackelem stack[STACK_SIZE];
  int ix;
  int max_ix;
} stack;

void eval_init();
int eval(const char *in);

#endif // _EVAL_H_
