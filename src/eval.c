#include "system.h"
#include "eval.h"
#include "miniutils.h"

typedef struct {
  unsigned int hash;
  int val;
} var_elem;

static stack stack_op;
static stack stack_num;
static var_elem var_stack[256];
static int var_stack_ix = 0;
static stackelem null_stackelem;
static unsigned short ix_table[100*2];


//#define DBG_EVAL(...)
#define DBG_EVAL(...) print( __VA_ARGS__)

static void stack_init(stack* s) {
  s->ix = 0;
  s->max_ix = 0;
}

//static stackelem* stack_peek(stack* s) {
//  return &s->stack[s->ix-1];
//}

static int stack_peek_int(stack* s) {
  if (s->stack[s->ix-1].is_var) DBG_EVAL("ERROR \n");
  return s->stack[s->ix-1].integer;
}

static void stack_push_int(stack* s, int n) {
  s->stack[s->ix].is_var = 0;
  s->stack[s->ix++].integer = n;
  if (s->ix > s->max_ix) s->max_ix = s->ix;
}

static void stack_push_var(stack* s, unsigned int hash) {
  s->stack[s->ix].is_var = 1;
  s->stack[s->ix++].var_hash = hash;
  if (s->ix > s->max_ix) s->max_ix = s->ix;
}

static void stack_push_elem(stack* s, stackelem* e) {
  s->stack[s->ix].is_var = e->is_var;
  s->stack[s->ix++].integer = e->integer;
  if (s->ix > s->max_ix) s->max_ix = s->ix;
}

static stackelem* stack_pop(stack* s) {
  return &s->stack[--s->ix];
}

static int stack_pop_int(stack* s) {
  if (s->stack[s->ix-1].is_var) DBG_EVAL("ERROR\n");
  return s->stack[--s->ix].integer;
}

static int stack_empty(stack* s) {
  return s->ix == 0;
}

static unsigned int var_hash(const char *var_name, unsigned int len) {
  unsigned int h = 0;
  while (len--) {
    h = (31 * h + (*var_name++)) ^ (h >> (32-5));
  }
  return h;
}

static int var_get(unsigned int var_hash) {
  int i;
  for (i = var_stack_ix; i >= 0; i--) {
    if (var_stack[i].hash != 0 && var_stack[i].hash == var_hash) {
      return var_stack[i].val;
    }
  }
  DBG_EVAL("ERROR: no such variable\n");
  return -1;
}

static void var_set(unsigned int var_hash, int n) {
  int i;
  int var_stack_found_ix = -1;

  for (i = var_stack_ix; i >= 0; i--) {
    if (var_stack[i].hash != 0 && var_stack[i].hash == var_hash) {
      var_stack_found_ix = i;
      break;
    }
  }
  if (var_stack_found_ix == -1) {
    var_stack[var_stack_ix].hash = var_hash;
    var_stack[var_stack_ix].val = n;
    var_stack_ix++;
  } else {
    var_stack[var_stack_found_ix].hash = var_hash;
    var_stack[var_stack_found_ix].val = n;
  }
}

static void tokenize(const char* str, unsigned int strlen, unsigned short* ix_table, unsigned int* ix_table_len) {
  unsigned int ix_table_ix = 0;
  unsigned int six;
  unsigned int opix = 0;
  unsigned int oplen = 0;
  e_state state = STATE_INI;
  for (six = 0; six < strlen; six++) {
    char c = str[six];
    // symbols
    if (c == '$' && state != STATE_VAR) {
      if (state != STATE_NBR && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six;
      }
      state = STATE_VAR;
      oplen++;
    }
    else if (((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') &&
        state != STATE_FUNC && oplen == 0) {
      if (state != STATE_FUNC && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six;
      }
      state = STATE_FUNC;
      oplen++;
    }
    else if (((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') &&
        (state == STATE_VAR || state == STATE_FUNC)) {
      oplen++;
    } else if (c >= '0' && c <= '9') {
      if (state != STATE_NBR && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six;
      }
      state = STATE_NBR;
      oplen++;
    }
    // operators single
    else if (c == '+' || c == '-' || c == '*' || c == '/' || c == '%' || c == '('
          || c == ')' || c == '!' || c == '^' || c == '?' || c == ':'  || c == '~'
          || c == ',') {
      if (state != STATE_OP && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six;
      }
      state = STATE_OP;
      oplen++;
      ix_table[ix_table_ix++] = opix;
      ix_table[ix_table_ix++] = oplen;
      oplen = 0;
      opix = six+1;
    // operators double
    } else if (c == '=' || c == '>' || c == '<') {
      if (state != STATE_OP2a && state != STATE_OP2b && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six;
        state = STATE_OP2a;
      }
      oplen++;
      if (state == STATE_OP2b) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
        oplen = 0;
        opix = six+1;
      }
      if (state == STATE_OP2a) {
        state = STATE_OP2b;
      }
    // crap
    } else {
      if (state != STATE_INI && oplen > 0) {
        ix_table[ix_table_ix++] = opix;
        ix_table[ix_table_ix++] = oplen;
      }
      oplen = 0;
      opix = six+1;
      state = STATE_INI;
    }
  } // for string
  if (state != STATE_INI && oplen > 0) {
    ix_table[ix_table_ix++] = opix;
    ix_table[ix_table_ix++] = oplen;
  }
  *ix_table_len = ix_table_ix;
}

static int strncmpx(const char* str_fix, const char* str, unsigned short len) {
  if (strlen(str_fix) != len) {
    return -1;
  } else {
    return strncmp(str_fix, str, len);
  }
}

int op_prio(const char* str, unsigned short ix, unsigned short len) {
  const char* s = &str[ix];
  if (strncmpx("!", s, len) == 0) {
    return 1000;
  } else if (strncmpx("~", s, len) == 0) {
    return 1000;
  } else if (strncmpx("|", s, len) == 0) {
    return 1000;
  } else if (strncmpx("&", s, len) == 0) {
    return 1000;
  } else if (strncmpx(">>", s, len) == 0) {
    return 1000;
  } else if (strncmpx("<<", s, len) == 0) {
    return 1000;
  } else if (strncmpx("^", s, len) == 0) {
    return 500;
  } else if (strncmpx("?", s, len) == 0) {
    return 7;
  } else if (strncmpx(":", s, len) == 0) {
    return 6;
  } else if (strncmpx("+", s, len) == 0) {
    return 100;
  } else if (strncmpx("-", s, len) == 0) {
    return 200;
  } else if (strncmpx("/", s, len) == 0) {
    return 300;
  } else if (strncmpx("%", s, len) == 0) {
    return 300;
  } else if (strncmpx("*", s, len) == 0) {
    return 400;
  }  else if (strncmpx("=", s, len) == 0) {
    return 5;
  } else if (strncmpx("==", s, len) == 0) {
    return 2000;
  } else if (strncmpx("<>", s, len) == 0) {
    return 2000;
  } else if (strncmpx(">=", s, len) == 0) {
    return 2000;
  } else if (strncmpx(">", s, len) == 0) {
    return 2000;
  } else if (strncmpx("<=", s, len) == 0) {
    return 2000;
  } else if (strncmpx("<", s, len) == 0) {
    return 2000;
  } else if (strncmpx("(", s, len) == 0) {
    return LEFT_PH;
  } else if (strncmpx(")", s, len) == 0) {
    return RIGHT_PH;
  } else if (strncmpx(",", s, len) == 0) {
    return COMMA;
  }
  return 0;
}

static int op_exe(stackelem* slh, stackelem* srh, const char* str, unsigned short ix, unsigned short len, stack* stack_op, stack* stack_num, unsigned short* ix_table) {
  int lh, rh;
  if (slh->is_var && (strncmpx("=", &str[ix], len) != 0)) {
    lh = var_get(slh->var_hash);
  } else {
    lh = slh->integer;
  }
  if (srh->is_var) {
    rh = var_get(srh->var_hash);
  } else {
    rh = srh->integer;
  }
  if (strncmpx("+", &str[ix], len) == 0) {
    return lh+rh;
  }
  else if (strncmpx("-", &str[ix], len) == 0) {
    return lh-rh;
  }
  else if (strncmpx("/", &str[ix], len) == 0) {
    return lh/rh;
  }
  else if (strncmpx("%", &str[ix], len) == 0) {
    return lh%rh;
  }
  else if (strncmpx("*", &str[ix], len) == 0) {
    return lh*rh;
  }
  else if (strncmpx(">>", &str[ix], len) == 0) {
    return lh>>rh;
  }
  else if (strncmpx("<<", &str[ix], len) == 0) {
    return lh<<rh;
  }
  else if (strncmpx("|", &str[ix], len) == 0) {
    return lh|rh;
  }
  else if (strncmpx("!", &str[ix], len) == 0) {
    stack_push_elem(stack_num, slh);
    return !rh;
  }
  else if (strncmpx("~", &str[ix], len) == 0) {
    stack_push_elem(stack_num, slh);
    return ~rh;
  }
  else if (strncmpx("&", &str[ix], len) == 0) {
    return lh&rh;
  }
  else if (strncmpx("^", &str[ix], len) == 0) {
    return lh^rh;
  }
  else if (strncmpx("==", &str[ix], len) == 0) {
    return lh==rh;
  }
  else if (strncmpx("<>", &str[ix], len) == 0) {
    return lh!=rh;
  }
  else if (strncmpx(">", &str[ix], len) == 0) {
    return lh>rh;
  }
  else if (strncmpx("<", &str[ix], len) == 0) {
    return lh<rh;
  }
  else if (strncmpx(">=", &str[ix], len) == 0) {
    return lh>=rh;
  }
  else if (strncmpx("<=", &str[ix], len) == 0) {
    return lh<=rh;
  }
  else if (strncmpx(":", &str[ix], len) == 0) {
    int op = stack_pop_int(stack_op);
    if (strncmpx("?", &str[ix_table[op]], ix_table[op+1]) != 0) {
      // error, no ? on stack for ternary
      return -1;
    }
    stackelem* eval = stack_pop(stack_num);
    int eval_ok = eval->is_var ? var_get(eval->var_hash) : eval->integer;
    return eval_ok ? lh:rh;
  }
  else if (strncmpx("=", &str[ix], len) == 0) {
    DBG_EVAL("[R%08x=%i] ", slh->var_hash, rh);
    var_set(slh->var_hash, rh);
    return rh;
  }
  // error
  return 0;
}

static int op_exe_d(stackelem* lh, stackelem* rh, const char* str, unsigned short ix, unsigned short len, stack* stack_op, stack* stack_num, unsigned short* ix_table) {
  char op[16];
  strncpy(op, &str[ix], len);
  op[len] = 0;
  DBG_EVAL("%c%08x %s %c%08x => ",
         lh->is_var?'R':' ',
             lh->is_var?lh->var_hash:lh->integer,
                 op,
                 rh->is_var?'R':' ',
                     rh->is_var?rh->var_hash:rh->integer);
  int res = op_exe(lh, rh, str, ix, len, stack_op, stack_num, ix_table);
  DBG_EVAL("%i\n", res);
  return res;
}

static int exe(const char* str, unsigned short* ix_table, unsigned short ix_table_len) {
  int res = -1;
  stack_init(&stack_op);
  stack_init(&stack_num);

  DBG_EVAL("building stack\n");
  int i;
  for (i = 0; i < ix_table_len; i += 2) {
    unsigned short token_ix = ix_table[i];
    unsigned short token_len = ix_table[i+1];
    int cur_prio = op_prio(str, token_ix, token_len);
    if (cur_prio != 0) {
      // got us an operator
      int prio_top = stack_empty(&stack_op) ?
          0 :
          op_prio(str, ix_table[stack_peek_int(&stack_op)], ix_table[stack_peek_int(&stack_op)+1]);
      int prio_this = cur_prio;
      while (prio_this != LEFT_PH
          && (prio_this == RIGHT_PH || prio_this == COMMA || prio_this <= prio_top)) {
        stackelem* rh = stack_pop(&stack_num);
        stackelem* lh = stack_empty(&stack_num) ? &null_stackelem : stack_pop(&stack_num);
        int op = stack_pop_int(&stack_op);
        res = op_exe_d(lh, rh, str, ix_table[op], ix_table[op+1], &stack_op, &stack_num, ix_table);
        stack_push_int(&stack_num, res);
        prio_top = stack_empty(&stack_op) ?
            0 :
            op_prio(str, ix_table[stack_peek_int(&stack_op)], ix_table[stack_peek_int(&stack_op)+1]);
        if (prio_top == LEFT_PH && prio_this == RIGHT_PH) {
          // pop away right paranthesis
          stack_pop(&stack_op);
          break;
        }
      } // collapsing due to prio inversion
      if (cur_prio != RIGHT_PH && cur_prio != COMMA) {
        // push operator (by pushing index)
        DBG_EVAL("push op prio %i\n", cur_prio);
        stack_push_int(&stack_op, i);
      } else {
        DBG_EVAL("op prio %i (not pushed)\n", cur_prio);
      }
    } else {
      // got us a value
      if (str[ix_table[i]] == '$') {
        unsigned int hash = var_hash(&str[ix_table[i]], ix_table[i+1]);
        DBG_EVAL("push var %08x\n", hash);
        stack_push_var(&stack_num, hash);
      } else {
        DBG_EVAL("push val %i\n", atoin(&str[ix_table[i]], 10, ix_table[i+1]));
        stack_push_int(&stack_num, atoin(&str[ix_table[i]], 10, ix_table[i+1]));
      }
    }
  }

  DBG_EVAL("collapsing stack\n");
  if (!stack_empty(&stack_num)) {
    stackelem rrh;
    stackelem* rh = stack_pop(&stack_num);
    rrh.is_var = rh->is_var;
    if (rh->is_var) {
      rrh.var_hash = rh->var_hash;
    } else {
      rrh.integer = rh->integer;
    }
    if (!stack_empty(&stack_op)) {
      while (!stack_empty(&stack_op)) {
        stackelem* lh = stack_pop(&stack_num);
        int op = stack_pop_int(&stack_op);
        res = op_exe_d(lh, &rrh, str, ix_table[op], ix_table[op+1], &stack_op, &stack_num, ix_table);
        rrh.is_var = 0;
        rrh.integer = res;
      }
    } else {
      res = rh->is_var ? var_get(rh->var_hash) : rh->integer;
    }
  }
  DBG_EVAL("stack num: %i\n", stack_num.max_ix);
  DBG_EVAL("stack op : %i\n", stack_op.max_ix);
  return res;
}

void eval_init() {
  memset(var_stack, 0, sizeof(var_stack));
  var_stack_ix = 0;
  null_stackelem.integer = 0;
  null_stackelem.is_var = 0;
}

int eval(const char *in) {
  unsigned int ix_table_len;
  DBG_EVAL("str:%s\n", in);
  tokenize(&in[0], strlen(in), &ix_table[0], &ix_table_len);
#if 0
  int i, j;
  DBG_EVAL("ix_table_len:%i\n", ix_table_len/2);
  for (i = 0; i < ix_table_len; i += 2) {
    DBG_EVAL("%i:  pos:%i  len:%i\t", i/2, ix_table[i], ix_table[i+1]);
    for (j = ix_table[i]; j < ix_table[i] + ix_table[i+1]; j++) {
      DBG_EVAL("%c", in[j]);
    }
    DBG_EVAL("\tprio:%i", op_prio(in, ix_table[i], ix_table[i+1]));
    DBG_EVAL("\n");
  }
#endif

  int r = exe(in, ix_table, ix_table_len);

#if 1
  int i;
  for (i = var_stack_ix; i >= 0; i--) {
    if (var_stack[i].hash != 0) {
      DBG_EVAL("R%08x = %i\n", var_stack[i].hash, var_stack[i].val);
    } else {
      DBG_EVAL("---\n");
    }
  }
#endif

  return r;
}
