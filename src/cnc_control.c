/*
 * cnc_control.c
 *
 *  Created on: 26 apr 2010
 *      Author: Peter
 */
#include "cnc_control.h"
#include "cnc_comm.h"
#include "miniutils.h"
#include "led.h"

#ifdef CONFIG_CNC

/**
 * Machine state
 */
static struct {
  /* Status register */
  volatile u8_t sr;
  /* Status register report mask */
  volatile u32_t sr_mask;
  /* Status register error mask */
  volatile u8_t sr_err;
  /* Status register error mask */
  volatile u8_t sr_err_mask;
  /* Status register change report callback */
  cnc_sr_callback sr_cb;
  cnc_pos_callback pos_cb;
  cnc_offs_callback offs_cb;

  CNC_Config_t config;

	/* Flag indicating if motor control is active, setting to 0 prohibits any motor activity */
	volatile u32_t cnc_timer_active;

  /* Current x position */
  volatile s32_t pos_x;
  /* Current y position */
  volatile s32_t pos_y;
  /* Current z position */
  volatile s32_t pos_z;

  /* Offset x position */
  volatile s32_t offs_pos_x;
  /* Offset y position */
  volatile s32_t offs_pos_y;
  /* Offset z position */
  volatile s32_t offs_pos_z;

	/* Current motion of all axes */
	CNC_Motion_t cur_motion;

	/* Pause granularity counter*/
	u32_t pause_tick;

	/* Pipeline active */
	volatile u32_t pipe_active;
	/* Pipelined motion definitions */
	CNC_Motion_t pipe[CNC_PIPE_CAPACITY];
	/* Index of first pipelined motion definition, at the end of current motion this will be active */
	u32_t pipe_start;
	/* Index of last pipelined motion definition */
	u32_t pipe_end;
	/* Number of pipelined motion definitions */
	u32_t pipe_len;
	cnc_pipe_callback pipe_cb;


	/* Motion register latch */
	CNC_Motion_t latch;
	/* Flag indicating if something needs to be latched into pipeline when current motion has ended */
	volatile u32_t latch_registers;
	/* Current flag id register */
	u32_t latch_id;

	/* Probe status flag */
	u32_t probe_status;
	/* Current probe count */
	u32_t probe_count;
	/* Current probe count */
	u32_t cur_probe_count;
	/* Z frequency on touch */
	u32_t probe_z_freq_on_touch;
} machine;


void CNC_init(cnc_sr_callback sr_f, cnc_pipe_callback pipe_f,
    cnc_pos_callback pos_f, cnc_offs_callback offs_f) {
  DBG(D_APP, D_DEBUG, "CNC init\n");

	machine.cnc_timer_active = FALSE;
	memset(&machine, 0, sizeof(machine));
	machine.sr_err_mask = 0xff;
	machine.pipe_active = TRUE;
	machine.probe_status = CNC_PROBE_DISABLED;
  machine.sr_cb = sr_f;
  machine.pipe_cb = pipe_f;
  machine.pos_cb = pos_f;
  machine.offs_cb = offs_f;

  (void)CNC_reset();
}

u32_t CNC_reset() {
  CNC_set_enabled(FALSE);
  CNC_pipeline_flush();
  CNC_set_regs_imm(0,0,0,0,0,0);
  CNC_set_probe(FALSE, 0, 0);
  CNC_pipeline_enable(FALSE);
  CNC_disable_error(0xff);
  machine.config.max_freq[X_AXIS] = CNC_MAX_STEP_FREQ;
  machine.config.max_freq[Y_AXIS] = CNC_MAX_STEP_FREQ;
  machine.config.max_freq[Z_AXIS] = CNC_MAX_STEP_FREQ;
  machine.config.rapid_delta[X_AXIS] = CNC_RAPID_ACC_DEC;
  machine.config.rapid_delta[Y_AXIS] = CNC_RAPID_ACC_DEC;
  machine.config.rapid_delta[Z_AXIS] = CNC_RAPID_ACC_DEC;
  return TRUE;
}

static bool update_axis_regs(CNC_Axis_t axis_def, CNC_Vector_t* pAxis, u32_t rapid) {
  if (pAxis->step_count == 0) {
    return FALSE;
  }
	pAxis->timer_counter +=
			MIN(
			    ((machine.config.max_freq[axis_def]) << CNC_FP_DECIMALS),
			    (pAxis->step_freq + pAxis->step_freq_adj));

	if (pAxis->timer_counter >= (CNC_TIMER_FREQ << CNC_FP_DECIMALS)) {
		pAxis->timer_counter -= (CNC_TIMER_FREQ << CNC_FP_DECIMALS);
		if (pAxis->step_count > 0) {
			pAxis->step_count--;
			if (rapid) {
				if (pAxis->step_count >= pAxis->step_count_half) {
					pAxis->step_freq_adj += machine.config.rapid_delta[axis_def];
				} else {
					pAxis->step_freq_adj -= machine.config.rapid_delta[axis_def];
				}
			}
			if (pAxis->step_count == 0) {
				// reached end of travel, reset motion registers
				pAxis->step_freq = 0;
				pAxis->step_freq_adj = 0;
				pAxis->timer_counter = 0;
			}
		}
		return TRUE;
	}
	return FALSE;
}

static void copy_axis_regs(CNC_Vector_t* pAxisDest, CNC_Vector_t* pAxisSrc) {
	pAxisDest->step_freq = pAxisSrc->step_freq;
	pAxisDest->step_freq_adj = pAxisSrc->step_freq_adj;
	pAxisDest->dir = pAxisSrc->dir;
	pAxisDest->step_count = pAxisSrc->step_count;
	pAxisDest->step_count_half = pAxisSrc->step_count_half;
}

static void copy_motion(CNC_Motion_t* pMotionDest, CNC_Motion_t* pMotionSrc) {
	pMotionDest->id = pMotionSrc->id;
	pMotionDest->rapid = pMotionSrc->rapid;
	copy_axis_regs(&pMotionDest->vector[X_AXIS], &pMotionSrc->vector[X_AXIS]);
	copy_axis_regs(&pMotionDest->vector[Y_AXIS], &pMotionSrc->vector[Y_AXIS]);
	copy_axis_regs(&pMotionDest->vector[Z_AXIS], &pMotionSrc->vector[Z_AXIS]);

	if (pMotionSrc->pause > 0) {
		if (pMotionDest == &machine.cur_motion) {
			machine.pause_tick = 0;
		}
	}
  pMotionDest->pause = pMotionSrc->pause;
}

void CNC_timer() {
  u32 sr = CNC_get_status();
  if ((machine.sr ^ sr) & machine.sr_mask) {
    machine.sr = sr;
    // report status change
    if (machine.sr_cb) {
      machine.sr_cb(sr);
    }
  } else {
    machine.sr = sr;
  }

  if (machine.sr_err & machine.sr_err_mask) {
    LED_disable(LED_CNC_WORK_BIT);
    LED_blink_single(LED_CNC_DISABLE_BIT, 16, 3, 1);
    LED_blink(LED_ERROR1 | LED_ERROR2 | LED_ERROR3, 32, 20, 1);
    return;
  }
	if (!machine.cnc_timer_active) {
		// prevent any tampering with cnc registers and control port
    LED_blink_single(LED_CNC_DISABLE_BIT, 16, 3, 1);
	  LED_blink_single(LED_CNC_WORK_BIT, 64, 1, 1);
		return;
	}

  if (sr & (1<<CNC_STATUS_BIT_MOVEMENT_PAUSE)) {
    LED_blink_single(LED_CNC_WORK_BIT, 16, 8, 1);
  } else if (sr & (1<<CNC_STATUS_BIT_MOVEMENT_STILL)) {
    LED_blink_single(LED_CNC_WORK_BIT, 16, 1, 1);
  } else if ((sr & ((1<<CNC_STATUS_BIT_MOVEMENT_PAUSE) | (1<<CNC_STATUS_BIT_MOVEMENT_STILL))) == 0) {
    LED_blink_single(LED_CNC_WORK_BIT, 16, 15, 1);
  }

	// probe sense control
	if (machine.probe_status != CNC_PROBE_DISABLED && machine.probe_status != CNC_PROBE_CONTACT) {
		int triggerPort = 0;//GP3DAT & 0xff;
		if ((triggerPort & (1<<1)) != 0) {
			machine.cur_motion.vector[Z_AXIS].step_freq = machine.probe_z_freq_on_touch;
			machine.probe_status = CNC_PROBE_SENSE;
			machine.cur_probe_count++;
			if (machine.cur_probe_count >= machine.probe_count) {
				machine.probe_status = CNC_PROBE_CONTACT;
				CNC_set_regs_imm(0,0,0,0,0,0);
			}
		} else {
			machine.cur_probe_count = 0;
		}
	}

	// movement
	{
	  // axes timer overflow flags
	  u32_t ov_axes = 0;

	  // check pause
    if (machine.cur_motion.pause > 0) {
      // paused, no motion
      if (machine.pause_tick > 0) {
        machine.pause_tick--;
      } else {
        machine.cur_motion.pause--;
        machine.pause_tick = CNC_TIMER_FREQ/1000;
      }
    } else {
      // control registers calculations
      ov_axes |=
          update_axis_regs(X_AXIS, &machine.cur_motion.vector[X_AXIS], machine.cur_motion.rapid) ?
              (1<<X_AXIS) : 0;
      ov_axes |=
          update_axis_regs(Y_AXIS, &machine.cur_motion.vector[Y_AXIS], machine.cur_motion.rapid) ?
              (1<<Y_AXIS) : 0;
      ov_axes |=
          update_axis_regs(Z_AXIS, &machine.cur_motion.vector[Z_AXIS], machine.cur_motion.rapid) ?
              (1<<Z_AXIS) : 0;
    }

    // control port setting
    bool f_step_x = machine.cur_motion.vector[X_AXIS].timer_counter <= ((CNC_TIMER_FREQ
        << CNC_FP_DECIMALS) / 2);
    bool f_step_y = machine.cur_motion.vector[Y_AXIS].timer_counter <= ((CNC_TIMER_FREQ
        << CNC_FP_DECIMALS) / 2);
    bool f_step_z = machine.cur_motion.vector[Z_AXIS].timer_counter <= ((CNC_TIMER_FREQ
        << CNC_FP_DECIMALS) / 2);

    CNC_GPIO_DEF(
        // set
        (machine.cur_motion.vector[X_AXIS].dir ? CNC_GPIO_DIR_X : 0) |
        (f_step_x ? CNC_GPIO_STEP_X : 0) |
        (machine.cur_motion.vector[Y_AXIS].dir ? CNC_GPIO_DIR_Y : 0) |
        (f_step_y ? CNC_GPIO_STEP_Y : 0) |
        (machine.cur_motion.vector[Z_AXIS].dir ? CNC_GPIO_DIR_Z : 0) |
        (f_step_z ? CNC_GPIO_STEP_Z : 0),
        // reset
        (machine.cur_motion.vector[X_AXIS].dir ? 0 : CNC_GPIO_DIR_X ) |
        (f_step_x ? 0 : CNC_GPIO_STEP_X ) |
        (machine.cur_motion.vector[Y_AXIS].dir ? 0 : CNC_GPIO_DIR_Y) |
        (f_step_y ? 0 : CNC_GPIO_STEP_Y) |
        (machine.cur_motion.vector[Z_AXIS].dir ? 0 : CNC_GPIO_DIR_Z ) |
        (f_step_z ? 0 : CNC_GPIO_STEP_Z)
    );

    // position calculations
    if (ov_axes & (1<<X_AXIS)) {
      machine.pos_x += ((machine.cur_motion.vector[X_AXIS].dir) ? 1 : -1);
    }
    if (ov_axes & (1<<Y_AXIS)) {
      machine.pos_y += ((machine.cur_motion.vector[Y_AXIS].dir) ? 1 : -1);
    }
    if (ov_axes & (1<<Z_AXIS)) {
      machine.pos_z += ((machine.cur_motion.vector[Z_AXIS].dir) ? 1 : -1);
    }
    if (machine.pos_cb && (ov_axes != 0)) {
      machine.pos_cb(machine.pos_x, machine.pos_y, machine.pos_z);
    }
	}

	// motion pipeline execution
	if (machine.pipe_active
	    && machine.cur_motion.vector[X_AXIS].step_count == 0
			&& machine.cur_motion.vector[Y_AXIS].step_count == 0
			&& machine.cur_motion.vector[Z_AXIS].step_count == 0
			&& machine.cur_motion.pause == 0) {
		// no current motion, something in the pipe?
		if (machine.pipe_len > 0) {
			copy_motion(&machine.cur_motion, &machine.pipe[machine.pipe_start]);
			machine.pipe[machine.pipe_start].id = 0; // clear id of used motion

			machine.pipe_start++;
			if (machine.pipe_start >= CNC_PIPE_CAPACITY) {
				machine.pipe_start = 0;
			}
			machine.pipe_len--;
			if (machine.pipe_cb) {
			  machine.pipe_cb(machine.cur_motion.id);
			}
		}
	}

	// motion latch
	// something to latch, is there room for it in our pipe
	if (machine.latch_registers && machine.pipe_len < CNC_PIPE_CAPACITY) {
		copy_motion(&machine.pipe[machine.pipe_end], &machine.latch);
		machine.pipe_end++;
		if (machine.pipe_end >= CNC_PIPE_CAPACITY) {
			machine.pipe_end = 0;
		}
		machine.pipe_len++;
		machine.latch_registers = FALSE;
	}
}

u32_t CNC_get_status() {
  u32_t sr = 0;
	sr |= ((machine.cnc_timer_active ? 1 : 0) << CNC_STATUS_BIT_CONTROL_ENABLED);
	sr |= ((machine.cur_motion.vector[X_AXIS].step_count == 0 &&
			machine.cur_motion.vector[Y_AXIS].step_count == 0 &&
			machine.cur_motion.vector[Z_AXIS].step_count	== 0 &&
			machine.cur_motion.pause == 0 ? 1 : 0) << CNC_STATUS_BIT_MOVEMENT_STILL);
	sr |= ((machine.cur_motion.pause != 0 ? 1 : 0) << CNC_STATUS_BIT_MOVEMENT_PAUSE);
	sr |= ((machine.cur_motion.rapid ? 1 : 0) << CNC_STATUS_BIT_MOVEMENT_RAPID);

	sr |= ((machine.pipe_active ? 1 : 0) << CNC_STATUS_BIT_PIPE_ACTIVE);
	sr |= ((machine.pipe_len == 0 ? 1 : 0) << CNC_STATUS_BIT_PIPE_EMPTY);
	sr |= ((machine.pipe_len >= CNC_PIPE_CAPACITY ? 1 : 0) << CNC_STATUS_BIT_PIPE_FULL);

	sr |= ((machine.latch_registers ? 1 : 0) << CNC_STATUS_BIT_LATCH_FULL);

	sr |= (machine.sr_err << 8) & 0xff00;

	return sr;
}

void CNC_set_enabled(u32_t enable) {
	machine.cnc_timer_active = enable;
}

void CNC_pipeline_enable(u32_t enable) {
	machine.pipe_active = enable;
}

u32_t CNC_is_latch_free() {
	return (!machine.latch_registers);
}

static void set_latch_motion_regs_for_axis(CNC_Vector_t* pAxis, s32_t steps,
    u32_t freq, u32_t rapid) {
	pAxis->dir = steps > 0 ? 1 : 0;
	if (steps < 0) {
		steps = -steps;
	}
	if (rapid) {
		pAxis->step_count_half = steps >> 1;
	}
	pAxis->step_count = freq == 0 ? 0 : steps;
	pAxis->step_freq = freq;
	pAxis->step_freq_adj = 0;
}

void CNC_set_latch_id(u32_t id) {
	machine.latch_id = id;
}

u32_t CNC_get_current_motion_id() {
	return machine.cur_motion.id;
}

void CNC_get_motion(CNC_Motion_t* pMotion) {
	copy_motion(pMotion, &machine.cur_motion);
}

u32_t CNC_latch_xyz(s32_t stepsX, u32_t freqX, s32_t stepsY, u32_t freqY, s32_t stepsZ,
    u32_t freqZ, u32_t rapid) {
  if (!CNC_is_latch_free()) {
    return CNC_ERR_LATCH_BUSY;
  }
  machine.latch.id = machine.latch_id++;
	machine.latch.rapid = rapid;
	set_latch_motion_regs_for_axis(&machine.latch.vector[X_AXIS], stepsX, freqX, rapid);
	set_latch_motion_regs_for_axis(&machine.latch.vector[Y_AXIS], stepsY, freqY, rapid);
	set_latch_motion_regs_for_axis(&machine.latch.vector[Z_AXIS], stepsZ, freqZ, rapid);
	machine.latch.pause = 0;

	machine.latch_registers = TRUE;

	return machine.latch.id;
}

u32_t CNC_latch_pause(u32_t timeInMs) {
  if (!CNC_is_latch_free()) {
    return CNC_ERR_LATCH_BUSY;
  }
  machine.latch.id = machine.latch_id++;
	set_latch_motion_regs_for_axis(&machine.latch.vector[X_AXIS], 0, 0, 0);
	set_latch_motion_regs_for_axis(&machine.latch.vector[Y_AXIS], 0, 0, 0);
	set_latch_motion_regs_for_axis(&machine.latch.vector[Z_AXIS], 0, 0, 0);
	machine.latch.pause = timeInMs == 0 ? 0 : 1 + timeInMs;
  machine.latch_registers = TRUE;

	return machine.latch.id;
}

static void set_imm_motion_regs_for_axis(CNC_Vector_t* pAxis,
    s32_t steps, u32_t freq) {
  pAxis->dir = steps > 0 ? 1 : 0;
  if (steps < 0) {
    steps = -steps;
  }
	pAxis->step_count = steps;
	pAxis->step_freq = freq;
	pAxis->step_freq_adj = 0;
}

void CNC_pipeline_flush() {
	int oldActive = machine.cnc_timer_active;
	machine.cnc_timer_active = FALSE;
	machine.pipe_len = 0;
	machine.pipe_start = 0;
	machine.pipe_end = 0;
	memset(&machine.pipe, 0, sizeof(machine.pipe) >> 1);
	memset(&machine.latch, 0, sizeof(machine.latch) >> 1);
	machine.latch_registers = FALSE;
	machine.cnc_timer_active = oldActive;
}

void CNC_set_x_imm(s32_t stepsX, u32_t freqX) {
	machine.cur_motion.rapid = FALSE;
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[X_AXIS], stepsX, freqX);
}

void CNC_set_y_imm(s32_t stepsY, u32_t freqY) {
	machine.cur_motion.rapid = FALSE;
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[Y_AXIS], stepsY, freqY);
}

void CNC_set_z_imm(s32_t stepsZ, u32_t freqZ) {
	machine.cur_motion.rapid = FALSE;
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[Z_AXIS], stepsZ, freqZ);
}

void CNC_set_regs_imm(s32_t stepsX, u32_t freqX, s32_t stepsY, u32_t freqY, s32_t stepsZ, u32_t freqZ) {
	machine.cur_motion.rapid = FALSE;
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[X_AXIS], stepsX, freqX);
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[Y_AXIS], stepsY, freqY);
	set_imm_motion_regs_for_axis(&machine.cur_motion.vector[Z_AXIS], stepsZ, freqZ);
}

void CNC_get_pos(s32_t* px, s32_t* py, s32_t* pz) {
	if (px != NULL) {
		*px = machine.pos_x + machine.offs_pos_x;
	}
	if (py != NULL) {
		*py = machine.pos_y + machine.offs_pos_y;
	}
	if (pz != NULL) {
		*pz = machine.pos_z + machine.offs_pos_z;
	}
}

void CNC_set_pos(s32_t x, s32_t y, s32_t z) {
  int oldActive = machine.cnc_timer_active;
  machine.cnc_timer_active = FALSE;
  machine.pos_x = x;
  machine.pos_y = y;
  machine.pos_z = z;
  if (machine.pos_cb) {
    machine.pos_cb(x,y,z);
  }

  machine.cnc_timer_active = oldActive;
}

void CNC_set_offs_pos(s32_t x, s32_t y, s32_t z) {
  int oldActive = machine.cnc_timer_active;
  machine.cnc_timer_active = FALSE;
  machine.offs_pos_x = x;
  machine.offs_pos_y = y;
  machine.offs_pos_z = z;
  if (machine.offs_cb) {
    machine.offs_cb(x,y,z);
  }

  machine.cnc_timer_active = oldActive;
}

void CNC_config_pos(s32_t x, s32_t y, s32_t z) {
  int oldActive = machine.cnc_timer_active;
  machine.cnc_timer_active = FALSE;
  machine.pos_x = x;
  machine.pos_y = y;
  machine.pos_z = z;
  machine.cnc_timer_active = oldActive;
}

void CNC_config_offs_pos(s32_t x, s32_t y, s32_t z) {
  int oldActive = machine.cnc_timer_active;
  machine.cnc_timer_active = FALSE;
  machine.offs_pos_x = x;
  machine.offs_pos_y = y;
  machine.offs_pos_z = z;
  machine.cnc_timer_active = oldActive;
}

void CNC_get_offs_pos(s32_t* px, s32_t* py, s32_t* pz) {
  if (px != NULL) {
    *px = machine.offs_pos_x;
  }
  if (py != NULL) {
    *py =  machine.offs_pos_y;
  }
  if (pz != NULL) {
    *pz = machine.offs_pos_z;
  }
}

void CNC_set_status_mask(u32_t mask) {
  machine.sr_mask = mask;
}

void CNC_set_error_mask(u32_t mask) {
  machine.sr_err_mask = mask;
}

void CNC_enable_error(u32_t error) {
  DBG(D_APP, D_FATAL, "CNC_ERROR: %08b\n", error);
  machine.sr_err |= error;
}

void CNC_disable_error(u32_t error) {
  machine.sr_err &= ~error;
}

void CNC_set_probe(u32_t enabled, u32_t contactCount, u32_t probeZFreqOnTouch) {
	if (enabled) {
		machine.probe_status = CNC_PROBE_NOCONTACT;
		machine.probe_count = contactCount;
		machine.cur_probe_count = 0;
		machine.probe_z_freq_on_touch = probeZFreqOnTouch;
	} else {
		machine.probe_status = CNC_PROBE_DISABLED;
	}
}

u32_t CNC_get_probe_status() {
	return machine.probe_status;
}

#define NIBBLES_CNC_DEC_HI ((32-CNC_FP_DECIMALS+3) / 4)
#define NIBBLES_CNC_DEC_LO ((CNC_FP_DECIMALS+3) / 4)
#define VEC_OUTPUT_STR "%s %c dir:%c steps:%8i f:%0_x.%0_x (%i Hz)\n"

static void print_vector(const char *prefix, const char vec_prefix, CNC_Vector_t *v) {
  u32_t step_freq_int = v->step_freq >> CNC_FP_DECIMALS;
  u32_t step_freq_dec = v->step_freq & ((1 << CNC_FP_DECIMALS) -1);
  u32_t hz = (v->step_freq>>CNC_FP_DECIMALS);
  char f[sizeof(VEC_OUTPUT_STR)];
  memcpy(f, VEC_OUTPUT_STR, sizeof(VEC_OUTPUT_STR));
  f[strchr(f,'_') - f] = '0' + NIBBLES_CNC_DEC_HI;
  f[strchr(f,'_') - f] = '0' + NIBBLES_CNC_DEC_LO;
  print(f,
      prefix, vec_prefix,
              v->dir ? '+' : '-',
              v->step_count,
              step_freq_int, step_freq_dec,
              hz);
}

static void print_motion(CNC_Motion_t *motion, const char *prefix) {
  print("%s id:%08x pause:%i rapid:%i\n", prefix, motion->id, motion->pause, motion->rapid);
  print_vector(prefix, 'X', &motion->vector[X_AXIS]);
  print_vector(prefix, 'Y', &motion->vector[Y_AXIS]);
  print_vector(prefix, 'Z', &motion->vector[Z_AXIS]);
}

u32_t CNC_dump() {
  print("CNC\n---\n");
  print("  active:%s sr:%16b report_mask:%08b\n", machine.cnc_timer_active ? "YES":"NO ", CNC_get_status(), machine.sr_mask);
  print("  errors:%08b mask:%08b\n", machine.sr_err & machine.sr_err_mask, machine.sr_err_mask);
  print("  config max_f x:%i y:%i z:%i\n", machine.config.max_freq[X_AXIS],
      machine.config.max_freq[Y_AXIS], machine.config.max_freq[Z_AXIS]);
  print("         rap_d x:%08x y:%08x z:%08x\n", machine.config.rapid_delta[X_AXIS],
      machine.config.rapid_delta[Y_AXIS], machine.config.rapid_delta[Z_AXIS]);
  print(" actual  x:%i y:%i z:%i\n", machine.pos_x, machine.pos_y, machine.pos_z);
  print(" offset  x:%i y:%i z:%i\n", machine.offs_pos_x, machine.offs_pos_y, machine.offs_pos_z);
  print(" current x:%i y:%i z:%i\n", machine.pos_x + machine.offs_pos_x, machine.pos_y + machine.offs_pos_y, machine.pos_z + machine.offs_pos_z);
  print_motion(&machine.cur_motion, "CNC curr motion");
  print(" pipe active:%s len:%i/%i\n", machine.pipe_active ? "YES" : "NO ",
      machine.pipe_len, CNC_PIPE_CAPACITY);
  int i;
  char pre[sizeof("CNC pipemotionX\0")];
  memcpy(pre, "CNC pipemotionX\0", sizeof("CNC pipemotionX\0"));
  for (i = 0; i < machine.pipe_len; i++) {
    int ix = machine.pipe_start + i;
    if (ix >= CNC_PIPE_CAPACITY) ix -= CNC_PIPE_CAPACITY;
    pre[sizeof("CNC pipemotion") - 1] = '1' + i;
    print_motion(&machine.pipe[ix], pre);
  }
  print(" latch:%s\n", machine.latch_registers ? "YES" : "NO ");
  if (machine.latch_registers) {
    print_motion(&machine.latch, "CNC latchmotion");
  }
  return 0;
}

void CNC_set_config(CNC_Config_t *config) {
  // TODO
}

CNC_Config_t *CNC_get_config() {
  return &machine.config;
}

void CNC_set_config_specific(u8_t config, u32_t value) {
  switch (config) {
  case COMM_PROTOCOL_CONFIG_MAX_X_FREQ:
    machine.config.max_freq[X_AXIS] = value;
    break;
  case COMM_PROTOCOL_CONFIG_MAX_Y_FREQ:
    machine.config.max_freq[Y_AXIS] = value;
    break;
  case COMM_PROTOCOL_CONFIG_MAX_Z_FREQ:
    machine.config.max_freq[Z_AXIS] = value;
    break;
  case COMM_PROTOCOL_CONFIG_RAPID_X_D:
    machine.config.rapid_delta[X_AXIS] = value;
    break;
  case COMM_PROTOCOL_CONFIG_RAPID_Y_D:
    machine.config.rapid_delta[Y_AXIS] = value;
    break;
  case COMM_PROTOCOL_CONFIG_RAPID_Z_D:
    machine.config.rapid_delta[Z_AXIS] = value;
    break;
  }
}
#endif // CONFIG_CNC
