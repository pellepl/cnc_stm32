/*
 * cnc_control.h
 *
 *  Created on: 26 apr 2010
 *      Author: Peter
 */

#ifndef CNC_CONTROL_H_
#define CNC_CONTROL_H_

#include "types.h"
#include "system.h"

#define CNC_TIMER_FREQ        (SYS_MAIN_TIMER_FREQ)

#define CNC_MAX_STEP_FREQ     (480*CNC_STEPS_PER_MM_X/60)
#define CNC_STEPS_PER_MM_X    (400)
#define CNC_STEPS_PER_MM_Y    (400)
#define CNC_STEPS_PER_MM_Z    (400)

#define CNC_FP_DECIMALS       (14)
#define CNC_PIPE_CAPACITY     (128)
#define CNC_RAPID_ACC_DEC     ((1<<CNC_FP_DECIMALS)/2)

#define CNC_PROBE_DISABLED    (-1)
#define CNC_PROBE_NOCONTACT   (0)
#define CNC_PROBE_SENSE   	   (1)
#define CNC_PROBE_CONTACT     (2)

#define CNC_STATUS_BIT_CONTROL_ENABLED   (0)

#define CNC_STATUS_BIT_MOVEMENT_STILL	  (1)
#define CNC_STATUS_BIT_MOVEMENT_PAUSE 	  (2)
#define CNC_STATUS_BIT_MOVEMENT_RAPID 	  (3)

#define CNC_STATUS_BIT_PIPE_ACTIVE		    (4)
#define CNC_STATUS_BIT_PIPE_EMPTY		    (5)
#define CNC_STATUS_BIT_PIPE_FULL		      (6)
#define CNC_STATUS_BIT_LATCH_FULL		    (7)

#define CNC_ERROR_BIT_EMERGENCY          (0)
#define CNC_ERROR_BIT_SETTINGS_CORRUPT   (1)
#define CNC_ERROR_BIT_COMM_LOST          (2)


typedef enum {
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS,
  AXES_COUNT
} CNC_Axis_t;

/**
 * Defines an 1D movement in an axis
 */
typedef struct CNC_Vector_s {
	volatile u32_t timer_counter;
	volatile u32_t step_freq;
	volatile u32_t step_count;
  volatile u32_t step_count_half;
  volatile u32_t step_freq_adj;
	volatile bool dir;
} CNC_Vector_t;

/**
 * Defines a 3D movement direction + pause for all axes
 */
typedef struct CNC_Motion_s {
  u32_t id;
	CNC_Vector_t vector[AXES_COUNT];
	volatile u32_t pause;
	volatile bool rapid;
} CNC_Motion_t;

typedef struct CNC_Config_s {
  u32_t max_freq[AXES_COUNT];
  u32_t rapid_delta[AXES_COUNT];
} CNC_Config_t;

typedef void (*cnc_sr_callback)(u32_t sr);
typedef void (*cnc_pipe_callback)(u32_t id);
typedef void (*cnc_pos_callback)(s32_t x, s32_t y, s32_t z);
typedef void (*cnc_offs_callback)(s32_t x, s32_t y, s32_t z);

void CNC_timer();

void CNC_init(cnc_sr_callback sr_f, cnc_pipe_callback pipe_f,
    cnc_pos_callback pos_f, cnc_offs_callback offs_f);

u32_t CNC_get_status();
void CNC_set_status_mask(u32_t mask);
u32_t CNC_get_error_mask();
void CNC_set_error_mask(u32_t mask);

u32_t CNC_is_latch_free();
void CNC_set_latch_id(u32_t id);
u32_t CNC_get_current_motion_id();
void CNC_get_motion(CNC_Motion_t* pMotion);

void CNC_pipeline_flush();
void CNC_pipeline_enable(u32_t enable);

void CNC_set_enabled(u32_t);

u32_t CNC_latch_pause(u32_t timeInMs);
u32_t CNC_latch_xyz(s32_t stepsX, u32_t freqX, s32_t stepsY, u32_t freqY, s32_t stepsZ, u32_t freqZ, u32_t rapid);

void CNC_set_regs_imm(s32_t stepsX, u32_t freqX, s32_t stepsY, u32_t freqY, s32_t stepsZ, u32_t freqZ);
void CNC_set_x_imm(s32_t stepsX, u32_t freqX);
void CNC_set_y_imm(s32_t stepsY, u32_t freqY);
void CNC_set_z_imm(s32_t stepsZ, u32_t freqZ);

void CNC_get_pos(s32_t* px, s32_t* py, s32_t* pz);
void CNC_set_pos(s32_t x, s32_t y, s32_t z);
void CNC_get_offs_pos(s32_t* px, s32_t* py, s32_t* pz);
void CNC_set_offs_pos(s32_t x, s32_t y, s32_t z);
void CNC_config_pos(s32_t x, s32_t y, s32_t z);
void CNC_config_offs_pos(s32_t x, s32_t y, s32_t z);

void CNC_set_probe(u32_t enabled, u32_t contactCount, u32_t probeZFreqOnTouch);
u32_t CNC_get_probe_status();

void CNC_set_error_mask(u32_t error_mask);
void CNC_enable_error(u32_t error);
void CNC_disable_error(u32_t error);

void CNC_set_config(CNC_Config_t *config);
CNC_Config_t *CNC_get_config();
void CNC_set_config_specific(u8_t config, u32_t value);

u32_t CNC_reset();

u32_t CNC_dump();

#endif /* CNC_CONTROL_H_ */
