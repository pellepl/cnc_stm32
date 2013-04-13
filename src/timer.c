/*
 * timer.c
 *
 *  Created on: Jun 23, 2012
 *      Author: petera
 */

#include "timer.h"
#include "system.h"
#include "stm32f10x.h"
#include "miniutils.h"
#include "uart.h"
#include "cnc_control.h"
#include "console.h"
#include "led.h"
#include "os.h"

void TIMER_irq() {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

#ifdef CONFIG_CNC
    CNC_timer();
#endif
    bool ms_update = SYS_timer();
    TASK_timer();
#ifdef CONFIG_LED
    LED_SHIFT_tick();
    LED_tick();
#endif
    if (ms_update) {
      __os_time_tick(SYS_get_time_ms());
    }
    DBG_timer();
  }
}
