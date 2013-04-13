/*
 * led.h
 *
 *  Created on: Sep 20, 2012
 *      Author: petera
 */

#ifndef LED_H_
#define LED_H_

#include "system.h"

typedef u32_t led_t;

#define LED_BLINK_FOREVER       0xff

#ifdef CONFIG_LED

void LED_enable(led_t led);
void LED_disable(led_t led);
void LED_set(led_t led_enable, led_t led_disable);
void LED_pulse(led_t led, u8_t ticks);
void LED_blink(led_t led, u8_t cycle, u8_t duty, u8_t blinks);
void LED_blink_single(u8_t led_ix, u8_t cycle, u8_t duty, u8_t blinks);

void LED_tick();
void LED_SHIFT_tick();

void LED_SHIFT_init(u8_t shift_reg_bit_size);
void LED_init(u16_t divisor);

#else

#define LED_enable(led)
#define LED_disable(led)
#define LED_set(led_enable, led_disable)
#define LED_pulse(led, ticks)
#define LED_blink(led, cycle, duty, blinks)
#define LED_blink_single(ix, cycle, duty, blinks)

#endif // CONFIG_LED

#endif /* LED_H_ */
