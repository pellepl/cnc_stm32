#include "led.h"
#ifdef CONFIG_LED

typedef struct {
  u8_t timer;
  u8_t cycle;
  u8_t duty;
  u8_t counter;
} led_status;

static struct {
  u16_t timer;
  u16_t divisor;
  led_t active_mask;
  led_status status[LED_COUNT];
} led_tim;

static struct {
  volatile led_t current_led;
  volatile char dirty;
  led_t shift_led;
  char shifting;
  u32_t timer_divisor;
  u32_t timer_counter;
  u32_t clock_pulse;
  u8_t bit_count;
  u8_t bits;
} led_shift;

static void LED_enable_intern(led_t led) {
  led_shift.current_led |= led;
  led_shift.dirty = TRUE;
  // TODO PETER REMOV
  if (led & LED_SPI_FLASH) {
    GPIO_enable(GPIOD, GPIO_Pin_13);
  }
}

static void LED_disable_intern(led_t led) {
  led_shift.current_led &= ~led;
  led_shift.dirty = TRUE;
  if (led & LED_SPI_FLASH) {
    GPIO_disable(GPIOD, GPIO_Pin_13);
  }
}

static void LED_set_intern(led_t led_enable, led_t led_disable) {
  led_t tmp = led_shift.current_led;
  tmp &= ~led_disable;
  tmp |= led_enable;
  led_shift.current_led = tmp;
  led_shift.dirty = TRUE;
}


void LED_enable(led_t leds) {
  led_tim.active_mask &= ~leds;
  LED_enable_intern(leds);
}

void LED_disable(led_t leds) {
  led_tim.active_mask &= ~leds;
  LED_disable_intern(leds);
}

void LED_set(led_t leds_enable, led_t leds_disable) {
  led_tim.active_mask &= ~(leds_enable | leds_disable);
  LED_set_intern(leds_enable, leds_disable);
}

static void LED_config_single(u8_t led_bit, u8_t cycle, u8_t duty, u8_t blinks) {
  led_t led = 1<<led_bit;
  if ((led_tim.active_mask & led) == 0) {
    led_tim.status[led_bit].timer = 0;
  }
  led_tim.status[led_bit].cycle = cycle;
  led_tim.status[led_bit].duty = duty;
  led_tim.status[led_bit].counter = blinks;
}

void LED_blink_single(u8_t led_ix, u8_t cycle, u8_t duty, u8_t blinks) {
  LED_config_single(led_ix, cycle, duty, blinks);
  led_tim.active_mask |= (1<<led_ix);
}

void LED_blink(led_t leds, u8_t cycle, u8_t duty, u8_t blinks) {
  if (leds == 0) return;
  int i;
  led_t l = leds;
  for (i = 0; l && i < LED_COUNT; i++) {
    if (l & 1) {
      LED_config_single(i, cycle, duty, blinks);
    }
    l >>= 1;
  }
  led_tim.active_mask |= leds;
}

void LED_pulse(led_t leds, u8_t ticks) {
  if (leds == 0) return;
  int i;
  led_t l = leds;
  for (i = 0; l && i < LED_COUNT; i++) {
    if (l & 1) {
      LED_config_single(i, 0,0,ticks);
    }
    l >>= 1;
  }
  led_tim.active_mask |= leds;
}


static void LED_update(led_t leds, led_status *s) {
  if (s->timer == 0) {
    LED_enable_intern(leds);
  } else if (s->timer == s->duty) {
    LED_disable_intern(leds);
  }
  if (s->timer++ >= s->cycle) {
    s->timer = 0;
    if (s->counter != LED_BLINK_FOREVER) {
      if (s->counter == 0) {
        LED_disable_intern(leds);
        led_tim.active_mask &= ~leds;
      } else {
        s->counter--;
      }
    }
  }
}

void LED_tick() {
  if (led_tim.timer == led_tim.divisor) {
    led_tim.timer = 0;
  } else {
    led_tim.timer++;
    return;
  }
  if (led_tim.active_mask == 0) return;
  int i;
  for (i = 0; i < LED_COUNT; i++) {
    if (led_tim.active_mask & (1<<i)) {
      LED_update(1<<i, &led_tim.status[i]);
    }
  }
}

void LED_SHIFT_tick() {
  if (led_shift.shifting) {

    if (led_shift.timer_counter++ >= led_shift.timer_divisor) {
      led_shift.timer_counter = 0;
    }
    LED_SHIFT_PORT->BSRR = LED_SHIFT_CLK << (led_shift.timer_counter > led_shift.clock_pulse ? 0:16);

    if (led_shift.timer_counter == 0) {
      char shift_bit = led_shift.shift_led & 1;
      LED_SHIFT_PORT->BSRR = LED_SHIFT_DAT << (shift_bit? 0:16);
      led_shift.shift_led >>= 1;

      if (led_shift.bit_count-- == 0) {
        led_shift.shifting = FALSE;
        LED_SHIFT_PORT->BSRR = LED_SHIFT_STR;
      }
    }
  } else if (led_shift.dirty) {
    led_shift.shift_led = led_shift.current_led;
    led_shift.dirty = FALSE;
    led_shift.timer_counter = 0;
    led_shift.bit_count = led_shift.bits;
    LED_SHIFT_PORT->BRR = LED_SHIFT_STR;
    led_shift.shifting = TRUE;
  }
}

void LED_SHIFT_init(u8_t bits) {
  memset(&led_shift, 0, sizeof(led_shift));
  LED_SHIFT_PORT->BRR = LED_SHIFT_STR;
  led_shift.bits = bits;
  led_shift.clock_pulse = 0;
  led_shift.timer_divisor = 2;
}

void LED_init(u16_t divisor) {
  memset(&led_tim, 0, sizeof(led_tim));
  led_tim.divisor = divisor;
}
#endif
