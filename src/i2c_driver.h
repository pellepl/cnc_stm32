/*
 * i2c_driver.h
 *
 *  Created on: Jul 8, 2013
 *      Author: petera
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "system.h"

/*
 * I2C bus
 */
typedef struct i2c_bus_s {
  I2C_TypeDef *hw;
  u8_t attached_devices;
  volatile bool busy;
  void (*i2c_bus_callback)(struct i2c_bus_s *s);
  void *user_p;
  volatile u32_t user_arg;
} i2c_bus;

extern i2c_bus __i2c_bus_vec[I2C_MAX_ID];

#define _I2C_BUS(x) (&__i2c_bus_vec[(x)])

void I2C_IRQ_err(i2c_bus *bus);
void I2C_IRQ_ev(i2c_bus *bus);
void I2C_init();
void I2C_test();


#endif /* I2C_DRIVER_H_ */
