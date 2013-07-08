/*
 * i2c_driver.c
 *
 *  Created on: Jul 8, 2013
 *      Author: petera
 */

#include "i2c_driver.h"

void I2C_init() {
  I2C_InitTypeDef  I2C_InitStruct;
  I2C_StructInit(&I2C_InitStruct);
  I2C_DeInit(I2C_PORT);

  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00; // never mind, will be master
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = 100000;

  I2C_ITConfig(I2C_PORT, I2C_IT_ERR | I2C_IT_BUF | I2C_IT_EVT, ENABLE);

  I2C_Init(I2C_PORT, &I2C_InitStruct);

  I2C_Cmd(I2C_PORT, ENABLE);
}

void I2C_test() {
  I2C_init();
  print("generate start\n");
  I2C_GenerateSTART(I2C_PORT, ENABLE);
  print("ok\n");
}

void I2C_IRQ_err() {
  print("i2c_err ");
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_SMBALERT))
  {
    print("SMBus Alert");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_SMBALERT);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_TIMEOUT))
  {
    print("Timeout or Tlow error");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_TIMEOUT);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_ARLO))
  {
    print("Arbitration lost");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_ARLO);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_PECERR))
  {
    print("PEC error");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_PECERR);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_OVR))
  {
    print("Overrun/Underrun flag");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_OVR);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_AF))
  {
    print("Acknowledge failure");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_AF);
  }
  if (I2C_GetITStatus(I2C_PORT, I2C_IT_BERR))
  {
    print("Bus error");
    I2C_ClearITPendingBit(I2C_PORT, I2C_IT_BERR);
  }
  print("\n");
}

void I2C_IRQ_ev() {
  u32_t ev = I2C_GetLastEvent(I2C_PORT);
  print("i2c_ev: %08x", ev);
  switch(ev)
  {
  case I2C_EVENT_MASTER_MODE_SELECT:
    print(" master mode");
    I2C_Send7bitAddress(I2C_PORT, 0x31, I2C_Direction_Transmitter);
  break;

  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    print(" master tx mode");
    I2C_SendData(I2C1, 0xaa);
  break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
    print(" master byte tx");
    I2C_GenerateSTOP(I2C_PORT, ENABLE);
  break;

  default:
  break;

  }
  print("\n");
}
