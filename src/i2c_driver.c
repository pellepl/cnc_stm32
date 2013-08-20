/*
 * i2c_driver.c
 *
 *  Created on: Jul 8, 2013
 *      Author: petera
 */

#include "i2c_driver.h"
#include "miniutils.h"

i2c_bus __i2c_bus_vec[I2C_MAX_ID];

int I2C_config(i2c_bus *bus, u32_t clock) {
  I2C_InitTypeDef  I2C_InitStruct;
  I2C_StructInit(&I2C_InitStruct);
  I2C_DeInit(bus->hw);

  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00; // never mind, will be master
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = clock;

  I2C_ITConfig(I2C1_PORT, I2C_IT_ERR | I2C_IT_BUF | I2C_IT_EVT, ENABLE);

  I2C_Init(I2C1_PORT, &I2C_InitStruct);

  I2C_Cmd(I2C1_PORT, ENABLE);

  I2C_AcknowledgeConfig(I2C1_PORT, ENABLE);
  I2C_StretchClockCmd(I2C1_PORT, ENABLE);

  return I2C_OK;
}

void I2C_init() {
  memset(__i2c_bus_vec, 0, sizeof(__i2c_bus_vec));
  _I2C_BUS(0)->hw = I2C1_PORT;
}

int I2C_rx(i2c_bus *bus, u8_t addr, u8_t *rx, u16_t len, bool gen_stop) {
  if (bus->state != I2C_S_IDLE) {
    return I2C_ERR_BUS_BUSY;
  }
  bus->state = I2C_S_GEN_START;
  bus->buf = rx;
  bus->len = len;
  bus->addr = addr & 0xfe;
  bus->gen_stop = gen_stop;
  I2C_AcknowledgeConfig(bus->hw, len > 1 ? ENABLE : DISABLE);
  I2C_GenerateSTOP(bus->hw, DISABLE);
  I2C_GenerateSTART(bus->hw, ENABLE);
  return I2C_OK;
}

int I2C_tx(i2c_bus *bus, u8_t addr, u8_t *tx, u16_t len, bool gen_stop) {
  if (bus->state != I2C_S_IDLE) {
    return I2C_ERR_BUS_BUSY;
  }
  bus->state = I2C_S_GEN_START;
  bus->buf = tx;
  bus->len = len;
  bus->addr = addr | 0x01;
  bus->gen_stop = gen_stop;
  I2C_GenerateSTOP(bus->hw, DISABLE);
  I2C_GenerateSTART(bus->hw, ENABLE);
  return I2C_OK;
}

int I2C_query(i2c_bus *bus, u8_t addr) {
  if (bus->state != I2C_S_IDLE) {
    return I2C_ERR_BUS_BUSY;
  }
  bus->state = I2C_S_GEN_START;
  bus->buf = 0;
  bus->len = 0;
  bus->addr = addr | 0x01;
  bus->gen_stop = TRUE;
  I2C_GenerateSTOP(bus->hw, DISABLE);
  I2C_GenerateSTART(bus->hw, ENABLE);
  return I2C_OK;
}

int I2C_set_callback(i2c_bus *bus, void (*i2c_bus_callback)(i2c_bus *bus, int res)) {
  if (bus->state != I2C_S_IDLE) {
    return I2C_ERR_BUS_BUSY;
  }
  bus->i2c_bus_callback = i2c_bus_callback;
  return I2C_OK;
}

int I2C_close(i2c_bus *bus) {
  I2C_ITConfig(I2C1_PORT, I2C_IT_ERR | I2C_IT_BUF | I2C_IT_EVT, DISABLE);
  I2C_Cmd(bus->hw, DISABLE);
  bus->user_arg = 0;
  bus->user_p = 0;
  if (bus->state != I2C_S_IDLE) {
    bus->state = I2C_S_IDLE;
    return I2C_ERR_BUS_BUSY;
  }
  return I2C_OK;
}

void I2C_register(i2c_bus *bus) {
  bus->attached_devices++;
}

void I2C_release(i2c_bus *bus) {
  bus->attached_devices--;
  if (bus->attached_devices == 0) {
    I2C_close(bus);
  }
}

bool I2C_is_busy(i2c_bus *bus) {
  return bus->state != I2C_S_IDLE;
}


void I2C_reset(i2c_bus *bus) {
  I2C_SoftwareResetCmd(bus->hw, ENABLE);
  I2C_SoftwareResetCmd(bus->hw, DISABLE);
  bus->state = I2C_S_IDLE;
}


static void i2c_error(i2c_bus *bus, int err, bool reset) {
  if (bus->state != I2C_S_IDLE) {
    if (reset) {
      I2C_reset(bus);
      I2C_GenerateSTOP(bus->hw, ENABLE);
      I2C_ReceiveData(bus->hw);
    }
    bus->state = I2C_S_IDLE;
    if (bus->i2c_bus_callback) {
      bus->i2c_bus_callback(bus, err);
    }
  }
}

void I2C_IRQ_err(i2c_bus *bus) {
  if (I2C_GetITStatus(bus->hw, I2C_IT_SMBALERT))
  {
    DBG(D_I2C, D_WARN, "i2c_err: SMBus Alert\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_SMBALERT);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_TIMEOUT))
  {
    DBG(D_I2C, D_WARN, "i2c_err: Timeout or Tlow error\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_TIMEOUT);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_ARLO))
  {
    DBG(D_I2C, D_WARN, "i2c_err: Arbitration lost\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_ARLO);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_PECERR))
  {
    DBG(D_I2C, D_WARN, "i2c_err: PEC error\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_PECERR);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_OVR))
  {
    DBG(D_I2C, D_WARN, "i2c_err: Overrun/Underrun flag\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_OVR);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_AF))
  {
    DBG(D_I2C, D_WARN, "i2c_err: Acknowledge failure\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_AF);
    i2c_error(bus, I2C_ERR_PHY, FALSE);
  }
  if (I2C_GetITStatus(bus->hw, I2C_IT_BERR))
  {
    DBG(D_I2C, D_WARN, "i2c_err: Bus error\n");
    I2C_ClearITPendingBit(bus->hw, I2C_IT_BERR);
    i2c_error(bus, I2C_ERR_PHY, TRUE);
  }
  //I2C_GenerateSTOP(bus->hw, ENABLE);
}

//#define I2C_HW_DEBUG(...) DBG(__VA_ARGS__)
#define I2C_HW_DEBUG(...)

void I2C_IRQ_ev(i2c_bus *bus) {
  u32_t ev = I2C_GetLastEvent(bus->hw);
  I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev: %08x\n", ev);
  switch (ev) {

  // EV5
  // from send start condition
  case I2C_EVENT_MASTER_MODE_SELECT:
    I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master mode\n");
    I2C_Send7bitAddress(bus->hw, bus->addr,
        (bus->addr & 0x01 ? I2C_Direction_Transmitter : I2C_Direction_Receiver));

  // ------- TX --------

  break;
  // EV6
  // from send 7 bit address tx
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master tx mode\n");
    if (bus->len > 0) {
      bus->state = I2C_S_TX;
      I2C_SendData(bus->hw, *bus->buf++);
      bus->len--;
    } else {
      // for address query
      I2C_GenerateSTOP(bus->hw, ENABLE);
      bus->state = I2C_S_IDLE;
      if (bus->i2c_bus_callback) {
        bus->i2c_bus_callback(bus, I2C_OK);
      }
    }
  break;
  // EV8
  // tx reg empty
  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
    I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master byte tx\n");
    if (bus->len > 0) {
      I2C_SendData(bus->hw, *bus->buf++);
      bus->len--;
    }
  break;
  // EV8_2
  // tx reg transmitted, became empty
  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    if (bus->state != I2C_S_IDLE) {
      I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master byte txed\n");
      if (bus->len == 0) {
        if (bus->gen_stop) {
          I2C_GenerateSTOP(bus->hw, ENABLE);
        }
        bool gen_cb = bus->state != I2C_S_IDLE;
        bus->state = I2C_S_IDLE;
        if (bus->i2c_bus_callback && gen_cb) {
          bus->i2c_bus_callback(bus, I2C_TX_OK);
        }
      }
    }
  break;


  // ------- RX --------

  // EV6
  // from send 7 bit address tx
  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master rx mode\n");
    bus->state = I2C_S_RX;
  if (bus->len <= 1) {
      if (bus->gen_stop) {
        I2C_GenerateSTOP(bus->hw, ENABLE);
      }
      I2C_AcknowledgeConfig(bus->hw, DISABLE);
    }
  break;
  // EV7
  // rx reg filled
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    I2C_HW_DEBUG(D_I2C, D_DEBUG, "i2c_ev:   master byte rxed\n");
    u8_t data = I2C_ReceiveData(bus->hw);
    *bus->buf++ = data;
    bus->len--;
    if (bus->len == 0) {
      if (bus->gen_stop) {
        I2C_GenerateSTOP(bus->hw, ENABLE);
      }
      I2C_AcknowledgeConfig(bus->hw, DISABLE);
      bool gen_cb = bus->state != I2C_S_IDLE;
      bus->state = I2C_S_IDLE;
      if (bus->i2c_bus_callback && gen_cb) {
        bus->i2c_bus_callback(bus, I2C_RX_OK);
      }
    }
    break;
  case 0x30000:
    // why oh why stm..?
    break;

  default:
    // bad event
    DBG(D_I2C, D_WARN, "i2c_err: bad event %08x\n", ev);
    i2c_error(bus, I2C_ERR_UNKNOWN_STATE, TRUE);

    break;
  }
}
