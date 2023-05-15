#include "sc16is752.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

namespace esphome {
namespace sc16is752 {

///////////////////////////////
// The SC16IS752Channel methods
///////////////////////////////

// The following functions implement the Interface part of the UARTComponent
void SC16IS752Channel::write_array(const uint8_t *data, size_t len) {}

bool SC16IS752Channel::peek_byte(uint8_t *data) {
  // int SC16IS750_peek(SC16IS750_t *dev, uint8_t channel) {
  //  if (dev->peek_flag == 0) {
  //    dev->peek_buf = SC16IS750_ReadByte(dev, channel);
  //    if (dev->peek_buf >= 0) {
  //      dev->peek_flag = 1;
  //    }
  //  }
  return true;
}

//  return dev->peek_buf;
// }

bool SC16IS752Channel::read_array(uint8_t *data, size_t len) { return true; }

int SC16IS752Channel::available() {
  return parent_->read_register(address(SC16IS752_REG_RXLVL, channel_), &buffer_, 1, true);
  // return SC16IS750_ReadRegister(dev, SC16IS750_REG_LSR) & 0x01;
}

// @TODO look strange
void SC16IS752Channel::flush() {
  do {
    buffer_ = parent_->read_register(address(SC16IS752_REG_LSR, channel_), &buffer_, 1);
  } while ((buffer_ & 0x20) == 0);
}

void SC16IS752Channel::check_logger_conflict() {}

// SC16IS752Channel::SC16IS752Channel() {
//   // initialize
//   reset_device();
//   enable_fifo(true);
//   // set_baud_rate();
//   // set_line();
// }
// void reset_device() {}
// uint8_t read_register(uint8_t reg) { return 0; }
// void write_register(uint8_t reg, uint8_t data) {}
// int available_in_fifo() { return 0; }

void SC16IS752Channel::fifo_enable(bool enable) {
  uint8_t fcr;
  fcr = parent_->read_register(address(SC16IS752_REG_FCR, channel_), &fcr, 1);
  fcr = enable ? (fcr & 0xFE) : (fcr | 0x01);
  parent_->write_register(address(SC16IS752_REG_FCR, channel_), &fcr, 1);
}

void SC16IS752Channel::set_line_param() {
  uint8_t lcr;
  lcr = parent_->read_register(address(SC16IS752_REG_LCR, channel_), &lcr, 1);
  lcr &= 0xC0;           // Clear the lower six bit of LCR (LCR[0] to LCR[5]
  switch (data_bits_) {  // data length settings
    case 5:
      break;
    case 6:
      lcr |= 0x01;
      break;
    case 7:
      lcr |= 0x02;
      break;
    case 8:
      lcr |= 0x03;
      break;
    default:
      lcr |= 0x03;
  }
  if (stop_bits_ == 2) {
    lcr |= 0x04;
  }
  switch (parity_) {              // parity selection length settings
    case UART_CONFIG_PARITY_ODD:  // odd parity
      lcr |= 0x08;
      break;
    case UART_CONFIG_PARITY_EVEN:  // even parity
      lcr |= 0x18;
      break;
    default:
  }
  parent_->write_register(address(SC16IS752_REG_FCR, channel_), &lcr, 1);
}

/* -------------------------------------------------------




void SC16IS750_WriteByte(SC16IS750_t *dev, uint8_t channel, uint8_t val) {
  uint8_t tmp_lsr;
  /*
    while ( SC16IS750_FIFOAvailableSpace(dev, channel) == 0 ){
  #ifdef	SC16IS750_DEBUG_PRINT
      printf("No available space\n");
  #endif

    };
  #ifdef	SC16IS750_DEBUG_PRINT
    printf("++++++++++++Data sent\n");
  #endif
    SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_THR, val);
  */
/* ----------------------------------------------------------------
 do {
   tmp_lsr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LSR);
 } while ((tmp_lsr & 0x20) == 0);

 SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_THR, val);
}

int SC16IS750_ReadByte(SC16IS750_t *dev, uint8_t channel) {
 volatile uint8_t val;
 if (SC16IS750_FIFOAvailableData(dev, channel) == 0) {
#ifdef SC16IS750_DEBUG_PRINT
   printf("No data available\n");
#endif
   return -1;

 } else {
#ifdef SC16IS750_DEBUG_PRINT
   printf("***********Data available***********\n");
#endif
   val = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_RHR);
   return val;
 }
}

void SC16IS750_EnableTransmit(SC16IS750_t *dev, uint8_t channel, uint8_t tx_enable) {
 uint8_t temp_efcr;
 temp_efcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_EFCR);
 if (tx_enable == 0) {
   temp_efcr |= 0x04;
 } else {
   temp_efcr &= 0xFB;
 }
 SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_EFCR, temp_efcr);

 return;
}

void SC16IS750_setTimeout(SC16IS750_t *dev, uint32_t time_out) { dev->timeout = time_out; }

size_t SC16IS750_readBytes(SC16IS750_t *dev, uint8_t channel, char *buffer, size_t length) {
 size_t count = 0;
 int16_t tmp;
 uint8_t _channel;

 while (count < length) {
   tmp = SC16IS750_readwithtimeout(dev, &_channel);
   if (tmp < 0) {
     break;
   }
   if (_channel == channel) {
     *buffer++ = (char) tmp;
     count++;
   }
 }

 return count;
}

int16_t SC16IS750_readwithtimeout(SC16IS750_t *dev, uint8_t *channel) {
 int16_t tmp;
 uint32_t time_stamp;
 time_stamp = millis();
 do {
   *channel = SC16IS750_CHANNEL_A;
   tmp = SC16IS750_read(dev, SC16IS750_CHANNEL_A);
   if (tmp >= 0)
     return tmp;
   if (dev->channels == SC16IS750_DUAL_CHANNEL) {
     *channel = SC16IS750_CHANNEL_B;
     tmp = SC16IS750_read(dev, SC16IS750_CHANNEL_B);
     if (tmp >= 0)
       return tmp;
   }
 } while (millis() - time_stamp < dev->timeout);
 return -1;  // -1 indicates timeout
}

// int SC16IS750_read(SC16IS750_t *dev, uint8_t channel) {
//   if (dev->peek_flag == 0) {
//     return SC16IS750_ReadByte(dev, channel);
//   } else {
//     dev->peek_flag = 0;
//     return dev->peek_buf;
//   }
// }

// void SC16IS750_write(SC16IS750_t *dev, uint8_t channel, uint8_t val) { SC16IS750_WriteByte(dev, channel, val); }

// void SC16IS750_pinMode(SC16IS750_t *dev, uint8_t pin, uint8_t i_o) { SC16IS750_GPIOSetPinMode(dev, pin, i_o); }

// void SC16IS750_digitalWrite(SC16IS750_t *dev, uint8_t pin, uint8_t value) {
//   SC16IS750_GPIOSetPinState(dev, pin, value);
// }

// uint8_t SC16IS750_digitalRead(SC16IS750_t *dev, uint8_t pin) { return SC16IS750_GPIOGetPinState(dev, pin); }

// uint8_t SC16IS750_ReadRegister(SC16IS750_t *dev, uint8_t channel, uint8_t reg_addr) {
//   uint8_t result;
//   // printf("ReadRegister channel=%d reg_addr=%x\n",channel, (reg_addr<<3 | channel<<1));

//   result = wiringPiI2CReadReg8(dev->i2c_fd, (reg_addr << 3 | channel << 1));
//   // printf("result=0x%x\n",result);

//   return result;
// }

void SC16IS750_WriteRegister(SC16IS750_t *dev, uint8_t channel, uint8_t reg_addr, uint8_t val) {
  wiringPiI2CWriteReg8(dev->i2c_fd, (reg_addr << 3 | channel << 1), val);
  return;
}

int16_t SC16IS750_SetBaudrate(SC16IS750_t *dev, uint8_t channel,
                              uint32_t baudrate)  // return error of baudrate parts per thousand
{
  uint16_t divisor;
  uint8_t prescaler;
  uint32_t actual_baudrate;
  int16_t error;
  uint8_t temp_lcr;
  if ((SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_MCR) & 0x80) == 0) {  // if prescaler==1
    prescaler = 1;
  } else {
    prescaler = 4;
  }

  // divisor = (SC16IS750_CRYSTCAL_FREQ/prescaler)/(baudrate*16);
  uint32_t divisor1 = dev->crystal_freq / prescaler;
  // printf("dev->crystal_freq=%ld prescaler=%d divisor1=%d\n",dev->crystal_freq, prescaler, divisor1);
  // uint32_t max_baudrate = divisor1/16;
  // printf("max_baudrate=%d\n",max_baudrate);
  uint32_t divisor2 = baudrate * 16;
  // printf("divisor2=%d\n",divisor2);
  divisor = (dev->crystal_freq / prescaler) / (baudrate * 16);
  if (divisor2 > divisor1) {
    printf("This baudrate (%d) is not support\n", baudrate);
    return 0;
  }
  // divisor = divisor1/divisor2;
  // divisor rounds up
  double wk = (double) divisor1 / (double) divisor2;
  divisor = wk + 0.999;
  // printf("baudrate=%d divisor=%d\n",baudrate,divisor);

  temp_lcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LCR);
  temp_lcr |= 0x80;
  SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_LCR, temp_lcr);
  // write to DLL
  SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_DLL, (uint8_t) divisor);
  // write to DLH
  SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_DLH, (uint8_t) (divisor >> 8));
  temp_lcr &= 0x7F;
  SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_LCR, temp_lcr);

#if 0
  actual_baudrate = divisor1 / divisor2;
  error = ((float)actual_baudrate-baudrate)*1000/baudrate;
#endif
  actual_baudrate = (divisor1 / divisor) / 16;
  error = baudrate - actual_baudrate;
  // printf("actual_baudrate=%d error=%d\n", actual_baudrate, error);
  if (error != 0) {
    printf("Warning:baudrate=%d actual_baudrate=%d\n", baudrate, actual_baudrate);
  }

#ifdef SC16IS750_DEBUG_PRINT
  printf("Desired baudrate: ");
  printf("%x\n", baudrate);
  printf("Calculated divisor: ");
  printf("%x\n", divisor);
  printf("Actual baudrate: ");
  printf("%x\n", actual_baudrate);
  printf("Baudrate error: ");
  printf("%x\n", error);
#endif

  return error;
}


void SC16IS750_GPIOSetPinMode(SC16IS750_t *dev, uint8_t pin_number, uint8_t i_o) {
  uint8_t temp_iodir;

  temp_iodir = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR);
  if (i_o == OUTPUT) {
    temp_iodir |= (0x01 << pin_number);
  } else {
    temp_iodir &= (uint8_t) ~(0x01 << pin_number);
  }

  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR, temp_iodir);
  return;
}

void SC16IS750_GPIOSetPinState(SC16IS750_t *dev, uint8_t pin_number, uint8_t pin_state) {
  uint8_t temp_iostate;

  temp_iostate = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
  if (pin_state == 1) {
    temp_iostate |= (0x01 << pin_number);
  } else {
    temp_iostate &= (uint8_t) ~(0x01 << pin_number);
  }

  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, temp_iostate);
  return;
}

uint8_t SC16IS750_GPIOGetPinState(SC16IS750_t *dev, uint8_t pin_number) {
  uint8_t temp_iostate;

  temp_iostate = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
  if ((temp_iostate & (0x01 << pin_number)) == 0) {
    return 0;
  }
  return 1;
}

uint8_t SC16IS750_GPIOGetPortState(SC16IS750_t *dev) {
  return SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
}

void SC16IS750_GPIOSetPortMode(SC16IS750_t *dev, uint8_t port_io) {
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR, port_io);
  return;
}

void SC16IS750_GPIOSetPortState(SC16IS750_t *dev, uint8_t port_state) {
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, port_state);
  return;
}

void SC16IS750_SetPinInterrupt(SC16IS750_t *dev, uint8_t io_int_ena) {
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOINTENA, io_int_ena);
  return;
}

void SC16IS752Channel::SC16IS750_ResetDevice(SC16IS750_t *dev) {
  uint8_t reg;

  reg = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
  reg |= 0x08;
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, reg);

  return;
}

void SC16IS750_ModemPin(SC16IS750_t *dev,
                        uint8_t gpio)  // gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
  uint8_t temp_iocontrol;

  temp_iocontrol = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
  if (gpio == 0) {
    temp_iocontrol |= 0x02;
  } else {
    temp_iocontrol &= 0xFD;
  }
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);

  return;
}

void SC16IS750_GPIOLatch(SC16IS750_t *dev, uint8_t latch) {
  uint8_t temp_iocontrol;

  temp_iocontrol = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
  if (latch == 0) {
    temp_iocontrol &= 0xFE;
  } else {
    temp_iocontrol |= 0x01;
  }
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);

  return;
}

void SC16IS750_FIFOReset(SC16IS750_t *dev, uint8_t channel, uint8_t rx_fifo) {
  uint8_t temp_fcr;

  temp_fcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_FCR);

  if (rx_fifo == 0) {
    temp_fcr |= 0x04;
  } else {
    temp_fcr |= 0x02;
  }
  SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_FCR, temp_fcr);

  return;
}

u
------------------------------------------------------- */

}  // namespace sc16is752
}  // namespace esphome
