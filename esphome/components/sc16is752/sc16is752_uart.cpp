#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

///////////////////////////////
// The SC16IS752Channel methods
///////////////////////////////
//
// The following functions implement the Interface part of the UARTComponent
//

// @TODO : timeout for read and write ???
void SC16IS752Channel::write_array(const uint8_t *data, size_t len) {
  if (tx_fifo_level() >= len)
    i2c::ErrorCode err = parent_->write_register(subaddress(SC16IS752_REG_RHR, channel_), data, len, true);
  else {
    // @TODO wait until more room in fifo
  }
}

// @TODO what do I return
bool SC16IS752Channel::read_array(uint8_t *data, size_t len) {
  if (!peek_.empty) {
    data[0] = peek_.byte;
    peek_.empty = true;
    data++;
    len--;
    if (len == 0)
      return true;
  }

  if (rx_fifo_level() >= len)
    i2c::ErrorCode err = parent_->read_register(subaddress(SC16IS752_REG_RHR, channel_), data, len, true);
  else {
    // @TODO wait until more chat avail with timeout ???
  }
  return true;
}

bool SC16IS752Channel::peek_byte(uint8_t *data) {
  if (peek_.empty) {
    peek_.empty = false;
    peek_.byte = read_register(SC16IS752_REG_RHR);
  }
  *data = peek_.byte;
  return true;  // TODO return value ???
}

int SC16IS752Channel::available() { return rx_fifo_level(); }

// TODO check
void SC16IS752Channel::flush() {
  do {
  } while ((read_register(SC16IS752_REG_LSR) & 0x20) == 0);  // loop until THR empty (bit[5])
}

// TODO DO!
void SC16IS752Channel::check_logger_conflict() {}

//
// local methods
//

uint8_t SC16IS752Channel::read_register(int address) {
  i2c::ErrorCode err = parent_->read_register(subaddress(address, channel_), &reg_buffer, 1, true);
  if (err != i2c::ERROR_OK)
    ;  // TODO LOG
  return reg_buffer;
}

void SC16IS752Channel::write_register(int address, uint8_t value) {
  reg_buffer = value;  // needed ?
  i2c::ErrorCode err = parent_->write_register(subaddress(SC16IS752_REG_FCR, channel_), &reg_buffer, 1, true);
  if (err != i2c::ERROR_OK)
    ;  // TODO LOG
}

void SC16IS752Channel::fifo_enable(bool enable) {
  auto fcr = read_register(SC16IS752_REG_FCR);
  enable ? (fcr &= 0xFE) : (fcr |= 0x01);
  write_register(SC16IS752_REG_FCR, fcr);
}

void SC16IS752Channel::set_line_param() {
  auto lcr = read_register(SC16IS752_REG_LCR);
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
      break;  // no parity
  }

  write_register(SC16IS752_REG_FCR, lcr);
}

void SC16IS752Channel::set_baudrate() {
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 14745600/16 = 192,000bps
  uint8_t pre_scaler;
  (read_register(SC16IS752_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;
  uint32_t upper_part = parent_->crystal_ / pre_scaler;
  uint32_t lower_part = baudrate_ * 16;
  uint32_t max_baudrate = upper_part / 16;
  ESP_LOGI(TAG, "crystal=%ld pre_scaler=%d max_baudrate=%ld", parent_->crystal_, pre_scaler, max_baudrate);

  if (lower_part > upper_part) {
    ESP_LOGE(TAG, "The requested baudrate (%ld) is not supported - set to 19200", baudrate_);
    baudrate_ = 19200;
    lower_part = baudrate_ * 16;
  }

  // we compute and round up the divisor
  uint32_t divisor = ceil((double) upper_part / (double) lower_part);

  auto lcr = read_register(SC16IS752_REG_LCR);
  lcr |= 0x80;  // set LCR[7] enable special registers
  write_register(SC16IS752_REG_LCR, lcr);
  write_register(SC16IS752_REG_DLL, (uint8_t) divisor);
  write_register(SC16IS752_REG_DLH, (uint8_t) (divisor >> 8));
  lcr &= 0x7F;  // reset LCR[7] disable special registers
  write_register(SC16IS752_REG_LCR, lcr);

  uint32_t actual_baudrate = (upper_part / divisor) / 16;
  ESP_LOGI(TAG, "Requested baudrate =%ld actual_baudrate=%ld", baudrate_, actual_baudrate);
}

/* -------------------------------------------------------
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

// uint8_t SC16IS750_digitalRead(SC16IS750_t *dev, uint8_t pin) { return SC16IS750_GPIOGetPinState(dev, pin); }

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

void SC16IS752Channel::SC16IS750_ResetDevice(SC16IS750_t *dev) {
  uint8_t reg;

  reg = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
  reg |= 0x08;
  SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, reg);

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

------------------------------------------------------- */

}  // namespace sc16is752
}  // namespace esphome
