/// @file sc16s752.cpp
/// @author @DrCoolzic
/// @brief sc16is752 implementation

#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

static const char *const TAG = "sc16is752";

///////////////////////////////////////////////////////////////////////////////
// The SC16IS752Component methods
///////////////////////////////////////////////////////////////////////////////

bool SC16IS752Component::check_model_() {
  // TODO
  // // we test the scratchpad reg for channel 1 if succeed then it is a 752
  // buffer_ = 0xAA;
  // write_register(subaddress_(SC16IS752_REG_SPR, 1), &buffer_, 1);
  // if ((read_register(subaddress_(SC16IS752_REG_SPR, 1), &buffer_, 1) == 0xAA) && (model_ == SC16IS752_MODEL))
  //   return true;
  // else
  //   return false;
  return true;
}

void SC16IS752Component::write_sc16is752_register_(uint8_t reg_address, uint8_t channel, const uint8_t *buffer,
                                                   size_t len) {
  auto sub = subaddress_(reg_address, channel);
  auto error = this->write_register(sub, buffer, len);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "I2CDevice::write_register(0x%2X[%s,%X], b=%X..., l=%d) - I2C return %d", sub,
              write_reg_to_str[reg_address], channel, *buffer, len, (int) error);
  } else {
    this->status_set_warning();
    ESP_LOGE(TAG, "I2CDevice::write_register(0x%02X[%s,%X], b=%X..., l=%d): I2C I/O error: %d", sub,
             write_reg_to_str[reg_address], channel, *buffer, len, (int) error);
  }
}

void SC16IS752Component::read_sc16is752_register_(uint8_t reg_address, uint8_t channel, uint8_t *buffer, size_t len) {
  auto sub = subaddress_(reg_address, channel);
  auto error = this->read_register(sub, buffer, len);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "I2CDevice::read_register(0x%02X[%s,%X], b=%X..., l=%d): I2C return %d", sub,
              read_reg_to_str[reg_address], channel, *buffer, len, (int) error);
  } else {
    this->status_set_warning();
    ESP_LOGE(TAG, "I2CDevice::read_register(0x%02X[%s,%X], b=%X..., l=%d): I2C I/O error: %d", sub,
             read_reg_to_str[reg_address], channel, *buffer, len, (int) error);
  }
}

void SC16IS752Component::write_io_register_(int reg_address, uint8_t value) {
  write_sc16is752_register_(reg_address, 0, &value, 1);
}

int SC16IS752Component::read_io_register_(int reg_address) {
  uint8_t data;
  read_sc16is752_register_(reg_address, 0, &data, 1);
  return data;
}

bool SC16IS752Component::read_pin_val_(uint8_t pin) {
  input_state_ = this->read_io_register_(SC16IS752_REG_IOPIN);
  return this->input_state_ & (1 << pin);
}

void SC16IS752Component::write_pin_val_(uint8_t pin, bool value) {
  if (value) {
    this->output_state_ |= (1 << pin);
  } else {
    this->output_state_ &= ~(1 << pin);
  }
  this->write_io_register_(SC16IS752_REG_IOPIN, this->output_state_);
}

void SC16IS752Component::set_pin_mode_(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT) {
    this->pin_config_ &= ~(1 << pin);  // clear bit (input mode)
    if (flags == gpio::FLAG_OUTPUT) {
      this->pin_config_ |= 1 << pin;   // set bit (output mode)
    }
    this->write_io_register_(SC16IS752_REG_IODIR, ~this->pin_config_);
  }
}

//
// overloaded functions from Component
//
void SC16IS752Component::setup() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s with %d UARTs...", model_name, (int) children.size());
  // we read anything just to test communication TODO
  if (this->read(&buffer_, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
  }
  if (!check_model_())
    ESP_LOGCONFIG(TAG, "Wrong model %s specified?", model_name);

  // we setup our children
  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "Setting up %s UART %d...", model_name, i);
    children[i]->fifo_enable_(true);
    children[i]->set_baudrate_();
    children[i]->set_line_param_();
  }
}

void SC16IS752Component::dump_config() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "%s:", model_name);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with %s failed!", model_name);
  }

  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "%s UART %d...", model_name, i);
  }
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS752Channel methods
///////////////////////////////////////////////////////////////////////////////

/// **IMPLEMENTATION DETAILS** - I am not aware of any formal definition of this
/// function written somewhere, but based on common sense and looking at Arduino
/// code it seems that the following is expected:
/// - the function tries to receive 'len' characters from the uart into buffer:
///   - terminates with true if requested number of characters have been read,
///   - terminates with false if we have a timeout condition
/// Note that the SC16IS75X UART has a 64 bytes internal buffer
bool SC16IS752Channel::read_array(uint8_t *buffer, size_t len) {
  if (!peek_.empty) {
    *buffer++ = peek_.byte;
    peek_.empty = true;
    len--;
    if (len == 0)
      return true;
  }

  if (!check_read_timeout_(len)) {  // timeout
    ESP_LOGE(TAG, "Reading from UART timed out ...");
    return false;
  }

  uint32_t start_time = millis();
  while (rx_fifo_level_() < len) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "Reading from UART timed out at byte %u!", this->available());
      return false;
    }
    yield();  // not sure what it does? I suppose reschedule thread to avoid blocking?
  }
  parent_->read_sc16is752_register_(SC16IS752_REG_RHR, channel_, buffer, len);
  return true;
}

/// @brief Please refer to @ref read_array() for more information
bool SC16IS752Channel::peek_byte(uint8_t *buffer) {
  if (peek_.empty) {
    peek_.empty = false;

    uint32_t start_time = millis();
    while (rx_fifo_level_() == 0) {
      if (millis() - start_time > 100) {
        ESP_LOGE(TAG, "Peeking from UART timed out!");
        return false;
      }
      yield();  // not sure what it does?
    }
    peek_.byte = read_uart_register_(SC16IS752_REG_RHR);
  }
  *buffer = peek_.byte;
  return true;
}

/// **IMPLEMENTATION DETAILS** - I am not aware of any formal definition of this
/// function written somewhere, but based on common sense and looking at Arduino
/// code it seems that the following is expected:
/// - the function tries to send 'len' characters through uart from buffer
///
/// Even though the read_byte function is poorly defined the write function is worse
/// for several reasons: There is no indication on available buffer size for writing
/// therefore you send without knowing if it will succeed, and even more importantly
/// when you call this function you do not know if the bytes have been sent
/// successfully. In other word you are totally blind when using this function!
/// Therefore here is what I do:
/// - if 'len' is less that available space in fifo I send all char into the
/// TX fifo and I return
/// - otherwise I return false
void SC16IS752Channel::write_array(const uint8_t *buffer, size_t len) {
  if (len > tx_fifo_level_())
    return;
  parent_->write_sc16is752_register_(SC16IS752_REG_RHR, channel_, buffer, len);
}

/// **IMPLEMENTATION DETAILS** - This function is the worse of all UART functions !!! To me it
/// does not make sense at all and I have no idea how someone could use it.  If we
/// refer to Serial.flush() in Arduino we have:
/// ** Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0,
/// this instead removed any buffered incoming serial data.) **
/// While flushing an input fifo make sense, flushing an output fifo make no sense!
/// Therefore I do nothing
void SC16IS752Channel::flush() {
  ESP_LOGE(TAG, "This functions is not implemented");
  parent_->mark_failed();
}

uint8_t SC16IS752Channel::read_uart_register_(int reg_address) {
  parent_->read_sc16is752_register_(reg_address, channel_, &parent_->buffer_, 1);
  return parent_->buffer_;
}

void SC16IS752Channel::write_uart_register_(int reg_address, uint8_t value) {
  parent_->write_sc16is752_register_(reg_address, channel_, &value, 1);
}

void SC16IS752Channel::fifo_enable_(bool enable) {
  auto fcr = read_uart_register_(SC16IS752_REG_FCR);
  enable ? (fcr &= 0xFE) : (fcr |= 0x01);
  write_uart_register_(SC16IS752_REG_FCR, fcr);
  ESP_LOGCONFIG(TAG, "UART channel %d fifo %s", channel_, enable ? "enabled" : "disabled");
}

void SC16IS752Channel::set_line_param_() {
  auto lcr = read_uart_register_(SC16IS752_REG_LCR);
  lcr &= 0xC0;  // Clear the lower six bit of LCR (LCR[0] to LCR[5]
  // data bits
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
  // stop bits
  if (stop_bits_ == 2) {
    lcr |= 0x04;
  }
  // parity
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
  // update
  write_uart_register_(SC16IS752_REG_FCR, lcr);
  ESP_LOGCONFIG(TAG, "UART channel %d line set to %d data_bits, %d stop_bits, and %s parity", channel_, data_bits_,
                stop_bits_,
                parity_ == UART_CONFIG_PARITY_ODD    ? "ODD"
                : parity_ == UART_CONFIG_PARITY_EVEN ? "EVEN"
                                                     : "NONE");
}

void SC16IS752Channel::set_baudrate_() {
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 14745600/16 = 192,000bps
  uint8_t pre_scaler;  // we never use it
  (read_uart_register_(SC16IS752_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;
  uint32_t upper_part = parent_->crystal_ / pre_scaler;
  uint32_t lower_part = baudrate_ * 16;
  // uint32_t max_baudrate = upper_part / 16;

  if (lower_part > upper_part) {
    ESP_LOGE(TAG, "The requested baudrate (%d) is too high - fallback to 19200", baudrate_);
    baudrate_ = 19200;
    lower_part = baudrate_ * 16;
  }

  // we compute and round up the divisor
  uint32_t divisor = ceil((double) upper_part / (double) lower_part);

  ESP_LOGVV(TAG, "crystal=%d baudrate=%d pre_scaler=%d divisor=%d", parent_->crystal_, baudrate_, pre_scaler, divisor);

  auto lcr = read_uart_register_(SC16IS752_REG_LCR);
  lcr |= 0x80;  // set LCR[7] enable special registers
  write_uart_register_(SC16IS752_REG_LCR, (uint8_t) lcr);
  write_uart_register_(SC16IS752_REG_DLL, (uint8_t) divisor);
  write_uart_register_(SC16IS752_REG_DLH, (uint8_t) (divisor >> 8));
  lcr &= 0x7F;  // reset LCR[7] disable special registers
  write_uart_register_(SC16IS752_REG_LCR, lcr);

  uint32_t actual_baudrate = (upper_part / divisor) / 16;
  ESP_LOGCONFIG(TAG, "Crystal %d Requested baudrate %d - actual baudrate %d", parent_->crystal_, baudrate_,
                actual_baudrate);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS752GPIOPin methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded GPIOPin
//
void SC16IS752GPIOPin::setup() { pin_mode(flags_); }
void SC16IS752GPIOPin::pin_mode(gpio::Flags flags) { this->parent_->set_pin_mode_(this->pin_, flags); }
bool SC16IS752GPIOPin::digital_read() { return this->parent_->read_pin_val_(this->pin_) != this->inverted_; }
void SC16IS752GPIOPin::digital_write(bool value) {
  this->parent_->write_pin_val_(this->pin_, value != this->inverted_);
}
std::string SC16IS752GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via SC16IS752", pin_);
  return buffer;
}

}  // namespace sc16is752
}  // namespace esphome
