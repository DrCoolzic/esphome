#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

static const char *const TAG = "sc16is752";

////////////////////////////////////////////////////////
// The SC16IS752Component methods
////////////////////////////////////////////////////////
bool SC16IS752Component::check_model_() {
  // we test the scratchpad reg for channel 1 if succeed then it is a 752
  reg_buffer = 0xAA;
  write_register(subaddress_(SC16IS752_REG_SPR, 1), &reg_buffer, 1);
  if ((read_register(subaddress_(SC16IS752_REG_SPR, 1), &reg_buffer, 1) == 0xAA) && (model_ == SC16IS752_MODEL))
    return true;
  else
    return false;
}

void SC16IS752Component::write_sc16is752_register_(uint8_t reg_address, uint8_t channel, const uint8_t *data,
                                                   size_t len) {
  auto error = this->write_register(subaddress_(reg_address, channel), data, len);
  if (error == i2c::ERROR_OK)
    this->status_clear_warning();
  else {
    this->status_set_warning();
    ESP_LOGE(TAG, "write_register_(): I2C I/O error: %d", (int) error);
  }
}

void SC16IS752Component::read_sc16is752_register_(uint8_t reg_address, uint8_t channel, uint8_t *data, size_t len) {
  auto error = this->read_register(subaddress_(reg_address, channel), data, len);
  if ((error == i2c::ERROR_OK))
    this->status_clear_warning();
  else {
    this->status_set_warning();
    ESP_LOGE(TAG, "read_register_(): I2C I/O error: %d", (int) error);
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
  std::string model_name = model_ == SC16IS750_MODEL ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s...", model_name);
  // we read anything just to test communication
  if (this->read(&reg_buffer, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
    return;
  }
  if (!check_model_())
    ESP_LOGCONFIG(TAG, "Wrong model %s specified?", model_name);
}

void SC16IS752Component::dump_config() {
  ESP_LOGCONFIG(TAG, "%s:", model_);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with %s failed!", model_);
  }
}

// nothing to do ?
void SC16IS752Component::update() {}

///////////////////////////////
// The SC16IS752Channel methods
///////////////////////////////

/// @TODO : timeout
void SC16IS752Channel::write_array(const uint8_t *data, size_t len) {
  int pos = 0;
  int remaining = len;
  do {
    // put as much bytes as we can
    auto put = std::min(tx_fifo_level_(), remaining);
    parent_->write_sc16is752_register_(SC16IS752_REG_RHR, channel_, &data[pos], put);
    remaining -= put;
    pos += put;
  } while (remaining > 0);
}

/// @TODO timeout
bool SC16IS752Channel::read_array(uint8_t *data, size_t len) {
  int pos = 0;
  int remaining = len;
  if (!peek_.empty) {
    data[pos++] = peek_.byte;
    peek_.empty = true;
    remaining--;
    if (remaining == 0)
      return true;
  }

  do {
    // get as much byte as we can
    auto get = std::min(rx_fifo_level_(), remaining);
    parent_->read_sc16is752_register_(SC16IS752_REG_RHR, channel_, &data[pos], get);
    remaining -= get;
    pos += get;
  } while (remaining > 0);
  return true;
}

/// @TODO timeout
bool SC16IS752Channel::peek_byte(uint8_t *data) {
  if (peek_.empty) {
    peek_.empty = false;
    do {  // wait until at least one char received
    } while (rx_fifo_level_() < 1);
    peek_.byte = read_uart_register_(SC16IS752_REG_RHR);
  }
  *data = peek_.byte;
  return true;
}

/// @TODO timeout
void SC16IS752Channel::flush() {
  do {
  } while ((read_uart_register_(SC16IS752_REG_LSR) & 0x20) == 0);  // loop until THR empty (bit[5])
}

uint8_t SC16IS752Channel::read_uart_register_(int reg_address) {
  parent_->read_sc16is752_register_(reg_address, channel_, &parent_->reg_buffer, 1);
  return parent_->reg_buffer;
}

void SC16IS752Channel::write_uart_register_(int reg_address, uint8_t value) {
  parent_->write_sc16is752_register_(reg_address, channel_, &value, 1);
}

void SC16IS752Channel::fifo_enable_(bool enable) {
  auto fcr = read_uart_register_(SC16IS752_REG_FCR);
  enable ? (fcr &= 0xFE) : (fcr |= 0x01);
  write_uart_register_(SC16IS752_REG_FCR, fcr);
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
}

void SC16IS752Channel::set_baudrate_() {
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 14745600/16 = 192,000bps
  uint8_t pre_scaler;  // we never use it
  (read_uart_register_(SC16IS752_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;
  uint32_t upper_part = parent_->crystal_ / pre_scaler;
  uint32_t lower_part = baudrate_ * 16;
  uint32_t max_baudrate = upper_part / 16;
  ESP_LOGI(TAG, "crystal=%ld pre_scaler=%d max_baudrate=%ld", parent_->crystal_, pre_scaler, max_baudrate);

  if (lower_part > upper_part) {
    ESP_LOGE(TAG, "The requested baudrate (%ld) is not supported - fallback to 19200", baudrate_);
    baudrate_ = 19200;
    lower_part = baudrate_ * 16;
  }

  // we compute and round up the divisor
  uint32_t divisor = ceil((double) upper_part / (double) lower_part);

  auto lcr = read_uart_register_(SC16IS752_REG_LCR);
  lcr |= 0x80;  // set LCR[7] enable special registers
  write_uart_register_(SC16IS752_REG_LCR, (uint8_t) lcr);
  write_uart_register_(SC16IS752_REG_DLL, (uint8_t) divisor);
  write_uart_register_(SC16IS752_REG_DLH, (uint8_t) (divisor >> 8));
  lcr &= 0x7F;  // reset LCR[7] disable special registers
  write_uart_register_(SC16IS752_REG_LCR, lcr);

  uint32_t actual_baudrate = (upper_part / divisor) / 16;
  ESP_LOGI(TAG, "Requested baudrate =%ld - actual baudrate=%ld", baudrate_, actual_baudrate);
}

///////////////////////////////////////////////////////////
// The SC16IS752GPIOPin methods
///////////////////////////////////////////////////////////
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
