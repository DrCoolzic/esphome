/// @file wk2132.cpp
/// @author @DrCoolzic
/// @brief wk2132 implementation

#include "wk2132.h"

namespace esphome {
namespace wk2132 {

static const char *const TAG = "wk2132";

/*! @page page_wk2132x_ WK2132Component documentation
This page gives some information about the details of implementation of
the WK2132Component class for ESPHome.

@section wk2132_component_ WK2132Component class
This class describes a WK2132 I²C component. It derives from two @ref esphome classes:
- The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
  @ref Component::dump_config() and @ref Component::get_setup_priority() methods
- The @ref i2c::I2CDevice class. From which we use some methods

We have a related class: @ref WK2132Channel class that takes cares of the UART related functions

All call to the i2c::I2CDevice register read and write are funneled through two functions:
- The WK2132Component::read_wk2132_register_() and
- The WK2132Component::write_wk2132_register_().

  @section wk2132_uart_ WK2132Channel (UART) class

TODO

*/

///////////////////////////////////////////////////////////////////////////////
// The WK2132Component methods
///////////////////////////////////////////////////////////////////////////////

int WK2132Component::count_{0};  // init static count of instances

inline const uint8_t i2c_address(uint8_t base_address, uint8_t channel, uint8_t fifo = 0) {
  // the address of the device is 0AA1 0CCF where AA is the address read from A1, A0
  // CC is the channel number (in practice 00 or 01) and F is one when accessing FIFO
  return base_address | channel << 1 | fifo;
}

inline const uint8_t real_reg_address(uint8_t reg_address, uint8_t channel) {
  // - the register address for the Global register is directly the reg_number
  // - the register address for the UART register is 00CC XXXX where CC is the channel
  //   number and XXXX the channel number.
  if ((reg_address == REG_WK2132_GENA) || (reg_address == REG_WK2132_GRST))
    return reg_address;
  return channel << 4 | reg_address;
}

const char *parity2string(uart::UARTParityOptions parity) {
  using namespace uart;
  switch (parity) {
    case UART_CONFIG_PARITY_NONE:
      return "NONE";
    case UART_CONFIG_PARITY_EVEN:
      return "EVEN";
    case UART_CONFIG_PARITY_ODD:
      return "ODD";
    default:
      return "UNKNOWN";
  }
}

i2c::ErrorCode WK2132Component::write_wk2132_register_(uint8_t reg_address, uint8_t channel, const uint8_t *data) {
  // update the i2c address
  address_ = i2c_address(base_address_, reg_address, channel);

  auto rra = real_reg_address(reg_address, channel);
  auto error = this->write_register(rra, data, 1);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "write_wk2132_register_(%s, %X [0x%02X], b=%02X [%s]): I2C code %d", write_reg_to_str[reg_address],
              channel, rra, *data, i2s_(*data), (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "write_wk2132_register_(%s, %X [0x%02X], b=%02X [%s]): I2C code %d", write_reg_to_str[reg_address],
             channel, rra, *data, i2s_(*data), (int) error);
  }
  return error;
}

i2c::ErrorCode WK2132Component::read_wk2132_register_(uint8_t reg_address, uint8_t channel, uint8_t *data) {
  // update the i2c address
  address_ = i2c_address(base_address_, channel);

  auto rra = real_reg_address(reg_address, channel);
  auto error = this->read_register(rra, data, 1);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "read_wk2132_register_(%s, %X [0x%02X], b=%02X [%s]): I2C code %d", read_reg_to_str[reg_address],
              channel, rra, *data, i2s_(*data), (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "read_wk2132_register_(%s, %X [0x%02X], b=%02X [%s]): I2C code %d", read_reg_to_str[reg_address],
             channel, rra, *data, i2s_(*data), (int) error);
  }
  return error;
}

//
// overloaded functions from Component
//
void WK2132Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WK2132:%d with %d UARTs...", get_num_(), (int) children.size());
  // we test communication
  uint8_t data;
  if (read_wk2132_register_(REG_WK2132_GENA, 0, &data) != i2c::ERROR_OK)
    ESP_LOGCONFIG(TAG, "... WK2132:%d failed", get_num_());
  this->mark_failed();

  // we can now setup our children
  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "  Setting up UART %d:%d...", get_num_(), i);
    children[i]->setup_channel();
  }
}

void WK2132Component::dump_config() {
  ESP_LOGCONFIG(TAG, "WK2132:%d with %d UARTs...", get_num_(), (int) children.size());
  ESP_LOGCONFIG(TAG, "  base address %02X", base_address_);
  ESP_LOGCONFIG(TAG, "  crystal %d", crystal_);

  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with WK2132 failed!");
  }

  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "  UART bus %d:%d...", get_num_(), i);
    ESP_LOGCONFIG(TAG, "    baudrate %d Bd", children[i]->baud_rate_);
    ESP_LOGCONFIG(TAG, "    data_bits %d", children[i]->data_bits_);
    ESP_LOGCONFIG(TAG, "    stop_bits %d", children[i]->stop_bits_);
    ESP_LOGCONFIG(TAG, "    parity %s", parity2string(children[i]->parity_));
  }
}

void WK2132Component::loop() {
  //
  // This loop is used only if the wk2132 component is in test mode
  //
  if (test_mode_ == 0)
    return;
  static int32_t end_time = 0;
  ESP_LOGI(TAG, "time between loop call %d ms...", millis() - end_time);
  end_time = millis();

  switch (test_mode_) {
    case 1:
      children[0]->test_uart_(true);
      // test_io__ TODO
      break;

    case 2:
      // TODO
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// The WK2132Channel methods
///////////////////////////////////////////////////////////////////////////////

void WK2132Channel::setup_channel() {
  // enable clock for this channel => bit 0 for chanel 0 and bit 1 for channel 1
  uint8_t gena = 0;
  parent_->read_wk2132_register_(REG_WK2132_GENA, 0, &gena);
  channel_ == 0 ? gena |= 0x01 : gena |= 0x02;
  parent_->write_wk2132_register_(REG_WK2132_GENA, 0, &gena);

  // enable transmit/receive fifo for this => bit 2 & 3
  uint8_t fsr = 0x0C;
  parent_->write_wk2132_register_(REG_WK2132_FCR, channel_, &fsr);

  // enable transmit/receive on this channel => bit 0 & 1
  uint8_t scr = 0x03;
  parent_->write_wk2132_register_(REG_WK2132_SCR, channel_, &scr);

  // set spage? TODO
  set_baudrate_();
  set_line_param_();
}

size_t WK2132Channel::tx_available() {
  parent_->read_wk2132_register_(REG_WK2132_TFV, channel_, &data_);
  return data_;
}

size_t WK2132Channel::rx_available() {
  parent_->read_wk2132_register_(REG_WK2132_RFV, channel_, &data_);
  return data_;
}

bool WK2132Channel::read_data(uint8_t *buffer, size_t len) {
  parent_->address_ = i2c_address(parent_->base_address_, channel_, 1);
  bool status = true;

  // TODO test enough received if not change len status false

  auto error = parent_->read(buffer, len);
  if ((error == i2c::ERROR_OK)) {
    parent_->status_clear_warning();
    ESP_LOGVV(TAG, "read_data(c=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
              (int) error);
  } else {  // error
    parent_->status_set_warning();
    ESP_LOGE(TAG, "read_data(c=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
             (int) error);
    status = false;
  }
  return status;
}

bool WK2132Channel::write_data(const uint8_t *buffer, size_t len) {
  parent_->address_ = i2c_address(parent_->base_address_, channel_, 1);
  bool status = true;

  // TODO test enough space if not change len status false

  auto error = parent_->write(buffer, len);
  if ((error == i2c::ERROR_OK)) {
    parent_->status_clear_warning();
    ESP_LOGVV(TAG, "write_data(c=%d buffer[0]=%02X [%s], len): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
              (int) error);
  } else {  // error
    parent_->status_set_warning();
    ESP_LOGE(TAG, "write_data(c=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
             (int) error);
    status = false;
  }
  return status;
}

void WK2132Channel::set_line_param_() {
  uint8_t lcr;
  parent_->read_wk2132_register_(REG_WK2132_LCR, channel_, &lcr);

  lcr &= 0xF0;  // Clear the lower 4 bit of LCR
  // stop bits
  if (stop_bits_ == 2) {
    lcr |= 0x01;
  }

  // parity
  switch (parity_) {                    // parity selection settings
    case uart::UART_CONFIG_PARITY_ODD:  // odd parity
      lcr |= 0x5 << 1;
      break;
    case uart::UART_CONFIG_PARITY_EVEN:  // even parity
      lcr |= 0x6 << 1;
      break;
    default:
      break;  // no parity
  }
  // update register
  parent_->write_wk2132_register_(REG_WK2132_LCR, channel_, &lcr);
  ESP_LOGV(TAG, "UART %d:%d line set to %d data_bits, %d stop_bits, and %s parity [%s]", parent_->get_num_(), channel_,
           data_bits_, stop_bits_, parity2string(parity_), i2s_(lcr));
}

void WK2132Channel::set_baudrate_() {
  uint16_t val_int = parent_->crystal_ / (baud_rate_ * 16) - 1;
  uint16_t val_dec = (parent_->crystal_ % (baud_rate_ * 16)) / (baud_rate_ * 16);
  uint8_t baud_high = (uint8_t) (val_int >> 8);
  uint8_t baud_low = (uint8_t) (val_int & 0xFF);
  while (val_dec > 0x0A)
    val_dec /= 0x0A;
  uint8_t baud_dec = (uint8_t) (val_dec);

  // switch Page
  // parent_->read_wk2132_register_(REG_WK2132_PAGE, channel_, &data_)  // save current
  uint8_t page = 1;
  parent_->write_wk2132_register_(REG_WK2132_PAGE, channel_, &page);
  parent_->write_wk2132_register_(REG_WK2132_BRH, channel_, &baud_high);
  parent_->write_wk2132_register_(REG_WK2132_BRL, channel_, &baud_low);
  parent_->write_wk2132_register_(REG_WK2132_BRD, channel_, &baud_dec);
  page = 0;
  parent_->write_wk2132_register_(REG_WK2132_PAGE, channel_, &page);

  ESP_LOGV(TAG, "UART %d:%d Crystal=%d baudrate(h,l,d)=%d (%d,%d,%d)", parent_->get_num_(), channel_, parent_->crystal_,
           baud_rate_, baud_high, baud_low, baud_dec);
}

}  // namespace wk2132
}  // namespace esphome
