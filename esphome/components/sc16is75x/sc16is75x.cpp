/// @file sc16s75x.cpp
/// @author @DrCoolzic
/// @brief sc16is75x implementation

#include "sc16is75x.h"

namespace esphome {
namespace sc16is75x {

static const char *const TAG = "sc16is75x";

/// @brief Converts the parity enumerator to a string
/// @param parity enum
/// @return the string
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
      return "INVALID";
  }
}

// convert an int to binary string
inline std::string i2s(uint8_t val) { return std::bitset<8>(val).to_string(); }
#define I2CS(val) (i2s(val).c_str())

// for more meaningful debug messages ...
static const char *write_reg_to_str[] = {"THR",   "IER",   "FCR", "LCR", "MCR", "LSR", "TCR", "SPR",
                                         "_INV_", "_INV_", "IOD", "IO",  "IOI", "IOC", "EFR"};
static const char *read_reg_to_str[] = {"RHR", "IER", "IIR", "LCR", "MCR", "LSR", "MSR", "SPR",
                                        "TXF", "RXF", "IOD", "IOP", "IOI", "IOC", "EFR"};

// the register address on the bus is build from the register value and the channel value.
// note that for I/O related register the chanel value is not significant so we set to zero
inline static uint8_t reg_bus_address(uint8_t reg_number, Channel channel = 0) {
  return (reg_number << 3 | channel << 1);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XComponent methods
///////////////////////////////////////////////////////////////////////////////
i2c::ErrorCode SC16IS75XComponent::write_sc16is75x_register_(uint8_t reg, Channel ch, const uint8_t *buf, size_t len) {
  auto rba = reg_bus_address(reg, ch);
  auto error = this->write_register(rba, buf, len);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "write_sc16is75x_register_ [%s, %d] => %02X, b=%02X, len=%d : I2C code %d", write_reg_to_str[reg],
              ch, rba, &buf, len, error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "write_sc16is75x_register_ [%s, %d] => %02X, b=%02X, len=%d : I2C code %d", write_reg_to_str[reg], ch,
             rba, &buf, len, error);
  }
  return error;
}

i2c::ErrorCode SC16IS75XComponent::read_sc16is75x_register_(uint8_t reg, Channel ch, uint8_t *buf, size_t len) {
  auto rba = reg_bus_address(reg, ch);
  auto error = this->read_register(rba, buf, len);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "read_sc16is75x_register_ [%s, %X] => %02X, b=%02X, len=%d : I2C code %d", read_reg_to_str[reg], ch,
              rba, &buf, len, error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "read_sc16is75x_register_ [%s, %X] => %02X, b=%02X, len=%d : I2C code %d", read_reg_to_str[reg], ch,
             rba, &buf, len, error);
  }
  return error;
}

bool SC16IS75XComponent::read_pin_val_(uint8_t pin) {
  this->read_sc16is75x_register_(SC16IS75X_REG_IOP, 0, &this->input_state_);
  ESP_LOGVV(TAG, "reading input pin %d in_state %s", pin, I2CS(input_state_));
  return this->input_state_ & (1 << pin);
}

void SC16IS75XComponent::write_pin_val_(uint8_t pin, bool value) {
  value ? this->output_state_ |= (1 << pin) : this->output_state_ &= ~(1 << pin);
  ESP_LOGV(TAG, "writing output pin %d out_state %s", pin, I2CS(this->output_state_));
  this->write_sc16is75x_register_(SC16IS75X_REG_IOP, 0, &this->output_state_);
}

void SC16IS75XComponent::set_pin_direction_(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT)
    this->pin_config_ &= ~(1 << pin);  // clear bit (input mode)
  else if (flags == gpio::FLAG_OUTPUT)
    this->pin_config_ |= 1 << pin;  // set bit (output mode)
  else
    ESP_LOGE(TAG, "pin %d direction invalid", pin);

  ESP_LOGD(TAG, "setting pin %d direction to %d pin_config=%s", pin, flags, I2CS(this->pin_config_));
  this->write_sc16is75x_register_(SC16IS75X_REG_IOD, 0, &this->pin_config_);  // TODO check ~
}

//
// overloaded methods from Component
//
void SC16IS75XComponent::setup() {
  const char *model_name = (this->model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up SC16IS75X:%s i2c_addr @%02X with %d UARTs...", this->get_name(), this->address_,
                this->children_.size());

  // // we read anything just to test communication
  // if (read_sc16is75x_register_(0, 0, &data_, 1) != i2c::ERROR_OK) {
  //   ESP_LOGCONFIG(TAG, "%s failed", model_name);
  //   this->mark_failed();
  // }

  // we can now setup our children
  for (auto child : this->children_)
    child->setup_channel();
}

void SC16IS75XComponent::dump_config() {
  const char *model_name = (this->model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "SC16IS75X:%s with %d UARTs...", this->get_name(), this->children_.size());
  ESP_LOGCONFIG(TAG, "  model %s", model_name);
  ESP_LOGCONFIG(TAG, "  crystal %d", this->crystal_);

  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with %s failed!", model_name);
  }

  for (auto child : this->children_)
    child->dump_channel();
  this->initialized_ = true;
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XChannel methods
///////////////////////////////////////////////////////////////////////////////
void SC16IS75XChannel::setup_channel() {
  ESP_LOGCONFIG(TAG, "  Setting up UART %s:%d...", this->parent_->get_name(), this->channel_);

  // reset and enable the fifo
  uint8_t fcr = 0x7;
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_FCR, this->channel_, &fcr);
  this->set_baudrate_();
  this->set_line_param_();
}

void SC16IS75XChannel::dump_channel() {
  ESP_LOGCONFIG(TAG, "  UART bus %s:%d...", this->parent_->get_name(), this->channel_);
  ESP_LOGCONFIG(TAG, "    baudrate %d Bd", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "    data_bits %d", this->data_bits_);
  ESP_LOGCONFIG(TAG, "    stop_bits %d", this->stop_bits_);
  ESP_LOGCONFIG(TAG, "    parity %s", parity_to_str(this->parity_));
}

void SC16IS75XChannel::set_line_param_() {
  uint8_t lcr;
  this->parent_->read_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);
  lcr &= 0xC0;                 // Clear the lower six bit of LCR (LCR[0] to LCR[5]) data bits
  switch (this->data_bits_) {  // data length settings
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
  if (this->stop_bits_ == 2) {
    lcr |= 0x04;
  }
  // parity
  switch (this->parity_) {              // parity selection settings
    case uart::UART_CONFIG_PARITY_ODD:  // odd parity
      lcr |= 0x08;
      break;
    case uart::UART_CONFIG_PARITY_EVEN:  // even parity
      lcr |= 0x18;
      break;
    default:
      break;  // no parity
  }
  // update register
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);
  ESP_LOGV(TAG, "UART %s:%d line set to %d data_bits, %d stop_bits, and %s parity [%s]", this->parent_->get_name(),
           this->channel_, this->data_bits_, this->stop_bits_, parity_to_str(this->parity_), I2CS(lcr));
}

void SC16IS75XChannel::set_baudrate_() {
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 14745600/16 = 192,000bps
  uint8_t pre_scaler = 1;  // we never use it... but we could if crystal is very high ...
  // (read_uart_register_(SC16IS75X_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;

  uint32_t upper_part = this->parent_->crystal_ / pre_scaler;
  uint32_t lower_part = this->baud_rate_ * 16;

  if (lower_part > upper_part) {  // sanity check
    ESP_LOGE(TAG, "The requested baudrate (%d) is too high - fallback to 19200", this->baud_rate_);
    this->baud_rate_ = 19200;
    lower_part = this->baud_rate_ * 16;
  }

  // we compute and round the divisor
  uint32_t divisor = (double) upper_part / (double) lower_part;
  uint32_t actual_baudrate = (upper_part / divisor) / 16;
  const uint8_t low = (uint8_t) divisor;
  const uint8_t high = (uint8_t) (divisor >> 8);

  uint8_t lcr;
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);
  lcr |= 0x80;  // set LCR[7] to enable special registers
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_DLL, this->channel_, &low);
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_DLH, this->channel_, &high);
  lcr &= 0x7F;  // reset LCR[7] to disable special registers
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);

  if (actual_baudrate == this->baud_rate_)
    ESP_LOGV(TAG, "UART %s:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", this->parent_->get_name(),
             this->channel_, this->parent_->crystal_, divisor, upper_part, lower_part, this->baud_rate_,
             actual_baudrate);
  else
    ESP_LOGW(TAG, "UART %s:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", this->parent_->get_name(),
             this->channel_, this->parent_->crystal_, divisor, upper_part, lower_part, this->baud_rate_,
             actual_baudrate);
}

bool SC16IS75XChannel::write_data_(const uint8_t *buffer, size_t len) {
  return this->parent_->write_sc16is75x_register_(SC16IS75X_REG_THR, this->channel_, buffer, len);
}

bool SC16IS75XChannel::read_data_(uint8_t *buffer, size_t len) {
  return this->parent_->read_sc16is75x_register_(SC16IS75X_REG_RHR, this->channel_, buffer, len);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XGPIOPin methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded GPIOPin
//

void SC16IS75XGPIOPin::setup() {
  ESP_LOGV(TAG, "Setting GPIO pin %d mode to %s", this->pin_,
           flags_ == gpio::FLAG_INPUT                        ? "Input"
           : (this->pin_, this->flags_ == gpio::FLAG_OUTPUT) ? "Output"
                                                             : "NOT SPECIFIED");
  // ESP_LOGCONFIG(TAG, "Setting GPIO pins direction/mode to '%s' %02X", i2s_(flags_), flags_);
  this->pin_mode(this->flags_);
}

void SC16IS75XGPIOPin::pin_mode(gpio::Flags flags) { this->parent_->set_pin_direction_(this->pin_, flags); }
bool SC16IS75XGPIOPin::digital_read() { return this->parent_->read_pin_val_(this->pin_) != this->inverted_; }
void SC16IS75XGPIOPin::digital_write(bool value) {
  this->parent_->write_pin_val_(this->pin_, value != this->inverted_);
}

std::string SC16IS75XGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via SC16IS75X:%s", this->pin_, this->parent_->get_name());
  return buffer;
}

///////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS BELOW
///////////////////////////////////////////////////////////////////////////////
#define TEST_COMPONENT
#ifdef TEST_COMPONENT

void SC16IS75XComponent::test_gpio_() {
  static bool init{false};
  if (!init) {
    init = true;
    input_state_ = 0;
    output_state_ = 0;
    for (int i = 0; i < 8; i++) {
      // set 4 first pins in input mode
      // set 4 next pins in output mode
      this->set_pin_direction_(i, (i < 4) ? esphome::gpio::Flags::FLAG_INPUT : esphome::gpio::Flags::FLAG_OUTPUT);
    }
    ESP_LOGI(TAG, "pins configuration: %s", I2CS(this->pin_config_));
  }
  for (int i = 0; i < 4; i++) {
    // read inputs
    auto val = this->read_pin_val_(i);
    // copy input to output
    this->write_pin_val_(i + 4, val);
  }
  ESP_LOGI(TAG, "input pins: %s", I2CS(this->input_state_));
  ESP_LOGI(TAG, "output pins: %s", I2CS(this->output_state_));
}

void SC16IS75XComponent::loop() {
  if (!this->initialized_)
    return;
  for (auto child : this->children_)
    child->rx_fifo_to_buffer_();

  //
  // Only use if component is in test mode
  //
  if (!this->test_mode_)
    return;

  char preamble[64];
  static int32_t loop_time = 0;
  ESP_LOGI(TAG, "%d ms since last loop call ...", millis() - loop_time);
  loop_time = millis();

  for (auto i = 0; i < children_.size(); i++) {
    snprintf(preamble, sizeof(preamble), "SC16IS75X_%s_Ch_%d", this->get_name(), i);
    children_[i]->uart_send_test(preamble);
    children_[i]->uart_receive_test(preamble, this->test_mode_ > 1);
  }

  // test_gpio_();

  ESP_LOGI(TAG, "loop execution time %d ms...", millis() - loop_time);
}
#endif

}  // namespace sc16is75x
}  // namespace esphome
