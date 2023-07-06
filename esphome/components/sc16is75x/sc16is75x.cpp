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

// convert byte to binary string
inline const char *i2s_(uint8_t val) { return std::bitset<8>(val).to_string().c_str(); }

// for more meaningful debug messages ...
static const char *write_reg_to_str[] = {"THR",   "IER",   "FCR", "LCR", "MCR", "LSR", "TCR", "SPR",
                                         "_INV_", "_INV_", "IOD", "IO",  "IOI", "IOC", "EFR"};
static const char *read_reg_to_str[] = {"RHR", "IER", "IIR", "LCR", "MCR", "LSR", "MSR", "SPR",
                                        "TXF", "RXF", "IOD", "IOP", "IOI", "IOC", "EFR"};

// the register address is composed of the register value and the channel value.
// note that for I/O related register the chanel value is not significant so we set to zero
inline static uint8_t reg_address(uint8_t reg_number, Channel channel = 0) { return (reg_number << 3 | channel << 1); }

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XComponent methods
///////////////////////////////////////////////////////////////////////////////
int SC16IS75XComponent::count_{0};  // initialize static count

i2c::ErrorCode SC16IS75XComponent::write_sc16is75x_register_(uint8_t reg_number, Channel channel, const uint8_t *buffer,
                                                             size_t len) {
  auto sub = reg_address(reg_number, channel);
  auto error = this->write_register(sub, buffer, len);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "write_sc16is75x_register_(%s, %X [0x%02X], b=%02X [%s], len=%d): I2C code %d",
              write_reg_to_str[reg_number], channel, sub, *buffer, i2s_(*buffer), len, (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "write_sc16is75x_register_(%s, %X [0x%02X], b=%02X [%s], len=%d): I2C code %d",
             write_reg_to_str[reg_number], channel, sub, *buffer, i2s_(*buffer), len, (int) error);
  }
  return error;
}

i2c::ErrorCode SC16IS75XComponent::read_sc16is75x_register_(uint8_t reg_number, Channel channel, uint8_t *buffer,
                                                            size_t len) {
  auto sub = reg_address(reg_number, channel);
  auto error = this->read_register(sub, buffer, len);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "read_sc16is75x_register_(%s, %X [0x%02X], b=%02X [%s], len=%d): I2C code %d",
              read_reg_to_str[reg_number], channel, sub, *buffer, i2s_(*buffer), len, (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "read_sc16is75x_register_(%s, %X [0x%02X], b=%02X [%s], len=%d): I2C code %d",
             read_reg_to_str[reg_number], channel, sub, *buffer, i2s_(*buffer), len, (int) error);
  }
  return error;
}

inline void SC16IS75XComponent::write_io_register_(int reg_number, uint8_t value) {
  this->write_sc16is75x_register_(reg_number, 0, &value, 1);
}

inline uint8_t SC16IS75XComponent::read_io_register_(int reg_number) {
  this->read_sc16is75x_register_(reg_number, 0, &buffer_, 1);
  return buffer_;
}

bool SC16IS75XComponent::read_pin_val_(uint8_t pin) {
  this->input_state_ = this->read_io_register_(SC16IS75X_REG_IOP);
  ESP_LOGVV(TAG, "reading input pin %d in_state %s", pin, i2s_(input_state_));
  // TODO log when value changed from last call ?
  return this->input_state_ & (1 << pin);
}

void SC16IS75XComponent::write_pin_val_(uint8_t pin, bool value) {
  value ? this->output_state_ |= (1 << pin) : this->output_state_ &= ~(1 << pin);
  ESP_LOGV(TAG, "writing output pin %d out_state %s", pin, i2s_(output_state_));
  this->write_io_register_(SC16IS75X_REG_IOP, this->output_state_);
}

void SC16IS75XComponent::set_pin_direction_(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT)
    this->pin_config_ &= ~(1 << pin);  // clear bit (input mode)
  else if (flags == gpio::FLAG_OUTPUT)
    this->pin_config_ |= 1 << pin;     // set bit (output mode)
  else
    ESP_LOGE(TAG, "pin %d direction invalid", pin);

  ESP_LOGD(TAG, "setting pin %d direction to %d pin_config=%s", pin, flags, i2s_(pin_config_));
  this->write_io_register_(SC16IS75X_REG_IOD, ~this->pin_config_);
}

//
// overloaded methods from Component
//
void SC16IS75XComponent::setup() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up SC16IS75X:%d i2c_addr @%02X with %d UARTs...", get_num_(), address_,
                (int) children_.size());
  // we read anything just to test communication
  if (read_sc16is75x_register_(0, 0, &buffer_, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
  }

  // we can now setup our children
  for (auto child : this->children_)
    child->setup_channel();
}

void SC16IS75XComponent::dump_config() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "SC16IS75X:%d with %d UARTs...", get_num_(), (int) children_.size());
  ESP_LOGCONFIG(TAG, "  model %s", model_name);
  ESP_LOGCONFIG(TAG, "  crystal %d", crystal_);

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

uint8_t SC16IS75XChannel::read_uart_register_(int reg_number) {
  this->parent_->buffer_ = 0;  // for debug help
  this->parent_->read_sc16is75x_register_(reg_number, this->channel_, &this->parent_->buffer_, 1);
  return this->parent_->buffer_;
}

void SC16IS75XChannel::write_uart_register_(int reg_number, uint8_t value) {
  this->parent_->write_sc16is75x_register_(reg_number, this->channel_, &value, 1);
}

void SC16IS75XChannel::setup_channel() {
  ESP_LOGCONFIG(TAG, "  Setting up UART %d:%d...", this->parent_->get_num_(), this->channel_);

  // reset and enable the fifo
  uint8_t fcr = 0x7;
  this->write_uart_register_(SC16IS75X_REG_FCR, fcr);
  this->set_baudrate_();
  this->set_line_param_();
}

void SC16IS75XChannel::dump_channel() {
  ESP_LOGCONFIG(TAG, "  UART bus %d:%d...", parent_->get_num_(), channel_);
  ESP_LOGCONFIG(TAG, "    baudrate %d Bd", baud_rate_);
  ESP_LOGCONFIG(TAG, "    data_bits %d", data_bits_);
  ESP_LOGCONFIG(TAG, "    stop_bits %d", stop_bits_);
  ESP_LOGCONFIG(TAG, "    parity %s", parity_to_str(parity_));
}

void SC16IS75XChannel::set_line_param_() {
  auto lcr = this->read_uart_register_(SC16IS75X_REG_LCR);
  lcr &= 0xC0;           // Clear the lower six bit of LCR (LCR[0] to LCR[5]) data bits
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
  switch (parity_) {                    // parity selection settings
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
  this->write_uart_register_(SC16IS75X_REG_LCR, lcr);
  ESP_LOGV(TAG, "UART %d:%d line set to %d data_bits, %d stop_bits, and %s parity [%s]", parent_->get_num_(), channel_,
           data_bits_, stop_bits_, parity_to_str(parity_), i2s_(lcr));
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

  auto lcr = read_uart_register_(SC16IS75X_REG_LCR);
  lcr |= 0x80;  // set LCR[7] to enable special registers
  this->write_uart_register_(SC16IS75X_REG_LCR, (uint8_t) lcr);
  this->write_uart_register_(SC16IS75X_REG_DLL, (uint8_t) divisor);
  this->write_uart_register_(SC16IS75X_REG_DLH, (uint8_t) (divisor >> 8));
  lcr &= 0x7F;  // reset LCR[7] to disable special registers
  this->write_uart_register_(SC16IS75X_REG_LCR, lcr);

  if (actual_baudrate == baud_rate_)
    ESP_LOGV(TAG, "UART %d:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", parent_->get_num_(), channel_,
             parent_->crystal_, divisor, upper_part, lower_part, baud_rate_, actual_baudrate);
  else
    ESP_LOGW(TAG, "UART %d:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", parent_->get_num_(), channel_,
             parent_->crystal_, divisor, upper_part, lower_part, baud_rate_, actual_baudrate);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XGPIOPin methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded GPIOPin
//

void SC16IS75XGPIOPin::setup() {
  ESP_LOGV(TAG, "Setting GPIO pin %d mode to %s", this->pin_,
           flags_ == gpio::FLAG_INPUT                  ? "Input"
           : (this->pin_, flags_ == gpio::FLAG_OUTPUT) ? "Output"
                                                       : "NOT SPECIFIED");
  // ESP_LOGCONFIG(TAG, "Setting GPIO pins direction/mode to '%s' %02X", i2s_(flags_), flags_);
  this->pin_mode(flags_);
}

void SC16IS75XGPIOPin::pin_mode(gpio::Flags flags) { this->parent_->set_pin_direction_(this->pin_, flags); }
bool SC16IS75XGPIOPin::digital_read() { return this->parent_->read_pin_val_(this->pin_) != this->inverted_; }
void SC16IS75XGPIOPin::digital_write(bool value) {
  this->parent_->write_pin_val_(this->pin_, value != this->inverted_);
}

std::string SC16IS75XGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via SC16IS75X:%d", this->pin_, this->parent_->get_num_());
  return buffer;
}

///////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS BELOW
///////////////////////////////////////////////////////////////////////////////
#define TEST_COMPONENT
#ifdef TEST_COMPONENT

void SC16IS75XComponent::test_gpio() {
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
    ESP_LOGI(TAG, "pins configuration: %s", i2s_(this->pin_config_));
  }
  for (int i = 0; i < 4; i++) {
    // read inputs
    auto val = this->read_pin_val_(i);
    // copy input to output
    this->write_pin_val_(i + 4, val);
  }
  ESP_LOGI(TAG, "input pins: %s", i2s_(this->input_state_));
  ESP_LOGI(TAG, "output pins: %s", i2s_(this->output_state_));
}

void SC16IS75XComponent::loop() {
  //
  // This loop is used only if the sc16is75x component is in test mode
  //
  if (!initialized_ || !test_mode_)
    return;

  char preamble[64];
  static int32_t loop_time = 0;
  ESP_LOGI(TAG, "%d ms since last loop call ...", millis() - loop_time);
  loop_time = millis();

  int i{0};
  for (auto child : this->children_) {
    snprintf(preamble, sizeof(preamble), "SC16IS75X_%d_Ch_%d", get_num_(), i++);
    child->uart_send_test(preamble);
    child->uart_receive_test(preamble, test_mode_ > 1);
  }

  test_gpio();

  ESP_LOGI(TAG, "loop execution time %d ms...", millis() - loop_time);
}
#endif

}  // namespace sc16is75x
}  // namespace esphome
