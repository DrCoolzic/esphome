/// @file sc16is75x_spi.cpp
/// @author @DrCoolzic
/// @brief sc16is75x_spi implementation

/*! @mainpage SC16IS75X documentation
Gives some information about the details of the implementation of
the SC16IS75X related classes.
*/

#include "sc16is75x_spi.h"

namespace esphome {
namespace sc16is75x_spi {

static const char *const TAG = "sc16is75x_spi";

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

/// @brief measure the time elapsed between two calls
/// @param last_time time od the previous call
/// @return the elapsed time in microseconds
uint32_t elapsed(uint32_t &last_time) {
  uint32_t e = millis() - last_time;
  last_time = millis();
  return e;
};

// for more meaningful debug messages ...
static const char *const WRITE_REG_TO_STR[2][16] = {
    {"THR", "IER", "FCR", "LCR", "MCR", "LSR", "TCR", "SPR", "???", "???", "IOD", "IOS", "IOI", "IOC", "EFR", "???"},
    {"DLL", "DHL", "EFR", "???", "XO1", "XO2", "XF1", "XF2", "???", "???", "???", "???", "???", "???", "???", "???"}};
static const char *const READ_REG_TO_STR[2][16] = {
    {"RHR", "IER", "IIR", "LCR", "MCR", "LSR", "MSR", "SPR", "TXF", "RXF", "IOD", "IOP", "IOI", "IOC", "EFR", "???"},
    {"DLL", "DHL", "EFR", "???", "XO1", "XO2", "XF1", "XF2", "???", "???", "???", "???", "???", "???", "???", "???"}};

enum TransferType { WRITE, READ };
// the address on the bus is build from the register value and the channel value combined with
// the w/r bit. For I/O registers the chanel value is not significant.
inline static uint8_t address_on_bus(uint8_t reg_number, Channel channel, TransferType transfer_type) {
  return (transfer_type << 7 | reg_number << 3 | channel << 1);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XSPIComponent methods
///////////////////////////////////////////////////////////////////////////////
void SC16IS75XSPIComponent::write_sc16is75x_register_(uint8_t reg, Channel channel, const uint8_t *data,
                                                      size_t length) {
  auto aob = address_on_bus(reg, channel, WRITE);
  this->enable();
  this->write_byte(aob);
  this->write_array(data, length);
  this->disable();
  ESP_LOGVV(TAG, "write_sc16is75x_register_ [%s, %d] => %02X, b=%02X, length=%d",
            WRITE_REG_TO_STR[this->special_reg_][reg], channel, aob, *data, length);
}

void SC16IS75XSPIComponent::read_sc16is75x_register_(uint8_t reg, Channel channel, uint8_t *data, size_t length) {
  auto aob = address_on_bus(reg, channel, READ);
  this->enable();
  this->write_byte(aob);
  this->read_array(data, length);
  this->disable();
  ESP_LOGVV(TAG, "read_sc16is75x_register_ [%s, %X] => %02X, b=%02X, length=%d",
            READ_REG_TO_STR[this->special_reg_][reg], channel, aob, *data, length);
}

bool SC16IS75XSPIComponent::read_pin_val_(uint8_t pin) {
  this->read_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &this->input_state_);
  ESP_LOGVV(TAG, "reading input pin %d = %d in_state %s", pin, this->input_state_ & (1 << pin), I2CS(input_state_));
  return this->input_state_ & (1 << pin);
}

void SC16IS75XSPIComponent::write_pin_val_(uint8_t pin, bool value) {
  value ? this->output_state_ |= (1 << pin) : this->output_state_ &= ~(1 << pin);
  ESP_LOGV(TAG, "writing output pin %d with %d out_state %s", pin, value, I2CS(this->output_state_));
  this->write_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &this->output_state_);
}

void SC16IS75XSPIComponent::set_pin_direction_(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT) {
    this->pin_config_ &= ~(1 << pin);  // clear bit (input mode)
  } else {
    if (flags == gpio::FLAG_OUTPUT) {
      this->pin_config_ |= 1 << pin;  // set bit (output mode)
    } else {
      ESP_LOGE(TAG, "pin %d direction invalid", pin);
    }
  }
  ESP_LOGD(TAG, "setting pin %d direction to %d pin_config=%s", pin, flags, I2CS(this->pin_config_));
  this->write_sc16is75x_register_(SC16IS75X_REG_IOD, 0, &this->pin_config_);  // TODO check ~
}

//
// overloaded methods from Component
//
void SC16IS75XSPIComponent::setup() {
  const char *model_name = (this->model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s:%s with %d UARTs...", model_name, this->get_name(), this->children_.size());
  this->spi_setup();
  // setup our children
  for (auto *child : this->children_)
    child->setup_channel_();
}

void SC16IS75XSPIComponent::dump_config() {
  const char *model_name = (this->model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "SC16IS75X SPI:%s with %d UARTs...", this->get_name(), this->children_.size());
  ESP_LOGCONFIG(TAG, "  Model: %s", model_name);
  LOG_PIN("  CS Pin: ", this->cs_);
  ESP_LOGCONFIG(TAG, "  Crystal: %d", this->crystal_);

  for (auto *child : this->children_)
    child->dump_channel_();
  this->initialized_ = true;
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XChannel methods
///////////////////////////////////////////////////////////////////////////////
void SC16IS75XChannel::setup_channel_() {
  ESP_LOGCONFIG(TAG, "  Setting up UART %s:%s...", this->parent_->get_name(), this->get_channel_name());

  uint8_t fcr = 0x7;  // reset and enable the fifo
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_FCR, this->channel_, &fcr);
  this->set_baudrate_();
  this->set_line_param_();
#ifdef USE_RING_BUFFER
  this->receive_buffer_ = new RingBuffer(this->buffer_size_);
#endif
}

void SC16IS75XChannel::dump_channel_() {
  ESP_LOGCONFIG(TAG, "  UART bus %s:%s...", this->parent_->get_name(), this->get_channel_name());
  ESP_LOGCONFIG(TAG, "    Baud Rate: %d Bd", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "    Data Bits: %d", this->data_bits_);
  ESP_LOGCONFIG(TAG, "    Stop Bits: %d", this->stop_bits_);
  ESP_LOGCONFIG(TAG, "    Parity: %s", parity2string(this->parity_));
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
           this->channel_, this->data_bits_, this->stop_bits_, parity2string(this->parity_), I2CS(lcr));
}

void SC16IS75XChannel::set_baudrate_() {
  uint8_t pre_scaler = 1;  // we never use it... but we could if crystal is very high ...
  // (read_register_(SC16IS75X_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;

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

  uint8_t lcr = 0;
  lcr |= 0x80;  // set LCR[7] to enable special registers
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);
  this->parent_->special_reg_ = 1;
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_DLL, this->channel_, &low);
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_DLH, this->channel_, &high);
  lcr &= 0x7F;  // reset LCR[7] to disable special registers
  this->parent_->special_reg_ = 0;
  this->parent_->write_sc16is75x_register_(SC16IS75X_REG_LCR, this->channel_, &lcr);

  if (actual_baudrate == this->baud_rate_) {
    ESP_LOGV(TAG, "UART %s:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", this->parent_->get_name(),
             this->channel_, this->parent_->crystal_, divisor, upper_part, lower_part, this->baud_rate_,
             actual_baudrate);
  } else {
    ESP_LOGW(TAG, "UART %s:%d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", this->parent_->get_name(),
             this->channel_, this->parent_->crystal_, divisor, upper_part, lower_part, this->baud_rate_,
             actual_baudrate);
  }
}

//
//  override UARTComponent methods
//

void SC16IS75XChannel::write_array(const uint8_t *buffer, size_t length) {
  if (length > FIFO_SIZE) {
    ESP_LOGE(TAG, "write_array() invalid call: requested %d bytes max size %d ...", length, FIFO_SIZE);
    length = FIFO_SIZE;
  }

  uint32_t const start_time = millis();
  // we wait until we have enough space in fifo
  while (length > (FIFO_SIZE - this->tx_in_fifo_())) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "write_array() overrun: %d bytes in fifo...", this->tx_in_fifo_());
      break;
    }
    yield();  // reschedule our thread to avoid blocking
  }
  this->write_data_(buffer, length);
}

void SC16IS75XChannel::flush() {
  uint32_t const start_time = millis();
  // we wait until fifo is empty
  while (this->tx_in_fifo_()) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "flush() timed out: still %d bytes not sent...", this->tx_in_fifo_());
      break;
    }
    yield();  // reschedule our thread to avoid blocking
  }
  ESP_LOGVV(TAG, "flush(): flushed the bytes in tx fifo");
}

#ifdef USE_RING_BUFFER
size_t SC16IS75XChannel::rx_fifo_to_buffer_() {
  // we look if some characters has been received in the fifo
  auto to_transfer = this->rx_in_fifo_();
  if (to_transfer) {
    uint8_t data[to_transfer];
    this->read_data_(data, to_transfer);
    auto free = this->receive_buffer_->free();
    if (to_transfer > free) {
      ESP_LOGW(TAG, "Ring buffer overrun --> bytes in fifo %d available in buffer %d", to_transfer, free);
      to_transfer = free;  // hopefully will do the rest next time
    }
    ESP_LOGV(TAG, "Transferred %d bytes from rx_fifo to buffer ring", to_transfer);
    for (size_t i = 0; i < to_transfer; i++)
      this->receive_buffer_->push(data[i]);
  }
  return to_transfer;
}
#endif

bool SC16IS75XChannel::read_array(uint8_t *buffer, size_t length) {
#ifdef USE_RING_BUFFER
  bool status = true;
  size_t received;

  if (length > this->buffer_size_) {
    ESP_LOGE(TAG, "read_array() invalid call: requested %d bytes max length %d ...", length, this->buffer_size_);
    return false;
  }

  // we wait until we have received the requested bytes
  uint32_t const start_time = millis();
  while (length > (received = this->available())) {
    this->rx_fifo_to_buffer_();  // try to transfer more
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "read_array() underrun: %d bytes requested %d received", length, this->rx_in_fifo_());
      return false;
    }
    yield();  // reschedule our thread to avoid blocking
  }

  for (size_t i = 0; i < received; i++) {
    this->receive_buffer_->pop(buffer[i]);
  }
  return status;

#else
  if (length > FIFO_SIZE) {
    ESP_LOGE(TAG, "read_array() invalid call: requested %d bytes max size %d ...", length, FIFO_SIZE);
    return false;
  }

  if (!peek_buffer_.empty) {  // test peek buffer
    *buffer++ = peek_buffer_.data;
    peek_buffer_.empty = true;
    if (length-- == 1)
      return true;
  }

  // we wait until we have received the requested bytes
  uint32_t const start_time = millis();
  while (length > this->rx_in_fifo_()) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "read_array() underrun: %d bytes requested %d received", length, this->rx_in_fifo_());
      return false;
    }
    yield();  // reschedule our thread to avoid blocking
  }
  this->read_data_(buffer, length);
  return true;
#endif
}

bool SC16IS75XChannel::peek_byte(uint8_t *buffer) {
#ifdef USE_RING_BUFFER
  return this->receive_buffer_->peek(*buffer);
#else
  if (peek_buffer_.empty && available() == 0)
    return false;
  if (peek_buffer_.empty) {
    peek_buffer_.empty = false;
    this->read_data_(&peek_buffer_.data, 1);
  }
  *buffer = peek_buffer_.data;
  return true;
#endif
}

int SC16IS75XChannel::available() {
#ifdef USE_RING_BUFFER
  auto available = this->receive_buffer_->count();
  // here if we do not have bytes in buffer we want to check if
  // there are bytes in the fifo,in which case we do not want to
  // delay reading them in the next loop.
  if (!available)
    available = this->rx_fifo_to_buffer_();
  ESP_LOGVV(TAG, "available(): %d bytes received", available);
  return available;
#else
  return (this->rx_in_fifo_() + (this->peek_buffer_.empty ? 0 : 1));
#endif
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XGPIOPin methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded GPIOPin
//

void SC16IS75XGPIOPin::setup() {
  ESP_LOGV(TAG, "Setting GPIO pin %d mode to %s", this->pin_,
           flags_ == gpio::FLAG_INPUT          ? "Input"
           : this->flags_ == gpio::FLAG_OUTPUT ? "Output"
                                               : "NOT SPECIFIED");
  // ESP_LOGCONFIG(TAG, "Setting GPIO pins direction/mode to '%s' %02X", i2s_(flags_), flags_);
  this->pin_mode(this->flags_);
}

std::string SC16IS75XGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via SC16IS75X:%s", this->pin_, this->parent_->get_name());
  return buffer;
}

///////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS BELOW
///////////////////////////////////////////////////////////////////////////////
#ifdef TEST_COMPONENT
class Increment {  // A increment "Functor" (A class object that acts like a method with state!)
 public:
  Increment() : i_(0) {}
  uint8_t operator()() { return i_++; }

 private:
  uint8_t i_;
};

void print_buffer(std::vector<uint8_t> buffer) {
  // quick and ugly hex converter to display buffer in hex format
  char hex_buffer[80];
  hex_buffer[50] = 0;
  for (size_t i = 0; i < buffer.size(); i++) {
    snprintf(&hex_buffer[3 * (i % 16)], sizeof(hex_buffer), "%02X ", buffer[i]);
    if (i % 16 == 15)
      ESP_LOGI(TAG, "   %s", hex_buffer);
  }
  if (buffer.size() % 16) {
    // null terminate if incomplete line
    hex_buffer[3 * (buffer.size() % 16) + 2] = 0;
    ESP_LOGI(TAG, "   %s", hex_buffer);
  }
}

/// @brief test the write_array method
void SC16IS75XChannel::uart_send_test_(char *message) {
  auto start_exec = micros();
  this->flush();  // we wait until they are gone
  std::vector<uint8_t> output_buffer(FIFO_SIZE);
  generate(output_buffer.begin(), output_buffer.end(), Increment());  // fill with incrementing number
  this->write_array(&output_buffer[0], FIFO_SIZE);                    // we send the buffer
  ESP_LOGV(TAG, "%s => sending %d bytes - exec time %d µs ...", message, FIFO_SIZE, micros() - start_exec);
}

/// @brief test read_array method
bool SC16IS75XChannel::uart_receive_test_(char *message) {
  auto start_exec = micros();
  bool status = true;
  size_t received;
  std::vector<uint8_t> buffer(FIFO_SIZE);

  // we wait until we have received all the bytes
  uint32_t const start_time = millis();
  while (FIFO_SIZE > (received = this->available())) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "uart_receive_test() timeout: only %d bytes received...", received);
      break;
    }
    yield();  // reschedule our thread to avoid blocking
  }

  uint8_t peek_value;
  this->peek_byte(&peek_value);
  if (peek_value != 0) {
    ESP_LOGE(TAG, "Peek first byte value error...");
    print_buffer(buffer);
    status = false;
  }

  status = status && this->read_array(&buffer[0], FIFO_SIZE);
  for (int i = 0; i < FIFO_SIZE; i++) {
    if (buffer[i] != i) {
      ESP_LOGE(TAG, "Read buffer contains error...");
      print_buffer(buffer);
      status = false;
      break;
    }
  }

  ESP_LOGV(TAG, "%s => %d bytes received status %s - exec time %d µs ...", message, FIFO_SIZE, status ? "OK" : "ERROR",
           micros() - start_exec);
  return status;
}

void SC16IS75XSPIComponent::test_gpio_input_() {
  static bool init_input{false};
  static uint8_t state, value;
  if (!init_input) {
    init_input = true;
    uint8_t io_dir = 0x00;
    // set all pins in input mode
    this->write_sc16is75x_register_(SC16IS75X_REG_IOD, 0, &io_dir);
    ESP_LOGI(TAG, "initializing all pins to input - configuration: %s", I2CS(io_dir));
    this->read_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &state);
    ESP_LOGI(TAG, "initial input register = %02X (%s)", state, I2CS(state));
  }
  this->read_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &value);
  if (value != state) {
    ESP_LOGI(TAG, "Input value changed from %02X to %02X (%s)", state, value, I2CS(value));
    state = value;
  }
}

void SC16IS75XSPIComponent::test_gpio_output_() {
  static bool init_output{false};
  static uint8_t state = 0x00;
  uint8_t value;
  if (!init_output) {
    init_output = true;
    uint8_t io_dir = 0xFF;
    // set all pins in output mode
    this->write_sc16is75x_register_(SC16IS75X_REG_IOD, 0, &io_dir);
    ESP_LOGI(TAG, "initializing all pins to output - configuration: %s", I2CS(io_dir));
    this->write_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &state);
    ESP_LOGI(TAG, "initial output register = %02X (%s)", state, I2CS(state));
  }

  value = ~state;
  this->write_sc16is75x_register_(SC16IS75X_REG_IOS, 0, &value);
  ESP_LOGI(TAG, "Flipping outputs from %02X to %02X (%s)", state, value, I2CS(value));
  state = value;
}

void SC16IS75XSPIComponent::loop() {
  if (!this->initialized_)
    return;

  static uint32_t loop_time = 0;
  static uint32_t loop_count = 0;
  uint32_t time = 0;

  if (test_mode_) {
    ESP_LOGV(TAG, "Component loop %d for %s : %d ms since last call ...", loop_count, this->get_name(),
             millis() - loop_time);
  }
  loop_time = millis();
  loop_count++;

  if (test_mode_ == 1) {
    char message[64];
    elapsed(time);  // set time to now
    for (auto *child : this->children_) {
      snprintf(message, sizeof(message), "%s:%s", this->get_name(), child->get_channel_name());
      child->uart_send_test_(message);
      ESP_LOGV(TAG, "uart_send_test - execution time %d ms...", elapsed(time));
      uint32_t const start_time = millis();
      while (child->rx_in_fifo_() < FIFO_SIZE) {  // wait until buffer empty
        if (millis() - start_time > 100) {
          ESP_LOGE(TAG, "Timed out waiting for bytes - %d bytes in buffer...", child->rx_in_fifo_());
          break;
        }
        yield();  // reschedule our thread to avoid blocking
      }
      bool status = child->uart_receive_test_(message);
      ESP_LOGI(TAG, "Loop %d send/received %d bytes %s", loop_count, FIFO_SIZE, status ? "correctly" : "with error");
      ESP_LOGV(TAG, "uart_receive_test - execution time %d ms...", elapsed(time));
    }
  }

  if (test_mode_ == 2) {
    elapsed(time);  // set time to now
    test_gpio_input_();
    ESP_LOGV(TAG, "test gpio input - execution time %d ms...", elapsed(time));
  }

  if (test_mode_ == 3) {
    elapsed(time);  // set time to now
    test_gpio_output_();
    ESP_LOGV(TAG, "test gpio output - execution time %d ms...", elapsed(time));
  }

  ESP_LOGV(TAG, "loop execution time %d ms...", millis() - loop_time);
}
#else
void SC16IS75XSPIComponent::loop() {}
#endif

}  // namespace sc16is75x_spi
}  // namespace esphome
