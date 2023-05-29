/// @file sc16s752.cpp
/// @author @DrCoolzic
/// @brief sc16is75x implementation

#include "sc16is75x.h"

namespace esphome {
namespace sc16is75x {

static const char *const TAG = "sc16is75x";

/*! @page page_sc1675x_ SC16IS75XComponent documentation
This page gives some information about the details of implementation of
the SC16IS75XComponent class for ESPHome.

@section sc16is75x_component_ SC16IS75XComponent class
This class describes a SC16IS75X I²C component. It derives from two @ref esphome classes:
- The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
  @ref Component::dump_config() and @ref Component::get_setup_priority() methods
- The @ref i2c::I2CDevice class. From which we use some methods
We have two related class :
- The @ref SC16IS75XChannel class that takes cares of the UART related functions
- The @ref SC16IS75XGPIOPin class
  that takes care of the details for the GPIO pins of the component.

All access to the i2c::I2CDevice class methods are done through two functions:
- The SC16IS75XComponent::read_sc16is75x_register_() and the
SC16IS75XComponent::write_sc16is75x_register_().
Maximum checks are performs at this level and other helper methods are available for io
and for uart.

  @section sc16is75x_uart_ SC16IS75XChannel (UART) class
Unfortunately I could not find any description of the uart::UARTDevice and
the uart::UARTComponent classes of @ref ESPHome, but it seems that they take their
roots in the Arduino library.\n
Most of the \b Serial methods provided in the Arduino library are poorly
defined and it seems that they have changed over time!\n
The esphome::uart::UARTDevice class directly relates to the **Serial Class**
in Arduino and they both derive from a **Stream class**.\n
For compatibility reason many simpler method are made available in ESPHome
but they very often loose the original status information ...\n

Therefore what I present here and what I have implemented represent
the best possible way I could think of for these classes/methods.

@subsection ra_ss_ bool read_array(uint8_t *buffer, size_t len);

This method receives 'len' characters from the uart into buffer:
- it returns true if requested number of characters have been read,
- it returns false if we have a timeout condition\n
Note that the SC16IS75X UART has a 64 bytes internal buffer that we first read
if character are available, then we wait for 100 ms for more characters.
In the Arduino library it is mentioned that user should not ask for more than
32 bytes with the read_byte() method. In a good usage of this method the caller
should test if characters are available before calling this method.

Typical code could be:
@code
  // ...
  auto len = uart.available();
  uint8_t buffer[64];
  if (len > 0) {
    auto status = uart.read(&buffer, len)
  }
  // test status ...
@endcode

@subsection pb_ss_ bool peek_byte(uint8_t *buffer);

This method returns the next byte (character) of incoming serial data without removing it from the internal serial
buffer.
- it returns true if a character have been read,
- it returns false if we have a timeout condition.\n
For more information refer to @ref ra_ss_.

@subsection wa_ss_ void write_array(uint8_t *buffer, size_t len);

This method sends 'len' characters from the uart into buffer.
Unfortunately compare to the Arduino equivalent this method
does not return any value and therefore you have no possibility
to know if the bytes has been transmitted correctly. To provide
a reasonable implementation the method does the following:
- it fills the output fifo with the provided bytes.
- it waits until either all the bytes have been sent or
  until a timeout of 100 ms has been reached.

Note:
- The internal buffer is 64 bytes long and therefore a request
  to send more characters can result in characters lost.
- Unfortunately neither Arduino neither ESPHome provide a method to
  find how much space is available in the transmit buffer...

Typical code to call this method could be:
@code
  // ...
  uint8_t buffer[64];
  size_t len;
  // ...
  len = ...
  while (len > 0) {
    send = max(len, 64);
    uart.read(&buffer, send);
    len -= send;
  }
  // ...
@endcode

@subsection fl_ss_ void SC16IS75XChannel::flush() {
This method is definitively the worse of all UART methods !!!
It is used apparently to flush the output buffer by waiting for all
characters to be sent? \n
If we refer to Serial.flush() in Arduino we have:
  - Waits for the transmission of outgoing serial data to complete.
  - Prior to Arduino 1.0, this instead removed any buffered incoming serial data.

While flushing an input fifo make sense, to me flushing an output fifo make very
little sense! Therefore currently I clear both the input and output fifo!


@section sc16is75x_gpio_ SC16IS75XGPIOPin (GPIO) class
  @TODO

*/

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XComponent methods
///////////////////////////////////////////////////////////////////////////////

i2c::ErrorCode SC16IS75XComponent::write_sc16is75x_register_(uint8_t reg_address, uint8_t channel,
                                                             const uint8_t *buffer, size_t len) {
  auto sub = subaddress_(reg_address, channel);
  auto error = this->write_register(sub, buffer, len);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "write_sc16is75x_register_(%s, %X [0x%02X], b=%02X, l=%d): I2C code %d",
              write_reg_to_str[reg_address], channel, sub, *buffer, len, (int) error);
  } else {
    this->status_set_warning();
    ESP_LOGE(TAG, "write_sc16is75x_register_(%s, %X [0x%02X], b=%02X, l=%d): I2C code %d",
             write_reg_to_str[reg_address], channel, sub, *buffer, len, (int) error);
  }
  return error;
}

i2c::ErrorCode SC16IS75XComponent::read_sc16is75x_register_(uint8_t reg_address, uint8_t channel, uint8_t *buffer,
                                                            size_t len) {
  auto sub = subaddress_(reg_address, channel);
  auto error = this->read_register(sub, buffer, len);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "read_sc16is75x_register_(%s, %X [0x%02X], b=%02X, l=%d): I2C code %d", read_reg_to_str[reg_address],
              channel, sub, *buffer, len, (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "read_sc16is75x_register_(%s, %X [0x%02X], b=%02X, l=%d): I2C code %d", read_reg_to_str[reg_address],
             channel, sub, *buffer, len, (int) error);
  }
  return error;
}

void SC16IS75XComponent::write_io_register_(int reg_address, uint8_t value) {
  write_sc16is75x_register_(reg_address, 0, &value, 1);
}

uint8_t SC16IS75XComponent::read_io_register_(int reg_address) {
  uint8_t buffer_ = 0;
  read_sc16is75x_register_(reg_address, 0, &buffer_, 1);
  return buffer_;
}

bool SC16IS75XComponent::read_pin_val_(uint8_t pin) {
  input_state_ = this->read_io_register_(SC16IS75X_REG_IOP);
  return this->input_state_ & (1 << pin);
}

void SC16IS75XComponent::write_pin_val_(uint8_t pin, bool value) {
  value ? this->output_state_ |= (1 << pin) : this->output_state_ &= ~(1 << pin);
  this->write_io_register_(SC16IS75X_REG_IOP, this->output_state_);
}

void SC16IS75XComponent::set_pin_mode_(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT)
    this->pin_config_ &= ~(1 << pin);  // clear bit (input mode)
  if (flags == gpio::FLAG_OUTPUT)
    this->pin_config_ |= 1 << pin;     // set bit (output mode)
  this->write_io_register_(SC16IS75X_REG_IOD, ~this->pin_config_);
}

//
// overloaded functions from Component
//
void SC16IS75XComponent::setup() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s (%x) with %d UARTs...", model_name, this, (int) children.size());
  // we read anything just to test communication
  if (read_sc16is75x_register_(0, 0, &buffer_, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
  }

  // we can now setup our children
  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "Setting up %s (%x) UART %d...", model_name, this, i);
    children[i]->fifo_enable_(true);
    children[i]->set_baudrate_();
    children[i]->set_line_param_();
  }
}

void SC16IS75XComponent::dump_config() {
  const char *model_name = (model_ == SC16IS750_MODEL) ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "%s (%x) with %d UARTs...", model_name, this, (int) children.size());
  ESP_LOGCONFIG(TAG, "  crystal %d", crystal_);
  // TODO pins info ?

  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with %s failed!", model_name);
  }

  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "  %s UART %d...", model_name, i);
    ESP_LOGCONFIG(TAG, "    baudrate %d Bd", children[i]->baud_rate_);
    ESP_LOGCONFIG(TAG, "    data_bits %d", children[i]->data_bits_);
    ESP_LOGCONFIG(TAG, "    stop_bits %d", children[i]->stop_bits_);
    ESP_LOGCONFIG(TAG, "    parity %s",
                  children[i]->parity_ == uart::UART_CONFIG_PARITY_ODD    ? "ODD"
                  : children[i]->parity_ == uart::UART_CONFIG_PARITY_EVEN ? "EVEN"
                                                                          : "NONE");
  }
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XChannel methods
///////////////////////////////////////////////////////////////////////////////

/// **IMPLEMENTATION DETAILS** - I am not aware of any formal definition of this
/// function written somewhere, but based on common sense and looking at Arduino
/// code it seems that the following is expected:
/// - the function **tries** to receive 'len' characters from the uart into buffer:
///   - it terminates with true if requested number of characters have been read,
///   - it terminates with false if we have a timeout condition
/// Note that the SC16IS75X UART has a 64 bytes internal buffer
bool SC16IS75XChannel::read_array(uint8_t *buffer, size_t len) {
  if (!peek_byte_.empty) {
    *buffer++ = peek_byte_.byte;
    peek_byte_.empty = true;
    if (--len == 0)
      return true;
  }

  if (!check_read_timeout_(len)) {  // check timeout if we read
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
  parent_->read_sc16is75x_register_(SC16IS75X_REG_RHR, channel_, buffer, len);
  return true;
}

/// @brief Please refer to @ref read_array() for more information on this method.
bool SC16IS75XChannel::peek_byte(uint8_t *buffer) {
  if (peek_byte_.empty) {
    peek_byte_.empty = false;
    uint32_t start_time = millis();
    while (rx_fifo_level_() == 0) {
      if (millis() - start_time > 100) {
        ESP_LOGE(TAG, "Peeking from UART timed out!");
        return false;
      }
      yield();  // not sure what it does?
    }
    peek_byte_.byte = read_uart_register_(SC16IS75X_REG_RHR);
  }
  *buffer = peek_byte_.byte;
  return true;
}

/// **IMPLEMENTATION DETAILS** - I am not aware of any formal definition of this
/// function written somewhere, but based on common sense and looking at Arduino
/// code it seems that the following is expected:
/// - the function **tries** to send 'len' characters to uart from buffer
///
/// Even though the read_byte function is poorly defined the write function is worse
/// for several reasons:
/// - There is no indication on available buffer size for writing
///   therefore you send without knowing if it will succeed...
/// - and even more importantly when you call this function you do not know
///   if the bytes have been sent successfully.
///
/// In other word you are **totally blind** when using this function!
/// Therefore here is what I do:
/// - if 'len' is greater than available space in fifo I just return
///   without doing anything
/// - otherwise I send all char into the TX fifo and I wait until all
///   the character has been sent or if timeout
void SC16IS75XChannel::write_array(const uint8_t *buffer, size_t len) {
  if (len > tx_fifo_level_())
    return;
  parent_->write_sc16is75x_register_(SC16IS75X_REG_RHR, channel_, buffer, len);
  uint32_t start_time = millis();
  while (tx_fifo_level_() != 0) {
    if (millis() - start_time > 100) {
      ESP_LOGW(TAG, "UART timed out in write!");
      return;
    }
    yield();  // not sure what it does?
  }
}

/// **IMPLEMENTATION DETAILS** - This function is the worse of all UART functions !!!
/// If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
/// of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed
/// any buffered incoming serial data.).
/// Therefore I do a mixture of both but not waiting: I clear the RX and TX fifos
void SC16IS75XChannel::flush() {
  ESP_LOGW(TAG, "This functions is not safe");
  fifo_enable_(true);
}

uint8_t SC16IS75XChannel::read_uart_register_(int reg_address) {
  parent_->buffer_ = 0;
  parent_->read_sc16is75x_register_(reg_address, channel_, &parent_->buffer_, 1);
  return parent_->buffer_;
}

void SC16IS75XChannel::write_uart_register_(int reg_address, uint8_t value) {
  parent_->write_sc16is75x_register_(reg_address, channel_, &value, 1);
}

/// @brief Enable/Disable FIFOs.
/// Enabling FIFOs also reset the two FIFOs
/// @param enable true -> enable, false -> disable
void SC16IS75XChannel::fifo_enable_(bool enable) {
  uint8_t fcr;
  fcr = enable ? 0x3 : 0x0;
  write_uart_register_(SC16IS75X_REG_FCR, fcr);
  ESP_LOGVV(TAG, "UART %d fifo %s", channel_, enable ? "enabled" : "disabled");
}

void SC16IS75XChannel::set_line_param_() {
  auto lcr = read_uart_register_(SC16IS75X_REG_LCR);
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
  // update
  write_uart_register_(SC16IS75X_REG_LCR, lcr);
  ESP_LOGVV(TAG, "UART %d line set to %d data_bits, %d stop_bits, and %s parity", channel_, data_bits_, stop_bits_,
            parity_ == uart::UART_CONFIG_PARITY_ODD    ? "ODD"
            : parity_ == uart::UART_CONFIG_PARITY_EVEN ? "EVEN"
                                                       : "NONE");
}

void SC16IS75XChannel::set_baudrate_() {
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 14745600/16 = 192,000bps
  uint8_t pre_scaler = 1;  // we never use it
  // (read_uart_register_(SC16IS75X_REG_MCR) & 0x80) == 0 ? pre_scaler = 1 : pre_scaler = 4;
  uint32_t upper_part = parent_->crystal_ / pre_scaler;
  uint32_t lower_part = baud_rate_ * 16;
  // uint32_t max_baudrate = upper_part / 16;

  if (lower_part > upper_part) {  // sanity check
    ESP_LOGE(TAG, "The requested baudrate (%d) is too high - fallback to 19200", baud_rate_);
    baud_rate_ = 19200;
    lower_part = baud_rate_ * 16;
  }

  // we compute and round up the divisor
  uint32_t divisor = ceil((double) upper_part / (double) lower_part);
  uint32_t actual_baudrate = (upper_part / divisor) / 16;
  ESP_LOGVV(TAG, "UART %d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", channel_, parent_->crystal_,
            divisor, upper_part, lower_part, baud_rate_, actual_baudrate);

  auto lcr = read_uart_register_(SC16IS75X_REG_LCR);
  lcr |= 0x80;  // set LCR[7] to enable special registers
  write_uart_register_(SC16IS75X_REG_LCR, (uint8_t) lcr);
  write_uart_register_(SC16IS75X_REG_DLL, (uint8_t) divisor);
  write_uart_register_(SC16IS75X_REG_DLH, (uint8_t) (divisor >> 8));
  lcr &= 0x7F;  // reset LCR[7] to disable special registers
  write_uart_register_(SC16IS75X_REG_LCR, lcr);

  if (actual_baudrate == baud_rate_)
    ESP_LOGVV(TAG, "UART %d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", channel_, parent_->crystal_,
              divisor, upper_part, lower_part, baud_rate_, actual_baudrate);
  else
    ESP_LOGW(TAG, "UART %d Crystal=%d div=%d(%d/%d) Requested=%d Bd => actual=%d Bd", channel_, parent_->crystal_,
             divisor, upper_part, lower_part, baud_rate_, actual_baudrate);
}

///////////////////////////////////////////////////////////////////////////////
// The SC16IS75XGPIOPin methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded GPIOPin
//

void SC16IS75XGPIOPin::setup() {
  // TODO message
  pin_mode(flags_);
}
void SC16IS75XGPIOPin::pin_mode(gpio::Flags flags) { this->parent_->set_pin_mode_(this->pin_, flags); }
bool SC16IS75XGPIOPin::digital_read() { return this->parent_->read_pin_val_(this->pin_) != this->inverted_; }
void SC16IS75XGPIOPin::digital_write(bool value) {
  this->parent_->write_pin_val_(this->pin_, value != this->inverted_);
}
std::string SC16IS75XGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via SC16IS75X", pin_);
  return buffer;
}

}  // namespace sc16is75x
}  // namespace esphome
