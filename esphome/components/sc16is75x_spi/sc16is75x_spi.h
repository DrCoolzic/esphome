/// @file sc16is75x_spi.h
/// @author @DrCoolzic
/// @brief sc16is75x_spi interface declaration

#pragma once
#include <bitset>
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is75x_spi {

#define TEST_COMPONENT
#define USE_RING_BUFFER

#ifdef USE_RING_BUFFER
///////////////////////////////////////////////////////////////////////////////
/// @brief This is an helper class that provides a simple ring buffers
/// that implements a FIFO function
///
/// This ring buffer is used to buffer the exchanges between the receiver
/// HW fifo and the client. Usually to read characters you first
/// check if bytes were received and if so you read them.
/// There is a problem if you try to the received bytes one by one.
/// If the registers are located on the chip everything is fine but with a
/// device like the wk2132 these registers are remote and therefore accessing
/// them requires several transactions on the I²C bus which is relatively slow.
/// One solution would be for the client to check the number of bytes available
/// and to read them all using the read_array() method. Unfortunately
/// most client I have reviewed are reading one character at a time in a
/// while loop which is the most inefficient way of doing things.
/// Therefore the solution I have chosen to implement is to
/// store received bytes locally in a buffer as soon as they arrive. With
/// this solution the bytes are stored locally and therefore accessible
/// very quickly when requested one by one.
/// @image html read_cycles.png
///////////////////////////////////////////////////////////////////////////////
class RingBuffer {
 public:
  /// @brief Ctor : initialize variables with the given size
  /// @param size size of the desired RingBuffer
  ///
  /// @note presence of this ctor invalidate usage of default ctor
  RingBuffer(const size_t size) : size_(size) { rb_.resize(size); }

  /// @brief pushes an item at the tail of the fifo
  /// @param item item to push
  /// @return true if item has been pushed, false il item was not pushed (buffer full)
  bool push(const uint8_t item) {
    if (is_full())
      return false;
    rb_[head_] = item;
    head_ = (head_ + 1) % rb_.size();
    count_++;
    return true;
  }

  /// @brief return and remove the item at head of the fifo
  /// @param item item read
  /// @return true if item has been retrieved, false il no item was found (buffer empty)
  bool pop(uint8_t &item) {
    if (is_empty())
      return false;
    item = rb_[tail_];
    tail_ = (tail_ + 1) % rb_.size();
    count_--;
    return true;
  }

  /// @brief return the value of the item at fifo's head without removing it
  /// @param item pointer to item to return
  /// @return true if item has been retrieved, false il no item was found (buffer empty)
  bool peek(uint8_t &item) {
    if (is_empty())
      return false;
    item = rb_[tail_];
    return true;
  }

  /// @brief test is the Ring Buffer is empty ?
  /// @return true if empty
  bool is_empty() { return (count_ == 0); }

  /// @brief test is the ring buffer is full ?
  /// @return true if full
  bool is_full() { return (count_ == rb_.size()); }

  /// @brief return the number of item in the ring buffer
  /// @return the count
  size_t count() { return count_; }

  /// @brief returns the number of free positions in the buffer
  /// @return how many items can be added
  size_t free() { return rb_.size() - count_; }

  /// @brief clear the buffer content
  void clear() { head_ = tail_ = count_ = 0; }

 private:
  std::vector<uint8_t> rb_;  // The ring buffer
  const size_t size_;        // size of the ring buffer
  int head_{0};              // points to the next element to write
  int tail_{0};              // points to the next element to read
  size_t count_{0};          // count number of element currently in the buffer
};

#endif

// size of the sc16is75x buffer fifo
constexpr size_t FIFO_SIZE = 64;

// General sc16is75x registers
constexpr uint8_t SC16IS75X_REG_RHR = 0x00;  ///< 00 receive holding register (r) with a 64-bytes FIFO
constexpr uint8_t SC16IS75X_REG_THR = 0X00;  ///< 00 transmit holding register (w) with a 64-bytes FIFO
constexpr uint8_t SC16IS75X_REG_IER = 0X01;  ///< 08 interrupt enable register (r/w)
constexpr uint8_t SC16IS75X_REG_IIR = 0X02;  ///< 10 interrupt identification register (r)
constexpr uint8_t SC16IS75X_REG_FCR = 0X02;  ///< 10 FIFO control register (w)
constexpr uint8_t SC16IS75X_REG_LCR = 0X03;  ///< 18 line control register (r/w)
constexpr uint8_t SC16IS75X_REG_MCR = 0X04;  ///< 20 modem control register (r/w) - only when EFR[4]=1
constexpr uint8_t SC16IS75X_REG_LSR = 0X05;  ///< 28 line status register (ro)
constexpr uint8_t SC16IS75X_REG_MSR = 0X06;  ///< 30 modem status register (ro)
constexpr uint8_t SC16IS75X_REG_TCR = 0X06;  ///< 30 transmission control register (r/w) when EFR[4]=1 & MRC[2]=1
constexpr uint8_t SC16IS75X_REG_SPR = 0X07;  ///< 38 scratchpad register (r/w)
constexpr uint8_t SC16IS75X_REG_TLR = 0X07;  ///< 38 trigger level register (r/w) when EFR[4]=1 & MRC[2]=1
constexpr uint8_t SC16IS75X_REG_TXF = 0X08;  ///< 40 transmit FIFO level register (ro)
constexpr uint8_t SC16IS75X_REG_RXF = 0X09;  ///< 48 receive FIFO level register (ro)
constexpr uint8_t SC16IS75X_REG_IOD = 0X0A;  ///< 50 I/O pin direction register (r/w)
constexpr uint8_t SC16IS75X_REG_IOS = 0X0B;  ///< 58 I/O pin state register (r/w)
constexpr uint8_t SC16IS75X_REG_IOI = 0X0C;  ///< 60 I/O interrupt enable register (r/w)
constexpr uint8_t SC16IS75X_REG_IOC = 0X0E;  ///< 70 I/O pin control register (r/w)
constexpr uint8_t SC16IS75X_REG_XFR = 0X0F;  ///< 78 extra features register (r/w)

// Special registers only if LCR[7] == 1 & LCR != 0xBF
constexpr uint8_t SC16IS75X_REG_DLL = 0x00;  ///< divisor latch lsb (r/w) only if LCR[7]=1 & LCR != 0xBF
constexpr uint8_t SC16IS75X_REG_DLH = 0X01;  ///< divisor latch msb (r/w) only if LCR[7]=1 & LCR != 0xBF

// Enhanced registers only if LCR == 0xBF
constexpr uint8_t SC16IS75X_REG_EFR = 0X02;  ///< enhanced features register only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XO1 = 0X04;  ///< Xon1 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XO2 = 0X05;  ///< Xon2 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XF1 = 0X06;  ///< Xoff1 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XF2 = 0X07;  ///< Xoff2 word (rw) only if LCR=0xBF (1011 1111)

enum SC16IS75XComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };  ///< chip models

class SC16IS75XChannel;   ///< forward declaration
using Channel = uint8_t;  ///< Channel definition

///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a SC16IS75XSPIComponent.
///
/// This class derives from two @ref esphome classes:
/// - The Virtual @ref Component class.
/// - The @ref spi::SPIDevice class.
///
/// We have two related class :
/// - The @ref SC16IS75XChannel class that takes cares of the UART related methods
/// - The @ref SC16IS75XGPIOPin class
///   that takes care of the details for the GPIO pins of the component.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XSPIComponent : public Component,
                              public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                    spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void set_model(SC16IS75XComponentModel model) { this->model_ = model; }
  void set_crystal(uint32_t crystal) { this->crystal_ = crystal; }
  void set_test_mode(int test_mode) { this->test_mode_ = test_mode; }
  void set_name(std::string name) { this->name_ = std::move(name); }
  const char *get_name() { return this->name_.c_str(); }

  //
  //  override Component methods
  //

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS - 0.5f; }

 protected:
  // we give access to protected objects to our friends :)
  friend class SC16IS75XChannel;
  friend class SC16IS75XGPIOPin;

  /// @brief All write calls to the component registers are done through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer to write
  /// @param length length of the buffer to write
  void write_sc16is75x_register_(uint8_t reg_address, Channel channel, const uint8_t *data, size_t length = 1);

  /// @brief All read calls to the component registers are done through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer to the buffer
  /// @param length number of bytes to read
  void read_sc16is75x_register_(uint8_t reg_address, Channel channel, uint8_t *data, size_t length = 1);

  /// Helper method to read the value of a pin.
  bool read_pin_val_(uint8_t pin);

  /// Helper method to write the value of a pin.
  void write_pin_val_(uint8_t pin, bool value);

  /// Helper method to set the pin mode of a pin.
  void set_pin_direction_(uint8_t pin, gpio::Flags flags);

#ifdef TEST_COMPONENT
  /// @brief for testing the GPIO pins
  void test_gpio_input_();
  void test_gpio_output_();
#endif

  /// pin config mask: 1 means OUTPUT, 0 means INPUT
  uint8_t pin_config_{0x00};
  /// output state: 1 means HIGH, 0 means LOW
  uint8_t output_state_{0x00};
  /// input pin states: 1 means HIGH, 0 means LOW
  uint8_t input_state_{0x00};

  SC16IS75XComponentModel model_;               ///< component's model
  uint32_t crystal_;                            ///< crystal frequency
  uint8_t data_{0};                             ///< one byte buffer
  std::vector<SC16IS75XChannel *> children_{};  ///< the list of SC16IS75XChannel UART children
  int test_mode_;                               ///< test_mode value (0 no test)
  std::string name_;                            ///< store name of entity
  uint8_t special_reg_{0};                      ///< 1 when accessing special register

  HighFrequencyLoopRequester high_freq_;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes the SC16IS75XChannel class
///
/// This class derives from the virtual @ref uart::UARTComponent class.
///
/// Unfortunately I have not found **any documentation** about the
/// uart::UARTDevice and uart::UARTComponent classes of @ref ESPHome.\n
/// However it seems that both of them are based on Arduino library.\n
///
/// Most of the interfaces provided by the Arduino Serial library are **poorly
/// defined** and it seems that the API has even \b changed over time!\n
/// The esphome::uart::UARTDevice class directly relates to the **Serial Class**
/// in Arduino and that derives from **Stream class**.\n
/// For compatibility reason (?) many helper methods are made available in
/// ESPHome to read and write. Unfortunately in many cases these helpers
/// are missing the critical status information and therefore are even
/// more unsafe to use...\n
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XChannel : public uart::UARTComponent {
 public:
  void set_parent(SC16IS75XSPIComponent *parent) {
    this->parent_ = parent;
    this->parent_->children_.push_back(this);  // add ourself to the vector list
  }
  void set_channel(Channel channel) { this->channel_ = channel; }
  void set_channel_name(std::string name) { this->name_ = std::move(name); }
  void set_buffer_size(int buffer_size) { this->buffer_size_ = buffer_size; }
  const char *get_channel_name() { return this->name_.c_str(); }

  //
  //  override UARTComponent methods
  //

  /// @brief Writes a specified number of bytes toward a serial port
  /// @param buffer pointer to the buffer
  /// @param length number of bytes to write
  ///
  /// This method sends 'length' characters from the buffer to the serial line.
  /// Unfortunately (unlike the Arduino equivalent) this method
  /// does not return any flag and therefore it is not possible to know
  /// if any/all bytes have been transmitted correctly. Another problem
  /// is that it is not possible to know ahead of time how many bytes we
  /// can safely send as there is no tx_available() method provided!
  /// To avoid overrun when using the write method you can use the flush()
  /// method to wait until the transmit fifo is empty.
  ///
  /// Typical usage could be:
  /// @code
  ///   // ...
  ///   uint8_t buffer[128];
  ///   // ...
  ///   write_array(&buffer, length);
  ///   flush();
  ///   // ...
  /// @endcode
  void write_array(const uint8_t *buffer, size_t length) override;

  /// @brief Reads a specified number of bytes from a serial port
  /// @param buffer buffer to store the bytes
  /// @param length number of bytes to read
  /// @return true if succeed, false otherwise
  ///
  /// Typical usage:
  /// @code
  ///   // ...
  ///   auto length = available();
  ///   uint8_t buffer[128];
  ///   if (length > 0) {
  ///     auto status = read_array(&buffer, length)
  ///     // test status ...
  ///   }
  /// @endcode
  bool read_array(uint8_t *buffer, size_t length) override;

  /// @brief Reads first byte in FIFO without removing it
  /// @param buffer pointer to the byte
  /// @return true if succeed reading one byte, false if no character available
  ///
  /// This method returns the next byte from receiving buffer without
  /// removing it from the internal fifo. It returns true if a character
  /// is available and has been read, false otherwise.\n
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Returns the number of bytes in the receive buffer
  /// @return the number of bytes available in the receiver fifo
  int available() override;

  /// @brief Flush the output fifo.
  ///
  /// If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
  /// of outgoing serial data to complete. (Prior to Arduino 1.0, this the method was
  /// removing any buffered incoming serial data.). **
  void flush() override;

 protected:
  friend class SC16IS75XSPIComponent;

  /// @brief this cannot happen with external uart
  void check_logger_conflict() override {}

  /// @brief returns the number of bytes currently in the receiver fifo
  /// @return the number of bytes
  size_t rx_in_fifo_() { return this->read_register_(SC16IS75X_REG_RXF); }

  /// @brief returns the number of bytes currently in the transmitter fifo
  /// @return the number of bytes
  size_t tx_in_fifo_() { return FIFO_SIZE - this->read_register_(SC16IS75X_REG_TXF); }

  /// @brief returns true if the transmit buffer is empty
  /// @return returns true if the transmit buffer is empty
  bool tx_fifo_is_not_empty_() { return !(this->read_register_(SC16IS75X_REG_LSR) & 0x40); }

  /// @brief Write data into the transmitter fifo
  /// @param buffer the input buffer
  /// @param length the number of bytes we want to transmit
  void write_data_(const uint8_t *buffer, size_t length) {
    this->parent_->write_sc16is75x_register_(SC16IS75X_REG_THR, this->channel_, buffer, length);
  }

  /// @brief Read data from the receiver fifo
  /// @param buffer the output buffer
  /// @param length the number of bytes we want to read
  void read_data_(uint8_t *buffer, size_t length) {
    this->parent_->read_sc16is75x_register_(SC16IS75X_REG_RHR, this->channel_, buffer, length);
  }

  inline uint8_t read_register_(int reg_address) {
    this->parent_->read_sc16is75x_register_(reg_address, this->channel_, &this->data_);
    return this->data_;
  }

#ifdef TEST_COMPONENT
  void uart_send_test_(char *message);
  bool uart_receive_test_(char *message);
#endif

  void set_line_param_();
  void set_baudrate_();
  void setup_channel_();
  void dump_channel_();

  SC16IS75XSPIComponent *parent_;  ///< our parent
  Channel channel_;                ///< our channel number
  uint8_t data_{0};                ///< one byte buffer
  std::string name_;               ///< name of the entity
  int buffer_size_{256};           // requested ring buffer size
#ifdef USE_RING_BUFFER
  RingBuffer *receive_buffer_;  // the receive buffer
  size_t rx_fifo_to_buffer_();  // method to transfer rx_fifo to buffer
#else                           // no ring buffer
  struct {
    uint8_t data;      ///< store the read data
    bool empty{true};  ///< peek buffer is empty ?
  } peek_buffer_;
#endif
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Helper class to expose a SC16IS75X pin as an internal input GPIO pin.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XGPIOPin : public GPIOPin {
 public:
  void set_parent(SC16IS75XSPIComponent *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }

  //
  // overriden GPIOPin methods
  //

  void setup() override;
  std::string dump_summary() const override;

  void pin_mode(gpio::Flags flags) override { this->parent_->set_pin_direction_(this->pin_, flags); }
  bool digital_read() override { return this->parent_->read_pin_val_(this->pin_) != this->inverted_; }
  void digital_write(bool value) override { this->parent_->write_pin_val_(this->pin_, value != this->inverted_); }

 protected:
  SC16IS75XSPIComponent *parent_{nullptr};
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace sc16is75x_spi
}  // namespace esphome
