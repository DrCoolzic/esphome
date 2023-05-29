/// @file sc16s752.h
/// @author @DrCoolzic
/// @brief sc16is75x interface

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is75x {

// General sc16is75x registers
const uint8_t SC16IS75X_REG_RHR = 0x00;  // receive holding register (r) with a 64-bytes FIFO
const uint8_t SC16IS75X_REG_THR = 0X00;  // transmit holding register (w) with a 64-bytes FIFO
const uint8_t SC16IS75X_REG_IER = 0X01;  // interrupt enable register (r/w)
const uint8_t SC16IS75X_REG_IIR = 0X02;  // interrupt identification register (r)
const uint8_t SC16IS75X_REG_FCR = 0X02;  // FIFO control register (w)
const uint8_t SC16IS75X_REG_LCR = 0X03;  // line control register (r/w)
const uint8_t SC16IS75X_REG_MCR = 0X04;  // modem control register (r/w) - only when EFR[4]=1
const uint8_t SC16IS75X_REG_LSR = 0X05;  // line status register (ro)
const uint8_t SC16IS75X_REG_MSR = 0X06;  // modem status register (ro)
const uint8_t SC16IS75X_REG_TCR = 0X06;  // transmission control register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS75X_REG_SPR = 0X07;  // scratchpad register (r/w)
const uint8_t SC16IS75X_REG_TLR = 0X07;  // trigger level register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS75X_REG_TXF = 0X08;  // transmit FIFO level register (ro)
const uint8_t SC16IS75X_REG_RXF = 0X09;  // receive FIFO level register (ro)
const uint8_t SC16IS75X_REG_IOD = 0X0A;  // I/O pin direction register (r/w)
const uint8_t SC16IS75X_REG_IOP = 0X0B;  // I/O pin state register (r/w)
const uint8_t SC16IS75X_REG_IOI = 0X0C;  // I/O interrupt enable register (r/w)
const uint8_t SC16IS75X_REG_IOC = 0X0E;  // I/O pin control register (r/w)
const uint8_t SC16IS75X_REG_XFR = 0X0F;  // extra features register (r/w)
// Special registers only if LCR[7] == 1 & LCR != 0xBF
const uint8_t SC16IS75X_REG_DLL = 0x00;  // divisor latch lsb (r/w) only if LCR[7]=1 & LCR != 0xBF
const uint8_t SC16IS75X_REG_DLH = 0X01;  // divisor latch msb (r/w) only if LCR[7]=1 & LCR != 0xBF
// Enhanced registers only if LCR == 0xBF
const uint8_t SC16IS75X_REG_EFR = 0X02;  // enhanced features register only if LCR=0xBF (1011 1111)
const uint8_t SC16IS75X_REG_XO1 = 0X04;  // Xon1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS75X_REG_XO2 = 0X05;  // Xon2 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS75X_REG_XF1 = 0X06;  // Xoff1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS75X_REG_XF2 = 0X07;  // Xoff2 word (rw) only if LCR=0xBF (1011 1111)

// for debug messages ...
static const char *write_reg_to_str[] = {"THR",   "IER",   "FCR", "LCR", "MCR", "LSR", "TCR", "SPR",
                                         "_INV_", "_INV_", "IOD", "IO",  "IOI", "IOC", "EFR"};
static const char *read_reg_to_str[] = {"RHR", "IER",  "IIR", "LCR", "MCR", "LSR", "MSR", "SPR",
                                        "TXF", "RXF", "IOD", "IOP", "IOI", "IOC", "EFR"};

/// @brief Parity options
enum UARTParityOptions {
  UART_CONFIG_PARITY_NONE,
  UART_CONFIG_PARITY_EVEN,
  UART_CONFIG_PARITY_ODD,
};

/// @brief supported chip models
enum SC16IS75XComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };
class SC16IS75XChannel;  // forward declaration

///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a SC16IS75X I²C component.
///
/// This class derives from two @ref esphome classes:
/// - The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
///   @ref Component::dump_config() and @ref Component::get_setup_priority() methods
/// - The @ref i2c::I2CDevice class. From which we use some methods
///
/// We have two related class :
/// - The @ref SC16IS75XChannel class that takes cares of the UART related functions
/// - The @ref SC16IS75XGPIOPin class
///   that takes care of the details for the GPIO pins of the component.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XComponent : public Component, public i2c::I2CDevice {
 public:
  void set_model(SC16IS75XComponentModel model) { model_ = model; }
  void set_crystal(uint32_t crystal) { crystal_ = crystal; }

  //
  //  override Component functions
  //

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }

 protected:
  // we give access to protected objects to our friends :)
  friend class SC16IS75XChannel;
  friend class SC16IS75XGPIOPin;

  /// @brief All write calls to I2C registers are funneled through this function
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to write
  /// @return the i2c error codes
  i2c::ErrorCode write_sc16is75x_register_(uint8_t reg_address, uint8_t channel, const uint8_t *buffer, size_t len);

  /// @brief All read calls to I2C registers are funneled through this function
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to read
  /// @return the i2c error codes
  i2c::ErrorCode read_sc16is75x_register_(uint8_t reg_address, uint8_t channel, uint8_t *buffer, size_t len);

  /// @brief Use to read GPIO related register. Channel 0 is used as it is not significant
  /// @param reg_address the register address
  /// @return the byte read from the register
  uint8_t read_io_register_(int reg_address);

  /// @brief Use to write GPIO related register. Channel 0 is used as it is not significant
  /// @param reg_address the register address
  /// @param value the value to write
  void write_io_register_(int reg_address, uint8_t value);

  /// Helper function to read the value of a pin.
  bool read_pin_val_(uint8_t pin);
  /// Helper function to write the value of a pin.
  void write_pin_val_(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void set_pin_mode_(uint8_t pin, gpio::Flags flags);
  // the register address is composed of the register value and the channel value.
  // note that for I/O related register the chanel value is not significant
  inline uint8_t subaddress_(uint8_t reg_addr, uint8_t channel = 0) { return (reg_addr << 3 | channel << 1); }

  /// Mask for the pin config - 1 means OUTPUT, 0 means INPUT
  uint8_t pin_config_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint8_t output_state_{0x00};
  /// The state of the actual input pin states - 1 means HIGH, 0 means LOW
  uint8_t input_state_{0x00};
  /// @brief The precise model of the component
  SC16IS75XComponentModel model_{SC16IS752_MODEL};
  /// crystal default on SC16IS750 => 14.7456MHz - on SC16IS752 => 3.072MHz
  uint32_t crystal_{3072000};
  /// one byte buffer used for most register operation to avoid allocation
  uint8_t buffer_{0};
  /// @brief the list of UART children
  std::vector<SC16IS75XChannel *> children{};
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes the UART part of a SC16IS75X I²C component.
///
/// This class derives from the @ref esphome @ref uart::UARTComponent class.
/// As the @ref uart::UARTComponent is a pure virtual class we need to
/// implement the following functions : @ref uart::UARTComponent::write_array(),
/// @ref uart::UARTComponent::read_array(), @ref uart::UARTComponent::peek_byte(),
/// @ref uart::UARTComponent::available(), @ref uart::UARTComponent::flush().
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XChannel : public uart::UARTComponent {
 public:
  void set_parent(SC16IS75XComponent *parent) {
    parent_ = parent;
    parent_->children.push_back(this);
  }
  void set_channel(uint8_t channel) { channel_ = channel; }

  // void set_baud_rate(uint32_t baudrate) { baudrate_ = baudrate; }
  // uint32_t get_baud_rate() const  { return baud_rate_; }
  // void set_stop_bits(uint8_t stop_bits) { stop_bits_ = stop_bits; }
  // uint8_t get_stop_bits() const { return this->stop_bits_; }
  // void set_data_bits(uint8_t data_bits) { data_bits_ = data_bits; }
  // uint8_t get_data_bits() const { return this->data_bits_; }
  // void set_parity(UARTParityOptions parity) { parity_ = parity; }
  // UARTParityOptions get_parity() const { return this->parity_; }

  //
  // overriden UARTComponent functions
  //

  /// @brief Write a specified number of bytes from a buffer to a serial port
  /// for more detail refer to implementation refer to @ref page_sc1675x_
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to write
  void write_array(const uint8_t *buffer, size_t len) override;

  /// @brief Read a specified number of bytes from a serial port to an buffer
  /// for more detail refer to implementation refer to @ref page_sc1675x_
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to read
  /// @return true if succeed false otherwise
  bool read_array(uint8_t *buffer, size_t len) override;

  /// @brief Read next byte available from serial buffer without removing it
  /// for more detail refer to implementation refer to @ref page_sc1675x_
  /// @param buffer pointer to the byte
  /// @return true if succeed false otherwise
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Return the number of bytes available for reading from the serial port.
  /// for more detail refer to implementation refer to @ref page_sc1675x_
  /// @return the number of bytes available in the fifo
  int available() override { return rx_fifo_level_(); }

  /// @brief Flush the input and output fifo
  /// for more detail refer to implementation refer to @ref page_sc1675x_
  void flush() override;

 protected:
  friend class SC16IS75XComponent;
  /// @brief cannot happen with our component!
  void check_logger_conflict() override {}

  uint8_t read_uart_register_(int reg_address);
  void write_uart_register_(int reg_address, uint8_t value);
  void set_line_param_();
  void set_baudrate_();
  void fifo_enable_(bool enable = true);
  inline int rx_fifo_level_() { return read_uart_register_(SC16IS75X_REG_RXF); }
  inline int tx_fifo_level_() { return read_uart_register_(SC16IS75X_REG_TXF); }

  SC16IS75XComponent *parent_;
  uint8_t channel_;
  // uint32_t baudrate_;
  // uint8_t stop_bits_;
  // uint8_t data_bits_;
  // UARTParityOptions parity_;
  struct {
    uint8_t byte;
    bool empty{true};
  } peek_byte_;
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Helper class to expose a SC16IS75X pin as an internal input GPIO pin.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XGPIOPin : public GPIOPin {
 public:

  //
  // overriden GPIOPin methods
  //
  
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(SC16IS75XComponent *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  SC16IS75XComponent *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace sc16is75x
}  // namespace esphome
