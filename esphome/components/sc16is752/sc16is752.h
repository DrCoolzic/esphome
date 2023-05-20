#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is752 {

// General registers
const uint8_t SC16IS752_REG_RHR = 0x00;    // receive holding register (r) with 64-bytes FIFO (if enabled)
const uint8_t SC16IS752_REG_THR = 0X00;    // transmit holding register (w) with 64-bytes FIFO if (enabled)
const uint8_t SC16IS752_REG_IER = 0X01;    // interrupt enable register (r/w)
const uint8_t SC16IS752_REG_IIR = 0X02;    // interrupt identification register (r)
const uint8_t SC16IS752_REG_FCR = 0X02;    // FIFO control register (w)
const uint8_t SC16IS752_REG_LCR = 0X03;    // line control register (r/w)
const uint8_t SC16IS752_REG_MCR = 0X04;    // modem control register (r/w) - only when EFR[4]=1
const uint8_t SC16IS752_REG_LSR = 0X05;    // line status register (ro)
const uint8_t SC16IS752_REG_MSR = 0X06;    // modem status register (ro)
const uint8_t SC16IS752_REG_TCR = 0X06;    // transmission control register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS752_REG_SPR = 0X07;    // scratchpad register (r/w)
const uint8_t SC16IS752_REG_TLR = 0X07;    // trigger level register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS752_REG_TXLVL = 0X08;  // transmit FIFO level register (ro)
const uint8_t SC16IS752_REG_RXLVL = 0X09;  // receive FIFO level register (ro)
const uint8_t SC16IS752_REG_IODIR = 0X0A;  // I/O pin direction register (r/w)
const uint8_t SC16IS752_REG_IOPIN = 0X0B;  // I/O pin state register (r/w)
const uint8_t SC16IS752_REG_IOINT = 0X0C;  // I/O interrupt enable register (r/w)
const uint8_t SC16IS752_REG_IOCTR = 0X0E;  // I/O pin control register (r/w)
const uint8_t SC16IS752_REG_EFCR = 0X0F;   // extra features register (r/w)
// Special registers only if LCR[7] == 1 & LCR != 0xBF
const uint8_t SC16IS752_REG_DLL = 0x00;  // divisor latch lsb (r/w) only if LCR[7]=1 & LCR != 0xBF
const uint8_t SC16IS752_REG_DLH = 0X01;  // divisor latch msb (r/w) only if LCR[7]=1 & LCR != 0xBF
// Enhanced registers only if LCR == 0xBF
const uint8_t SC16IS752_REG_EFR = 0X02;    // enhanced features register only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON1 = 0X04;   // Xon1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON2 = 0X05;   // Xon2 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF1 = 0X06;  // Xoff1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF2 = 0X07;  // Xoff2 word (rw) only if LCR=0xBF (1011 1111)
// Parity options
enum UARTParityOptions {
  UART_CONFIG_PARITY_NONE,
  UART_CONFIG_PARITY_EVEN,
  UART_CONFIG_PARITY_ODD,
};
// supported chip model
enum SC16IS752ComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };



/// @brief Describes a SC16IS752 I²C component.
/// We have two related class :tThe SC16IS752Channel class that
/// take cares of the UART related functions and the SC16IS752GPIOPin
/// that take care of the details for the GPIO pins of the component.
class SC16IS752Component : public Component, public i2c::I2CDevice {
 public:
  void set_model(SC16IS752ComponentModel model) { model_ = model; }
  void set_crystal(uint32_t crystal) { crystal_ = crystal; }

  //  overloaded Component virtual functions
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void update();

 protected:
  // we give access to protected object to our friends
  friend class SC16IS752Channel;
  friend class SC16IS752GPIOPin;

  void write_sc16is752_register_(uint8_t reg_address, uint8_t channel, const uint8_t *data, size_t len);
  void read_sc16is752_register_(uint8_t reg_address, uint8_t channel, uint8_t *data, size_t len);
  int read_io_register_(int reg_address);
  void write_io_register_(int reg_address, uint8_t value);
  bool check_model_();
  /// Helper function to read the value of a pin.
  bool read_pin_val_(uint8_t pin);
  /// Helper function to write the value of a pin.
  void write_pin_val_(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void set_pin_mode_(uint8_t pin, gpio::Flags flags);
  // the register address is composed of the register value and the channel value.
  // note that for I/O the chanel value is not significant
  inline int subaddress_(int reg_addr, int channel = 0) { return (reg_addr << 3 | channel << 1); }

  /// Mask for the pin config - 1 means OUTPUT, 0 means INPUT
  uint8_t pin_config_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint8_t output_state_{0x00};
  /// The state of the actual input pin states - 1 means HIGH, 0 means LOW
  uint8_t input_state_{0x00};
  /// @brief The precise model of the component
  SC16IS752ComponentModel model_;
  /// crystal default on SC16IS750 => 14.7456MHz -  on SC16IS752 => 3.072MHz
  uint32_t crystal_;
  /// buffer used for register operatio
  uint8_t reg_buffer;
};



/// @brief Describes the UART part of a SC16IS752 I²C component.
class SC16IS752Channel : public uart::UARTComponent {
 public:
  SC16IS752Channel() { fifo_enable_(); }  // ctor
  void set_parent(SC16IS752Component *parent) { parent_ = parent; }
  void set_channel(uint8_t channel) { channel_ = channel; }
  void set_baudrate(uint32_t baudrate) { baudrate_ = baudrate; }
  void set_stop_bits(int stop_bits) { stop_bits_ = stop_bits; }
  void set_data_bits(int data_bits) { data_bits_ = data_bits; }
  void set_parity(UARTParityOptions parity) { parity_ = parity; }

  //
  // overriden UARTComponent functions
  //

  /// @brief Write a specified number of bytes from a buffer to a serial port
  /// @param data pointer to the buffer
  /// @param len number of bytes to write
  void write_array(const uint8_t *data, size_t len) override;

  /// @brief Read a specified number of bytes from a serial port to an buffer
  /// @param data pointer to the buffer
  /// @param len number of bytes to read
  /// @return seems to always return true
  bool read_array(uint8_t *data, size_t len) override;

  /// @brief Read next byte available from serial buffer without removing it
  /// @param data pointer to the byte
  /// @return seems to always return true;
  bool peek_byte(uint8_t *data) override;

  /// @brief Get the number of bytes available for reading from the serial port.
  /// @return the number of bytes available
  int available() override { return rx_fifo_level_(); }

  /// @brief Clears the buffer ??? output **once all outgoing characters have been sent**. ???
  void flush() override;

 protected:
  /// @brief cannot happen with our component!
  void check_logger_conflict() override {}

  uint8_t read_uart_register_(int reg_address);
  void write_uart_register_(int reg_address, uint8_t value);
  void set_line_param_();
  void set_baudrate_();
  void fifo_enable_(bool enable = true);
  inline int rx_fifo_level_() { return read_uart_register_(SC16IS752_REG_RXLVL); }
  inline int tx_fifo_level_() { return read_uart_register_(SC16IS752_REG_TXLVL); }

  SC16IS752Component *parent_;
  uint8_t channel_;
  uint32_t baudrate_;
  uint8_t stop_bits_;
  uint8_t data_bits_;
  UARTParityOptions parity_;
  struct {
    uint8_t byte;
    bool empty{true};
  } peek_;
};



/// Helper class to expose a SC16IS752 pin as an internal input GPIO pin.
class SC16IS752GPIOPin : public GPIOPin {
 public:
  //
  // overriden GPIOPin methods
  //
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(SC16IS752Component *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  SC16IS752Component *parent_;
  uint8_t pin_;
  bool inverted_;  // not used
  gpio::Flags flags_;
};

}  // namespace sc16is752
}  // namespace esphome
