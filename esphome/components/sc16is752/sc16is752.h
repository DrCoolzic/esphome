#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is752 {

// General registers
const uint8_t SC16IS752_REG_RHR = 0x00;        // receive holding register (r) with 64-bytes FIFO (if enabled)
const uint8_t SC16IS752_REG_THR = 0X00;        // transmit holding register (w) with 64-bytes FIFO if (enabled)
const uint8_t SC16IS752_REG_IER = 0X01;        // interrupt enable register (r/w)
const uint8_t SC16IS752_REG_IIR = 0X02;        // interrupt identification register (r)
const uint8_t SC16IS752_REG_FCR = 0X02;        // FIFO control register (w)
const uint8_t SC16IS752_REG_LCR = 0X03;        // line control register (r/w)
const uint8_t SC16IS752_REG_MCR = 0X04;        // modem control register (r/w) - only when EFR[4]=1
const uint8_t SC16IS752_REG_LSR = 0X05;        // line status register (ro)
const uint8_t SC16IS752_REG_MSR = 0X06;        // modem status register (ro)
const uint8_t SC16IS752_REG_TCR = 0X06;        // transmission control register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS752_REG_SPR = 0X07;        // scratchpad register (r/w)
const uint8_t SC16IS752_REG_TLR = 0X07;        // trigger level register (r/w) when EFR[4]=1 & MRC[2]=1
const uint8_t SC16IS752_REG_TXLVL = 0X08;      // transmit FIFO level register (ro)
const uint8_t SC16IS752_REG_RXLVL = 0X09;      // receive FIFO level register (ro)
const uint8_t SC16IS752_REG_IODIR = 0X0A;      // I/O pin direction register (r/w)
const uint8_t SC16IS752_REG_IOSTATE = 0X0B;    // I/O state register (r/w)
const uint8_t SC16IS752_REG_IOINTENA = 0X0C;   // I/O interrupt enable register (r/w)
const uint8_t SC16IS752_REG_IOCONTROL = 0X0E;  // I/O pin control register (r/w)
const uint8_t SC16IS752_REG_EFCR = 0X0F;       // extra features register (r/w)
// Special registers only if LCR[7] == 1 & LCR != 0xBF
const uint8_t SC16IS752_REG_DLL = 0x00;  // divisor latch lsb (r/w) only if LCR[7]=1 & LCR != 0xBF
const uint8_t SC16IS752_REG_DLH = 0X01;  // divisor latch msb (r/w) only if LCR[7]=1 & LCR != 0xBF
// Enhanced registers only if LCR == 0xBF
const uint8_t SC16IS752_REG_EFR = 0X02;    // enhanced features register only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON1 = 0X04;   // Xon1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON2 = 0X05;   // Xon2 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF1 = 0X06;  // Xoff1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF2 = 0X07;  // Xoff2 word (rw) only if LCR=0xBF (1011 1111)

enum UARTParityOptions {
  UART_CONFIG_PARITY_NONE,
  UART_CONFIG_PARITY_EVEN,
  UART_CONFIG_PARITY_ODD,
};
enum SC16IS752ComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };

static uint8_t reg_buffer;  // buffer used for register operation
static const char *const TAG = "sc16is752";

// the register address is composed of the register value and the channel value.
// note that for I/O the chanel value is not significant
inline int subaddress(uint8_t reg_addr, uint8_t channel = 0) { return (reg_addr << 3 | channel << 1); }

/// @brief Describes a SC16IS752 I²C component.
/// We have two related class :tThe SC16IS752Channel class that
/// take cares of the UART related functions and the SC16IS752GPIOPin
/// that take care of the details for the GPIO pins of the component.
class SC16IS752Component : public Component, public i2c::I2CDevice {
 public:
  bool check_model();

  /// @brief Set the model (750/752) of the component
  void set_model(SC16IS752ComponentModel model) { model_ = model; }
  void set_crystal(uint32_t crystal) { crystal_ = crystal; }

  //
  //  overloaded Component virtual functions
  //

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void update();

  //
  // GPIO related
  //

  /// Helper function to read the value of a pin.
  bool digital_read(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void pin_mode(uint8_t pin, gpio::Flags flags);

 protected:
  friend class SC16IS752Channel;

  // uint8_t pin_;
  // bool inverted_;
  // gpio::Flags flags_;
  // bool read_inputs_;
  // bool write_register_uint8_t reg, uint8_t value;

  /// Mask for the pin config - 1 means OUTPUT, 0 means INPUT
  uint8_t config_mask_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint8_t output_mask_{0x00};
  /// The state of the actual input pin states - 1 means HIGH, 0 means LOW
  uint8_t input_mask_{0x00};
  /// @brief The precise model of the component
  SC16IS752ComponentModel model_;

  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 3072000/16 = 192,000bps
  uint32_t crystal_;
};

/// @brief Describes the UART part of a SC16IS752 I²C component.
class SC16IS752Channel : public uart::UARTComponent {
 public:
  SC16IS752Channel() { fifo_enable(); }  // ctor
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
  /// @return ??? TODO ???
  bool read_array(uint8_t *data, size_t len) override;

  /// @brief Read next byte from serial buffer without removing it ???
  /// @param data pointer to the byte
  /// @return ? TODO dont know
  bool peek_byte(uint8_t *data) override;

  /// @brief Get the number of bytes available for reading from the serial port.
  /// @return the number of bytes available
  int available() override;

  /// @brief Clears the buffer ??? output **once all outgoing characters have been sent**. ???
  void flush() override;

 protected:
  /// @brief ???
  void check_logger_conflict() override;

  uint8_t read_register(int reg_address);
  void write_register(int reg_address, uint8_t value);
  void set_line_param();
  void set_baudrate();
  void fifo_enable(bool enable = true);
  int rx_fifo_level() { return read_register(SC16IS752_REG_RXLVL); }
  int tx_fifo_level() { return read_register(SC16IS752_REG_TXLVL); }

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
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace sc16is752
}  // namespace esphome
