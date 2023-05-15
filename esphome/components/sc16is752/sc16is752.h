#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is752 {

// // Device Address (A=VDD - B=GND - C=SCL - D=SDA)
// const uint8_t SC16IS752_ADDRESS_AA = 0X90;
// const uint8_t SC16IS752_ADDRESS_AB = 0X92;
// const uint8_t SC16IS752_ADDRESS_AC = 0X94;
// const uint8_t SC16IS752_ADDRESS_AD = 0X96;
// const uint8_t SC16IS752_ADDRESS_BA = 0X98;
// const uint8_t SC16IS752_ADDRESS_BB = 0X9A;
// const uint8_t SC16IS752_ADDRESS_BC = 0X9C;
// const uint8_t SC16IS752_ADDRESS_BD = 0X9E;
// const uint8_t SC16IS752_ADDRESS_CA = 0XA0;
// const uint8_t SC16IS752_ADDRESS_CB = 0XA2;
// const uint8_t SC16IS752_ADDRESS_CC = 0XA4;
// const uint8_t SC16IS752_ADDRESS_CD = 0XA6;
// const uint8_t SC16IS752_ADDRESS_DA = 0XA8;
// const uint8_t SC16IS752_ADDRESS_DB = 0XAA;
// const uint8_t SC16IS752_ADDRESS_DC = 0XAC;
// const uint8_t SC16IS752_ADDRESS_DD = 0XAE;

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
const uint8_t SC16IS752_REG_EFR = 0X02;         // enhanced features register only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON1 = 0X04;        // Xon1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XON2 = 0X05;        // Xon2 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF1 = 0X06;       // Xoff1 word (rw) only if LCR=0xBF (1011 1111)
const uint8_t SC16IS752_REG_XOFF2 = 0X07;       // Xoff2 word (rw) only if LCR=0xBF (1011 1111)

const uint8_t SC16IS752_SINGLE_CHANNEL = 0x01;  // SC16IS750
const uint8_t SC16IS752_DUAL_CHANNEL = 0x02;    // SC16IS752

enum UARTParityOptions {
  UART_CONFIG_PARITY_NONE,
  UART_CONFIG_PARITY_EVEN,
  UART_CONFIG_PARITY_ODD,
};

inline int address(uint8_t reg_addr, uint8_t channel) { return (reg_addr << 3 | channel << 1); }

/// @brief Describes a SC16IS752 I²C component. This class
/// contains the parameter and method global to the component.
/// We have two related class The SC16IS752Channel class that
/// take cares of the UART related things and the SC16IS752GPIOPin
/// that take care of the details for the GPIO pins of the component.
/// This component can have one (SC16IS750) or two (SC16IS752) UART
/// channels and always have 8 GPIO pins.
/// Some important methods to note from the I²C class are:
///   ErrorCode read(uint8_t *data, size_t len)
///   ErrorCode read_register(uint8_t a_register, uint8_t *data, size_t len, bool stop = true)
///   ErrorCode write(const uint8_t *data, uint8_t len, bool stop = true)
///   ErrorCode write_register(uint8_t a_register, const uint8_t *data, size_t len, bool stop = true)
/// ** Note that we do not need the address as it is stored in the I²C device **
class SC16IS752Component : public Component, public i2c::I2CDevice {
 public:
  /// Where the component's initialization should happen.
  /// Defaults to doing nothing.
  void setup() override;
  /// @brief Used to dump the configuration
  void dump_config() override;
  /// @brief priority of setup. higher -> executed earlier
  /// @return the priority
  float get_setup_priority() const override { return setup_priority::IO; }
  /// @brief You'll only need this when creating your own custom sensor
  void update();

  i2c::ErrorCode read(uint8_t *data, size_t len) { return read(data, len); }
  //   ErrorCode read_register(uint8_t a_register, uint8_t *data, size_t len, bool stop = true) {
  //     ErrorCode err = this->write(&a_register, 1, stop);
  //     if (err != ERROR_OK)
  //       return err;
  //     return this->read(data, len);
  //   }

  //   ErrorCode write(const uint8_t *data, uint8_t len, bool stop = true) { return bus_->write(address_, data, len,
  //   stop); } ErrorCode write_register(uint8_t a_register, const uint8_t *data, size_t len, bool stop = true) {
  //     WriteBuffer buffers[2];
  //     buffers[0].data = &a_register;
  //     buffers[0].len = 1;
  //     buffers[1].data = data;
  //     buffers[1].len = len;
  //     return bus_->writev(address_, buffers, 2, stop);
  //   }

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
  // uint8_t current_channel_ = 255;

  // bool read_inputs_;
  // bool write_register_uint8_t reg, uint8_t value;

  /// Mask for the pin config - 1 means OUTPUT, 0 means INPUT
  uint8_t config_mask_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint8_t output_mask_{0x00};
  /// The state of the actual input pin states - 1 means HIGH, 0 means LOW
  uint8_t input_mask_{0x00};
  /// Storage for last I2C error seen
  esphome::i2c::ErrorCode last_error_;
};

/// @brief Describes the UART part of a SC16IS752 I²C component.
/// We keep track of our SC16IS752Component parent. This class
/// only contains the methods and parameters that relates to the
/// UART function.
class SC16IS752Channel : public uart::UARTComponent {
 public:
  /// @brief We belongs to a SC16IS752Component
  /// @param parent our parent
  void set_parent(SC16IS752Component *parent) { parent_ = parent; }

  /// @brief Set our channel number
  /// @param channel channel number
  void set_channel(uint8_t channel) { channel_ = channel; }

  void set_baudrate(int baudrate) { baud_rate_ = baudrate; }
  void set_stop_bits(int stop_bits) { stop_bits_ = stop_bits; }
  void set_data_bits(int data_bits) { data_bits_ = data_bits; }
  void set_parity(UARTParityOptions parity) { parity_ = parity; }

  /// @brief Write a specified number of bytes from a buffer to a serial port
  /// @param data pointer to the buffer
  /// @param len number of bytes to write
  void write_array(const uint8_t *data, size_t len) override;

  /// @brief Read a specified number of bytes from a serial port to an buffer
  /// @param data pointer to the buffer
  /// @param len number of bytes to read
  bool read_array(uint8_t *data, size_t len) override;

  /// @brief Read next byte from serial buffer without removing it
  /// @param data pointer to the byte
  /// @return ? TODO
  bool peek_byte(uint8_t *data) override;

  /// @brief Get the number of bytes available for reading from the serial port.
  /// @return the number of bytes available
  int available() override;

  /// @brief Clears the buffer ??? **once all outgoing characters have been sent**. ???
  void flush() override;

 protected:
  void check_logger_conflict() override;

  void fifo_enable(bool enable = true);
  void set_line_param();

  // uint8_t fifo_available_space() { return parent_->read_register(address(SC16IS752_REG_TXLVL, channel_), &reg_buf,
  // 1); }

  /// @brief the channel number of this UART
  uint8_t channel_;
  /// @brief The SC16IS752Component we belongs to
  SC16IS752Component *parent_;
  /// @brief baud rate for this channel
  uint32_t baudrate_;
  /// @brief channel's number of stop bits
  uint8_t stop_bits_;
  /// @brief channel's number of data bits
  uint8_t data_bits_;
  /// @brief channel's parity option
  UARTParityOptions parity_;
  uint8_t buffer_;
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
