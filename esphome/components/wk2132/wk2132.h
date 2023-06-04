/// @file wk2132.h
/// @author @DrCoolzic
/// @brief wk2132 interface declaration

#pragma once
#include <bitset>
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/external_uart/external_uart.h"

namespace esphome {
namespace wk2132 {

/// @brief Global register
const uint8_t REG_WK2132_GENA = 0x00;  ///< Global control register
const uint8_t REG_WK2132_GRST = 0x01;  ///< Global UART channel reset register

/// @brief Page control register
const uint8_t REG_WK2132_PAGE = 0x03;  //< UART page control register

/// @brief UART Channel register SPAGE0
const uint8_t REG_WK2132_SCR = 0x04;   ///< serial control register
const uint8_t REG_WK2132_LCR = 0x05;   ///< line control register
const uint8_t REG_WK2132_FCR = 0x06;   ///< FIFO control register
const uint8_t REG_WK2132_IER = 0x07;   ///< interrupt enable register
const uint8_t REG_WK2132_IFR = 0x08;   ///< interrupt flag register
const uint8_t REG_WK2132_TFV = 0x09;   ///< transmit FIFO value register
const uint8_t REG_WK2132_RFV = 0x0A;   ///< receive FIFO value register
const uint8_t REG_WK2132_FSR = 0x0B;   ///< FIFO status register
const uint8_t REG_WK2132_LSR = 0x0C;   ///< receive status register
const uint8_t REG_WK2132_FDAT = 0x0D;  ///< FIFO data register (r/w)

///@brief UART Channel register SPAGE1
const uint8_t REG_WK2132_BRH = 0x04;   ///< Channel baud rate configuration register high byte
const uint8_t REG_WK2132_BRL = 0x05;   ///< Channel baud rate configuration register low byte
const uint8_t REG_WK2132_BRD = 0x06;   ///< Channel baud rate configuration register decimal part
const uint8_t REG_WK2132_RFTL = 0x07;  ///< Channel receive FIFO interrupt trigger configuration register
const uint8_t REG_WK2132_TFTL = 0x08;  ///< Channel transmit FIFO interrupt trigger configuration register

// for debug messages ...
static const char *write_reg_to_str[] = {"THR",   "IER",   "FCR", "LCR", "MCR", "LSR", "TCR", "SPR",
                                         "_INV_", "_INV_", "IOD", "IO",  "IOI", "IOC", "EFR"};
static const char *read_reg_to_str[] = {"RHR", "IER", "IIR", "LCR", "MCR", "LSR", "MSR", "SPR",
                                        "TXF", "RXF", "IOD", "IOP", "IOI", "IOC", "EFR"};

// convert byte to binary string
inline const char *i2s_(uint8_t val) { return std::bitset<8>(val).to_string().c_str(); }

class WK2132Channel;  // forward declaration

///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a WK2132 I²C component.
///
/// This class derives from two @ref esphome classes:
/// - The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
///   @ref Component::dump_config() and @ref Component::get_setup_priority() methods
/// - The @ref i2c::I2CDevice class. From which we use some methods
///
/// We have one related class :
/// - The @ref WK2132Channel class that takes cares of the UART related functions
///////////////////////////////////////////////////////////////////////////////
class WK2132Component : public Component, public i2c::I2CDevice {
 public:
  // we store the base_address and we increment number of instances
  WK2132Component() : base_address_{address_} { ++count_; }
  void set_test_mode(int test_mode) { test_mode_ = test_mode; }

  //
  //  override Component functions
  //

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void loop() override;

 protected:
  friend class WK2132Channel;

  /// @brief All write calls to I2C registers are funneled through this function
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer
  /// @return the i2c error codes
  i2c::ErrorCode write_wk2132_register_(uint8_t reg_address, uint8_t channel, const uint8_t *buffer);

  /// @brief All read calls to I2C registers are funneled through this function
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer
  /// @return the i2c error codes
  i2c::ErrorCode read_wk2132_register_(uint8_t reg_address, uint8_t channel, uint8_t *buffer);

  int get_num_() const { return num_; }

  /// crystal default on SC16IS750 => 14.7456MHz - on SC16IS752 => 3.072MHz
  uint32_t crystal_{14745600L};
  uint8_t base_address_{0x00};
  int test_mode_{0};
  /// @brief the list of WK2132Channel UART children
  std::vector<WK2132Channel *> children{};
  static int count_;  // count number of instances
  int num_{count_};   // take current count
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes a UART channel of a WK2132 I²C component.
///
/// This class derives from the @ref external_uart::ExternalUARTComponent class.
/// As the @ref external_uart::ExternalUARTComponent is a pure virtual class we need to
/// implement the following functions : @ref
///////////////////////////////////////////////////////////////////////////////
class WK2132Channel : public external_uart::ExternalUARTComponent {
 public:
  void set_parent(WK2132Component *parent) {
    parent_ = parent;
    parent_->children.push_back(this);
  }
  void set_channel(uint8_t channel) { channel_ = channel; }
  void setup_channel();

  //
  // overriden UARTComponent functions
  //

  /// @brief Should return the number of bytes available in the receiver fifo
  /// @return the number of bytes we can read
  size_t rx_available() override;

  /// @brief Should return the number of bytes available in the transmitter fifo
  /// @return the number of bytes we can write
  virtual size_t tx_available() override;

  /// @brief Read data from the receiver fifo to a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to read
  /// @return true if succeed false otherwise
  bool read_data(uint8_t *buffer, size_t len) override;

  /// @brief Write data to the transmitter fifo from a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to write
  /// @return true if succeed false otherwise
  bool write_data(const uint8_t *buffer, size_t len) override;

  /// @brief Query the size of the component's fifo
  /// @return the size
  virtual size_t fifo_size() override { return 256; }

 protected:
  friend class WK2132Component;

  void set_line_param_();
  void set_baudrate_();

  WK2132Component *parent_;
  uint8_t channel_;
  uint8_t data_;  // one byte buffer
};

}  // namespace wk2132
}  // namespace esphome
