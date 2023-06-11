/// @file wk2132.h
/// @author @DrCoolzic
/// @brief wk2132 interface declaration

#pragma once
#include <bitset>
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sc16is75x/gen_uart.h"

namespace esphome {
namespace wk2132 {

/// @brief Global register
const uint8_t REG_WK2132_GENA = 0x00;  ///< Global control register
const uint8_t REG_WK2132_GRST = 0x01;  ///< Global UART channel reset register
const uint8_t REG_WK2132_GMUT = 0x02;  ///< Global main UART control register, and will be used only when the main UART
                                       ///< is selected as UART, no need to be set here.

/// @brief UART Channel register when PAGE = 0
const uint8_t REG_WK2132_SPAGE = 0x03;  //< UART page control register
const uint8_t REG_WK2132_SCR = 0x04;    ///< serial control register
const uint8_t REG_WK2132_LCR = 0x05;    ///< line control register
const uint8_t REG_WK2132_FCR = 0x06;    ///< FIFO control register
const uint8_t REG_WK2132_SIER = 0x07;   ///< interrupt enable register
const uint8_t REG_WK2132_SIFR = 0x08;   ///< interrupt flag register
const uint8_t REG_WK2132_TFCNT = 0x09;  ///< transmit FIFO value register
const uint8_t REG_WK2132_RFCNT = 0x0A;  ///< receive FIFO value register
const uint8_t REG_WK2132_FSR = 0x0B;    ///< FIFO status register
const uint8_t REG_WK2132_LSR = 0x0C;    ///< receive status register
const uint8_t REG_WK2132_FDA = 0x0D;    ///< FIFO data register (r/w)

///@brief UART Channel register PAGE = 1
const uint8_t REG_WK2132_BRH = 0x04;  ///< Channel baud rate configuration register high byte
const uint8_t REG_WK2132_BRL = 0x05;  ///< Channel baud rate configuration register low byte
const uint8_t REG_WK2132_BRD = 0x06;  ///< Channel baud rate configuration register decimal part
const uint8_t REG_WK2132_RFI = 0x07;  ///< Channel receive FIFO interrupt trigger configuration register
const uint8_t REG_WK2132_TFI = 0x08;  ///< Channel transmit FIFO interrupt trigger configuration register

class WK2132Channel;                  // forward declaration
///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a WK2132 I²C component.
///
/// This class derives from two @ref esphome classes:
/// - The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
///   @ref Component::dump_config() and @ref Component::get_setup_priority() methods
/// - The @ref i2c::I2CDevice class. From which we use some methods
///
/// We have one related class :
/// - The @ref WK2132Channel class that takes cares of the UART related methods
///////////////////////////////////////////////////////////////////////////////
class WK2132Component : public Component, public i2c::I2CDevice {
 public:
  // we store the base_address and we increment number of instances
  WK2132Component() : base_address_{address_} { ++counter_; }
  void set_crystal(uint32_t crystal) { crystal_ = crystal; }
  void set_test_mode(int test_mode) { test_mode_ = test_mode; }

  //
  //  override Component methods
  //

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void loop() override;

 protected:
  friend class WK2132Channel;
  const char *reg_to_str(int val);

  /// @brief All write calls to I2C registers are funneled through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer to buffer
  /// @param len length of buffer
  /// @return the i2c error codes
  void write_wk2132_register_(uint8_t reg_address, uint8_t channel, const uint8_t *buffer, size_t len);

  /// @brief All read calls to I2C registers are funneled through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer buffer pointer
  /// @param len length of buffer
  /// @return the i2c error codes
  uint8_t read_wk2132_register_(uint8_t reg_address, uint8_t channel, uint8_t *buffer, size_t len);

  int get_num_() const { return num_; }

  uint32_t crystal_{14745600L};  // crystal default value;
  uint8_t base_address_;         ///< base address of I2C device
  int test_mode_{0};
  uint8_t data_;                 // temporary buffer
  /// @brief the list of WK2132Channel UART children
  std::vector<WK2132Channel *> children_{};
  static int counter_;       // count number of instances
  int num_{counter_};        // current counter
  bool page1_{false};        // set to true when in page1 mode
  bool initialized_{false};  // true when initialized finished
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes a UART channel of a WK2132 I²C component.
///
/// This class derives from the @ref gen_uart::GenUARTChannel class.
/// As the @ref gen_uart::GenUARTChannel is a pure virtual class we need to
/// implement the following methods : @ref
///////////////////////////////////////////////////////////////////////////////
class WK2132Channel : public gen_uart::GenUARTChannel {
 public:
  void set_parent(WK2132Component *parent) {
    parent_ = parent;
    parent_->children_.push_back(this);
  }
  void set_channel(uint8_t channel) { channel_ = channel; }
  void setup_channel();

  //
  // overriden UARTComponent functions
  //

  /// @brief Should return the number of bytes available in the receiver fifo
  /// @return the number of bytes we can read
  size_t rx_in_fifo() override;

  /// @brief Should return the number of bytes available in the transmitter fifo
  /// @return the number of bytes we can write
  size_t tx_in_fifo() override;

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
  virtual size_t fifo_size() override { return 128; }

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
