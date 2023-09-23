/// @file sc16s75x.h
/// @author @DrCoolzic
/// @brief sc16is75x interface declaration

#pragma once
#include <bitset>
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
// #include "esphome/components/uart/uart.h"
#include "uart_base.h"

namespace esphome {
namespace sc16is75x {

// General sc16is75x registers
constexpr uint8_t SC16IS75X_REG_RHR = 0x00;  // 00 receive holding register (r) with a 64-bytes FIFO
constexpr uint8_t SC16IS75X_REG_THR = 0X00;  // 00 transmit holding register (w) with a 64-bytes FIFO
constexpr uint8_t SC16IS75X_REG_IER = 0X01;  // 08 interrupt enable register (r/w)
constexpr uint8_t SC16IS75X_REG_IIR = 0X02;  // 10 interrupt identification register (r)
constexpr uint8_t SC16IS75X_REG_FCR = 0X02;  // 10 FIFO control register (w)
constexpr uint8_t SC16IS75X_REG_LCR = 0X03;  // 18 line control register (r/w)
constexpr uint8_t SC16IS75X_REG_MCR = 0X04;  // 20 modem control register (r/w) - only when EFR[4]=1
constexpr uint8_t SC16IS75X_REG_LSR = 0X05;  // 28 line status register (ro)
constexpr uint8_t SC16IS75X_REG_MSR = 0X06;  // 30 modem status register (ro)
constexpr uint8_t SC16IS75X_REG_TCR = 0X06;  // 30 transmission control register (r/w) when EFR[4]=1 & MRC[2]=1
constexpr uint8_t SC16IS75X_REG_SPR = 0X07;  // 38 scratchpad register (r/w)
constexpr uint8_t SC16IS75X_REG_TLR = 0X07;  // 38 trigger level register (r/w) when EFR[4]=1 & MRC[2]=1
constexpr uint8_t SC16IS75X_REG_TXF = 0X08;  // 40 transmit FIFO level register (ro)
constexpr uint8_t SC16IS75X_REG_RXF = 0X09;  // 48 receive FIFO level register (ro)
constexpr uint8_t SC16IS75X_REG_IOD = 0X0A;  // 50 I/O pin direction register (r/w)
constexpr uint8_t SC16IS75X_REG_IOP = 0X0B;  // 58 I/O pin state register (r/w)
constexpr uint8_t SC16IS75X_REG_IOI = 0X0C;  // 60 I/O interrupt enable register (r/w)
constexpr uint8_t SC16IS75X_REG_IOC = 0X0E;  // 70 I/O pin control register (r/w)
constexpr uint8_t SC16IS75X_REG_XFR = 0X0F;  // 78 extra features register (r/w)

// Special registers only if LCR[7] == 1 & LCR != 0xBF
constexpr uint8_t SC16IS75X_REG_DLL = 0x00;  // divisor latch lsb (r/w) only if LCR[7]=1 & LCR != 0xBF
constexpr uint8_t SC16IS75X_REG_DLH = 0X01;  // divisor latch msb (r/w) only if LCR[7]=1 & LCR != 0xBF

// Enhanced registers only if LCR == 0xBF
constexpr uint8_t SC16IS75X_REG_EFR = 0X02;  // enhanced features register only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XO1 = 0X04;  // Xon1 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XO2 = 0X05;  // Xon2 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XF1 = 0X06;  // Xoff1 word (rw) only if LCR=0xBF (1011 1111)
constexpr uint8_t SC16IS75X_REG_XF2 = 0X07;  // Xoff2 word (rw) only if LCR=0xBF (1011 1111)

enum SC16IS75XComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };  ///< chip models

class SC16IS75XChannel;  ///< forward declaration
using Channel = uint8_t;

///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a SC16IS75X I²C component.
///
/// This class derives from two @ref esphome classes:
/// - The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
///   @ref Component::dump_config() and @ref Component::get_setup_priority() methods
/// - The @ref i2c::I2CDevice class. From which we use some methods
///
/// We have two related class :
/// - The @ref SC16IS75XChannel class that takes cares of the UART related methods
/// - The @ref SC16IS75XGPIOPin class
///   that takes care of the details for the GPIO pins of the component.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XComponent : public Component, public i2c::I2CDevice {
 public:
  void set_model(SC16IS75XComponentModel model) { model_ = model; }
  void set_crystal(uint32_t crystal) { crystal_ = crystal; }
  void set_test_mode(int test_mode) { test_mode_ = test_mode; }
  void set_name(std::string name) { this->name_ = std::move(name); }
  const char *get_name() { return this->name_.c_str(); }

  //
  //  override Component methods
  //

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void loop() override;

 protected:
  // we give access to protected objects to our friends :)
  friend class SC16IS75XChannel;
  friend class SC16IS75XGPIOPin;

  /// @brief All write calls to component registers are done through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param data to write
  i2c::ErrorCode write_sc16is75x_register_(uint8_t reg_address, Channel channel, const uint8_t *data, size_t len = 1);

  /// @brief All read calls to I2C registers are done through this method
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to read
  /// @return the i2c error codes
  i2c::ErrorCode read_sc16is75x_register_(uint8_t reg_address, Channel channel, uint8_t *buffer, size_t len = 1);

  /// Helper method to read the value of a pin.
  bool read_pin_val_(uint8_t pin);

  /// Helper method to write the value of a pin.
  void write_pin_val_(uint8_t pin, bool value);

  /// Helper method to set the pin mode of a pin.
  void set_pin_direction_(uint8_t pin, gpio::Flags flags);

  void test_gpio_();

  /// pin config mask: 1 means OUTPUT, 0 means INPUT
  uint8_t pin_config_{0x00};
  /// output state: 1 means HIGH, 0 means LOW
  uint8_t output_state_{0x00};
  /// input pin states: 1 means HIGH, 0 means LOW
  uint8_t input_state_{0x00};

  SC16IS75XComponentModel model_{SC16IS752_MODEL};  ///< model of the component
  uint32_t crystal_;                                ///< default crystal for SC16IS752
  uint8_t data_{0};                                 ///< one byte buffer
  std::vector<SC16IS75XChannel *> children_{};      ///< the list of SC16IS75XChannel UART children
  int test_mode_{0};                                ///< test_mode value (0 no test)
  std::string name_;                                ///< store name of entity
  bool initialized_{false};                         ///< true when component initialized
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes the UART part of a SC16IS75X I²C component.
///
/// This class derives from the @ref gen_uart::GenUARTChannel virtual class.
/// we must therefore provide several methods for the virtual class
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XChannel : public uart_base::UARTBase {
 public:
  void set_parent(SC16IS75XComponent *parent) {
    this->parent_ = parent;
    this->parent_->children_.push_back(this);  // add ourself to the list (vector)
  }
  void set_channel(Channel channel) { this->channel_ = channel; }
  void set_channel_name(std::string name) { this->name_ = std::move(name); }
  const char *get_channel_name() { return this->name_.c_str(); }
  void setup_channel();
  void dump_channel();

 protected:
  friend class SC16IS75XComponent;
  /// @brief Should return the number of bytes available in the receiver fifo
  /// @return the number of bytes we can read
  size_t rx_in_fifo_() override { return this->read_uart_register_(SC16IS75X_REG_RXF); }

  /// @brief Should return the number of bytes available in the transmitter fifo
  /// @return the number of bytes we can write
  virtual size_t tx_in_fifo_() override { return uart_base::FIFO_SIZE - this->read_uart_register_(SC16IS75X_REG_TXF); }

  inline bool tx_fifo_is_not_empty_() override {
    return !this->read_uart_register_(SC16IS75X_REG_LSR) & 0x40;  // TODO check
  }

  /// @brief Write data to the transmitter fifo from a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to writefor
  /// @return true if succeed false otherwise
  bool write_data_(const uint8_t *buffer, size_t len) override;

  /// @brief Read data from the receiver fifo to a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to read
  /// @return true if succeed false otherwise
  bool read_data_(uint8_t *buffer, size_t len) override;

  // helpers
  uint8_t read_uart_register_(int reg_address) {
    this->parent_->read_sc16is75x_register_(reg_address, this->channel_, &this->data_);
    return this->data_;
  }
  // void write_uart_register_(int reg_address, uint8_t value);
  void set_line_param_();
  void set_baudrate_();

  SC16IS75XComponent *parent_;
  Channel channel_;
  uint8_t data_;      ///< one byte buffer
  std::string name_;  ///< name of the entity
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Helper class to expose a SC16IS75X pin as an internal input GPIO pin.
///////////////////////////////////////////////////////////////////////////////
class SC16IS75XGPIOPin : public GPIOPin {
 public:
  void set_parent(SC16IS75XComponent *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }

  //
  // overriden GPIOPin methods
  //

  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

 protected:
  SC16IS75XComponent *parent_{nullptr};
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace sc16is75x
}  // namespace esphome
