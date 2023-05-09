#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sc16is752 {

enum UARTParityOptions {
  UART_CONFIG_PARITY_NONE,
  UART_CONFIG_PARITY_EVEN,
  UART_CONFIG_PARITY_ODD,
};

/// @brief Describes a SC16IS752 I²C component. This components can
/// have up to two UART channels and 8 GPIO pins
class SC16IS752Component : public Component, public i2c::I2CDevice {
 public:
  /// Where the component's initialization should happen.
  /// Analogous to Arduino's setup(). This method is guaranteed to only be called once.
  /// Defaults to doing nothing.
  void setup() override;
  /// @brief I guess it dumps the configuration
  void dump_config() override;
  /// @brief priority of setup(). higher -> executed earlier
  /// @return the priority
  float get_setup_priority() const override { return setup_priority::IO; }
  /// @brief You'll only need this when creating your own custom sensor
  void update();

  /// Helper function to read the value of a pin.
  bool digital_read(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void pin_mode(uint8_t pin, gpio::Flags flags);

 protected:
  friend class SC16IS752Channel;

  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
  uint8_t current_channel_ = 255;

  bool read_inputs_();
  bool write_register_(uint8_t reg, uint8_t value);

  /// Mask for the pin config - 1 means OUTPUT, 0 means INPUT
  uint8_t config_mask_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint8_t output_mask_{0x00};
  /// The state of the actual input pin states - 1 means HIGH, 0 means LOW
  uint8_t input_mask_{0x00};
  /// Storage for last I2C error seen
  esphome::i2c::ErrorCode last_error_;
};

/// @brief Describes the UART component part of a SC16IS752 I²C component.
class SC16IS752Channel : public uart::UARTComponent {
 public:
  // nicer and more efficient but a bit inconsistent with other ESPHome class!
  // /// get/set accessors. Usage chanel() = xxx
  // auto channel() -> uint8_t & { return channel_; }
  // /// get/set accessors. Usage parrent() = xxx
  // auto parent() -> SC16IS752Component & { return *parent_; }

  /// @brief We belongs to a SC16IS752Component
  /// @param parent our parrent
  void set_parent(SC16IS752Component *parent) { parent_ = parent; }
  /// @brief Set our channel number
  /// @param channel channel number
  void set_pin(uint8_t channel) { channel_ = channel; }

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
  /// @brief Clears the buffer **once all outgoing characters have been sent**.
  void flush() override;

 protected:
  /// @brief the channel number of this UART
  uint8_t channel_;
  /// @brief The SC16IS752Component we belongs to
  SC16IS752Component *parent_;

  /// @brief baud rate for this channel
  uint32_t baud_rate_;
  /// @brief channel's number of stop bits
  uint8_t stop_bits_;
  /// @brief channel's number of data bits
  uint8_t data_bits_;
  /// @brief channel's parity option
  UARTParityOptions parity_;
};

/// Helper class to expose a SC16IS752 pin as an internal input GPIO pin.
class SC16IS752GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  // std::string dump_summary() const override;

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
