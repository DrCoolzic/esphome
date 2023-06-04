/// @file external_uart.h
/// @author @DrCoolzic
/// @brief interface declaration of an external uart bus

#pragma once
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace external_uart {

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes an external (i.e. not in the ESP) UART bus.
///
/// External UART should be able to use this code to implement the required
/// UARTComponent virtual methods to avoid code duplication. This class
/// implement the following functions : @ref uart::UARTComponent::write_array(),
/// @ref uart::UARTComponent::read_array(), @ref uart::UARTComponent::peek_byte(),
/// @ref uart::UARTComponent::available(), @ref uart::UARTComponent::flush(). TODO ?
/// **Note that this class a virtual class**
///////////////////////////////////////////////////////////////////////////////
class ExternalUARTComponent : public uart::UARTComponent {
 public:
  //
  // we implement the UARTComponent pure virtual methods
  //

  /// @brief Write a specified number of bytes from a buffer to a serial port
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to write
  void write_array(const uint8_t *buffer, size_t len) override;

  /// @brief Read a specified number of bytes from a serial port to an buffer
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to read
  /// @return true if succeed false otherwise
  bool read_array(uint8_t *buffer, size_t len) override;

  /// @brief Read next byte available from serial buffer without removing it
  /// @param buffer pointer to the byte
  /// @return true if succeed false if no character available
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Return the number of bytes available for reading from the serial port.
  /// @return the number of bytes available in the receiver fifo
  int available() override { return rx_available(); }

  // @brief Flush the input and output fifo
  virtual void flush();

  /// @brief This function can be used to test the uart bus
  /// @param safe if true we use safe write method by calling the tx_available()
  void test_uart_(bool safe);

  /// @brief Should return the number of bytes available in the receiver fifo
  /// @return the number of bytes we can read
  virtual size_t rx_available() = 0;

  /// @brief Should return the number of bytes available in the transmitter fifo
  /// @return the number of bytes we can write
  virtual size_t tx_available() = 0;

  /// @brief Read data from the receiver fifo to a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to read
  /// @return true if succeed false otherwise
  virtual bool read_data(uint8_t *buffer, size_t len) = 0;

  /// @brief Write data to the transmitter fifo from a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to write
  /// @return true if succeed false otherwise
  virtual bool write_data(const uint8_t *buffer, size_t len) = 0;

  /// @brief Query the size of the component's fifo
  /// @return the size
  virtual size_t fifo_size() = 0;

 protected:
  /// @brief cannot happen with external uart
  void check_logger_conflict() override {}

  struct {
    uint8_t data;
    bool empty{true};
  } peek_buffer_;
};

}  // namespace external_uart
}  // namespace esphome
