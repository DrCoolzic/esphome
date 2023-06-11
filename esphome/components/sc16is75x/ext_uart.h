/// @file ext_uart.h
/// @author @DrCoolzic
/// @brief interface declaration of an external uart bus

#pragma once
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ext_uart {

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes an "external" (i.e. not on the ESP board) UART bus.
///
/// Any "external" UART should be able to use this code to implement the required
/// UARTComponent virtual methods to avoid code duplication. This class
/// implement the following virtual functions:
/// - @ref uart::UARTComponent::write_array(),
/// - @ref uart::UARTComponent::read_array(),
/// - @ref uart::UARTComponent::peek_byte(),
/// - @ref uart::UARTComponent::available(),
/// - @ref uart::UARTComponent::flush(). TODO ?
/// @n **Note that this class a pure virtual class**
///////////////////////////////////////////////////////////////////////////////
class ExtUARTComponent : public uart::UARTComponent {
 public:
  //
  // we implement the UARTComponent pure virtual methods
  //

  ///
  /// ** Important remark about the maximum buffer size **
  /// The maximum number of bytes you can read or write in one call
  /// is passed with the "len" parameter. This parameter should not
  /// be bigger than the value returned by fifo_size()
  ///

  /// @brief Write a specified number of bytes from a buffer to a serial port
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to write
  void write_array(const uint8_t *buffer, size_t len) override;

  /// @brief Read a specified number of bytes from a serial port to a buffer
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to read
  /// @return true if succeed, false otherwise
  bool read_array(uint8_t *buffer, size_t len) override;

  /// @brief Read next byte available from serial buffer without removing it
  /// from the FIFO
  /// @param buffer pointer to the byte
  /// @return true if succeed reading one byte, false if no character available
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Return the number of bytes available for reading from the serial port.
  /// @return the number of bytes available in the receiver fifo
  int available() override { return rx_in_fifo(); }

  /// @brief Flush the output fifo. This is the only way to wait until all the bytes
  /// in the transmit FIFO have been sent. The function timeout after 100 ms. Therefore
  /// at very low speed you can't be sure all characters are gone.
  virtual void flush();

  /// @brief Returns the number of bytes available in receive fifo
  /// @return the number of bytes we can read
  virtual size_t rx_in_fifo() = 0;

  /// @brief Returns the number of bytes still in the transmitter fifo
  /// @return the number of bytes not yet sent
  virtual size_t tx_in_fifo() = 0;

  /// @brief Reads data from the receiver fifo and write to a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to read
  /// @return true if succeed, false otherwise
  virtual bool read_data(uint8_t *buffer, size_t len) = 0;

  /// @brief Write data to the transmitter fifo from a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to write
  /// @return true if succeed false otherwise
  virtual bool write_data(const uint8_t *buffer, size_t len) = 0;

  /// @brief Query the size of the component's fifo.
  /// @n This is the maximum length you can use for the buffer
  /// during read and write data functions.
  /// @return the size
  virtual size_t fifo_size() = 0;

 protected:
  /// @brief cannot happen with external uart
  void check_logger_conflict() override {}
  size_t buffered() { return peek_buffer_.empty ? 0 : 1; }

  bool safe_{true};  // false will speed up operation but is unsafe
  struct {
    uint8_t data;
    bool empty{true};
  } peek_buffer_;  // temporary storage when you peek data

  void uart_send_test(uint8_t channel);
  void uart_receive_test(uint8_t channel, bool print_buf = true);
};

}  // namespace ext_uart
}  // namespace esphome
