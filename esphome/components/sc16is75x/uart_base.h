/// @file uart_base.h
/// @author @DrCoolzic
/// @brief declaration of UARTBase class and RingBuffer Class

#pragma once
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart_base {

///////////////////////////////////////////////////////////////////////////////
/// @brief This is an helper class that provides a simple ring buffers
/// that implements a FIFO function
///
/// This ring buffer is used to buffer the exchanges between the receiver
/// HW fifo and the client. Usually to read characters you first
/// check if bytes were received and if so you read them.
/// There is a problem if you try to the received bytes one by one.
/// If the registers are located on the chip everything is fine but with a
/// device like the wk2132 these registers are remote and therefore accessing
/// them requires several transactions on the I²C bus which is relatively slow.
/// One solution would be for the client to check the number of bytes available
/// and to read them all using the read_array() method. Unfortunately
/// most client I have reviewed are reading one character at a time in a
/// while loop which is the most inefficient way of doing things.
/// Therefore the solution I have chosen to implement is to
/// store received bytes locally in a buffer as soon as they arrive. With
/// this solution the bytes are stored locally and therefore accessible
/// very quickly when requested one by one.
/// @image html read_cycles.png
///////////////////////////////////////////////////////////////////////////////
class RingBuffer {
 public:
  /// @brief Ctor : initialize variables with the given size
  /// @param size size of the desired RingBuffer
  RingBuffer(const size_t size) : size_(size) { rb_.resize(size); }

  /// @brief pushes an item at the tail of the fifo
  /// @param item item to push
  /// @return true if item has been pushed, false il item was not pushed (buffer full)
  bool push(const uint8_t item) {
    if (is_full())
      return false;
    rb_[head_] = item;
    head_ = (head_ + 1) % rb_.size();
    count_++;
    return true;
  }

  /// @brief return and remove the item at head of the fifo
  /// @param item item read
  /// @return true if item has been retrieved, false il no item was found (buffer empty)
  bool pop(uint8_t &item) {
    if (is_empty())
      return false;
    item = rb_[tail_];
    tail_ = (tail_ + 1) % rb_.size();
    count_--;
    return true;
  }

  /// @brief return the value of the item at fifo's head without removing it
  /// @param item pointer to item to return
  /// @return true if item has been retrieved, false il no item was found (buffer empty)
  bool peek(uint8_t &item) {
    if (is_empty())
      return false;
    item = rb_[tail_];
    return true;
  }

  /// @brief test is the Ring Buffer is empty ?
  /// @return true if empty
  bool is_empty() { return (count_ == 0); }

  /// @brief test is the ring buffer is full ?
  /// @return true if full
  bool is_full() { return (count_ == rb_.size()); }

  /// @brief return the number of item in the ring buffer
  /// @return the count
  size_t count() { return count_; }

  /// @brief returns the number of free positions in the buffer
  /// @return how many items can be added
  size_t free() { return rb_.size() - count_; }

  /// @brief clear the buffer content
  void clear() { head_ = tail_ = count_ = 0; }

 private:
  std::vector<uint8_t> rb_;  // The ring buffer
  const size_t size_;        // size of the ring buffer
  int head_{0};              // points to the next element to write
  int tail_{0};              // points to the next element to read
  size_t count_{0};          // count number of element currently in the buffer
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Describes the UARTBase class that implements the basic read/write
/// functions of UARTComponent so we do not duplicate the code
///
/// This class derives from the virtual @ref uart::UARTComponent class.
///
/// Unfortunately I have not found **any documentation** about the
/// uart::UARTDevice and uart::UARTComponent classes of @ref ESPHome.\n
/// However it seems that both of them are based on Arduino library.\n
///
/// Most of the interfaces provided by the Arduino Serial library are **poorly
/// defined** and it seems that the API has even \b changed over time!\n
/// The esphome::uart::UARTDevice class directly relates to the **Serial Class**
/// in Arduino and that derives from **Stream class**.\n
/// For compatibility reason (?) many helper methods are made available in
/// ESPHome to read and write. Unfortunately in many cases these helpers
/// are missing the critical status information and therefore are even
/// more unsafe to use...\n
///////////////////////////////////////////////////////////////////////////////
class UARTBase : public uart::UARTComponent {
 public:
  UARTBase(const size_t size) : receive_buffer_(RingBuffer(size)) {}

  //
  // we override the virtual class from UARTComponent
  //

  /// @brief Writes a specified number of bytes toward a serial port
  /// @param buffer pointer to the buffer
  /// @param len number of bytes to write
  ///
  /// This method sends 'len' characters from the buffer to the serial line.
  /// Unfortunately (unlike the Arduino equivalent) this method
  /// does not return any flag and therefore it is not possible to know
  /// if any/all bytes have been transmitted correctly. Another problem
  /// is that it is not possible to know ahead of time how many bytes we
  /// can safely send as there is no tx_available() method provided!
  /// To avoid overrun when using the write method you can use the flush()
  /// method to wait until the transmit fifo is empty.
  ///
  /// Typical usage could be:
  /// @code
  ///   // ...
  ///   uint8_t buffer[128];
  ///   // ...
  ///   write_array(&buffer, len);
  ///   flush();
  ///   // ...
  /// @endcode
  void write_array(const uint8_t *buffer, size_t len) override;

  /// @brief Reads a specified number of bytes from a serial port
  /// @param buffer buffer to store the bytes
  /// @param len number of bytes to read
  /// @return true if succeed, false otherwise
  ///
  /// Typical usage:
  /// @code
  ///   // ...
  ///   auto len = available();
  ///   uint8_t buffer[128];
  ///   if (len > 0) {
  ///     auto status = read_array(&buffer, len)
  ///     // test status ...
  ///   }
  /// @endcode
  bool read_array(uint8_t *buffer, size_t len) override;

  /// @brief Reads first byte in FIFO without removing it
  /// @param buffer pointer to the byte
  /// @return true if succeed reading one byte, false if no character available
  ///
  /// This method returns the next byte from receiving buffer without
  /// removing it from the internal fifo. It returns true if a character
  /// is available and has been read, false otherwise.\n
  bool peek_byte(uint8_t *buffer) override;

  /// @brief Returns the number of bytes in the receive buffer
  /// @return the number of bytes available in the receiver fifo
  int available() override;

  /// @brief Flush the output fifo.
  ///
  /// If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
  /// of outgoing serial data to complete. (Prior to Arduino 1.0, this the method was
  /// removing any buffered incoming serial data.). **
  void flush() override;

 protected:
  /// @brief this cannot happen with external uart
  void check_logger_conflict() override {}

  /// @brief transfer the bytes from the HW fifo to the ring buffer
  /// @return the number of bytes transferred
  ///
  /// This method should be called from the loop() in derived class
  size_t rx_fifo_to_buffer_();

  //
  // what needs to be provided by children
  //

  /// @brief Returns the number of bytes in the receive fifo
  /// @return the number of bytes in the fifo
  virtual size_t rx_in_fifo_() = 0;

  /// @brief Returns the number of bytes in the transmit fifo
  /// @return the number of bytes in the fifo
  virtual size_t tx_in_fifo_() = 0;

  /// @brief test if transmit buffer is not empty
  /// @return true if not empty
  virtual bool tx_fifo_is_not_empty_() = 0;

  /// @brief Reads data from the receive fifo to a buffer
  /// @param buffer the buffer
  /// @param len the number of bytes we want to read
  /// @return true if succeed false otherwise
  virtual bool read_data_(uint8_t *buffer, size_t len) = 0;

  /// @brief Writes data from a buffer to the transmit fifo
  /// @param buffer the buffer
  /// @param len the number of bytes we want to write
  /// @return true if succeed false otherwise
  virtual bool write_data_(const uint8_t *buffer, size_t len) = 0;

  /// @brief return the size of the fifo
  virtual const size_t fifo_size_() = 0;

  void uart_send_test(char *preamble);
  void uart_receive_test(char *preamnle, bool print_buf = true);

  bool flush_requested_{false};  ///< flush was requested but not honored
  RingBuffer receive_buffer_;    ///< The receive buffer
};

}  // namespace uart_base
}  // namespace esphome
