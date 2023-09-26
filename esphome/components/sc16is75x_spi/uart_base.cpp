/// @file gen_uart.cpp
/// @author @DrCoolzic
/// @brief external uart implementation

#include "uart_base.h"

namespace esphome {
namespace uart_base {

static const char *const TAG = "uart_base";

/*! @page page_gen_uart_bus_ GenUARTChannel documentation
This page gives some information about the details of the implementation of
the GenUARTChannel class for ESPHome.

  @section gen_uart_bus_ GenUARTChannel (UART) class
Unfortunately I have not found any documentation about the uart::UARTDevice and
uart::UARTComponent classes of @ref ESPHome.
@n However it seems that both of them are based on equivalent in Arduino library.\n

Most of the interfaces provided by the Arduino Serial library are **poorly
defined** and it seems that the API has even \b changed over time!\n
The esphome::uart::UARTDevice class directly relates to the **Serial Class**
in Arduino and derives from **Stream class**.\n
For compatibility reason (?) many helper methods are made available in ESPHome to
read and write. Unfortunately in many cases these helpers are missing the critical
status information and therefore even more unsafe to use...\n

 @subsection ra_ss_ bool read_array(uint8_t *buffer, size_t len);

This method receives 'len' characters from the uart and transfer them into
a buffer. It returns
- true if requested number of characters have been transferred,
- false if we have a timeout condition\n

Note: If the characters requested are available in the fifo we read them otherwise
we wait up to 100 ms to get them. To avoid problems it is highly recommended to call
read() with the length set to the number of bytes returned by available()

Typical usage:
@code
  // ...
  auto len = available();
  uint8_t buffer[64];
  if (len > 0) {
    auto status = read_array(&buffer, len)
  }
  // test status ...
@endcode

 @subsection pb_ss_ bool peek_byte(uint8_t *buffer);

This method returns the next byte from incoming serial line without
removing it from the internal fifo. It returns: true if a character
is available and has been read, false otherwise.\n

 @subsection wa_ss_ void write_array(uint8_t *buffer, size_t len);

This method sends 'len' characters from the buffer to the serial line.
Unfortunately (unlike the Arduino equivalent) this method
does not return any value and therefore it is not possible
to know if the bytes has been transmitted correctly.
Another problem is that it is not possible to know how many bytes we
can safely send as there is no tx_available() method provided!
To avoid overrun when using write use flush() to wait until fifo is
empty.

Typical usage could be:
@code
  // ...
  uint8_t buffer[64];
  // ...
  flush();
  write_array(&buffer, len);
  // ...
@endcode

 @subsection fl_ss_ bool flush();

If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed
any buffered incoming serial data.). **

The method waits until all characters inside the fifo have been sent.
Timeout  after 100 ms

Typical usage see @ref wa_ss_

*/

///////////////////////////////////////////////////////////////////////////////
// The UARTBase methods
///////////////////////////////////////////////////////////////////////////////
void UARTBase::write_array(const uint8_t *buffer, size_t len) {
  if (len > this->fifo_size_()) {
    ESP_LOGE(TAG, "Write buffer invalid call: requested %d bytes max size %d ...", len, this->fifo_size_());
    len = this->fifo_size_();
  }

  // if we had a flush request it is time to check it has been honored
  if (this->flush_requested_) {
    if (this->tx_fifo_is_not_empty_()) {
      // if flush requested and fifo is not empty we have to wait until it gets empty
      uint32_t const start_time = millis();
      while (this->tx_fifo_is_not_empty_()) {  // wait until buffer empty
        if (millis() - start_time > 100) {
          ESP_LOGE(TAG, "Flush timed out: still %d bytes not sent...", this->tx_in_fifo_());
          return;
        }
        yield();  // reschedule our thread to avoid blocking
      }
    }
    this->flush_requested_ = false;  // we are all set
  }
  this->write_data_(buffer, len);  // send the buffer
}

bool UARTBase::read_array(uint8_t *buffer, size_t len) {
  bool status = true;
  auto available = this->receive_buffer_.count();

  // If we do not have any bytes in buffer we want to check if
  // there are bytes in the fifo,in which case we read them
  // immediately so we do not delay until the next loop transfer.
  if (!available)
    available = this->rx_fifo_to_buffer_();

  if ((len > this->fifo_size_()) || (len > available)) {
    ESP_LOGVV(TAG, "read_array buffer underflow requested %d bytes available %d ...", len, available);
    len = available;
    status = false;  // invalid call or not enough char
  }
  // retrieve the bytes from ring buffer
  for (size_t i = 0; i < len; i++) {
    this->receive_buffer_.pop(buffer[i]);
  }
  return status;
}

inline bool UARTBase::peek_byte(uint8_t *buffer) { return this->receive_buffer_.peek(*buffer); }

int UARTBase::available() {
  auto available = this->receive_buffer_.count();

  // here if we do not have bytes in buffer we want to check if
  // there are bytes in the fifo,in which case we do not want to
  // delay reading them in the next loop.
  if (!available)
    available = this->rx_fifo_to_buffer_();

  return available;
}

void UARTBase::flush() {
  // here we just record the request but we do not wait at this time.
  // Tje next write_array() call will first check that everything
  // is gone otherwise it will wait. This gives time for the bytes
  // to go
  this->flush_requested_ = true;
}

size_t UARTBase::rx_fifo_to_buffer_() {
  // we look if some characters has been received in the fifo
  auto to_transfer = this->rx_in_fifo_();
  if (to_transfer) {
    uint8_t data[to_transfer];
    this->read_data_(data, to_transfer);
    auto free = this->receive_buffer_.free();
    if (to_transfer > free) {
      ESP_LOGV(TAG, "Ring buffer overrun --> requested %d available %d", to_transfer, free);
      to_transfer = free;  // hopefully will do the rest next time
    }
    ESP_LOGV(TAG, "Transferred %d bytes from rx_fifo to buffer ring", to_transfer);
    for (size_t i = 0; i < to_transfer; i++)
      this->receive_buffer_.push(data[i]);
  }
  return to_transfer;
}

///////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS BELOW
///////////////////////////////////////////////////////////////////////////////
#define TEST_COMPONENT
#ifdef TEST_COMPONENT

class Increment {  // A increment "Functor" (A class object that acts like a method with state!)
 public:
  Increment() : i_(0) {}
  uint8_t operator()() { return i_++; }

 private:
  uint8_t i_;
};

void print_buffer(std::vector<uint8_t> buffer) {
  // quick and ugly hex converter to display buffer in hex format
  char hex_buffer[80];
  hex_buffer[50] = 0;
  for (size_t i = 0; i < buffer.size(); i++) {
    snprintf(&hex_buffer[3 * (i % 16)], sizeof(hex_buffer), "%02X ", buffer[i]);
    if (i % 16 == 15)
      ESP_LOGI(TAG, "   %s", hex_buffer);
  }
  if (buffer.size() % 16) {
    // null terminate if incomplete line
    hex_buffer[3 * (buffer.size() % 16) + 2] = 0;
    ESP_LOGI(TAG, "   %s", hex_buffer);
  }
}

/// @brief test the write_array method
void UARTBase::uart_send_test(char *preamble) {
  auto start_exec = millis();
  uint8_t to_send = this->fifo_size_() - tx_in_fifo_();
  uint8_t to_flush = tx_in_fifo_();  // byte in buffer before execution
  this->flush();                     // we wait until they are gone
  uint8_t remains = tx_in_fifo_();   // remaining bytes if not null => flush timeout

  if (to_send > 0) {
    std::vector<uint8_t> output_buffer(to_send);
    generate(output_buffer.begin(), output_buffer.end(), Increment());  // fill with incrementing number
    output_buffer[0] = to_send;                     // we send as the first byte the length of the buffer
    this->write_array(&output_buffer[0], to_send);  // we send the buffer
    ESP_LOGI(TAG, "%s pre flushing %d, remains %d => sending %d bytes - exec time %d ms ...", preamble, to_flush,
             remains, to_send, millis() - start_exec);
  }
}

/// @brief test read_array method
void UARTBase::uart_receive_test(char *preamble, bool print_buf) {
  auto start_exec = millis();
  bool status = true;
  uint8_t to_read = rx_in_fifo_();

  if (to_read > 0) {
    std::vector<uint8_t> buffer(to_read);
    status = read_array(&buffer[0], to_read);
    if (print_buf)
      print_buffer(buffer);
  }
  ESP_LOGI(TAG, "%s => %d bytes received status %s - exec time %d ms ...", preamble, to_read, status ? "OK" : "ERROR",
           millis() - start_exec);
}
#endif

}  // namespace uart_base
}  // namespace esphome
