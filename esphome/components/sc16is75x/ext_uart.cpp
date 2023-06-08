/// @file ext_uart.cpp
/// @author @DrCoolzic
/// @brief external uart implementation

#include "ext_uart.h"

namespace esphome {
namespace ext_uart {

static const char *const TAG = "ext_uart";

/*! @page page_ext_uart_bus_ ExtUARTComponent documentation
This page gives some information about the details of implementation of
the ExtUARTComponent class for ESPHome.

  @section ext_uart_bus_ ExtUARTComponent (UART) class
Unfortunately I have not found any documentation on the uart::UARTDevice and
uart::UARTComponent classes of @ref ESPHome. However it seems that both of
them take their roots from the Arduino library.\n

Most of the interfaces provided by the Arduino Serial library are **poorly
defined** and it seems that the API has even \b changed over time!\n
The esphome::uart::UARTDevice class directly relates to the **Serial Class**
in Arduino and both of them derive from a **Stream class**.\n
For compatibility reason (?) many helper methods are made available in ESPHome,
but unfortunately in some cases they are missing critical status information ...\n

 @subsection ra_ss_ bool read_array(uint8_t *buffer, size_t len);

This method receives 'len' characters from the uart and transfer them into
a buffer. It returns:
- true if requested number of characters have been transferred,
- false if we have a timeout condition\n

Note: If characters requested are available in the fifo we read them otherwise
we wait up to 100 ms to get them. To avoid problem it is recommended to call
read() with the number of bytes returned by available()

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

This method returns the next byte from incoming serial byte without
removing it from the internal fifo. It returns: true if a character
is available and has been read, false otherwise.\n

@subsection wa_ss_ void write_array(uint8_t *buffer, size_t len);

This method sends 'len' characters from the buffer to the serial line.
Unfortunately (unlike the Arduino equivalent) this method
does not return any value and therefore it is not possible
to know if the bytes has been transmitted correctly.
Another problem is that it is not possible to know how many bytes we
can safely send as there is no tx_available method provided!
When writing continuously to avoid overrun use flush()

Typical usage could be:
@code
  // ...
  uint8_t buffer[64];
  // ...
  write_array(&buffer, len);
  flush();
  // ...
@endcode

 @subsection fl_ss_ bool peek_byte(uint8_t *buffer);

If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed
any buffered incoming serial data.). **

The function waits until all characters has been sent.
Timeout  after 100 ms

*/

///////////////////////////////////////////////////////////////////////////////
// The ExtUARTComponent methods
///////////////////////////////////////////////////////////////////////////////

bool ExtUARTComponent::read_array(uint8_t *buffer, size_t len) {
  if (len > fifo_size()) {
    ESP_LOGE(TAG, "Read buffer invalid call: requested %d bytes max size %d ...", len, fifo_size());
    return false;
  }

  if (!peek_buffer_.empty) {  // test peek buffer
    *buffer++ = peek_buffer_.data;
    peek_buffer_.empty = true;
    if (len-- == 1)
      return true;
  }

  bool status = true;
  uint32_t start_time = millis();
  // in safe mode we check that we have received the character
  while (safe_ && rx_in_fifo() < len) {
    // we wait as much as 100 ms
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "Read buffer underrun: requested %d bytes only received %d ...", len, rx_in_fifo());
      len = rx_in_fifo();  // set length to what is in the buffer
      status = false;
      break;
    }
    yield();
  }
  read_data(buffer, len);
  return status;
}

bool ExtUARTComponent::peek_byte(uint8_t *buffer) {
  if (safe_ && peek_buffer_.empty && available() == 0)
    return false;
  if (peek_buffer_.empty) {
    peek_buffer_.empty = false;
    read_data(&peek_buffer_.data, 1);
  }
  *buffer = peek_buffer_.data;
  return true;
}

void ExtUARTComponent::write_array(const uint8_t *buffer, size_t len) {
  if (len > fifo_size()) {
    ESP_LOGE(TAG, "Write buffer invalid call: requested %d bytes max size %d ...", len, fifo_size());
    return;
  }
  if (safe_ && (len > (fifo_size() - tx_in_fifo()))) {
    len = fifo_size() - tx_in_fifo();  // send as much as possible
    ESP_LOGE(TAG, "Write buffer overrun: can only send %d bytes ...", len);
  }
  write_data(buffer, len);
}

void ExtUARTComponent::flush() {
  uint32_t start_time = millis();
  while (tx_in_fifo()) {  // wait until buffer empty
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "Flush timed out: still %d bytes not sent...", fifo_size() - tx_in_fifo());
      return;
    }
    yield();  // I suppose this func reschedule thread to avoid blocking?
  }
}

///////////////////////////////////////////////////////////////////////////////
/// TEST FUNCTIONS BELOW
///////////////////////////////////////////////////////////////////////////////
#define TEST_COMPONENT
#ifdef TEST_COMPONENT

class Increment {  // A increment "Functor" (A class object that acts like a function with state!)
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
void ExtUARTComponent::uart_send_test(uint8_t channel) {
  auto start_exec = millis();
  uint8_t to_send = fifo_size() - tx_in_fifo();

  if (to_send > 0) {
    std::vector<uint8_t> output_buffer(to_send);
    generate(output_buffer.begin(), output_buffer.end(), Increment());  // fill with incrementing number
    output_buffer[0] = to_send;                     // we send as the first byte the length of the buffer
    this->write_array(&output_buffer[0], to_send);  // we send the buffer

    uint8_t to_flush = tx_in_fifo();                // byte in buffer after execution
    this->flush();                                  // we wait until they are gone

    uint8_t remains = tx_in_fifo();                 // remaining bytes if not null => flush timeout
    ESP_LOGI(TAG, "Channel %d: %d bytes transmitted need flushing %d, remains %d - exec time %d ms ...", channel,
             to_send, to_flush, remains, millis() - start_exec);
  }
}

/// @brief test read_array method
void ExtUARTComponent::uart_receive_test(uint8_t channel) {
  auto start_exec = millis();
  bool status = true;
  uint8_t to_read = rx_in_fifo();

  if (to_read > 0) {
    std::vector<uint8_t> buffer(to_read);
    status = read_array(&buffer[0], to_read);
    // print_buffer(buffer);
  }
  ESP_LOGI(TAG, "Channel %d: %d bytes received status %s - exec time %d ms ...", channel, to_read,
           status ? "OK" : "ERROR", millis() - start_exec);
}
#endif

}  // namespace ext_uart
}  // namespace esphome
