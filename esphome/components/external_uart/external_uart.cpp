/// @file external_uart.cpp
/// @author @DrCoolzic
/// @brief external uart implementation

#include "external_uart.h"

namespace esphome {
namespace external_uart {

static const char *const TAG = "external_uart";

/*! @page page_external_uart_bus_ ExternalUARTComponent documentation
This page gives some information about the details of implementation of
the ExternalUARTComponent class for ESPHome.

  @section external_uart_bus_ ExternalUARTComponent (UART) class
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
// The ExternalUARTComponent methods
///////////////////////////////////////////////////////////////////////////////

bool ExternalUARTComponent::read_array(uint8_t *buffer, size_t len) {
  if (len > fifo_size()) {
    ESP_LOGE(TAG, "Read buffer invalid call: requested %d bytes fifo size %d ...", len, fifo_size());
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
  while (rx_available() < len) {
    // we wait as much as 100 ms
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "Read buffer underrun: requested %d bytes only received %d ...", len, rx_available());
      len = rx_available();  // set length to what is in the buffer
      status = false;
      break;
    }
    yield();
  }
  read_data(buffer, len);
  return status;
}

bool ExternalUARTComponent::peek_byte(uint8_t *buffer) {
  if (peek_buffer_.empty && available() == 0)
    return false;
  if (peek_buffer_.empty) {
    peek_buffer_.empty = false;
    read_data(&peek_buffer_.data, 1);
  }
  *buffer = peek_buffer_.data;
  return true;
}

void ExternalUARTComponent::write_array(const uint8_t *buffer, size_t len) {
  if (len > tx_available()) {
    ESP_LOGE(TAG, "Write buffer overrun: requested %d can only send %d bytes ...", len, tx_available());
    len = tx_available();  // send as much as possible
  }
  write_data(buffer, len);
}

void ExternalUARTComponent::flush() {
  uint32_t start_time = millis();
  while (tx_available() < fifo_size()) {  // we wait for buffer to refill
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "Flush timed out: still %d bytes not sent...", fifo_size() - tx_available());
      return;
    }
    yield();  // I suppose this func reschedule thread to avoid blocking?
  }
}

/// @brief test the uart send/receive methods
/// - here we assume the user has connected all the rx pins to tx pins
/// @param safe
/// - true: how things should be if we had a function to know space
///   available in the transmit buffer.
/// - false: we call blindly the write function and this may result in an
///   eventual buffer overrun. This allow to test how error are catch in
///   write_array method.
void ExternalUARTComponent::test_uart_(bool safe) {
  class Increment {  // A increment functor (A class object that acts like a function with state)
   public:
    Increment() : i_(0) {}
    uint8_t operator()() { return i_++; }

   private:
    uint8_t i_;
  };

  // test the write_array method
  auto start_exec = millis();
  auto available = this->tx_available();
  if (!safe)
    available = fifo_size();                              // might result in overrun
  if (available > 0) {
    std::vector<uint8_t> buffer(available);               // set buffer size to available
    generate(buffer.begin(), buffer.end(), Increment());  // fill with incrementing number
    this->write_array(&buffer[0], available);
    this->flush();                                        // give time to recover
    ESP_LOGI(TAG, "sending %d char - exec time %d ms ...", available, millis() - start_exec);
  }

  // test read_array method - we try to get as much as we can
  start_exec = millis();
  available = this->rx_available();
  if (available > 0) {
    std::vector<uint8_t> buffer(available);
    char status[32];
    snprintf(status, sizeof(status), "%s", this->read_array(&buffer[0], available) ? "OK" : "ERROR");
    ESP_LOGI(TAG, "received %d char %s - exec time %d ms ...", available, status, millis() - start_exec);
    // quick and ugly hex converter to display buffer in hex format
    char hex_buffer[64];
    hex_buffer[50] = 0;
    for (size_t i = 0; i < available; i++) {
      snprintf(&hex_buffer[3 * (i % 16)], sizeof(hex_buffer), "%02X ", buffer[i]);
      if (i % 16 == 15)
        ESP_LOGI(TAG, "   %s", hex_buffer);
    }
    if (available % 16) {
      hex_buffer[3 * (available % 16) + 2] = 0;
      ESP_LOGI(TAG, "   %s", hex_buffer);
    }
  }
}

}  // namespace external_uart
}  // namespace esphome
