/// @file gen_uart.cpp
/// @author @DrCoolzic
/// @brief external uart implementation

/*! @mainpage UARTBase documentation
Gives some information about the details of the implementation of
the UARTBase class.
*/

#include "uart_base.h"

namespace esphome {
namespace sc16is75x_spi {

static const char *const TAG = "uart_base";

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
  // there are bytes in the fifo, in which case we read them
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
  // here we just record the fact thar flush was requested but we do not wait
  // at this time. In the next write_array() call we will check that everything
  // is gone otherwise we will wait. This gives time for the bytes to go.
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
void UARTBase::uart_send_test(char *message) {
  auto start_exec = micros();
  uint8_t to_send = this->fifo_size_() - tx_in_fifo_();
  this->flush();  // we wait until they are gone

  if (to_send > 0) {
    std::vector<uint8_t> output_buffer(to_send);
    generate(output_buffer.begin(), output_buffer.end(), Increment());  // fill with incrementing number
    this->write_array(&output_buffer[0], to_send);                      // we send the buffer
    //    ESP_LOGI(TAG, "%s => sending %d bytes - exec time %d ms ...", message, to_send, millis() - start_exec);
    ESP_LOGI(TAG, "%s => sending %d bytes - exec time %d µs ...", message, to_send, micros() - start_exec);
  }
}

/// @brief test read_array method
void UARTBase::uart_receive_test(char *message) {
  auto start_exec = micros();
  bool status = true;
  uint8_t to_read = rx_in_fifo_();
  if (to_read < this->fifo_size_())
    ESP_LOGI(TAG, "%s => %d bytes received expected %d ...", message, to_read, this->fifo_size_());

  if (to_read > 0) {
    std::vector<uint8_t> buffer(to_read);
    status = read_array(&buffer[0], to_read);
    for (int i = 0; i < to_read; i++) {
      if (buffer[i] != i) {
        ESP_LOGE(TAG, "%s => invalid bytes received...");
        print_buffer(buffer);
        break;
      }
    }
  }
  ESP_LOGI(TAG, "%s => %d bytes received status %s - exec time %d µs ...", message, to_read, status ? "OK" : "ERROR",
           micros() - start_exec);
}
#endif

}  // namespace sc16is75x_spi
}  // namespace esphome
