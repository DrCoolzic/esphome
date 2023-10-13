/// @file spi2uart.cpp
/// @author @DrCoolzic
/// @brief spi2uart implementation

/*! @mainpage SPI2UART documentation
Gives some information about the details of the implementation of
the SPI2UART related classes.
*/

#include "spi2uart.h"

namespace esphome {
namespace spi2uart {

static const char *const TAG = "spi2uart";

// // convert an int to binary string
// inline std::string i2s(uint8_t val) { return std::bitset<8>(val).to_string(); }
// #define I2CS(val) (i2s(val).c_str())

/// @brief measure the time elapsed between two calls
/// @param last_time time of the previous call
/// @return the elapsed time in microseconds
uint32_t elapsed(uint32_t &last_time) {
  uint32_t e = millis() - last_time;
  last_time = millis();
  return e;
};

// the address on the bus is build from the register value and the channel value.
inline static uint8_t address_on_bus(uint8_t reg_number, Channel channel) { return (reg_number | channel); }

///////////////////////////////////////////////////////////////////////////////
// The SPI2UART_Component methods
///////////////////////////////////////////////////////////////////////////////
//
// overloaded methods from Component
//
void SPI2UART_Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPI2UART:%s with %d UARTs...", this->get_name(), this->children_.size());
  this->spi_setup();
  // setup our children
  for (auto child : this->children_)
    child->setup_channel_();
}

void SPI2UART_Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SPI2UART:%s with %d UARTs...", this->get_name(), this->children_.size());
  LOG_PIN("  CS Pin: ", this->cs_);
  for (auto child : this->children_)
    child->dump_channel_();
  this->initialized_ = true;
}

///////////////////////////////////////////////////////////////////////////////
// The SPI2UARTChannel methods
///////////////////////////////////////////////////////////////////////////////
void SPI2UARTChannel::setup_channel_() {
  ESP_LOGCONFIG(TAG, "  Setting up UART %s:%s...", this->parent_->get_name(), this->get_channel_name());
  switch (this->baud_rate_) {
    case 1200:
      write_register_(SPI2UART_REG_BDR, 0);
      break;
    case 2400:
      write_register_(SPI2UART_REG_BDR, 1);
      break;
    case 4800:
      write_register_(SPI2UART_REG_BDR, 2);
      break;
    case 9600:
      write_register_(SPI2UART_REG_BDR, 3);
      break;
    case 19200:
      write_register_(SPI2UART_REG_BDR, 4);
      break;
    case 38400:
      write_register_(SPI2UART_REG_BDR, 5);
      break;
    case 57600:
      write_register_(SPI2UART_REG_BDR, 6);
      break;
    case 115200:
      write_register_(SPI2UART_REG_BDR, 7);
      break;
    case 31250:
      write_register_(SPI2UART_REG_BDR, 8);
      break;
    case 62500:
      write_register_(SPI2UART_REG_BDR, 9);
      break;
    default:
      ESP_LOGE(TAG, "invalid baud rate UART %s:%d %d Bd", this->parent_->get_name(), this->channel_, this->baud_rate_);
      return;
  }
  ESP_LOGV(TAG, "UART %s:%d %d Bd", this->parent_->get_name(), this->channel_, this->baud_rate_);
}

void SPI2UARTChannel::dump_channel_() {
  ESP_LOGCONFIG(TAG, "  UART bus %s:%s...", this->parent_->get_name(), this->get_channel_name());
  ESP_LOGCONFIG(TAG, "    baudrate %d Bd", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "    data_bits 8");
  ESP_LOGCONFIG(TAG, "    stop_bits 1");
  ESP_LOGCONFIG(TAG, "    no parity");

  // just in case we flush rx fifo
  size_t to_flush = this->rx_in_fifo_();
  if (to_flush) {
    uint8_t buf[256]{0};
    auto aob = address_on_bus(SPI2UART_REG_RXD, channel_);
    ESP_LOGE(TAG, "    need to flush %d bytes in rx_fifo!", to_flush);
    this->parent_->enable();
    this->parent_->write_byte(aob);
    this->parent_->write_byte(to_flush);  // ??? strange but seems necessary
    this->parent_->read_array(buf, to_flush);
    this->parent_->disable();
  }
  // just in case we flush tx fifo
  to_flush = this->tx_in_fifo_();
  if (to_flush) {
    auto aob = address_on_bus(SPI2UART_REG_TXD, channel_);
    ESP_LOGE(TAG, "    need to flush %d bytes in tx_fifo!", to_flush);
    this->parent_->enable();
    this->parent_->write_byte(aob);
    this->parent_->write_byte(1);  // ??? strange but seems necessary
    this->parent_->write_byte(0);  // dummy
    this->parent_->disable();
  }
}

void SPI2UARTChannel::write_array(const uint8_t *buffer, size_t length) {
  if (length > FIFO_SIZE) {
    ESP_LOGE(TAG, "write_array() invalid call: requested %d bytes max size %d ...", length, FIFO_SIZE);
    length = FIFO_SIZE;
  }

  uint32_t const start_time = millis();
  // we wait until we have enough space in fifo
  while (length > (FIFO_SIZE - this->tx_in_fifo_())) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "write_array() overrun: %d bytes in fifo...", this->tx_in_fifo_());
      break;
    }
    yield();  // reschedule our thread to avoid blocking
  }
  auto aob = address_on_bus(SPI2UART_REG_TXD, channel_);
  this->parent_->enable();
  this->parent_->write_byte(aob);
  this->parent_->write_byte(length);  // ??? strange but seems necessary
  this->parent_->write_array(buffer, length);
  this->parent_->disable();
  ESP_LOGVV(TAG, "write_array(): a=%02X, b=%02X, l=%d", aob, *buffer, length);
}

void SPI2UARTChannel::flush() {
  uint32_t const start_time = millis();
  // we wait until fifo is empty
  while (this->tx_in_fifo_()) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "flush() timed out: still %d bytes not sent...", this->tx_in_fifo_());
      break;
    }
    yield();  // reschedule our thread to avoid blocking
  }
  ESP_LOGVV(TAG, "flush(): flushed bytes in tx fifo");
}

bool SPI2UARTChannel::read_array(uint8_t *buffer, size_t length) {
  if (length > FIFO_SIZE) {
    ESP_LOGE(TAG, "read_array() invalid call: requested %d bytes max size %d ...", length, FIFO_SIZE);
    return false;
  }

  // we wait until we have received the requested bytes
  uint32_t const start_time = millis();
  while (length > this->rx_in_fifo_()) {
    if (millis() - start_time > 100) {
      ESP_LOGE(TAG, "read_array() underrun: %d bytes requested %d received", length, this->rx_in_fifo_());
      return false;
    }
    yield();  // reschedule our thread to avoid blocking
  }

  auto aob = address_on_bus(SPI2UART_REG_RXD, channel_);
  this->parent_->enable();
  this->parent_->write_byte(aob);
  this->parent_->write_byte(length);  // ??? strange but seems necessary
  this->parent_->read_array(buffer, length);
  this->parent_->disable();
  ESP_LOGV(TAG, "read_array: a=%02X, b=%02X, length=%d", aob, *buffer, length);
  return true;
}

void SPI2UARTChannel::write_register_(uint8_t reg, const uint8_t data) {
  auto aob = address_on_bus(reg, channel_);
  this->parent_->enable();
  this->parent_->write_byte(aob);
  this->parent_->write_byte(data);
  this->parent_->disable();
  ESP_LOGVV(TAG, "write_register_(): a=%02X, b=%02X", aob, data);
}

uint8_t SPI2UARTChannel::read_register_(uint8_t reg) {
  auto aob = address_on_bus(reg, channel_);
  this->parent_->enable();
  this->parent_->write_byte(aob);
  // const uint8_t value(this->parent_->read_byte());
  uint8_t value = this->parent_->read_byte();
  this->parent_->disable();
  ESP_LOGVV(TAG, "read_register_(): a=%02X, b=%02X", aob, value);
  return value;
}

void SPI2UART_Component::loop() {
  if (!this->initialized_)
    return;

  static uint32_t loop_time = 0;
  static uint32_t loop_count = 0;
  uint32_t time = 0;

  if (test_mode_) {
    ESP_LOGV(TAG, "Component loop %d for %s : %d ms since last call ...", loop_count, this->get_name(),
             millis() - loop_time);
  }
  loop_time = millis();
  loop_count++;

#ifdef TEST_COMPONENT
  if (test_mode_ == 1) {
    char message[64];
    elapsed(time);  // set time to now
    for (auto child : this->children_) {
      snprintf(message, sizeof(message), "%s:%s", this->get_name(), child->get_channel_name());
      child->uart_send_test_(message);
      ESP_LOGV(TAG, "uart_send_test - execution time %d ms...", elapsed(time));
      uint32_t const start_time = millis();
      while (child->rx_in_fifo_() < FIFO_SIZE) {  // wait until buffer empty
        if (millis() - start_time > 100) {
          ESP_LOGE(TAG, "Timed out waiting for bytes - %d bytes in buffer...", child->rx_in_fifo_());
          break;
        }
        yield();  // reschedule our thread to avoid blocking
      }
      bool status = child->uart_receive_test_(message);
      ESP_LOGI(TAG, "Loop %d send/received on %s %d bytes %s", loop_count, message, FIFO_SIZE,
               status ? "correctly" : "with error");
      ESP_LOGV(TAG, "uart_receive_test - execution time %d ms...", elapsed(time));
    }
  }

  ESP_LOGV(TAG, "loop execution time %d ms...", millis() - loop_time);
}
#endif

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
void SPI2UARTChannel::uart_send_test_(char *message) {
  auto start_exec = micros();
  size_t to_send = FIFO_SIZE;
  this->flush();  // we wait until they are gone

  if (to_send > 0) {
    std::vector<uint8_t> output_buffer(to_send);
    generate(output_buffer.begin(), output_buffer.end(), Increment());  // fill with incrementing number
    this->write_array(&output_buffer[0], to_send);                      // we send the buffer
    ESP_LOGV(TAG, "%s => sending %d bytes - exec time %d µs ...", message, to_send, micros() - start_exec);
  }
}

/// @brief test read_array method
bool SPI2UARTChannel::uart_receive_test_(char *message) {
  auto start_exec = micros();
  bool status = true;
  uint8_t to_read = rx_in_fifo_();

  if (to_read > 0) {
    std::vector<uint8_t> buffer(to_read);
    status = read_array(&buffer[0], to_read);
    for (int i = 0; i < to_read; i++) {
      if (buffer[i] != i) {
        ESP_LOGE(TAG, "Read buffer for %s contains error...", message);
        print_buffer(buffer);
        status = false;
        size_t to_flush = this->rx_in_fifo_();
        if (to_flush) {
          uint8_t buf[256]{0};
          auto aob = address_on_bus(SPI2UART_REG_RXD, channel_);
          ESP_LOGV(TAG, "need to flush %d bytes in rx_fifo", to_flush);
          this->parent_->enable();
          this->parent_->write_byte(aob);
          this->parent_->write_byte(to_flush);  // ??? strange but seems necessary
          this->parent_->read_array(buf, to_flush);
          this->parent_->disable();
        }
        break;
      }
    }
  }
  if (to_read < FIFO_SIZE) {
    ESP_LOGE(TAG, "%s => %d bytes received expected %d ...", message, to_read, FIFO_SIZE);
    status = false;
  }
  ESP_LOGV(TAG, "%s => %d bytes received status %s - exec time %d µs ...", message, to_read, status ? "OK" : "ERROR",
           micros() - start_exec);
  return status;
}
#endif

}  // namespace spi2uart
}  // namespace esphome
