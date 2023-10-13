/// @file spi2uart.h
/// @author @DrCoolzic
/// @brief spi2uart interface declaration

#pragma once
#include <bitset>
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace spi2uart {

#define TEST_COMPONENT

// size of the fifo
constexpr size_t FIFO_SIZE = 128;

// SPI2UART registers
constexpr uint8_t SPI2UART_REG_RXN = 0x10;  ///< ro reads number of bytes in the channel receive buffer
constexpr uint8_t SPI2UART_REG_RXD = 0X20;  ///< ro reads data byte(s) from the channel receive buffer
constexpr uint8_t SPI2UART_REG_TXN = 0X30;  ///< ro reads number of bytes in the channel receive buffer
constexpr uint8_t SPI2UART_REG_TXD = 0X40;  ///< wo writes data byte into the channel transmit buffer
constexpr uint8_t SPI2UART_REG_BDR = 0X80;  ///< wo sets the channel baud rate

enum SPI2UARTComponentModel { SC16IS750_MODEL, SC16IS752_MODEL };  ///< chip models

class SPI2UARTChannel;    ///< forward declaration
using Channel = uint8_t;  ///< Channel definition

///////////////////////////////////////////////////////////////////////////////
/// @brief This class describes a SPI2UART_Component.
///
/// This class derives from two @ref esphome classes:
/// - The Virtual @ref Component class.
/// - The @ref spi::SPIDevice class.
///
/// We have two related class :
/// - The @ref SPI2UARTChannel class that takes cares of the UART related methods
/// - The @ref SPI2UARTGPIOPin class
///   that takes care of the details for the GPIO pins of the component.
///////////////////////////////////////////////////////////////////////////////
class SPI2UART_Component : public Component,
                           public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                 spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_75KHZ> {
 public:
  void set_test_mode(int test_mode) { this->test_mode_ = test_mode; }
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
  friend class SPI2UARTChannel;

  std::vector<SPI2UARTChannel *> children_{};  ///< the list of SPI2UARTChannel UART children
  int test_mode_;                              ///< test_mode value (0 no test)
  std::string name_;                           ///< store name of entity
  bool initialized_{false};                    ///< true when component initialized
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
class SPI2UARTChannel : uart::UARTComponent {
 public:
  // SPI2UARTChannel() : UARTBase(FIFO_SIZE) {}
  void set_parent(SPI2UART_Component *parent) {
    this->parent_ = parent;
    this->parent_->children_.push_back(this);  // add ourself to the vector list
  }
  void set_channel(Channel channel) { this->channel_ = channel; }
  void set_baud_rate(uint32_t baud_rate) { this->baud_rate_ = baud_rate; }
  void set_channel_name(std::string name) { this->name_ = std::move(name); }
  const char *get_channel_name() { return this->name_.c_str(); }

  /// @brief Writes a specified number of bytes toward a serial port
  /// @param buffer pointer to the buffer
  /// @param length number of bytes to write
  ///
  /// This method sends 'length' characters from the buffer to the serial line.
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
  ///   write_array(&buffer, length);
  ///   flush();
  ///   // ...
  /// @endcode
  void write_array(const uint8_t *buffer, size_t length) override;

  /// @brief Reads a specified number of bytes from a serial port
  /// @param buffer buffer to store the bytes
  /// @param length number of bytes to read
  /// @return true if succeed, false otherwise
  ///
  /// Typical usage:
  /// @code
  ///   // ...
  ///   auto length = available();
  ///   uint8_t buffer[128];
  ///   if (length > 0) {
  ///     auto status = read_array(&buffer, length)
  ///     // test status ...
  ///   }
  /// @endcode
  bool read_array(uint8_t *buffer, size_t length) override;

  /// @brief Reads first byte in FIFO without removing it
  /// @param buffer pointer to the byte
  /// @return true if succeed reading one byte, false if no character available
  ///
  /// This method returns the next byte from receiving buffer without
  /// removing it from the internal fifo. It returns true if a character
  /// is available and has been read, false otherwise.\n
  bool peek_byte(uint8_t *buffer) override { return false; }  // TODO

  /// @brief Returns the number of bytes in the receive buffer
  /// @return the number of bytes available in the receiver fifo
  int available() override { return this->rx_in_fifo_(); }

  /// @brief Flush the output fifo.
  ///
  /// If we refer to Serial.flush() in Arduino it says: ** Waits for the transmission
  /// of outgoing serial data to complete. (Prior to Arduino 1.0, this the method was
  /// removing any buffered incoming serial data.). **
  void flush() override;

 protected:
  friend class SPI2UART_Component;

  /// @brief this cannot happen with external uart
  void check_logger_conflict() override {}

  void setup_channel_();
  void dump_channel_();

  /// @brief write data in the specified register
  /// @param reg_address the register address
  /// @param channel the channel number
  /// @param data to write
  void write_register_(uint8_t reg_address, const uint8_t data);

  /// @brief read data from specified register
  /// @param reg_address the register address
  /// @param channel the channel number. Only significant for UART registers
  uint8_t read_register_(uint8_t reg_address);

  /// @brief returns the number of bytes currently in the receiver fifo
  /// @return the number of bytes
  size_t rx_in_fifo_() { return this->read_register_(SPI2UART_REG_RXN); }

  /// @brief returns the number of bytes currently in the transmitter fifo
  /// @return the number of bytes
  size_t tx_in_fifo_() { return this->read_register_(SPI2UART_REG_TXN); }

#ifdef TEST_COMPONENT
  // for test
  void uart_send_test_(char *message);
  bool uart_receive_test_(char *message);
#endif

  SPI2UART_Component *parent_;  ///< our parent
  Channel channel_;             ///< our channel number
  std::string name_;            ///< name of the entity
};

}  // namespace spi2uart
}  // namespace esphome
