/// @file wk2132.cpp
/// @author @DrCoolzic
/// @brief wk2132 implementation

#include "wk2132.h"

#define ClassicReadMethods

namespace esphome {
namespace wk2132 {

static const char *const TAG = "wk2132";

/*! @page page_wk2132x_ WK2132Component documentation
This page gives some information about the details of implementation of
the WK2132Component class for ESPHome.

@section wk2132_component_ WK2132Component class
This class describes a WK2132 I²C component. It derives from two @ref esphome classes:
- The @ref Virtual Component class. From this class we redefine the @ref Component::setup(),
  @ref Component::dump_config() and @ref Component::get_setup_priority() methods
- The @ref i2c::I2CDevice class. From which we use some methods

We have a related class: @ref WK2132Channel class that takes cares of the UART related functions

All call to the i2c::I2CDevice register read and write are funneled through two functions:
- The WK2132Component::read_wk2132_register_() and
- The WK2132Component::write_wk2132_register_().

  @section wk2132_uart_ WK2132Channel (UART) class

TODO

*/

static const char *reg_to_str_p0[] = {"GENA", "GRST", "GMUT",  "SPAGE", "SCR", "LCR", "FCR",
                                      "SIER", "SIFR", "TFCNT", "RFCNT", "FSR", "LSR", "FDAT"};
static const char *reg_to_str_p1[] = {"GENA", "GRST", "GMUT", "SPAGE", "BAUD1", "BAUD0", "PRES", "RFTL", "TFTL"};

// convert an int to binary string
inline const char *i2s_(uint8_t val) { return std::bitset<8>(val).to_string().c_str(); }

int WK2132Component::count_{0};  // init static count of instances

/// @brief Computes the I²C Address to access the component
/// @param base_address the base address of the component as set by the A1 A0 pins
/// @param channel (0-3) the UART channel
/// @param fifo (0-1) if 0 access to internal register, if 1 direct access to fifo
/// @return the i2c address to use
inline const uint8_t i2c_address(uint8_t base_address, uint8_t channel, uint8_t fifo) {
  // the address of the device is 0AA1 0CCF (eg: 0001 0000) where:
  // - AA is the address read from A1,A0
  // - CC is the channel number (in practice only 00 or 01)
  // - F is 0 when accessing register one when accessing FIFO
  uint8_t addr = base_address | channel << 1 | fifo;
  // ESP_LOGI(TAG, "i2c_address %02X [%s] => b=%02X c=%02X f=%d", addr, i2s_(addr), base_address, channel, fifo);
  return addr;
}

// TODO seems we do not need

/// @brief Computes the register address
/// @param reg_number the register number
/// @param channel the channel number
/// @return the register address
inline const uint8_t reg_address(uint8_t reg_number, uint8_t channel) {
  // - the register address for the Global register is directly equal
  //   to the reg_number. Global reg XXXX XXGG
  // - the register address for the UART channel is 00CC NNNN where
  //   - CC is the channel number (0-1)
  //   - NNNN the register number (3-15)
  uint8_t ra;
  if (reg_number & 0x3 < 3)  // test global register
    ra = reg_number;
  else
    ra = channel << 4 | reg_number;
  // ESP_LOGI(TAG, "reg address %02X [%s] => reg_num=%02X channel=%d", ra, i2s_(ra), reg_number, channel);
  return ra;
}

/// @brief Converts the parity enum to a string
/// @param parity enum
/// @return the string
const char *parity2string(uart::UARTParityOptions parity) {
  using namespace uart;
  switch (parity) {
    case UART_CONFIG_PARITY_NONE:
      return "NONE";
    case UART_CONFIG_PARITY_EVEN:
      return "EVEN";
    case UART_CONFIG_PARITY_ODD:
      return "ODD";
    default:
      return "UNKNOWN";
  }
}

///////////////////////////////////////////////////////////////////////////////
// The WK2132Component methods
///////////////////////////////////////////////////////////////////////////////

// for debug messages ...
inline const char *WK2132Component::reg_to_str(int val) { return page1_ ? reg_to_str_p1[val] : reg_to_str_p0[val]; }

void WK2132Component::write_wk2132_register_(uint8_t reg_number, uint8_t channel, const uint8_t *buffer, size_t len) {
  address_ = i2c_address(base_address_, channel, 0);  // update the i2c address
  auto error = this->write_register(reg_number, buffer, len);
  if (error == i2c::ERROR_OK) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "write_wk2132_register_(@%02X %s, %d b=%02X [%s], len=%d): I2C code %d", address_,
              reg_to_str(reg_number), channel, *buffer, i2s_(*buffer), len, (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "write_wk2132_register_(@%02X %s, %d b=%02X [%s], len=%d): I2C code %d", address_,
             reg_to_str(reg_number), channel, *buffer, i2s_(*buffer), len, (int) error);
  }
}

uint8_t WK2132Component::read_wk2132_register_(uint8_t reg_number, uint8_t channel, uint8_t *buffer, size_t len) {
  address_ = i2c_address(base_address_, channel, 0);  // update the i2c address
  auto error = this->read_register(reg_number, buffer, len);
  if ((error == i2c::ERROR_OK)) {
    this->status_clear_warning();
    ESP_LOGVV(TAG, "read_wk2132_register_(@%02X %s, %d b=%02X [%s], len=%d): I2C code %d", address_,
              reg_to_str(reg_number), channel, *buffer, i2s_(*buffer), len, (int) error);
  } else {  // error
    this->status_set_warning();
    ESP_LOGE(TAG, "read_wk2132_register_(@%02X %s, %d b=%02X [%s], len=%d): I2C code %d", address_,
             reg_to_str(reg_number), channel, *buffer, i2s_(*buffer), len, (int) error);
  }
  return *buffer;
}

//
// overloaded functions from Component
//
void WK2132Component::setup() {
  base_address_ = address_;
  ESP_LOGCONFIG(TAG, "Setting up WK2132:%d @%02X with %d UARTs...", get_num_(), base_address_, (int) children.size());
  // we test communication
  read_wk2132_register_(REG_WK2132_GENA, 0, &data_, 1);

  // we can now setup our children
  for (auto i = 0; i < children.size(); i++)
    children[i]->setup_channel();
}

void WK2132Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Config WK2132:%d with %d UARTs...", get_num_(), (int) children.size());
  ESP_LOGCONFIG(TAG, "  crystal %d", crystal_);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with WK2132 failed!");
  }

  for (auto i = 0; i < children.size(); i++) {
    ESP_LOGCONFIG(TAG, "  UART bus %d:%d...", get_num_(), i);
    ESP_LOGCONFIG(TAG, "    baudrate %d Bd", children[i]->baud_rate_);
    ESP_LOGCONFIG(TAG, "    data_bits %d", children[i]->data_bits_);
    ESP_LOGCONFIG(TAG, "    stop_bits %d", children[i]->stop_bits_);
    ESP_LOGCONFIG(TAG, "    parity %s", parity2string(children[i]->parity_));
  }
}

void WK2132Component::loop() {
  //
  // This loop is used only if the wk2132 component is in test mode
  //
  if (test_mode_ == 0)
    return;
  static int32_t loop_time = 0;
  ESP_LOGI(TAG, "time between loop call %d ms...", millis() - loop_time);
  loop_time = millis();

  switch (test_mode_) {
    case 1:
      children[0]->test_uart_(true);
      break;
  }
  ESP_LOGI(TAG, "loop exec time %d ms...", millis() - loop_time);
}

///////////////////////////////////////////////////////////////////////////////
// The WK2132Channel methods
///////////////////////////////////////////////////////////////////////////////

void WK2132Channel::setup_channel() {
  ESP_LOGCONFIG(TAG, "  Setting up UART %d:%d...", parent_->get_num_(), channel_);

  //  GENA description of global control register:
  //  * -------------------------------------------------------------------------
  //  * |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  //  * -------------------------------------------------------------------------
  //  * |   M1   |   M0   |              RESERVED             |  UT2EN |  UT1EN |
  //  * -------------------------------------------------------------------------
  uint8_t gena;
  parent_->read_wk2132_register_(REG_WK2132_GENA, 0, &gena, 1);
  (this->channel_ == 0) ? gena |= 0x01 : gena |= 0x02;
  parent_->write_wk2132_register_(REG_WK2132_GENA, 0, &gena, 1);

  //  GRST description of global reset register:
  //  * -------------------------------------------------------------------------
  //  * |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  //  * -------------------------------------------------------------------------
  //  * |       RSV       | UT2SLE | UT1SLE |       RSV       | UT2RST | UT1RST |
  //  * -------------------------------------------------------------------------
  // software reset UART channels
  uint8_t grst = 0;
  parent_->read_wk2132_register_(REG_WK2132_GRST, 0, &gena, 1);
  (this->channel_ == 0) ? grst |= 0x01 : grst |= 0x02;
  parent_->write_wk2132_register_(REG_WK2132_GRST, 0, &grst, 1);

  // set page 0
  uint8_t page = 0;
  parent_->page1_ = false;
  parent_->write_wk2132_register_(REG_WK2132_SPAGE, channel_, &page, 1);

  // FCR description of UART FIFO control register:
  // -------------------------------------------------------------------------
  // |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  // -------------------------------------------------------------------------
  // |      TFTRIG     |      RFTRIG     |  TFEN  |  RFEN  |  TFRST |  RFRST |
  // -------------------------------------------------------------------------
  uint8_t fsr = 0x0D;  // 0000 1101
  parent_->write_wk2132_register_(REG_WK2132_FCR, channel_, &fsr, 1);

  // SCR description of UART control register:
  //  -------------------------------------------------------------------------
  //  |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  //  -------------------------------------------------------------------------
  //  |                     RSV                    | SLEEPEN|  TXEN  |  RXEN  |
  //  -------------------------------------------------------------------------
  uint8_t scr = 0x3;  // 0000 0011
  parent_->write_wk2132_register_(REG_WK2132_SCR, channel_, &scr, 1);

  set_baudrate_();
  set_line_param_();
}

void WK2132Channel::set_baudrate_() {
  uint16_t val_int = parent_->crystal_ / (baud_rate_ * 16) - 1;
  uint16_t val_dec = (parent_->crystal_ % (baud_rate_ * 16)) / (baud_rate_ * 16);
  uint8_t baud_high = (uint8_t) (val_int >> 8);
  uint8_t baud_low = (uint8_t) (val_int & 0xFF);
  while (val_dec > 0x0A)
    val_dec /= 0x0A;
  uint8_t baud_dec = (uint8_t) (val_dec);

  uint8_t page = 1;  // switch to page 1
  parent_->write_wk2132_register_(REG_WK2132_SPAGE, channel_, &page, 1);
  parent_->page1_ = true;
  parent_->write_wk2132_register_(REG_WK2132_BRH, channel_, &baud_high, 1);
  parent_->write_wk2132_register_(REG_WK2132_BRL, channel_, &baud_low, 1);
  parent_->write_wk2132_register_(REG_WK2132_BRD, channel_, &baud_dec, 1);
  page = 0;  // switch back to page 0
  parent_->write_wk2132_register_(REG_WK2132_SPAGE, channel_, &page, 1);
  parent_->page1_ = false;

  ESP_LOGCONFIG(TAG, "  Crystal=%d baudrate(h,l,d)=%d (%d,%d,%d)", parent_->crystal_, baud_rate_, baud_high, baud_low,
                baud_dec);
}

void WK2132Channel::set_line_param_() {
  data_bits_ = 8;  // not used - should be 8
  uint8_t lcr;
  parent_->read_wk2132_register_(REG_WK2132_LCR, channel_, &lcr, 1);
  // LCR description of line configuration register:
  //  -------------------------------------------------------------------------
  //  |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  //  -------------------------------------------------------------------------
  //  |        RSV      |  BREAK |  IREN  |  PAEN  |      PAM        |  STPL  |
  //  -------------------------------------------------------------------------
  lcr &= 0xF0;                           // Clear the lower 4 bit of LCR
  if (stop_bits_ == 2)
    lcr |= 0x01;                         // 0001

  switch (parity_) {                     // parity selection settings
    case uart::UART_CONFIG_PARITY_ODD:   // odd parity
      lcr |= 0x5 << 1;                   // 101x
      break;
    case uart::UART_CONFIG_PARITY_EVEN:  // even parity
      lcr |= 0x6 << 1;                   // 110x
      break;
    default:
      break;  // no parity 000x
  }
  parent_->write_wk2132_register_(REG_WK2132_LCR, channel_, &lcr, 1);
  ESP_LOGCONFIG(TAG, "  line set to %d data_bits, %d stop_bits, parity %s reg=%s", data_bits_, stop_bits_,
                parity2string(parity_), i2s_(lcr));
}

size_t WK2132Channel::tx_available() {
  //  * -------------------------------------------------------------------------
  //  * |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
  //  * -------------------------------------------------------------------------
  //  * |  RFOE  |  RFBI  |  RFFE  |  RFPE  |  RDAT  |  TDAT  |  TFULL |  TBUSY |
  //  * -------------------------------------------------------------------------
  uint8_t available = 0;
  uint8_t fsr = parent_->read_wk2132_register_(REG_WK2132_FSR, channel_, &data_, 1);
  if ((fsr & 0x4) == 1) {  // transmit fifo *not* empty
    ESP_LOGE(TAG, "fifo NOT empty!");
    if (fsr & 02 == 1) {   // transmit fifo full
      ESP_LOGE(TAG, "fifo is full!");
      available = fifo_size();
    } else {
      available = fifo_size() - parent_->read_wk2132_register_(REG_WK2132_TFCNT, channel_, &data_, 1);
      ESP_LOGE(TAG, "fifo neither full nor empty - space available %d", available);
    }
  } else {
    ESP_LOGE(TAG, "fifo empty");
    available = fifo_size();
  }
  return available;
}

size_t WK2132Channel::rx_available() {
  uint8_t available = 0;
  uint8_t fsr = parent_->read_wk2132_register_(REG_WK2132_FSR, channel_, &data_, 1);
  if ((fsr & 0x8) == 1)
    uint8_t received = parent_->read_wk2132_register_(REG_WK2132_RFCNT, channel_, &data_, 1);
  ESP_LOGE(TAG, "bytes waiting in fifo %d", available);
  return available;
}

bool WK2132Channel::read_data(uint8_t *buffer, size_t len) {
#ifdef ClassicReadMethods
  parent_->read_wk2132_register_(REG_WK2132_FDA, channel_, buffer, len);
  return true;

#else
  parent_->address_ = i2c_address(parent_->base_address_, channel_, 1);
  auto error = parent_->read(buffer, len);
  if ((error == i2c::ERROR_OK)) {
    parent_->status_clear_warning();
    ESP_LOGVV(TAG, "read_data(ch=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
              (int) error);
    return true;
  } else {  // error
    parent_->status_set_warning();
    ESP_LOGE(TAG, "read_data(ch=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
             (int) error);
    return false;
  }
#endif
}

bool WK2132Channel::write_data(const uint8_t *buffer, size_t len) {
  // TODO test status for full **
  parent_->address_ = i2c_address(parent_->base_address_, channel_, 1);

#ifdef ClassicWriteMethods
  parent_->write_wk2132_register_(REG_WK2132_FDA, channel_, buffer, len);
  return true;

#else
  auto error = parent_->write(buffer, len);
  if ((error == i2c::ERROR_OK)) {
    parent_->status_clear_warning();
    ESP_LOGVV(TAG, "write_data(ch=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
              (int) error);
    return true;
  } else {  // error
    parent_->status_set_warning();
    ESP_LOGE(TAG, "write_data(ch=%d buffer[0]=%02X [%s], len=%d): I2C code %d", channel_, *buffer, i2s_(*buffer), len,
             (int) error);
    return false;
  }
#endif
}

}  // namespace wk2132
}  // namespace esphome
