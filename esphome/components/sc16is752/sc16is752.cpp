#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

////////////////////////////////////////////////////////
// The SC16IS752Component methods
////////////////////////////////////////////////////////
void SC16IS752Component::check_model() {
  if (model_ != UNKNOWN_MODEL)
    return;  // already set by user in configuration
  // we test the scratchpad reg for channel 1 if succeed then it is a 752
  reg_buffer = 0xAA;
  write_register(subaddress(SC16IS752_REG_SPR, 1), &reg_buffer, 1);
  if (read_register(subaddress(SC16IS752_REG_SPR, 1), &reg_buffer, 1) == 0xAA)
    model_ = SC16IS752_MODEL;
  else
    model_ = SC16IS750_MODEL;
}

//
// overloaded functions from Component
//
void SC16IS752Component::setup() {
  check_model();
  std::string model_name = model_ == SC16IS750_MODEL ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s...", model_name);
  uint8_t buffer;
  // we read anything just to test communication
  if (this->read(&buffer, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
    return;
  }
  // crystal on SC16IS750 is 14.7456MHz => max speed 14745600/16 = 921,600bps.
  // crystal on SC16IS752 is 3.072MHz => max speed 3072000/16 = 192,000bps
  if (crystal_ == 0) {
    model_ == SC16IS750_MODEL ? crystal_ = 14745600 : 3072000;
    ESP_LOGCONFIG(TAG, "crystal value %ld", crystal_);
  }
}

void SC16IS752Component::dump_config() {
  ESP_LOGCONFIG(TAG, "%s:", model_);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with %s failed!", model_);
  }
}

void SC16IS752Component::update() {}

//
// GPIO related
//
bool SC16IS752Component::digital_read(uint8_t pin) { return true; }
void SC16IS752Component::digital_write(uint8_t pin, bool value) {}
void SC16IS752Component::pin_mode(uint8_t pin, gpio::Flags flags) {}

///////////////////////////////////////////////////////////
// The SC16IS752GPIOPin methods
///////////////////////////////////////////////////////////
void SC16IS752GPIOPin::setup() {}
void SC16IS752GPIOPin::pin_mode(gpio::Flags flags) {}
bool SC16IS752GPIOPin::digital_read() { return true; }
void SC16IS752GPIOPin::digital_write(bool value) {}
std::string SC16IS752GPIOPin::dump_summary() const { return "dummy"; }

}  // namespace sc16is752
}  // namespace esphome
