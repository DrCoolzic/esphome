#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

////////////////////////////////////////////////////////
// The SC16IS752Component methods
////////////////////////////////////////////////////////
bool SC16IS752Component::check_model() {
  // we test the scratchpad reg for channel 1 if succeed then it is a 752
  reg_buffer = 0xAA;
  write_register(subaddress(SC16IS752_REG_SPR, 1), &reg_buffer, 1);
  if ((read_register(subaddress(SC16IS752_REG_SPR, 1), &reg_buffer, 1) == 0xAA) && (model_ == SC16IS752_MODEL))
    return true;
  else
    return false;
}

//
// overloaded functions from Component
//
void SC16IS752Component::setup() {
  std::string model_name = model_ == SC16IS750_MODEL ? "SC16IS750" : "SC16IS752";
  ESP_LOGCONFIG(TAG, "Setting up %s...", model_name);
  // we read anything just to test communication
  if (this->read(&reg_buffer, 1) != i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "%s failed", model_name);
    this->mark_failed();
    return;
  }
  if (!check_model())
    ESP_LOGCONFIG(TAG, "wrong model %s specified?", model_name);
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
void SC16IS752Component::pin_mode(uint8_t pin, gpio::Flags flags) {}
bool SC16IS752Component::digital_read(uint8_t pin) { return true; }
void SC16IS752Component::digital_write(uint8_t pin, bool value) {}

///////////////////////////////////////////////////////////
// The SC16IS752GPIOPin methods
///////////////////////////////////////////////////////////
void SC16IS752GPIOPin::setup() {}
void SC16IS752GPIOPin::pin_mode(gpio::Flags flags) {}
bool SC16IS752GPIOPin::digital_read() { return true; }
void SC16IS752GPIOPin::digital_write(bool value) {}
std::string SC16IS752GPIOPin::dump_summary() const { return "TODO"; }

}  // namespace sc16is752
}  // namespace esphome
