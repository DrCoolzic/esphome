#include "sc16is752.h"

namespace esphome {
namespace sc16is752 {

// const uint8_t INPUT_REG = 0;
// const uint8_t OUTPUT_REG = 1;
// const uint8_t INVERT_REG = 2;
// const uint8_t CONFIG_REG = 3;

static const char *const TAG = "sc16is752";

/////////////////////////////////
// The SC16IS752Component methods
/////////////////////////////////
void SC16IS752Component::setup() {}
void SC16IS752Component::dump_config() {}
void SC16IS752Component::update() {}

bool SC16IS752Component::digital_read(uint8_t pin) { return true; }
void SC16IS752Component::digital_write(uint8_t pin, bool value) {}
void SC16IS752Component::pin_mode(uint8_t pin, gpio::Flags flags) {}

// bool SC16IS752Component::read_inputs_() {
//   uint8_t inputs;

//   if (this->is_failed()) {
//     ESP_LOGD(TAG, "Device marked failed");
//     return false;
//   }

//   if ((this->last_error_ = this->read_register(INPUT_REG, &inputs, 1, true)) != esphome::i2c::ERROR_OK) {
//     this->status_set_warning();
//     ESP_LOGE(TAG, "read_register_(): I2C I/O error: %d", (int) this->last_error_);
//     return false;
//   }
//   this->status_clear_warning();
//   this->input_mask_ = inputs;
//   return true;
// }

// bool SC16IS752Component::write_register_(uint8_t reg, uint8_t value) {
//   if ((this->last_error_ = this->write_register(reg, &value, 1, true)) != esphome::i2c::ERROR_OK) {
//     this->status_set_warning();
//     ESP_LOGE(TAG, "write_register_(): I2C I/O error: %d", (int) this->last_error_);
//     return false;
//   }

//   this->status_clear_warning();
//   return true;
// }

///////////////////////////////
// The SC16IS752Channel methods
///////////////////////////////

// void SC16IS752Channel::setup() {}
// void SC16IS752Channel::dump_config() {}
// void SC16IS752Channel::update() {}
// uint32_t SC16IS752Channel::get_config() { return 1; }
void SC16IS752Channel::write_array(const uint8_t *data, size_t len) {}
bool SC16IS752Channel::peek_byte(uint8_t *data) { return true; }
bool SC16IS752Channel::read_array(uint8_t *data, size_t len) { return true; }
int SC16IS752Channel::available() { return 1; }
void SC16IS752Channel::flush() {}
void SC16IS752Channel::check_logger_conflict() {}

///////////////////////////////
// The SC16IS752GPIOPin methods
///////////////////////////////

void SC16IS752GPIOPin::setup() {}
void SC16IS752GPIOPin::pin_mode(gpio::Flags flags) {}
bool SC16IS752GPIOPin::digital_read() { return true; }
void SC16IS752GPIOPin::digital_write(bool value) {}
std::string SC16IS752GPIOPin::dump_summary() const { return "dummy"; }

}  // namespace sc16is752
}  // namespace esphome
