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

// ///////////////////////////////
// // The SC16IS752Channel methods
// ///////////////////////////////

// void SC16IS752Channel::write_array(const uint8_t *data, size_t len) {}
// bool SC16IS752Channel::peek_byte(uint8_t *data) { return true; }
// bool SC16IS752Channel::read_array(uint8_t *data, size_t len) { return true; }
// int SC16IS752Channel::available() { return 1; }
// void SC16IS752Channel::flush() {}
// void SC16IS752Channel::check_logger_conflict() {}

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
