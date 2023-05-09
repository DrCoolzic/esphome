import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_OUTPUT,
    CONF_CHANNEL,
    CONF_CHANNELS,
)
CODEOWNERS = ["@xxx"]

DEPENDENCIES = ["i2c", "uart"]

sc16is752_ns = cg.esphome_ns.namespace("sc16is752")
SC16IS752Component = sc16is752_ns.class_("SC16IS752Component", cg.Component, i2c.I2CDevice)
SC16IS752Channel = sc16is752_ns.class_("SC16IS752Channel", uart.UARTDevice)
SC16IS752GPIOPin = sc16is752_ns.class_(
    "SC16IS752GPIOPin", cg.GPIOPin, cg.Parented.template(SC16IS752Component)
)

CONF_SC16IS752 = "sc16is752"
MULTI_CONF = True

CONF_BUS_ID = "bus_id"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SC16IS752Component),
            cv.Optional(CONF_CHANNELS, default=[]): cv.ensure_list(
                {
                    cv.Required(CONF_BUS_ID): cv.declare_id(SC16IS752Channel),
                    cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=2),
                }
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x90))
    .extend(cv.COMPONENT_SCHEMA)
)

def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value

SC16IS752_PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(SC16IS752GPIOPin),
        cv.Required(CONF_SC16IS752): cv.use_id(SC16IS752Component),
        cv.Required(CONF_NUMBER): cv.int_range(min=0, max=8),
        cv.Optional(CONF_MODE, default={}): cv.All(
            {
                cv.Optional(CONF_INPUT, default=False): cv.boolean,
                cv.Optional(CONF_OUTPUT, default=False): cv.boolean,
            },
            validate_mode,
        ),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)

@pins.PIN_SCHEMA_REGISTRY.register("sc16is752", SC16IS752_PIN_SCHEMA)
async def sc16is752_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_SC16IS752])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for conf in config[CONF_CHANNELS]:
        chan = cg.new_Pvariable(conf[CONF_BUS_ID])
        cg.add(chan.set_parent(var))
        cg.add(chan.set_channel(conf[CONF_CHANNEL]))
