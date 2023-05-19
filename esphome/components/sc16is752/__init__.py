import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.components import uart
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_CHANNEL,
    CONF_CHANNELS,
    CONF_ID,
    CONF_INPUT,
    CONF_INVERTED,
    CONF_MODE,
    CONF_NUMBER,
    CONF_OUTPUT,
    CONF_UART_ID,
)

CODEOWNERS = ["@DrCoolZic"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["uart"]

sc16is752_ns = cg.esphome_ns.namespace("sc16is752")
SC16IS752Component = sc16is752_ns.class_(
    "SC16IS752Component", cg.Component, i2c.I2CDevice
)
SC16IS752Channel = sc16is752_ns.class_(
    "SC16IS752Channel", cg.Component, uart.UARTComponent
)
SC16IS752GPIOPin = sc16is752_ns.class_(
    "SC16IS752GPIOPin", cg.GPIOPin, cg.Parented.template(SC16IS752Component)
)

CONF_SC16IS752 = "sc16is752"
MULTI_CONF = True
UARTParityOptions = sc16is752_ns.enum("UARTParityOptions")
UART_PARITY_OPTIONS = {
    "NONE": UARTParityOptions.UART_CONFIG_PARITY_NONE,
    "EVEN": UARTParityOptions.UART_CONFIG_PARITY_EVEN,
    "ODD": UARTParityOptions.UART_CONFIG_PARITY_ODD,
}
CONF_STOP_BITS = "stop_bits"
CONF_DATA_BITS = "data_bits"
CONF_PARITY = "parity"

SC16IS752ComponentModel = sc16is752_ns.enum("SC16IS752ComponentModel")
SC16IS752_MODELS = {
    "SC16IS750": SC16IS752ComponentModel.SC16IS750_MODEL,
    "SC16IS752": SC16IS752ComponentModel.SC16IS752_MODEL,
}
CONF_MODEL = "model"
CONF_CRYSTAL = "crystal"


def post_validate(value):
    channel_count = len(value[CONF_CHANNELS])
    if value[CONF_MODEL] == "SC16IS750":
        if channel_count > 1:
            raise cv.Invalid("SC16IS750 can only have one channel")
        if channel_count > 0 and value[CONF_CHANNELS][0][CONF_CHANNEL] == 1:
            raise cv.Invalid("Only channel 0 is authorized for a SC16IS750")
        if value.get(CONF_CRYSTAL) is None:
            value[CONF_CRYSTAL] = 14745600
    else:  # SC16IS752
        if (
            channel_count > 1
            and value[CONF_CHANNELS][0][CONF_CHANNEL]
            == value[CONF_CHANNELS][1][CONF_CHANNEL]
        ):
            raise cv.Invalid("Duplicate channel number")
        if value.get(CONF_CRYSTAL) is None:
            value[CONF_CRYSTAL] = 3072000
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SC16IS752Component),
            cv.Optional(CONF_MODEL, default="SC16IS752"): cv.enum(
                SC16IS752_MODELS, upper=True
            ),
            cv.Optional(CONF_CRYSTAL): cv.int_,
            cv.Optional(CONF_CHANNELS, default=[]): cv.ensure_list(
                {
                    cv.Required(CONF_UART_ID): cv.declare_id(SC16IS752Channel),
                    cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=1),
                    cv.Required(CONF_BAUD_RATE): cv.int_range(min=1),
                    cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
                    cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=5, max=8),
                    cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                        UART_PARITY_OPTIONS, upper=True
                    ),
                }
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x90))
    .extend(cv.COMPONENT_SCHEMA),
    post_validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_crystal(config[CONF_CRYSTAL]))
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    for conf in config[CONF_CHANNELS]:
        chan = cg.new_Pvariable(conf[CONF_UART_ID])
        cg.add(chan.set_parent(var))
        cg.add(chan.set_channel(conf[CONF_CHANNEL]))
        cg.add(chan.set_baud_rate(conf[CONF_BAUD_RATE]))
        cg.add(chan.set_stop_bits(conf[CONF_STOP_BITS]))
        cg.add(chan.set_data_bits(conf[CONF_DATA_BITS]))
        cg.add(chan.set_parity(conf[CONF_PARITY]))


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
