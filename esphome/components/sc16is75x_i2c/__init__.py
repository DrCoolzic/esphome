import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.components import uart
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_CHANNEL,
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

sc16is75x_i2c_ns = cg.esphome_ns.namespace("sc16is75x_i2c")
SC16IS75XComponent = sc16is75x_i2c_ns.class_(
    "SC16IS75XComponent", cg.Component, i2c.I2CDevice
)
SC16IS75XChannel = sc16is75x_i2c_ns.class_(
    "SC16IS75XChannel", cg.Component, uart.UARTComponent
)
SC16IS75XGPIOPin = sc16is75x_i2c_ns.class_(
    "SC16IS75XGPIOPin", cg.GPIOPin, cg.Parented.template(SC16IS75XComponent)
)

CONF_SC16IS75X = "sc16is75x_i2c"
MULTI_CONF = True
CONF_STOP_BITS = "stop_bits"
CONF_DATA_BITS = "data_bits"
CONF_PARITY = "parity"
CONF_MODEL = "model"
CONF_CRYSTAL = "crystal"
CONF_UART = "uart"
CONF_TEST_MODE = "test_mode"

SC16IS75XComponentModel = sc16is75x_i2c_ns.enum("SC16IS75XComponentModel")
SC16IS75X_MODELS = {
    "SC16IS750": SC16IS75XComponentModel.SC16IS750_MODEL,
    "SC16IS752": SC16IS75XComponentModel.SC16IS752_MODEL,
}


def post_validate(value):
    for uart_elem in value[CONF_UART]:
        if CONF_CHANNEL not in uart_elem:
            uart_elem[CONF_CHANNEL] = 0
    uart_count = len(value[CONF_UART])
    if value[CONF_MODEL] == "SC16IS750":
        if uart_count > 1:
            raise cv.Invalid("SC16IS750 can only have one channel")
        if uart_count > 0 and value[CONF_UART][0][CONF_CHANNEL] == 1:
            raise cv.Invalid("Channel must be 0 for a SC16IS750")
        if CONF_CRYSTAL not in value:
            value[CONF_CRYSTAL] = 14745600
    else:  # SC16IS752
        if (
            uart_count > 1
            and value[CONF_UART][0][CONF_CHANNEL] == value[CONF_UART][1][CONF_CHANNEL]
        ):
            raise cv.Invalid("Duplicate channel number")
        if CONF_CRYSTAL not in value:
            value[CONF_CRYSTAL] = 3072000
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SC16IS75XComponent),
            cv.Optional(CONF_MODEL, default="SC16IS752"): cv.enum(
                SC16IS75X_MODELS, upper=True
            ),
            cv.Optional(CONF_CRYSTAL): cv.int_,
            cv.Optional(CONF_TEST_MODE, default=0): cv.int_,
            cv.Optional(CONF_UART, default=[]): cv.ensure_list(
                {
                    cv.Required(CONF_UART_ID): cv.declare_id(SC16IS75XChannel),
                    cv.Optional(CONF_CHANNEL): cv.int_range(min=0, max=1),
                    cv.Required(CONF_BAUD_RATE): cv.int_range(min=1),
                    cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
                    cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=5, max=8),
                    cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                        uart.UART_PARITY_OPTIONS, upper=True
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
    cg.add(var.set_name(str(config[CONF_ID])))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_crystal(config[CONF_CRYSTAL]))
    cg.add(var.set_test_mode(config[CONF_TEST_MODE]))
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    for uart_elem in config[CONF_UART]:
        chan = cg.new_Pvariable(uart_elem[CONF_UART_ID])
        cg.add(chan.set_channel_name(str(uart_elem[CONF_UART_ID])))
        cg.add(chan.set_parent(var))
        cg.add(chan.set_channel(uart_elem[CONF_CHANNEL]))
        cg.add(chan.set_baud_rate(uart_elem[CONF_BAUD_RATE]))
        cg.add(chan.set_stop_bits(uart_elem[CONF_STOP_BITS]))
        cg.add(chan.set_data_bits(uart_elem[CONF_DATA_BITS]))
        cg.add(chan.set_parity(uart_elem[CONF_PARITY]))


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


SC16IS75X_PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(SC16IS75XGPIOPin),
        cv.Required(CONF_SC16IS75X): cv.use_id(SC16IS75XComponent),
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


@pins.PIN_SCHEMA_REGISTRY.register("sc16is75x_i2c", SC16IS75X_PIN_SCHEMA)
async def sc16is75x_i2c_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_SC16IS75X])
    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var
