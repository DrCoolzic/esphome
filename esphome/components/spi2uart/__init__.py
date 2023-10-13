import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.components import uart
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_CHANNEL,
    CONF_ID,
    CONF_UART_ID,
)

CODEOWNERS = ["@DrCoolZic"]
DEPENDENCIES = ["spi"]

spi2uart_ns = cg.esphome_ns.namespace("spi2uart")
SPI2UART_Component = spi2uart_ns.class_(
    "SPI2UART_Component", cg.Component, spi.SPIDevice
)
SPI2UARTChannel = spi2uart_ns.class_(
    "SPI2UARTChannel", cg.Component, uart.UARTComponent
)

CONF_SPI2UART = "spi2uart"
MULTI_CONF = True
CONF_UART = "uart"
CONF_TEST_MODE = "test_mode"


def post_validate(value):
    for uart_elem in value[CONF_UART]:
        if CONF_CHANNEL not in uart_elem:
            uart_elem[CONF_CHANNEL] = 0
    uart_count = len(value[CONF_UART])
    if value[0] == "SC16IS750":
        if uart_count > 1:
            raise cv.Invalid("SC16IS750 can only have one channel")
        if uart_count > 0 and value[CONF_UART][0][CONF_CHANNEL] == 1:
            raise cv.Invalid("Channel must be 0 for a SC16IS750")
    else:  # SC16IS752
        if (
            uart_count > 1
            and value[CONF_UART][0][CONF_CHANNEL] == value[CONF_UART][1][CONF_CHANNEL]
        ):
            raise cv.Invalid("Duplicate channel number")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SPI2UART_Component),
            cv.Optional(CONF_TEST_MODE, default=0): cv.int_,
            cv.Required(CONF_UART): cv.ensure_list(
                {
                    cv.Required(CONF_UART_ID): cv.declare_id(SPI2UARTChannel),
                    cv.Optional(CONF_CHANNEL): cv.int_range(min=0, max=3),
                    cv.Required(CONF_BAUD_RATE): cv.int_range(min=1),
                }
            ),
        }
    )
    .extend(spi.spi_device_schema())
    .extend(cv.COMPONENT_SCHEMA),
    # post_validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_name(str(config[CONF_ID])))
    cg.add(var.set_test_mode(config[CONF_TEST_MODE]))
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    for uart_elem in config[CONF_UART]:
        chan = cg.new_Pvariable(uart_elem[CONF_UART_ID])
        cg.add(chan.set_channel_name(str(uart_elem[CONF_UART_ID])))
        cg.add(chan.set_parent(var))
        cg.add(chan.set_channel(uart_elem[CONF_CHANNEL]))
        cg.add(chan.set_baud_rate(uart_elem[CONF_BAUD_RATE]))
