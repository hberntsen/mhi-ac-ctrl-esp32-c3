import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.core import coroutine
from esphome import pins
from . import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_SCLK_PIN = "sclk_pin"
CONF_CS_IN_PIN = "cs_in_pin"
CONF_CS_OUT_PIN = "cs_out_pin"

TYPES = [
    CONF_MOSI_PIN,
    CONF_MISO_PIN, 
    CONF_SCLK_PIN, 
    CONF_CS_IN_PIN, 
    CONF_CS_OUT_PIN,
]

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MhiAcCtrl),
        cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        cv.Optional(CONF_MOSI_PIN, default="GPIO7"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_MISO_PIN, default="GPIO2"): pins.gpio_output_pin_schema,
        cv.Optional(CONF_SCLK_PIN, default="GPIO6"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_CS_IN_PIN, default="GPIO10"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_CS_OUT_PIN, default="GPIO9"): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)


@coroutine
async def to_code(config):
    paren = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    await climate.register_climate(paren, config)

    mosi_pin = await cg.gpio_pin_expression(config[CONF_MOSI_PIN])
    cg.add(paren.set_mosi_pin(mosi_pin))

    miso_pin = await cg.gpio_pin_expression(config[CONF_MISO_PIN])
    cg.add(paren.set_miso_pin(miso_pin))

    sclk_pin = await cg.gpio_pin_expression(config[CONF_SCLK_PIN])
    cg.add(paren.set_sclk_pin(sclk_pin))

    cs_in_pin = await cg.gpio_pin_expression(config[CONF_CS_IN_PIN])
    cg.add(paren.set_cs_in_pin(cs_in_pin))

    cs_out_pin = await cg.gpio_pin_expression(config[CONF_CS_OUT_PIN])
    cg.add(paren.set_cs_out_pin(cs_out_pin))
