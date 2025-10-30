import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import climate, sensor
from esphome import pins

AUTO_LOAD = ["sensor", "climate"]
CODEOWNERS = ["hberntsen"]

CONF_MHI_AC_CTRL_ID = "mhi_ac_ctrl_id"

mhi_core_ns = cg.global_ns.namespace("mhi_ac")
ConfigStruct = mhi_core_ns.struct("Config")
MhiAcCtrl = cg.global_ns.class_("MhiAcCtrl", cg.Component, climate.Climate)

CONF_EXTERNAL_TEMPERATURE_SENSOR = 'external_temperature_sensor'
CONF_USE_LONG_FRAME = "use_long_frame"
CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_SCLK_PIN = "sclk_pin"
CONF_CS_IN_PIN = "cs_in_pin"
CONF_CS_OUT_PIN = "cs_out_pin"

TYPES = [
    CONF_EXTERNAL_TEMPERATURE_SENSOR,
    CONF_USE_LONG_FRAME,
    CONF_MOSI_PIN,
    CONF_MISO_PIN,
    CONF_SCLK_PIN,
    CONF_CS_IN_PIN,
    CONF_CS_OUT_PIN,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MhiAcCtrl),
        cv.Optional(CONF_EXTERNAL_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_USE_LONG_FRAME, default=True): cv.boolean,
        cv.Optional(CONF_MOSI_PIN, default="GPIO7"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_MISO_PIN, default="GPIO2"): pins.gpio_output_pin_schema,
        cv.Optional(CONF_SCLK_PIN, default="GPIO6"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_CS_IN_PIN, default="GPIO10"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_CS_OUT_PIN, default="GPIO9"): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    config_struct = cg.StructInitializer(
        ConfigStruct,
        ("use_long_frame", config[CONF_USE_LONG_FRAME]),
        ("mosi_pin", (await cg.gpio_pin_expression(config[CONF_MOSI_PIN])).get_pin()),
        ("miso_pin", (await cg.gpio_pin_expression(config[CONF_MISO_PIN])).get_pin()),
        ("sclk_pin", (await cg.gpio_pin_expression(config[CONF_SCLK_PIN])).get_pin()),
        ("cs_in_pin", (await cg.gpio_pin_expression(config[CONF_CS_IN_PIN])).get_pin()),
        ("cs_out_pin", (await cg.gpio_pin_expression(config[CONF_CS_OUT_PIN])).get_pin()),
    )

    var = cg.new_Pvariable(config[CONF_ID], config_struct)
    await cg.register_component(var, config)

    if CONF_EXTERNAL_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_EXTERNAL_TEMPERATURE_SENSOR])
        cg.add(var.set_external_room_temperature_sensor(sens))
