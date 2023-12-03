import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_POWER,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_WATT,
    UNIT_WATT_HOURS,
    ICON_FAN,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
)
from . import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

CONF_FRAME_ERRORS = "frame_errors"
CONF_FAN_RAW = "fan_raw"
CONF_FAN_OLD = "fan_old"
CONF_TOTAL_ENERGY = "total_energy"

TYPES = [
    CONF_FAN_RAW,
    CONF_FAN_OLD,
    CONF_TOTAL_ENERGY,
    CONF_POWER,
    CONF_FRAME_ERRORS,
]

MhiFrameErrors = cg.global_ns.class_("MhiFrameErrors", cg.Component, sensor.Sensor)
MhiTotalEnergy = cg.global_ns.class_("MhiTotalEnergy", cg.Component, sensor.Sensor)
MhiPower = cg.global_ns.class_("MhiPower", cg.Component, sensor.Sensor)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MhiAcCtrl),
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Optional(CONF_FAN_RAW): sensor.sensor_schema(icon=ICON_FAN),
            cv.Optional(CONF_FAN_OLD): sensor.sensor_schema(icon=ICON_FAN),
            cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(
                class_=MhiTotalEnergy,
                unit_of_measurement=UNIT_WATT_HOURS,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                class_=MhiPower,
                unit_of_measurement=UNIT_WATT,
                device_class=DEVICE_CLASS_POWER,
            ),
            cv.Optional(CONF_FRAME_ERRORS): sensor.sensor_schema(
                class_=MhiFrameErrors, state_class=STATE_CLASS_TOTAL_INCREASING
            ),
        }
    )
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        sens = await sensor.new_sensor(conf)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))


async def to_code(config):
    paren = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    for key in TYPES:
        await setup_conf(config, key, paren)
