import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_POWER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_WATT,
    UNIT_WATT_HOURS,
    ICON_FAN,
    ICON_THERMOMETER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    UNIT_HOUR,
    ICON_TIMER,
    DEVICE_CLASS_DURATION,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL,
    UNIT_EMPTY,
    DEVICE_CLASS_FREQUENCY,
    UNIT_HERTZ,
    DEVICE_CLASS_CURRENT,
    UNIT_AMPERE,
    UNIT_KILOWATT_HOURS,
    DEVICE_CLASS_ENERGY,
    ICON_COUNTER
)
from . import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

CONF_FRAME_ERRORS = "frame_errors"
CONF_OPERATION_DATA_TIMEOUTS = "operation_data_timeouts"
CONF_INTEGRATED_TOTAL_ENERGY = "integrated_total_energy"
CONF_SET_TEMPERATURE = "set_temperature"
CONF_CLIMATE_CURRENT_TEMPERATURE = "climate_current_temperature"
CONF_RETURN_AIR_TEMPERATURE = "return_air_temperature"
CONF_INDOOR_U_BEND_TEMPERATURE = "indoor_u_bend_temperature"
CONF_INDOOR_CAPILLARY_TEMPERATURE = "indoor_capillary_temperature"
CONF_INDOOR_SUCTION_HEADER_TEMPERATURE = "indoor_suction_header_temperature"
CONF_INDOOR_FAN_SPEED = "indoor_fan_speed"
CONF_INDOOR_TOTAL_RUN_HOURS = "indoor_total_run_hours"
CONF_OUTDOOR_AIR_TEMPERATURE = "outdoor_air_temperature"
CONF_OUTDOOR_HEAT_EXCHANGER_TEMPERATURE_1 = "outdoor_heat_exchanger_temperature_1"
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_DISCHARGE_PIPE_TEMPERATURE = "discharge_pipe_temperature"
CONF_CURRENT = "current"
CONF_COMPRESSOR_DISCHARGE_PIPE_SUPER_HEAT_TEMPERATURE = "compressor_discharge_pipe_super_heat_temperature"
CONF_COMPRESSOR_PROTECTION_STATE_NUMBER = "compressor_protection_state_number"
CONF_OUTDOOR_FAN_SPEED = "outdoor_fan_speed"
CONF_COMPRESSOR_TOTAL_RUN_HOURS = "compressor_total_run_hours"
CONF_OUTDOOR_EXPANSION_VALVE_PULSE_RATE = "outdoor_expansion_valve_pulse_rate"
CONF_ENERGY_USED = "energy_used"

TYPES = [
    CONF_INTEGRATED_TOTAL_ENERGY,
    CONF_SET_TEMPERATURE,
    CONF_POWER,
    CONF_FRAME_ERRORS,
    CONF_OPERATION_DATA_TIMEOUTS,
    CONF_CLIMATE_CURRENT_TEMPERATURE,
    CONF_RETURN_AIR_TEMPERATURE,
    CONF_INDOOR_U_BEND_TEMPERATURE,
    CONF_INDOOR_CAPILLARY_TEMPERATURE,
    CONF_INDOOR_SUCTION_HEADER_TEMPERATURE,
    CONF_INDOOR_FAN_SPEED,
    CONF_INDOOR_TOTAL_RUN_HOURS,
    CONF_OUTDOOR_AIR_TEMPERATURE,
    CONF_OUTDOOR_HEAT_EXCHANGER_TEMPERATURE_1,
    CONF_COMPRESSOR_FREQUENCY,
    CONF_DISCHARGE_PIPE_TEMPERATURE,
    CONF_CURRENT,
    CONF_COMPRESSOR_DISCHARGE_PIPE_SUPER_HEAT_TEMPERATURE,
    CONF_COMPRESSOR_PROTECTION_STATE_NUMBER,
    CONF_OUTDOOR_FAN_SPEED,
    CONF_COMPRESSOR_TOTAL_RUN_HOURS,
    CONF_OUTDOOR_EXPANSION_VALVE_PULSE_RATE,
    CONF_ENERGY_USED
]

MhiFrameErrors = cg.global_ns.class_("MhiFrameErrors", cg.Component, sensor.Sensor)
MhiIntegratedTotalEnergy = cg.global_ns.class_("MhiIntegratedTotalEnergy", cg.Component, sensor.Sensor)
MhiPower = cg.global_ns.class_("MhiPower", cg.Component, sensor.Sensor)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MhiAcCtrl),
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Optional(CONF_INTEGRATED_TOTAL_ENERGY): sensor.sensor_schema(
                class_=MhiIntegratedTotalEnergy,
                unit_of_measurement=UNIT_WATT_HOURS,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_SET_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                class_=MhiPower,
                unit_of_measurement=UNIT_WATT,
                device_class=DEVICE_CLASS_POWER,
            ),
            cv.Optional(CONF_FRAME_ERRORS): sensor.sensor_schema(
                class_=MhiFrameErrors,
                icon=ICON_COUNTER,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_OPERATION_DATA_TIMEOUTS): sensor.sensor_schema(
                icon=ICON_COUNTER,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_CLIMATE_CURRENT_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                device_class=DEVICE_CLASS_TEMPERATURE,
                icon=ICON_THERMOMETER,
            ),
            cv.Optional(CONF_RETURN_AIR_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_INDOOR_U_BEND_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_INDOOR_CAPILLARY_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_EMPTY,
            ),
            cv.Optional(CONF_INDOOR_SUCTION_HEADER_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_INDOOR_FAN_SPEED): sensor.sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_FAN,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INDOOR_TOTAL_RUN_HOURS): sensor.sensor_schema(
                device_class=DEVICE_CLASS_DURATION,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_TIMER,
                state_class=STATE_CLASS_TOTAL,
                unit_of_measurement=UNIT_HOUR,
            ),
            cv.Optional(CONF_OUTDOOR_AIR_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_OUTDOOR_HEAT_EXCHANGER_TEMPERATURE_1): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_FREQUENCY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:sine-wave",
                unit_of_measurement=UNIT_HERTZ
            ),
            cv.Optional(CONF_DISCHARGE_PIPE_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_CURRENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                unit_of_measurement=UNIT_AMPERE,
            ),
            cv.Optional(CONF_COMPRESSOR_DISCHARGE_PIPE_SUPER_HEAT_TEMPERATURE): sensor.sensor_schema(
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_EMPTY,
            ),
            cv.Optional(CONF_COMPRESSOR_PROTECTION_STATE_NUMBER): sensor.sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:shield-alert-outline"
            ),
            cv.Optional(CONF_OUTDOOR_FAN_SPEED): sensor.sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_FAN,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPRESSOR_TOTAL_RUN_HOURS): sensor.sensor_schema(
                device_class=DEVICE_CLASS_DURATION,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_TIMER,
                state_class=STATE_CLASS_TOTAL,
                unit_of_measurement=UNIT_HOUR,
            ),
            cv.Optional(CONF_OUTDOOR_EXPANSION_VALVE_PULSE_RATE): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:valve",
                state_class=STATE_CLASS_MEASUREMENT,
                unit_of_measurement=UNIT_EMPTY,
            ),
            cv.Optional(CONF_ENERGY_USED): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:lightning-bolt",
                unit_of_measurement=UNIT_KILOWATT_HOURS,
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
