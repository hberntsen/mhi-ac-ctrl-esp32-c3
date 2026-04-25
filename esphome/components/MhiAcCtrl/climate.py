import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from . import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

CONFIG_SCHEMA = (
    climate.climate_schema(MhiAcCtrl)
    .extend(
        {
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    paren = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    await climate.register_climate(paren, config)
