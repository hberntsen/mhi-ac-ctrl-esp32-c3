import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ENTITY_CATEGORY_NONE
from . import MhiAcCtrl, CONF_MHI_AC_CTRL_ID

CONF_VANES_UD = "vanes_ud"
CONF_VANES_LR = "vanes_lr"

TYPES = [
    CONF_VANES_UD,
    CONF_VANES_LR
]

MhiVanesUD = cg.global_ns.class_("MhiVanesUD", cg.Component, select.Select)
MhiVanesLR = cg.global_ns.class_("MhiVanesLR", cg.Component, select.Select)

OPTIONS = {
        "vanes_ud": ["Up", "Up/Center", "Center/Down", "Down", "Swing",
                     "3D Auto", "See IR Remote"],
        "vanes_lr": ["Left", "Left/Center", "Center", "Center/Right", "Right",
                     "Wide", "Spot", "Swing", "3D Auto"]
        }

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MhiAcCtrl),
            cv.GenerateID(CONF_MHI_AC_CTRL_ID): cv.use_id(MhiAcCtrl),
            cv.Optional(CONF_VANES_UD): select.select_schema(
                class_=MhiVanesUD,
                entity_category=ENTITY_CATEGORY_NONE,
                icon="mdi:arrow-up-down",
            ),
            cv.Optional(CONF_VANES_LR): select.select_schema(
                class_=MhiVanesLR,
                entity_category=ENTITY_CATEGORY_NONE,
                icon="mdi:arrow-left-right",
            ),
        }
    )
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        # The code depends on the exact values specified here, so change it in
        # both places.
        sens = await select.new_select(conf, options=OPTIONS[key])
        cg.add(getattr(hub, f"set_{key}_select")(sens))


async def to_code(config):
    paren = await cg.get_variable(config[CONF_MHI_AC_CTRL_ID])
    for key in TYPES:
        await setup_conf(config, key, paren)
