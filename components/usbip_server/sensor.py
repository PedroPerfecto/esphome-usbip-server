import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC, STATE_CLASS_TOTAL_INCREASING

from . import CONF_USBIP_SERVER_ID, USBIPServer

DEPENDENCIES = ["usbip_server"]

CONF_IMPORT_COUNT = "import_count"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_USBIP_SERVER_ID): cv.use_id(USBIPServer),
        # successful imports (client connections) since ESP32 boot;
        # total_increasing: HA recognizes the reset after a reboot
        cv.Optional(CONF_IMPORT_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:counter",
        ),
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_USBIP_SERVER_ID])
    if conf := config.get(CONF_IMPORT_COUNT):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_import_count_sensor(sens))
