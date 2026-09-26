import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_USBIP_SERVER_ID, USBIPServer

DEPENDENCIES = ["usbip_server"]

CONF_CLIENT_ADDRESS = "client_address"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_USBIP_SERVER_ID): cv.use_id(USBIPServer),
        # IP address of the client with the imported device, empty when none
        cv.Optional(CONF_CLIENT_ADDRESS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:ip-network",
        ),
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_USBIP_SERVER_ID])
    if conf := config.get(CONF_CLIENT_ADDRESS):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(server.set_client_address_text_sensor(sens))
