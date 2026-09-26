import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_USBIP_SERVER_ID, USBIPServer

DEPENDENCIES = ["usbip_server"]

CONF_USB_CONNECTED = "usb_connected"
CONF_CLIENT_CONNECTED = "client_connected"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_USBIP_SERVER_ID): cv.use_id(USBIPServer),
        # the ESP32 sees the USB device and exports it
        cv.Optional(CONF_USB_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:usb",
        ),
        # a client has an active session with the imported device
        cv.Optional(CONF_CLIENT_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:lan-connect",
        ),
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_USBIP_SERVER_ID])
    if conf := config.get(CONF_USB_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(server.set_usb_connected_binary_sensor(sens))
    if conf := config.get(CONF_CLIENT_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(server.set_client_connected_binary_sensor(sens))
