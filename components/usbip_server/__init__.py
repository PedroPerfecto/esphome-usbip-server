import esphome.codegen as cg
from esphome.components.usb_host import CONF_PID, CONF_VID, USBClient, register_usb_client
import esphome.config_validation as cv
from esphome.const import CONF_PORT

AUTO_LOAD = ["usb_host"]
DEPENDENCIES = ["network"]
CODEOWNERS = []

CONF_BUSID = "busid"
CONF_MAX_TRANSFER_SIZE = "max_transfer_size"
CONF_MAX_PENDING = "max_pending"
CONF_DMA_CHUNK_SIZE = "dma_chunk_size"
CONF_INTERFACES = "interfaces"
CONF_USBIP_SERVER_ID = "usbip_server_id"

usbip_server_ns = cg.esphome_ns.namespace("usbip_server")
USBIPServer = usbip_server_ns.class_("USBIPServer", USBClient)


def _validate_busid(value):
    value = cv.string_strict(value)
    if not 1 <= len(value) <= 31:
        raise cv.Invalid("busid must be 1-31 characters")
    return value


CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(USBIPServer),
        # 0/0 = export any connected device
        cv.Optional(CONF_VID, default=0): cv.hex_uint16_t,
        cv.Optional(CONF_PID, default=0): cv.hex_uint16_t,
        cv.Optional(CONF_PORT, default=3240): cv.port,
        cv.Optional(CONF_BUSID, default="1-1"): _validate_busid,
        # largest transfer from the client; kept in PSRAM (Linux usb-storage sends up to ~120 KB)
        cv.Optional(CONF_MAX_TRANSFER_SIZE, default=131072): cv.int_range(min=64, max=1048576),
        # largest single USB transfer in internal RAM (DMA); larger bulk transfers are split
        cv.Optional(CONF_DMA_CHUNK_SIZE, default=16384): cv.int_range(min=64, max=65536),
        cv.Optional(CONF_MAX_PENDING, default=16): cv.int_range(min=1, max=32),
        # exported interface numbers (bInterfaceNumber); omitted = all
        cv.Optional(CONF_INTERFACES): cv.All(
            cv.ensure_list(cv.int_range(min=0, max=31)), cv.Length(min=1)
        ),
    }
)


async def to_code(config):
    var = await register_usb_client(config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_busid(config[CONF_BUSID]))
    cg.add(var.set_max_transfer_size(config[CONF_MAX_TRANSFER_SIZE]))
    cg.add(var.set_max_pending(config[CONF_MAX_PENDING]))
    cg.add(var.set_dma_chunk_size(config[CONF_DMA_CHUNK_SIZE]))
    if CONF_INTERFACES in config:
        mask = 0
        for i in config[CONF_INTERFACES]:
            mask |= 1 << i
        cg.add(var.set_interface_mask(mask))
