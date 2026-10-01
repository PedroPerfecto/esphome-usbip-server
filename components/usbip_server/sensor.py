import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC, STATE_CLASS_TOTAL_INCREASING

from . import CONF_USBIP_SERVER_ID, USBIPServer

DEPENDENCIES = ["usbip_server"]

CONF_IMPORT_COUNT = "import_count"

# Diagnostic transfer counters, data endpoints only (not ep0/control). Since boot;
# total_increasing, like CONF_IMPORT_COUNT: HA recognizes the reset after a reboot.
# See usbip_server.h/.cpp (publish_diag_ and the fetch_add call sites) for exact semantics.
CONF_IN_SUBMITTED = "in_submitted"
CONF_IN_DONE_DATA = "in_done_data"
CONF_IN_DONE_EMPTY = "in_done_empty"
CONF_IN_DONE_ERR = "in_done_err"
CONF_IN_RET_SENT = "in_ret_sent"
CONF_OUT_SUBMITTED = "out_submitted"
CONF_OUT_DONE_OK = "out_done_ok"
CONF_UNLINK_RX = "unlink_rx"
CONF_DROPPED_SILENT = "dropped_silent"
CONF_STALE_SKIP = "stale_skip"
CONF_SEND_ERRORS = "send_errors"
CONF_SLOT_EXHAUSTED = "slot_exhausted"

_DIAG_COUNTERS = (
    CONF_IN_SUBMITTED,
    CONF_IN_DONE_DATA,
    CONF_IN_DONE_EMPTY,
    CONF_IN_DONE_ERR,
    CONF_IN_RET_SENT,
    CONF_OUT_SUBMITTED,
    CONF_OUT_DONE_OK,
    CONF_UNLINK_RX,
    CONF_DROPPED_SILENT,
    CONF_STALE_SKIP,
    CONF_SEND_ERRORS,
    CONF_SLOT_EXHAUSTED,
)

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
        **{
            cv.Optional(key): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:counter",
            )
            for key in _DIAG_COUNTERS
        },
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_USBIP_SERVER_ID])
    if conf := config.get(CONF_IMPORT_COUNT):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_import_count_sensor(sens))
    for key in _DIAG_COUNTERS:
        if conf := config.get(key):
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(server, f"set_{key}_sensor")(sens))
