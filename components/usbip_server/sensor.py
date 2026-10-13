import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_MILLISECOND,
)

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
CONF_UNLINK_DISCARDED = "unlink_discarded"
CONF_IN_RESUBMITTED = "in_resubmitted"
CONF_IN_SEND_FAILED = "in_send_failed"
CONF_CB_QUEUE_FULL = "cb_queue_full"
# Interrupt endpoints without HALT (usbip_intrq.h): counters since boot, total_increasing.
CONF_INTR_Q_FULL = "intr_q_full"
CONF_INTR_DROPPED_NOSESSION = "intr_dropped_nosession"
CONF_INTR_FLUSHED_IMPORT = "intr_flushed_import"
CONF_INTR_UNLINK_WAITING = "intr_unlink_waiting"
CONF_OUT_UNLINK_WAIT_TIMEOUT = "out_unlink_wait_timeout"
CONF_INTR_OVERFLOW = "intr_overflow"
CONF_INTR_ZLP = "intr_zlp"
# Stuck detector (usbip_outwatch.h): counters since boot, total_increasing.
CONF_OUT_STUCK_DETECTED = "out_stuck_detected"
CONF_UNLINK_ORPHANED = "unlink_orphaned"
CONF_SEND_TIMEOUTS = "send_timeouts"
CONF_STALL_SOFT = "stall_soft"
CONF_STALL_RECOVERED = "stall_recovered"

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
    CONF_UNLINK_DISCARDED,
    CONF_IN_RESUBMITTED,
    CONF_IN_SEND_FAILED,
    CONF_CB_QUEUE_FULL,
    CONF_INTR_Q_FULL,
    CONF_INTR_DROPPED_NOSESSION,
    CONF_INTR_FLUSHED_IMPORT,
    CONF_INTR_UNLINK_WAITING,
    CONF_OUT_UNLINK_WAIT_TIMEOUT,
    CONF_INTR_OVERFLOW,
    CONF_INTR_ZLP,
    CONF_OUT_STUCK_DETECTED,
    CONF_UNLINK_ORPHANED,
    CONF_SEND_TIMEOUTS,
    CONF_STALL_SOFT,
    CONF_STALL_RECOVERED,
)

# Longest dev_lock_ hold around the usb_host_* calls within the last diag period (10 s).
# Not a counter: a peak that restarts at 0 every period, hence measurement.
CONF_DEV_LOCK_HOLD_MAX = "dev_lock_hold_max"
UNIT_MICROSECOND = "µs"  # not defined in esphome.const

# Highest number of packets stored in an interrupt IN queue within the last diag period (10 s).
# A peak that restarts every period, hence measurement.
CONF_INTR_Q_DEPTH_MAX = "intr_q_depth_max"

# Longest interrupt OUT stall since boot that ended by itself (submission to completion).
# A maximum that only grows until the next boot, hence measurement.
CONF_STALL_MAX_MS = "stall_max_ms"

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
        cv.Optional(CONF_DEV_LOCK_HOLD_MAX): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROSECOND,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:timer-sand",
        ),
        cv.Optional(CONF_INTR_Q_DEPTH_MAX): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:tray-full",
        ),
        cv.Optional(CONF_STALL_MAX_MS): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLISECOND,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:timer-alert-outline",
        ),
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
    if conf := config.get(CONF_DEV_LOCK_HOLD_MAX):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_dev_lock_hold_max_sensor(sens))
    if conf := config.get(CONF_INTR_Q_DEPTH_MAX):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_intr_q_depth_max_sensor(sens))
    if conf := config.get(CONF_STALL_MAX_MS):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_stall_max_ms_sensor(sens))
