"""
Uponor KNX RF Thermostat – ESPHome External Component
======================================================
Supports Uponor T-45 (and T-33, T-37, T-54, T-55, T-75) KNX RF thermostats.
Hardware: CC1101 RF module + ESP32 (WROOM or any variant).

ESPHome 2025.2+ required  (external_components API).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_EMPTY,
)

CODEOWNERS = ["@your_github_username"]
DEPENDENCIES = []
AUTO_LOAD = ["sensor"]

# --------------------------------------------------------------------------
# Namespace / class registration
# --------------------------------------------------------------------------
uponor_knx_rf_ns = cg.esphome_ns.namespace("uponor_knx_rf")
UponorKnxRf = uponor_knx_rf_ns.class_("UponorKnxRf", cg.Component)

# --------------------------------------------------------------------------
# Config keys
# --------------------------------------------------------------------------
CONF_GDO0_PIN    = "gdo0_pin"
CONF_CS_PIN      = "cs_pin"
CONF_MOSI_PIN    = "mosi_pin"
CONF_MISO_PIN    = "miso_pin"
CONF_SCK_PIN     = "sck_pin"
CONF_THERMOSTATS = "thermostats"
CONF_SERIAL      = "serial"
CONF_TEMPERATURE = "temperature"
CONF_SETPOINT    = "setpoint"
CONF_BATTERY     = "battery"

# --------------------------------------------------------------------------
# Schema for a single thermostat entry
# --------------------------------------------------------------------------
THERMOSTAT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SERIAL): cv.string,
        cv.Required(CONF_NAME):   cv.string,
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SETPOINT): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:battery",
        ),
    }
)

# --------------------------------------------------------------------------
# Top-level component schema
# --------------------------------------------------------------------------
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UponorKnxRf),
            cv.Optional(CONF_GDO0_PIN,  default=2):  cv.int_,
            cv.Optional(CONF_CS_PIN,    default=5):  cv.int_,
            cv.Optional(CONF_MOSI_PIN,  default=23): cv.int_,
            cv.Optional(CONF_MISO_PIN,  default=19): cv.int_,
            cv.Optional(CONF_SCK_PIN,   default=18): cv.int_,
            cv.Required(CONF_THERMOSTATS): cv.ensure_list(THERMOSTAT_SCHEMA),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


# --------------------------------------------------------------------------
# Code generation
# --------------------------------------------------------------------------
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_gdo0_pin(config[CONF_GDO0_PIN]))
    cg.add(var.set_cs_pin(config[CONF_CS_PIN]))
    cg.add(var.set_mosi_pin(config[CONF_MOSI_PIN]))
    cg.add(var.set_miso_pin(config[CONF_MISO_PIN]))
    cg.add(var.set_sck_pin(config[CONF_SCK_PIN]))

    for ts in config[CONF_THERMOSTATS]:
        serial = ts[CONF_SERIAL]
        name   = ts[CONF_NAME]

        temp_s = None
        if CONF_TEMPERATURE in ts:
            temp_s = await sensor.new_sensor(ts[CONF_TEMPERATURE])

        setp_s = None
        if CONF_SETPOINT in ts:
            setp_s = await sensor.new_sensor(ts[CONF_SETPOINT])

        batt_s = None
        if CONF_BATTERY in ts:
            batt_s = await sensor.new_sensor(ts[CONF_BATTERY])

        cg.add(
            var.add_thermostat(
                serial,
                name,
                temp_s if temp_s is not None else cg.nullptr,
                setp_s if setp_s is not None else cg.nullptr,
                batt_s if batt_s is not None else cg.nullptr,
            )
        )
