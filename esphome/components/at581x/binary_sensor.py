import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.components import i2c, binary_sensor
from esphome.automation import maybe_simple_id
from esphome.const import (
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_DISTANCE,
    UNIT_CENTIMETER,
    UNIT_PERCENT,
    CONF_LIGHT,
    CONF_FREQUENCY,
    DEVICE_CLASS_ILLUMINANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_SIGNAL,
    ICON_FLASH,
    ICON_MOTION_SENSOR,
    ICON_LIGHTBULB,
)

from . import CONF_AT581X_ID, AT581XComponent, at581x_ns


CODEOWNERS = ["@X-Ryl669"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True


# Actions
AT581XResetAction = at581x_ns.class_(
    "at581xResetAction", automation.Action
)
AT581XSettingsAction = at581x_ns.class_(
    "at581xSettingsAction", automation.Action
)

CONF_SENSING_DISTANCE = "sensing_distance"
CONF_FACTORY_RESET = "factory_reset"
CONF_SENSITIVITY = "sensitivity"
CONF_DETECTION_PIN = "detection_pin"
CONF_POWERON_SELFCHECK_TIME = "poweron_selfcheck_time"
CONF_PROTECT_TIME = "protect_time"
CONF_TRIGGER_BASE = "trigger_base"
CONF_TRIGGER_KEEP = "trigger_keep"
CONF_STAGE_GAIN = "stage_gain"
CONF_POWER_CONSUMPTION = "power_consumption"


CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(
        AT581XComponent,
        device_class=DEVICE_CLASS_MOTION,
        icon=ICON_MOTION_SENSOR,
    ).extend(
        {
            cv.GenerateID(): cv.declare_id(AT581XComponent),
            cv.Optional(CONF_DETECTION_PIN, default=21): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x28))
)



async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if detection_pin := config.get(CONF_DETECTION_PIN):
        pin = await cg.gpio_pin_expression(config[CONF_DETECTION_PIN])
        cg.add(var.set_detection_pin(pin))

@automation.register_action(
    "at581x.reset",
    AT581XResetAction,
    maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(AT581XComponent),
        }
    ),
)
async def at581x_reset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    return var

RADAR_SETTINGS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(AT581XComponent),
        cv.Optional(CONF_FACTORY_RESET): cv.templatable(cv.boolean),
        cv.Optional(CONF_FREQUENCY, default=5800): cv.one_of(5696, 5715, 5730, 5748, 5765, 5784, 5800, 5819, 5836, 5851, 5869, 5888, int=True),
        cv.Optional(CONF_SENSING_DISTANCE, default=823): cv.int_range(min=0, max=1023),
        cv.Optional(CONF_POWERON_SELFCHECK_TIME, default=2000): cv.int_range(min=0, max=65535),

        cv.Optional(CONF_POWER_CONSUMPTION, default=70): cv.one_of(48, 56, 63, 70, 77, 91, 105, 115, 40, 44, 47, 51, 54, 61, 68, 78, int=True),
        cv.Optional(CONF_PROTECT_TIME, default=1000): cv.int_range(min=1, max=65535),
        cv.Optional(CONF_TRIGGER_BASE, default=500): cv.int_range(min=1, max=65535),
        cv.Optional(CONF_TRIGGER_KEEP, default=1500): cv.int_range(min=1, max=65535),
        cv.Optional(CONF_STAGE_GAIN, default=3): cv.int_range(min=0, max=12),
    }
).add_extra(
    cv.has_at_least_one_key(
        CONF_FACTORY_RESET,
        CONF_FREQUENCY,
        CONF_SENSING_DISTANCE,
    )
)

@automation.register_action(
    "at581x.settings",
    AT581XSettingsAction,
    RADAR_SETTINGS_SCHEMA,
)
async def at581x_settings_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])

    # Radar configuration
    if CONF_FACTORY_RESET in config:
        cg.add(var.set_factory_reset())

    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))

    if CONF_SENSING_DISTANCE in config:
        cg.add(var.set_sensing_distance(config[CONF_SENSING_DISTANCE]))

    if CONF_POWERON_SELFCHECK_TIME in config:
        cg.add(var.set_poweron_selfcheck_time(config[CONF_POWERON_SELFCHECK_TIME]))

    if CONF_PROTECT_TIME in config:
        cg.add(var.set_protect_time(config[CONF_PROTECT_TIME]))

    if CONF_TRIGGER_BASE in config:
        cg.add(var.set_trigger_base(config[CONF_TRIGGER_BASE]))

    if CONF_TRIGGER_KEEP in config:
        cg.add(var.set_trigger_keep(config[CONF_TRIGGER_KEEP]))

    if CONF_STAGE_GAIN in config:
        cg.add(var.set_stage_gain(config[CONF_STAGE_GAIN]))

    if CONF_POWER_CONSUMPTION in config:
        cg.add(var.set_power_consumption(config[CONF_POWER_CONSUMPTION]))

    return var
