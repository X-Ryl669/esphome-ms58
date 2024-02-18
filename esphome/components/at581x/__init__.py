import esphome.codegen as cg
from esphome import pins, automation
from esphome.components import i2c, binary_sensor


at581x_ns = cg.esphome_ns.namespace("at581x")
AT581XComponent = at581x_ns.class_("AT581XComponent", cg.Component, i2c.I2CDevice, binary_sensor.BinarySensor)

# Actions
AT581XResetAction = at581x_ns.class_(
    "at581xResetAction", automation.Action
)
AT581XSettingsAction = at581x_ns.class_(
    "at581xSettingsAction", automation.Action
)

CONF_AT581X_ID = "at581x_id"
