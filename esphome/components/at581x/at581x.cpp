#include "at581x.h"

/* Select gain for AT581X (3dB per step for level1, 6dB per step for level 2), high value = small gain. (p12) */
const uint8_t GainAddrTable[] = { 0x5c, 0x63 };
const uint8_t Gain5CTable[]   = { 0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78, 0x88, 0x98, 0xa8, 0xb8, 0xc8  };
const uint8_t Gain63Table[]   = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
const uint8_t Gain61Value     = 8;

/*!< Power consumption configuration table (p12). */
const uint8_t Power_48uA        = (0x00<<4 | 0x01);
const uint8_t Power_56uA        = (0x01<<4 | 0x01);
const uint8_t Power_63uA        = (0x02<<4 | 0x01);
const uint8_t Power_70uA        = (0x03<<4 | 0x01);
const uint8_t Power_77A_uA      = (0x04<<4 | 0x01);
const uint8_t Power_91uA        = (0x05<<4 | 0x01);
const uint8_t Power_105uA       = (0x06<<4 | 0x01);
const uint8_t Power_115uA       = (0x07<<4 | 0x01);

const uint8_t Power_40uA        = (0x00<<4 | 0x03);
const uint8_t Power_44uA        = (0x01<<4 | 0x03);
const uint8_t Power_47uA        = (0x02<<4 | 0x03);
const uint8_t Power_51uA        = (0x03<<4 | 0x03);
const uint8_t Power_54uA        = (0x04<<4 | 0x03);
const uint8_t Power_61uA        = (0x05<<4 | 0x03);
const uint8_t Power_68uA        = (0x06<<4 | 0x03);
const uint8_t Power_77B_uA      = (0x07<<4 | 0x03);
/* Reverse table from position to value for debugging purpose */
const uint8_t PowerTable[]   = { 48,   56,  63,  70,  77,  91, 105, 115,  40,  44,  47,  51,  54,  61,  68,  78 };
const uint8_t Power67Table[] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7 };
const uint8_t Power68Table[] = { 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8,  24,  24,  24,  24,  24,  24,  24,  24 }; // See Page 12, shift by 3 bits

/*!< Frequency Configuration table (p14/15 of datasheet). */
const uint8_t FreqAddr      = 0x61;
const uint8_t FreqTable[]   = { 5696, 5715, 5730, 5748, 5765, 5784, 5800, 5819, 5836, 5851, 5869, 5888 };
const uint8_t Freq5FTable[] = { 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x40, 0x41, 0x42, 0x43 };
const uint8_t Freq60Table[] = { 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9e, 0x9e, 0x9e, 0x9e };

/*!< Value for RF and analog modules switch (p10). */
const uint8_t RFOffTable[] = { 0x46, 0xaa, 0x50 };
const uint8_t RFOnTable[]  = { 0x45, 0x55, 0xA0 };
const uint8_t RFRegAddr[]  = { 0x5d, 0x62, 0x51 }

/*!< Registers of Lighting delay time. Unit: ms, min 2s (p8) */
const uint8_t HighLevelDelayControlAddr = 0x41;    /*!< Time_flag_out_ctrl 0x01 */
const uint8_t HighLevelDelayValueAddr   = 0x42;    /*!< Time_flag_out_1 Bit<7:0> */

const uint8_t AT581X_RA_Reset  = 0x00;

/*!< Sensing distance address */
const uint8_t SignalDetectionThresholdAddrLo = 0x10;
const uint8_t SignalDetectionThresholdAddrHi = 0x11;

/*!< Bit field value for power registers */
const uint8_t PowerThresholdAddrHi = 0x68;
const uint8_t PowerThresholdAddrLo = 0x67;
const uint8_t PwrWorkTimeEn  = 8; // Reg 0x67
const uint8_t PwrBurstTimeEn = 32; // Reg 0x68
const uint8_t PwrThreshEn    = 64; // Reg 0x68
const uint8_t PwrThreshValEn = 128; // Reg 0x67

/*!< Times */
const uint8_t TriggerBaseTimeAddr = 0x3D; // 4 bytes, so up to 0x40
const uint8_t ProtectTimeAddr     = 0x4E; // 2 bytes, up to 0x4F
const uint8_t TriggerKeepTimeAddr = 0x42; // 4 bytes, so up to 0x45
const uint8_t Time41Value         = 1;
const uint8_t SelfCheckTimeAddr   = 0x38; // 2 bytes, up to 0x39

namespace esphome {
namespace at581x {

static const char *const TAG = "at581x";

bool AT581XComponent::i2c_write_reg(uint8_t addr, uint8_t data) {
  return write_register(addr, &data, 1) == esphome::i2c::NO_ERROR;
}
bool AT581XComponent::i2c_write_reg(uint8_t addr, uint32_t data) {
  return    write_register(addr, data & 0xFF, 1) == esphome::i2c::NO_ERROR
         && write_register(addr + 1, (data >> 8) & 0xFF , 1) == esphome::i2c::NO_ERROR
         && write_register(addr + 2, (data >> 16) & 0xFF , 1) == esphome::i2c::NO_ERROR
         && write_register(addr + 3, (data >> 24) & 0xFF , 1) == esphome::i2c::NO_ERROR;
}
bool AT581XComponent::i2c_write_reg(uint8_t addr, uint16_t data) {
  return    write_register(addr, data & 0xFF, 1) == esphome::i2c::NO_ERROR
         && write_register(addr + 1, (data >> 8) & 0xFF , 1) == esphome::i2c::NO_ERROR;
}


bool AT581XComponent::i2c_read_reg(uint8_t addr, uint8_t & data) {
  return read_register(addr, &data, 1) == esphome::i2c::NO_ERROR;
}

void AT581XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AT581X '%s'...", this->name_.c_str());
  this->detection_pin_->setup();
  if (!i2c_write_config()) {
    ESP_LOGCONFIG(TAG, "Setting up AT581X '%s' failed...", this->name_.c_str());
  }
}
void AT581XComponent::update() override {
  // The main operation is to detect the human presence
  bool state = this->detection_pin_->digital_read();
  this->publish_state(state);
}
void AT581XComponent::dump_config() {
  LOG_BINARY_SENSOR("", "AT581X", this);
  LOG_PIN("  Pin: ", this->detection_pin_);
  LOG_I2C_DEVICE(this);
}
#define ArrSz(X)  sizeof(X) / sizeof(X[0])
bool AT581XComponent:;i2c_write_config() {
  // Set frequency point
  if (!i2c_write_reg(FreqAddr, 0xD0 | Gain61Value)) {
    ESP_LOGE(TAG, "Failed to write AT581X Freq mode");
    return false;
  }
  // Find the current frequency from the table to know what value to write
  for (uint16_t i = 0; i < ArrSz(FreqTable)+1; i++) {
    if (i == ArrSz(FreqTable)) {
      ESP_LOGE(TAG, "Set frequency not found");
      return false;
    }
    if (FreqTable[i] == this->freq_) {
      if (!i2c_write_reg(0x5F, Freq5FTable[i]) || !i2c_write_reg(0x60, Freq60Table[i])) {
        ESP_LOGE(TAG, "Failed to write AT581X Freq value");
        return false;
      }
      break;
    }
  }

  // Set distance
  if (!i2c_write_reg(SignalDetectionThresholdAddrLo, this->distance_ & 0xFF)
   || !i2c_write_reg(SignalDetectionThresholdAddrHi, this->distance_ >> 8)) {
    ESP_LOGE(TAG, "Failed to write AT581X sensing distance low");
    return false;
  }
  // Set power setting
  uint8_t pwr67 = PwrThreshValEn | PwrWorkTimeEn,
          pwr68 = PwrBurstTimeEn | PwrThreshEn;
  for (uint16_t i = 0; i < ArrSz(PowerTable)+1; i++) {
    if (i == ArrSz(PowerTable)) {
      ESP_LOGE(TAG, "Set power not found");
      return false;
    }
    if (PowerTable[i] == this->power_) {
      pwr67 |= Power67Table[i];
      pwr68 |= Power68Table[i]; // See Page 12
      break;
    }
  }

  if (!i2c_write_reg(PowerThresholdAddrLo, pwr67)
   || !i2c_write_reg(PowerThresholdAddrHi, pwr68)) {
    ESP_LOGE(TAG, "Failed to write AT581X power registers");
    return false;
  }

  // Set gain
  if (!i2c_write_reg(GainAddrTable[0], Gain5CTable[this->gain_])
     || !i2c_write_reg(GainAddrTable[1], Gain63Table[this->gain_ >> 1])) {
    ESP_LOGE(TAG, "Failed to write AT581X gain registers");
    return false;
  }

  // Set times
  if (!i2c_write_reg(TriggerBaseTimeAddr, (uint32)this->trigger_base_time_ms_)) {
    ESP_LOGE(TAG, "Failed to write AT581X trigger base time registers");
    return false;
  }
  if (!i2c_write_reg(TriggerKeepTimeAddr, (uint32)this->trigger_keep_time_ms_)) {
    ESP_LOGE(TAG, "Failed to write AT581X trigger keep time registers");
    return false;
  }
  if (!i2c_write_reg(ProtectTimeAddr, (uint16)this->protect_time_ms_)) {
    ESP_LOGE(TAG, "Failed to write AT581X protect time registers");
    return false;
  }
  if (!i2c_write_reg(SelfCheckTimeAddr, (uint16)this->self_check_time_ms_)) {
    ESP_LOGE(TAG, "Failed to write AT581X self check time registers");
    return false;
  }
  if (!i2c_write_reg(0x41, Tim41Value)) {
    ESP_LOGE(TAG, "Failed to enable AT581X time registers");
    return false;
  }


  // Don't know why it's required in other code, it's not in datasheet
  if (!i2c_write_reg(0x55, 0x04)) {
    ESP_LOGE(TAG, "Failed to enable AT581X");
    return false;
  }

  // Ok, config is written, let's reset the chip so it's using the new config
  return set_factory_reset();
}

float AT581XComponent::get_setup_priority() const override;
bool AT581XComponent::set_factory_reset() {
  if (!i2c_write_reg(AT581X_RA_Reset, 0) || !i2c_write_reg(AT581X_RA_Reset, 1)) {
    ESP_LOGE(TAG, "Failed to reset AT581X component");
  }
}

void AT581XComponent::set_rf_mode(bool enable) {
  const uint8_t ** p = enable ? &RFOnTable : &RFOffTable;
  for (size_t i = 0; i < ArrSz(RFRegAddr); i++)
    if (!i2c_write_reg(RFRegAddr[i], (*p)[i])) {
      ESP_LOGE(TAG, "Failed to write AT581X RF mode");
      return;
    }
}

}  // namespace at581x
}  // namespace esphome
