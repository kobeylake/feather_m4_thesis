// bmi3_arduino_common.cpp
#include "bmi3_arduino_common.h"
#include <Arduino.h>
#include <Wire.h>

// Default I2C address: 0x68 (SDO->GND) or 0x69 (SDO->VCC)
static const uint8_t BMI3_I2C_ADDR = 0x69;

// ---------------------------------------------------------------------------
// I2C read/write functions for Bosch BMI323
// ---------------------------------------------------------------------------

static int8_t i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *) {
  Wire.beginTransmission(BMI3_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return BMI3_E_COM_FAIL;
  }
  uint32_t bytes_read = Wire.requestFrom(BMI3_I2C_ADDR, (uint8_t)len);
  if (bytes_read != len) {
    return BMI3_E_COM_FAIL;
  }
  for (uint32_t i = 0; i < len && Wire.available(); i++) {
    data[i] = Wire.read();
  }
  return BMI323_OK;
}

static int8_t i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *) {
  Wire.beginTransmission(BMI3_I2C_ADDR);
  Wire.write(reg);
  for (uint32_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  if (Wire.endTransmission() != 0) {
    return BMI3_E_COM_FAIL;
  }
  return BMI323_OK;
}

static void delay_us(uint32_t us, void *) {
  delayMicroseconds(us);
}

// ---------------------------------------------------------------------------
// Initialise interface for BMI323 (I2C variant)
// ---------------------------------------------------------------------------

int8_t bmi3_interface_init(struct bmi3_dev *dev, uint8_t intf) {
  if (!dev) return BMI323_E_NULL_PTR;

  Wire.begin();
  Wire.setClock(400000);   // 400 kHz I2C Fast Mode

  dev->intf           = BMI3_I2C_INTF;
  dev->read           = i2c_read;
  dev->write          = (bmi3_write_fptr_t)i2c_write;
  dev->delay_us       = delay_us;
  dev->intf_ptr       = nullptr;
  dev->read_write_len = 32;
  return BMI323_OK;
}

void bmi3_coines_deinit(void) {}

// ---------------------------------------------------------------------------
// Error printer for debugging
// ---------------------------------------------------------------------------
void bmi3_error_codes_print_result(const char *api, int8_t rslt) {
  if (rslt == BMI323_OK) return;
  Serial.print("[BMI323] "); Serial.print(api); Serial.print(" -> err=");
  switch (rslt) {
    case BMI3_E_NULL_PTR:              Serial.println("BMI3_E_NULL_PTR"); break;
    case BMI3_E_COM_FAIL:              Serial.println("BMI3_E_COM_FAIL"); break;
    case BMI3_E_FEATURE_ENGINE_STATUS: Serial.println("BMI3_E_FEATURE_ENGINE_STATUS"); break;
    default:                           Serial.println(rslt); break;
  }
}