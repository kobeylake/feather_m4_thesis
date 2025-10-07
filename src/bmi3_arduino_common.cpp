// bmi3_arduino_common.cpp
#include "bmi3_arduino_common.h"
#include <Arduino.h>
#include <SPI.h>

const int BMI3_PIN_CS = 10;                      // <-- make this global
SPISettings bmiSPI(1000000, MSBFIRST, SPI_MODE0); // <-- make this global too

static inline void cs_low()  { digitalWrite(BMI3_PIN_CS, LOW); }
static inline void cs_high() { digitalWrite(BMI3_PIN_CS, HIGH); }

static int8_t spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *) {
  SPI.beginTransaction(bmiSPI);
  cs_low();
  SPI.transfer(reg | 0x80);                  // READ
  for (uint32_t i = 0; i < len; i++) {
    data[i] = SPI.transfer(0x00);
  }
  cs_high();
  SPI.endTransaction();
  return BMI323_OK;
}

static int8_t spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *) {
  SPI.beginTransaction(bmiSPI);
  cs_low();
  SPI.transfer(reg & 0x7F);                  // WRITE
  for (uint32_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  cs_high();
  SPI.endTransaction();
  return BMI323_OK;
}

static void delay_us(uint32_t us, void *) { delayMicroseconds(us); }

int8_t bmi3_interface_init(struct bmi3_dev *dev, uint8_t /*intf*/) {
  if (!dev) return BMI323_E_NULL_PTR;

  pinMode(BMI3_PIN_CS, OUTPUT);
  cs_high();
  SPI.begin();

  dev->intf           = BMI3_SPI_INTF;
  dev->read           = spi_read;
  dev->write          = (bmi3_write_fptr_t)spi_write;
  dev->delay_us       = delay_us;
  dev->intf_ptr       = nullptr;
  dev->read_write_len = 32;
  return BMI323_OK;
}

void bmi3_coines_deinit(void) {}

void bmi3_error_codes_print_result(const char *api, int8_t rslt) {
  if (rslt == BMI323_OK) return;
  Serial.print("[BMI323] "); Serial.print(api); Serial.print(" -> err=");
  switch (rslt) {
    case BMI3_E_NULL_PTR:               Serial.println("BMI3_E_NULL_PTR"); break;
    case BMI3_E_COM_FAIL:               Serial.println("BMI3_E_COM_FAIL"); break;
    case BMI3_E_FEATURE_ENGINE_STATUS:  Serial.println("BMI3_E_FEATURE_ENGINE_STATUS"); break;
    default:                            Serial.println(rslt); break;
  }
}
