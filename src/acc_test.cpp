#include <Arduino.h>
#include <Wire.h>
#include "bmi323.h"
#include "bmi3_arduino_common.h"

static bmi3_dev dev = {0};

// Configure accelerometer parameters
static int8_t set_accel_config(bmi3_dev *d) {
  int8_t rslt;
  bmi3_sens_config cfg = {0};

  cfg.type = BMI323_ACCEL;

  rslt = bmi323_get_sensor_config(&cfg, 1, d);
  bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
  if (rslt != BMI323_OK) return rslt;

  cfg.cfg.acc.odr      = BMI3_ACC_ODR_100HZ;
  cfg.cfg.acc.range    = BMI3_ACC_RANGE_2G;
  cfg.cfg.acc.bwp      = BMI3_ACC_BW_ODR_HALF;
  cfg.cfg.acc.avg_num  = BMI3_ACC_AVG16;
  cfg.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

  rslt = bmi323_set_sensor_config(&cfg, 1, d);
  bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
  if (rslt != BMI323_OK) return rslt;

  // Map DRDY interrupt (optional)
  bmi3_map_int map_int = {0};
  map_int.acc_drdy_int = BMI3_INT1;
  rslt = bmi323_map_interrupt(map_int, d);
  bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

  // Clear any latched status
  uint16_t tmp = 0; (void)bmi323_get_int1_status(&tmp, d);
  return rslt;
}

static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width) {
  const float half_scale = (float)((1UL << bit_width) / 2.0f);
  return (val * g_range) / half_scale;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Start I2C bus
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initialising BMI323 over I2C...");

  // Initialise device over I2C
  int8_t rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
  bmi3_error_codes_print_result("bmi3_interface_init", rslt);
  if (rslt != BMI323_OK) return;

  // Soft reset
  (void)bmi323_soft_reset(&dev);
  dev.delay_us(20000, dev.intf_ptr);

  // Initialise the BMI323 sensor
  rslt = bmi323_init(&dev);
  bmi3_error_codes_print_result("bmi323_init", rslt);
  if (rslt != BMI323_OK) return;

  // Configure accelerometer
  rslt = set_accel_config(&dev);
  if (rslt == BMI323_OK) {
    Serial.println("idx,raw_x,raw_y,raw_z,gx,gy,gz");
  }
}

void loop() {
  static uint16_t idx = 0;
  uint16_t int_status = 0;

  if (bmi323_get_int1_status(&int_status, &dev) == BMI323_OK) {
    if (int_status & BMI3_INT_STATUS_ACC_DRDY) {
      bmi3_sensor_data sd = {0};
      sd.type = BMI323_ACCEL;
      if (bmi323_get_sensor_data(&sd, 1, &dev) == BMI323_OK) {
        float gx = lsb_to_g(sd.sens_data.acc.x, 2.0f, dev.resolution);
        float gy = lsb_to_g(sd.sens_data.acc.y, 2.0f, dev.resolution);
        float gz = lsb_to_g(sd.sens_data.acc.z, 2.0f, dev.resolution);
        Serial.print(idx); Serial.print(',');
        Serial.print(sd.sens_data.acc.x); Serial.print(',');
        Serial.print(sd.sens_data.acc.y); Serial.print(',');
        Serial.print(sd.sens_data.acc.z); Serial.print(',');
        Serial.print(gx, 4); Serial.print(',');
        Serial.print(gy, 4); Serial.print(',');
        Serial.println(gz, 4);
        idx++;
      }
    }
  }
  delay(1);
}
