#ifndef BMI3_ARDUINO_COMMON_H
#define BMI3_ARDUINO_COMMON_H

#include <Arduino.h>
#include <Wire.h>
#include "bmi323.h"

// Interface init/deinit
int8_t bmi3_interface_init(struct bmi3_dev *dev, uint8_t intf);
void bmi3_coines_deinit(void);

// Error print helper
void bmi3_error_codes_print_result(const char *api, int8_t rslt);

#endif // BMI3_ARDUINO_COMMON_H
