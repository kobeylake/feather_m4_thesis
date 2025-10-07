#include <Arduino.h>
#include "wiring_private.h"   // for g_APinDescription, pinPeripheral()

// ------ pick your analog pin here ------
#define ADC_PIN   A1          // try A1 first (avoids A0/DAC quirks)
// --------------------------------------

static inline void wait_sync_all() {
  while (ADC0->SYNCBUSY.reg) { /* wait */ }
}

static bool adc_convert_once(uint16_t &out) {
  // Clear stale flag, start single conversion
  ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  ADC0->SWTRIG.bit.START = 1;

  // Wait with timeout (avoid hard hang)
  const uint32_t t0 = millis();
  while (!ADC0->INTFLAG.bit.RESRDY) {
    if (millis() - t0 > 50) return false;  // 50 ms guard
  }
  out = ADC0->RESULT.reg & 0x0FFFu;        // 12-bit result
  return true;
}

void ADC_Init(uint8_t arduinoPin) {
  // 0) Make sure the pad is in ANALOG mode (turns off digital buffer/pulls)
  pinPeripheral(arduinoPin, PIO_ANALOG);

  // 1) Enable bus clock to ADC0
  MCLK->APBDMASK.bit.ADC0_ = 1;

  // 2) Feed **GCLK0** to ADC0 (GCLK0 is running by default on SAME51 cores)
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

  // 3) Reset
  ADC0->CTRLA.bit.SWRST = 1;
  while (ADC0->CTRLA.bit.SWRST) {}

  // 4) Prescaler (slow & stable)
  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV256;
  wait_sync_all();

  // 5) Reference: use external AREF (≈3.3 V on Feather M4)
  ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
  ADC0->REFCTRL.bit.REFCOMP = 1;
  while (ADC0->SYNCBUSY.bit.REFCTRL) {}
  wait_sync_all();

  // 6) 12-bit, single-shot
  ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
  while (ADC0->SYNCBUSY.bit.CTRLB) {}
  ADC0->CTRLB.bit.FREERUN = 0;
  while (ADC0->SYNCBUSY.bit.CTRLB) {}

  // 7) Sample time (a little longer for stability) and averging
  ADC0->SAMPCTRL.bit.SAMPLEN = 63;
  while (ADC0->SYNCBUSY.bit.SAMPCTRL) {}
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(4);
  while (ADC0->SYNCBUSY.bit.AVGCTRL) {}

  // 8) Map Arduino pin -> correct ADC channel from variant table
  uint8_t muxpos = g_APinDescription[arduinoPin].ulADCChannelNumber; // channel index for this pin
  ADC0->INPUTCTRL.bit.MUXPOS = muxpos;         // program the actual channel
#if defined(ADC_INPUTCTRL_MUXNEG_GND_Val)
  ADC0->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; // single-ended to GND
#endif
  while (ADC0->SYNCBUSY.bit.INPUTCTRL) {}

  // 9) Enable ADC
  ADC0->CTRLA.bit.ENABLE = 1;
  while (ADC0->SYNCBUSY.bit.ENABLE) {}
  delayMicroseconds(50);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println();
  Serial.println("******************************");
  Serial.print  ("  SAME51 ADC (registers) on ");
  Serial.println(ADC_PIN == A0 ? "A0" : (ADC_PIN == A1 ? "A1" : "Ax"));
  Serial.println("  Ref: AREFA ~ 3.3 V, GCLK0 ");
  Serial.println("******************************");

  ADC_Init(ADC_PIN);
}

void loop() {
  static uint32_t samples = 0;
  static uint16_t last = 0;

  uint16_t v;
  if (adc_convert_once(v)) {
    last = v - 10; // offset at 0V
    samples++;
  } else {
    // show a one-off hint if conversion didn't complete
    static bool said = false;
    if (!said) { Serial.println("ADC timeout: check clock/ref/pinmux/channel."); said = true; }
  }

  static uint32_t t0 = 0;
  if (millis() - t0 >= 1000) {
    float volts = last * (3.3f / 4095.0f);
    Serial.print("Samples/sec: "); Serial.println(samples);
    Serial.print("Raw: ");         Serial.print(last);
    Serial.print("   Volts: ");    Serial.print(volts, 4); Serial.println(" V");
    samples = 0;
    t0 = millis();
  }
}
