//**************************************************************************
// FreeRTOS on SAME51 (Feather M4 CAN)
// Original: Scott Briscoe
// Modified: Added LED blink task
//**************************************************************************

#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (2000)

#include <Arduino.h>
#include <FreeRTOS.h>
#include <FreeRTOS_SAMD51.h>
#include <ACANFD_FeatherM4CAN.h>
#include <task.h>
#include <SPI.h>
#include "wiring_private.h"   // for g_APinDescription, pinPeripheral()




//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define ERROR_LED_PIN  13 // Error LED Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  HIGH // LED state that turns the error LED ON
#define configCHECK_FOR_STACK_OVERFLOW 2

// ADC PIN
#define ADC_PIN   A1       // A1 to avoid A0/DAC functionality

// static CANFDMessage can_message;

// Select the serial port for output
//#define SERIAL          Serial // For Adafruit SAME51 boards

//**************************************************************************
// Global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_ADCTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_ledTask;
TaskHandle_t Handle_canTxTask;
volatile u_int16_t adc_1; 


//**************************************************************************
// LVDT struct
//**************************************************************************
typedef struct __attribute__((packed)) {
	uint16_t lvdt_right;
	uint16_t lvdt_left;
} LvdtFrame_t;


//**************************************************************************
// Delay helpers
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay(us / portTICK_PERIOD_US);  
}

void myDelayMs(int ms)
{
  vTaskDelay((ms * 1000) / portTICK_PERIOD_US);  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil(previousWakeTime, (ms * 1000) / portTICK_PERIOD_US);  
}

//**************************************************************************
// ADC helpers
//**************************************************************************

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
  myDelayMs(50);
}

int send_can_msg(u_int16_t can_id, u_int8_t can_msg_len, u_int8_t *data, u_int8_t data_len) {
    if ((can_msg_len < 0) || (can_msg_len > 64) || (data_len > can_msg_len)) {
        return 2; // failure, data too long
    }
 
    CANFDMessage can_message;
    can_message.idx = 0;
    can_message.ext = false;
    can_message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
    can_message.id = can_id & 0x1FFFFFFF;
    can_message.len = can_msg_len;
 
    memcpy(can_message.data, data, data_len);
 
    if (!can_message.ext) {
        can_message.id &= 0x7FF ;
    }
   
    for (int i = 0; i < 100; i++) {
        if (can1.tryToSendReturnStatusFD(can_message) == 0) {
            return 0; // success
        }
        myDelayUs(1);
    }
 
    return 1; // failure
}


//*****************************************************************
// Thread A: Prints "A" every 500 ms for 100 times, then deletes itself
//*****************************************************************
static void threadA(void *pvParameters) 
{
  Serial.println("Thread A: Started");
  for (int x = 0; x < 100; ++x)
  {
    Serial.print("A");
    Serial.flush();
    myDelayMs(500);
  }
  Serial.println("Thread A: Deleting");
  vTaskDelete(NULL);
}

//*****************************************************************
// Thread B: Prints "B" every 2 seconds forever
//*****************************************************************
static void threadADC(void *pvParameters) 
{
  Serial.println("Thread ADC: Started");

  // Initialise ADC once
  ADC_Init(ADC_PIN);
  Serial.println("ADC initialised on A1");

  adc_1 = 69;
  float voltage = 0.0f;
  uint32_t sampleCount = 0;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    u_int16_t adc_1_read;
    if (adc_convert_once(adc_1_read)) {
      adc_1 = adc_1_read;
      sampleCount++;
      voltage = (adc_1 - 10) * (3.3f / 4095.0f); // calibration offset like before
      Serial.print("ADC Raw: "); Serial.print(adc_1);
      Serial.print("   Voltage: "); Serial.print(voltage, 4);
      Serial.println(" V");
    } else {
      Serial.println("ADC timeout or read error");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2500));  // 2 Hz sampling rate
  }
  // while (1)
  // {
  //   Serial.println("ADC test thread");
  //   Serial.flush();
  //   myDelayMs(2000);
  // }
}

//*****************************************************************
// LED Blink Task: Toggles LED_BUILTIN every 500 ms forever
//*****************************************************************
static void ledTask(void *pvParameters)
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("LED Blink Task: Started");
  
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    myDelayMs(500);
  }
}

//*****************************************************************
// CAN TX Task: Sends "testing" message every second
//*****************************************************************
static void canTxTask(void *pvParameters) {
  Serial.println("CAN TX Task: Started");
  

   while (1) {
    // send lvdt vals
    LvdtFrame_t vals; 
    vals.lvdt_left = 1; 
    vals.lvdt_right = adc_1; // adc value here
    int error = send_can_msg(0x22, 8, (u_int8_t*)&vals, sizeof(vals));
    if (error == 1) {
      Serial.println("CAN TX fail");
    }
    myDelayMs(2000);
    Serial.print("adc val canTX:");
    Serial.println(adc_1);
   }
  



  // // Initialize CAN at 500 kbps
  // if (!CAN.begin(500E3)) {
  //   Serial.println("Starting CAN failed!");
  //   vTaskDelete(NULL);
  // }

  // while (1) {
  //   CAN.beginPacket(0x123); // Arbitrary CAN ID
  //   CAN.write('t');
  //   CAN.write('e');
  //   CAN.write('s');
  //   CAN.write('t');
  //   CAN.write('i');
  //   CAN.write('n');
  //   CAN.write('g');
  //   CAN.endPacket();

  //   Serial.println("Sent CAN message: testing");
  //   myDelayMs(1000);
  // }
}

//*****************************************************************
// Monitor Task: Prints heap, task stats, and stack usage every 10 s
//*****************************************************************
static char ptrTaskList[1024];

void taskMonitor(void *pvParameters)
{
  int measurement;

  Serial.println("Task Monitor: Started");

  while (1)
  {
    myDelayMs(10000); // print every 10 seconds

    Serial.println("\n****************************************************");
    Serial.print("Free Heap: ");
    Serial.print(xPortGetFreeHeapSize());
    Serial.println(" bytes");

    Serial.print("Min Heap: ");
    Serial.print(xPortGetMinimumEverFreeHeapSize());
    Serial.println(" bytes");

    Serial.println("****************************************************");
    Serial.println("Task            ABS             %Util");
    Serial.println("****************************************************");

    vTaskGetRunTimeStats(ptrTaskList);
    Serial.println(ptrTaskList);

    Serial.println("****************************************************");
    Serial.println("Task            State   Prio    Stack   Num     Core");
    Serial.println("****************************************************");

    vTaskList(ptrTaskList);
    Serial.println(ptrTaskList);

    Serial.println("****************************************************");
    Serial.println("[Stacks Free Bytes Remaining] ");

    measurement = uxTaskGetStackHighWaterMark(Handle_aTask);
    Serial.print("Thread A: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_ADCTask);
    Serial.print("Thread ADC: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_monitorTask);
    Serial.print("Monitor Stack: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_ledTask);
    Serial.print("LED Task: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_canTxTask);
    Serial.print("CAN TX Task: "); 
    Serial.println(measurement);

    Serial.println("****************************************************");
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete(NULL);
}

//*****************************************************************
// Setup: Initializes Serial, LEDs, and creates tasks
//*****************************************************************
void setup() 
{
  Serial.begin(115200);
  delay(1000);
  //while (!Serial);

  Serial.println("");
  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");

  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);
  vSetErrorSerial(&Serial);

  // 500kBps arbitration, 4MBps data
  ACANFD_FeatherM4CAN_Settings settings(
    ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz,
    500 * 1000,
    875,
    DataBitRateFactor::x8,
    700
  );
   
  settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD;

  settings.mBitRatePrescaler          = 1;   // BRP
  settings.mArbitrationPhaseSegment1  = 85;  // PropSeg + PhaseSeg1
  settings.mArbitrationPhaseSegment2  = 10;  // PhaseSeg2
  settings.mArbitrationSJW            = 1;

  settings.mDataPhaseSegment1         = 8;   // dPropSeg + dPhaseSeg1
  settings.mDataPhaseSegment2         = 3;   // dPhaseSeg2
  settings.mDataSJW                   = 1;
  can1.beginFD(settings);

  xTaskCreate(threadA,     "Task A",       256, NULL, tskIDLE_PRIORITY + 3, &Handle_aTask);
  xTaskCreate(threadADC,     "Task ADC",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_ADCTask);
  xTaskCreate(taskMonitor, "Task Monitor", 512, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);
  xTaskCreate(ledTask,     "LED Blink",    128, NULL, tskIDLE_PRIORITY + 1, &Handle_ledTask);
  xTaskCreate(canTxTask,   "CAN TX",       512, NULL, tskIDLE_PRIORITY + 2, &Handle_canTxTask);

  vTaskStartScheduler();

  while (1)
  {
    Serial.println("Scheduler Failed!");
    delay(1000);
  }
}

//*****************************************************************
// Loop: Now acts as RTOS idle hook
//*****************************************************************
void loop() 
{
  Serial.print(".");
  delay(100);
}
