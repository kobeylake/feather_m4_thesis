//**************************************************************************
// FreeRTOS on SAME51 (Feather M4 CAN)
// Original: Scott Briscoe
// Modified: Added ...
//**************************************************************************

#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (2000)

#include <Arduino.h>
#include <FreeRTOS.h>
#include <FreeRTOS_SAMD51.h>
#include <ACANFD_FeatherM4CAN.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h>
#include "bmi323.h"
#include "bmi3_arduino_common.h"
#include "wiring_private.h"   // for g_APinDescription, pinPeripheral()



//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define ERROR_LED_PIN  13 // Error LED Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  HIGH // LED state that turns the error LED ON
#define configCHECK_FOR_STACK_OVERFLOW 2

// ADC PIN
#define ADC_PIN   A1       // A1 to avoid A0/DAC functionality
#define ADC_PIN2  A2


// static CANFDMessage can_message;

// Select the serial port for output
//#define SERIAL          Serial // For Adafruit SAME51 boards

//**************************************************************************
// Global variables
//**************************************************************************
TaskHandle_t Handle_accelTask;
TaskHandle_t Handle_ADCTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_canADCTxTask;
TaskHandle_t Handle_canAccelTxTask;

SemaphoreHandle_t accelDataMutex;

volatile u_int16_t adc_1; 
volatile u_int16_t adc_2;
volatile u_int16_t accel_x[3200];
volatile u_int16_t accel_y[3200];

static bmi3_dev dev = {0};

//**************************************************************************
// LVDT struct
//**************************************************************************
typedef struct __attribute__((packed)) {
	uint16_t lvdt_right;
	uint16_t lvdt_left;
} LvdtFrame_t;


//**************************************************************************
// Accelerometer struct
//**************************************************************************
typedef struct __attribute__((packed)) {
	int16_t acc_x[3200];
	int16_t acc_y[3200];
} AccFrame_t;

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

// Configure accelerometer parameters
static int8_t set_accel_config(bmi3_dev *d) {
  int8_t rslt;
  bmi3_sens_config cfg = {0};

  cfg.type = BMI323_ACCEL;

  rslt = bmi323_get_sensor_config(&cfg, 1, d);
  bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
  if (rslt != BMI323_OK) return rslt;

  cfg.cfg.acc.odr      = BMI3_ACC_ODR_3200HZ;
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


//**************************************************************************
// Helper to construct and send CAN Message
//**************************************************************************
int send_can_msg(u_int16_t can_id, u_int8_t can_msg_len, u_int8_t *data, u_int8_t data_len) {
    if ((can_msg_len < 0) || (can_msg_len > 64) || (data_len > can_msg_len)) {
        return 2; // failure, data too long
    }
 
    static CANFDMessage can_message;
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
// Thread Accel : Reads accelerometer data and fills in the global arrays
//*****************************************************************
static void threadAccel(void *pvParameters) 
{
  Serial.println("Thread Acc: Started");
  while(1)
  {
    static uint16_t idx = 0;
    uint16_t int_status = 0;

    if( xSemaphoreTake( accelDataMutex, ( TickType_t ) 5 ) == pdTRUE ) {
      if (bmi323_get_int1_status(&int_status, &dev) == BMI323_OK) {
        if (int_status & BMI3_INT_STATUS_ACC_DRDY) {
          bmi3_sensor_data sd = {0};
          sd.type = BMI323_ACCEL;
          if (bmi323_get_sensor_data(&sd, 1, &dev) == BMI323_OK) {

            // store raw values in global arrays
            accel_x[idx] = sd.sens_data.acc.x;
            accel_y[idx] = sd.sens_data.acc.y;
            idx++;

            // wrap around when we hit 3200 samples
            if (idx >= 3200) {
              idx = 0;
              Serial.println("Accel buffer full, wrapping around");
            }

            

             if (idx % 100 == 0) {
              float gx = lsb_to_g(sd.sens_data.acc.x, 2.0f, dev.resolution);
              float gy = lsb_to_g(sd.sens_data.acc.y, 2.0f, dev.resolution);
              float gz = lsb_to_g(sd.sens_data.acc.z, 2.0f, dev.resolution);
              Serial.print("Sample ");
              Serial.print(idx);
              Serial.print(": ");

              // raw values
              Serial.print("Raw [");
              Serial.print(sd.sens_data.acc.x);
              Serial.print(", ");
              Serial.print(sd.sens_data.acc.y);
              Serial.print(", ");
              Serial.print(sd.sens_data.acc.z);
              Serial.print("]  ");

              // converted g-values
              Serial.print("G [");
              Serial.print(gx, 4);
              Serial.print(", ");
              Serial.print(gy, 4);
              Serial.print(", ");
              Serial.print(gz, 4);
              Serial.println("]");
            }

          }
        }
      }
      xSemaphoreGive(accelDataMutex);
    }
    myDelayUs(100);  // 100us delay gives other tasks a chance to run

  }
}

//*****************************************************************
// Thread ADC : Prints ADC value every 2 seconds forever
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

    
      // debugging LED
      if (adc_1 > 2000) {
        // rgb led on
        digitalWrite(LED_BUILTIN, LOW);
      }
      if (adc_1 <= 2000) {
        // rgb led off
        digitalWrite(LED_BUILTIN, HIGH);
      }


    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2500));  // sample every 2.5sec
  }
  // while (1)
  // {
  //   Serial.println("ADC test thread");
  //   Serial.flush();
  //   myDelayMs(2000);
  // }
}



//*****************************************************************
// CAN TX Task: Sends status of LVDT every 2 second
//*****************************************************************
static void canADCTxTask(void *pvParameters) {
  Serial.println("CAN ADC TX Task: Started");
  

   while (1) {
    // send lvdt vals
    static LvdtFrame_t vals; 
    vals.lvdt_left = 1; 
    vals.lvdt_right = adc_1; // adc value here
    int error = send_can_msg(0x22, 8, (u_int8_t*)&vals, sizeof(vals));
    if (error == 1) {
      Serial.println("CAN TX fail (LVDT)");
    }
    myDelayMs(2000);
    Serial.print("adc val canTX:");
    Serial.println(adc_1);
   }
  
}


//*****************************************************************
// CAN TX Accel Task: Sends accelerometer data every 30 sec
//*****************************************************************
static void canAccelTxTask(void *pvParameters) {
  Serial.println("CAN Accel TX Task: Started");

  int16_t chunk[32];
  
  while (1) {
    // CAN FD supports up to 64 bytes per message
    // 64 bytes = 32 uint16_t values per message
    // we have 3200 samples, so we need 3200/32 = 100 messages per axis


    myDelayMs(30000);

    Serial.println("Freezinf buffer to start accel data transmission");
    xSemaphoreTake(accelDataMutex, portMAX_DELAY);

    
    // send x-axis data
    for (int i = 0; i < 3200; i += 32) {
      for (int j = 0; j < 32; j++) {
        chunk[j] = accel_x[i + j];
      }
      
      int error = send_can_msg(0x23, 64, (uint8_t*)chunk, sizeof(chunk));
      if (error == 1) {
        Serial.println("CAN Accel X TX fail");
      }
      myDelayUs(100); // small delay between messages
    }
    
    Serial.println("Sent all X axis data");
    
    // send y-axis data
    for (int i = 0; i < 3200; i += 32) {
      for (int j = 0; j < 32; j++) {
        chunk[j] = accel_y[i + j];
      }
      
      int error = send_can_msg(0x24, 64, (uint8_t*)chunk, sizeof(chunk));
      if (error == 1) {
        Serial.println("CAN Accel Y TX fail");
      }
      myDelayUs(100);
    }
    
    Serial.println("un-freezing buffer after sending all Y axis data");
    xSemaphoreGive(accelDataMutex);
    
    
  }
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

    measurement = uxTaskGetStackHighWaterMark(Handle_accelTask);
    Serial.print("Thread Accel: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_ADCTask);
    Serial.print("Thread ADC: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_monitorTask);
    Serial.print("Monitor Stack: ");
    Serial.println(measurement);

    measurement = uxTaskGetStackHighWaterMark(Handle_canADCTxTask);
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

  // debugging LED 
  pinMode(LED_BUILTIN, OUTPUT);

  // Start with LED off
  digitalWrite(LED_BUILTIN, LOW);

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

  // Initialise the BMI323 sensor - error somewhere in these 3 lines which make code stop
  rslt = bmi323_init(&dev);
  bmi3_error_codes_print_result("bmi323_init", rslt);
  
  // while (1){
  //   Serial.print("bmi323_init result: ");
  //   Serial.println(rslt);
  //   Serial.flush();
  // }
  // getting a return of -2 here. a BMI3_E_COM_FAIL 

  if (rslt != BMI323_OK) return;

  // Configure accelerometer
  rslt = set_accel_config(&dev);
  if (rslt == BMI323_OK) {
    Serial.println("idx,raw_x,raw_y,raw_z,gx,gy,gz");
  }


  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);
  vSetErrorSerial(&Serial);

  // Create mutex for protecting accelerometer data
  accelDataMutex = xSemaphoreCreateMutex();
  if (accelDataMutex == NULL) {
    while(1) {
      Serial.println("Failed to create mutex!");
    }    
  }

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


  // Create tasks and start kernal
  xTaskCreate(threadAccel,     "Task Accelerometer",       512, NULL, tskIDLE_PRIORITY + 2, &Handle_accelTask);
  xTaskCreate(threadADC,     "Task ADC",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_ADCTask);
  xTaskCreate(taskMonitor, "Task Monitor", 512, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);
  xTaskCreate(canADCTxTask,   "CAN ADC TX",       512, NULL, tskIDLE_PRIORITY + 2, &Handle_canADCTxTask);
  xTaskCreate(canAccelTxTask,   "CAN Accelerometer TX",       1024, NULL, tskIDLE_PRIORITY + 2, &Handle_canAccelTxTask);

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
  //Serial.print(".");
  delay(100);
}
