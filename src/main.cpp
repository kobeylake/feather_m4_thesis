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

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define ERROR_LED_PIN  13 // Error LED Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE  HIGH // LED state that turns the error LED ON
#define configCHECK_FOR_STACK_OVERFLOW 2

// static CANFDMessage can_message;

// Select the serial port for output
//#define SERIAL          Serial // For Adafruit SAME51 boards

//**************************************************************************
// Global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_ledTask;
TaskHandle_t Handle_canTxTask;


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
static void threadB(void *pvParameters) 
{
  Serial.println("Thread B: Started");

  while (1)
  {
    Serial.println("B");
    Serial.flush();
    myDelayMs(2000);
  }
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

  CANFDMessage can_message;

  can_message.idx = 0 ;
  can_message.ext = false; //always false
  can_message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH; //always configured this way
  // can_message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
  can_message.id = 0x3 & 0x1FFFFFFF ; // rail condition (me) number 3

  if (!can_message.ext) {
      can_message.id &= 0x7FF ;
  }

  // message.len = Valid values are: 0, 1, ..., 8, 12, 16, 20, 24, 32, 48, 64
  can_message.len = 20;

  const int i = 21;

  memcpy(&can_message.data, &i, sizeof(int));

  while (1) {
    const uint32_t sendStatus = can1.tryToSendReturnStatusFD(can_message);
    if (sendStatus != 0) {
        Serial.print("CAN TX Error: ");
        Serial.println(sendStatus);
    }
    myDelayMs(1000);
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

    measurement = uxTaskGetStackHighWaterMark(Handle_bTask);
    Serial.print("Thread B: ");
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
  while (!Serial);

  Serial.println("");
  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");

  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);
  vSetErrorSerial(&Serial);

  // 500kBps arbitration, 4MBps data
  ACANFD_FeatherM4CAN_Settings settings(ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz, 500 * 1000, DataBitRateFactor::x8);

  settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD ;

  can1.beginFD(settings);

  xTaskCreate(threadA,     "Task A",       256, NULL, tskIDLE_PRIORITY + 3, &Handle_aTask);
  xTaskCreate(threadB,     "Task B",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
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
