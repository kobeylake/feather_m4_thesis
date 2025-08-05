
#ifndef ARDUINO_FEATHER_M4_CAN
  #error "This sketch should be compiled for Arduino Feather M4 CAN (SAME51)"
#endif

//-----------------------------------------------------------------
// IMPORTANT:
//   <ACANFD_FeatherM4CAN.h> should be included only from the .ino file
//   From an other file, include <ACANFD_FeatherM4CAN-from-cpp.h>
//   Before including <ACANFD_FeatherM4CAN.h>, you should define 
//   Message RAM size for CAN0 and Message RAM size for CAN1.
//   Maximum required size is 4,352 (4,352 32-bit words).
//   A 0 size means the CAN module is not configured; its TxCAN and RxCAN pins
//   can be freely used for an other function.
//   The begin method checks if actual size is greater or equal to required size.
//   Hint: if you do not want to compute required size, print
//   can1.messageRamRequiredMinimumSize () for getting it.

#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (2000)

#include <Wire.h>
#include <time.h>
#include <ACANFD_FeatherM4CAN.h>
#include <SparkFunTMP102.h>

typedef struct __attribute__((packed)) {
    uint8_t packet_type;
    int16_t sleeper_temp;
    uint8_t sleeper_humidity;
    uint8_t ballast_moisture[2];
    bool water_presence;
    uint16_t rainfall_count;
    int16_t ballast_temp[4];
    bool train_present;
} EnvironmentalPacket1;

void toggle_led(void) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

//-----------------------------------------------------------------

void print_packet(const EnvironmentalPacket1 &p);
EnvironmentalPacket1 make_packet_1(void);
void show_can_settings(ACANFD_FeatherM4CAN_Settings settings);

TMP102 sensor0;

void setup () {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Wire.begin();
    while (!Serial) {
        delay(50);
        digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
    }
    srand(time(NULL));

    if(!sensor0.begin())
    {
        Serial.println("Cannot connect to TMP102.");
        Serial.println("Is the board connected? Is the device ID correct?");
        while(1);
    }

    // 500kBps arbitration, 4MBps data
    ACANFD_FeatherM4CAN_Settings settings(ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz, 500 * 1000, DataBitRateFactor::x8);

    settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD ;

    Serial.println ("CAN1 CAN FD HIL Tester") ;
    show_can_settings(settings);

    const uint32_t errorCode = can1.beginFD(settings) ;

    Serial.print ("Message RAM required minimum size: ") ;
    Serial.print (can1.messageRamRequiredMinimumSize ()) ;
    Serial.println (" words") ;

    if (0 == errorCode) {
        Serial.println ("can configuration ok") ;
    }else{
        Serial.print ("Error can configuration: 0x") ;
        Serial.println (errorCode, HEX) ;
    }
}

//-----------------------------------------------------------------

static CANFDMessage can_message ;
u_int64_t loop_index = 0;
float temperature;

void loop () {
    sensor0.wakeup();
    temperature = sensor0.readTempC();
    sensor0.sleep();

    Serial.print("Temperature: ");
    Serial.println(temperature);


    can_message.idx = 0 ;
    can_message.ext = false;
    can_message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
    // can_message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
    can_message.id = 0x2 & 0x1FFFFFFF ;

    if (!can_message.ext) {
        can_message.id &= 0x7FF ;
    }

    EnvironmentalPacket1 src = make_packet_1();

    can_message.len = 20;
    // message.len = Valid values are: 0, 1, ..., 8, 12, 16, 20, 24, 32, 48, 64
    memcpy(&can_message.data, &src, sizeof(EnvironmentalPacket1));

    const uint32_t sendStatus = can1.tryToSendReturnStatusFD(can_message);


    if (sendStatus == 0) {
        Serial.print ("Packet number: ") ;
        Serial.print (loop_index++) ;
        Serial.print (", id 0x") ;
        Serial.print (can_message.id, HEX) ;
        Serial.print (", type ") ;
        Serial.print (can_message.type) ;
        Serial.print (" (") ;
        switch (can_message.type) {
            case CANFDMessage::CAN_REMOTE :
                Serial.print ("remote") ;
                break ;
            case CANFDMessage::CAN_DATA :
                Serial.print ("CAN 2.0B data") ;
                break ;
            case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH :
                Serial.print ("CANFD, no BRS") ;
                break ;
            case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH :
                Serial.print ("CANFD, BRS") ;
                break ;
        }
        Serial.print ("), length ") ;
        Serial.print (can_message.len) ;
        Serial.println() ;
    } else if (sendStatus == 0) {
        Serial.print("Sent error 0x");
        Serial.println(sendStatus);
    }

    delay(10000);
}
 