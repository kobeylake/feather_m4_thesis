#include <Arduino.h>

#include <Wire.h>

#define INC_ADDRESS 0x69
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

uint16_t x, y, z;
uint16_t gyr_x, gyr_y, gyr_z;

uint16_t  temperature = 0;
float     temperatureInDegree = 0.f;


//Write data in 16 bits
void writeRegister16(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  //Low 
  Wire.write((uint16_t)value & 0xff);
  //High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

void softReset(){  
  writeRegister16(CMD, 0xDEAF);
  delay(50);    
}


//Read data in 16 bits
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 2);  
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }  
  return (data[0]   | (uint16_t)data[1] << 8);
}

//Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 20);
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  
  int offset = 0;  
  x =             (data[offset + 0]   | (uint16_t)data[offset + 1] << 8);  //0x03
  y =             (data[offset + 2]   | (uint16_t)data[offset + 3] << 8);  //0x04
  z =             (data[offset + 4]   | (uint16_t)data[offset + 5] << 8);  //0x05
  gyr_x =         (data[offset + 6]   | (uint16_t)data[offset + 7] << 8);  //0x06
  gyr_y =         (data[offset + 8]   | (uint16_t)data[offset + 9] << 8);  //0x07
  gyr_z =         (data[offset + 10]  | (uint16_t)data[offset + 11] << 8); //0x08
  temperature =   (data[offset + 12]  | (uint16_t)data[offset + 13] << 8); //0x09
  temperatureInDegree = (temperature/512.f) + 23.0f;  
}


void setup(void) {  
  Serial.begin(115200); 
  //Accelerometer
  Wire.begin();  
  //Wire.setClock(400000);      // I2C Fast Mode (400kHz)  
  //softReset();  
  /*
   * Acc_Conf P.91
   * mode:        0x7000  -> High
   * average:     0x0000  -> No
   * filtering:   0x0080  -> ODR/4
   * range:       0x0000  -> 2G
   * ODR:         0x000B  -> 800Hz
   * Total:       0x708B
   */
  //writeRegister16(ACC_CONF,0x708B);//Setting accelerometer  
  /*
   * Gyr_Conf P.93
   * mode:        0x7000  -> High
   * average:     0x0000  -> No
   * filtering:   0x0080  -> ODR/4
   * range:       0x0000  -> 125kdps
   * ODR:         0x000B  -> 800Hz
   * Total:       0x708B
   */
  //writeRegister16(GYR_CONF,0x708B);//Setting gyroscope    
}

void loop() {
  Wire.beginTransmission(INC_ADDRESS);  // I2C address
  Wire.write(0x00);              // Read CHIP_ID
  int error = Wire.endTransmission(false);
  if( error == 0)
    Serial.println( "register selected");
  else{
    Serial.print( "Error n°: ");  
    Serial.println(error);      
  }
  int n = Wire.requestFrom(INC_ADDRESS, 2, true);
  if( n == 2)
  {
    Serial.println("Got 2");
    int data1 = Wire.read();
    int data2 = Wire.read();
    Serial.println( data1, HEX);
    Serial.println( data2, HEX);
  }
  else
  {
    Serial.println("Ouch, that hurts");
  }
  delay(250);

  /*
  if(readRegister16(0x02) == 0x00) {
    //Read ChipID
    //Serial.print("ChipID:");
    //Serial.println(readRegister16(0x00));    
    readAllAccel();             // read all accelerometer/gyroscope/temperature data     
    Serial.print("x:");
    Serial.print(x);
    Serial.print(" | y:");
    Serial.print(y);
    Serial.print(" | z:");
    Serial.print(z);
    Serial.print(" | gyr_x:");
    Serial.print(gyr_x);
    Serial.print(" | gyr_y:");
    Serial.print(gyr_y);
    Serial.print(" | gyr_z:");
    Serial.print(gyr_z);
    Serial.print(" | temp:");
    Serial.println(temperatureInDegree);
    
  }
  */
}