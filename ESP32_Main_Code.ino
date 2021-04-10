#include "BluetoothSerial.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

#define SHT_LOX1 15
#define SHT_LOX2 19
#define SHT_LOX3 32

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;


int sensorPin = 33;    // select the input pin for the potentiometer      // select the pin for the LED
int16_t sensorValue = 0;  // variable to store the value coming from the sensor

char left='x',right='x',back='x',warn='x',fall='x';
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void setup() {
  Serial.begin(115200);// declare the ledPin as an OUTPUT:
  SerialBT.begin("SeismoFrost"); //Bluetooth device name
  pinMode(33, INPUT);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
 
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  
  setID();
  //setting up MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  sensorValue = (int) (sensorValue/50);
  sensorValue = map(sensorValue, 18, 54, 0, 80);
  if(sensorValue >= 70){
    warn = 'd';
  }else{
    warn = 'x';
  }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  read_dual_sensors();
  if(a.acceleration.y*100 <= -750.00 || a.acceleration.y*100 >= 750.00){
    //while(a.acceleration.y*100 <= -750.00 || a.acceleration.y*100 >= 750.00){
    //sensors_event_t a, g, temp;
    //mpu.getEvent(&a, &g, &temp);
    fall = 'f';
  } else{
    fall = 'x';
  }
 
  SerialBT.print(sensorValue);
  SerialBT.print("|");
  SerialBT.print(warn);
  SerialBT.print("|");
  SerialBT.print(back);
  SerialBT.print("|");
  SerialBT.print(left);
  SerialBT.print("|");
  SerialBT.print(right);
  SerialBT.print("|");
  SerialBT.print(fall);
  SerialBT.print("|");
  delay(10);
  
 /* Serial.print(sensorValue);
  Serial.print("/");
  Serial.print(warn);
  Serial.print("/");
  Serial.print(back);
  Serial.print("/");
  Serial.print(left);
  Serial.print("/");
  Serial.print(right);
  Serial.println("/");
  Serial.println(fall);
  delay(10);*/
 
}


void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  if(measure1.RangeStatus != 4){
    if(measure1.RangeMilliMeter <=100){
    left = 'a';
  }else{
    left = 'x';
  }
  }
  if(measure2.RangeStatus != 4){
    if(measure2.RangeMilliMeter <= 100){
       right = 'b';
    }else{
      right = 'x';
    }
   
  }
  if(measure3.RangeStatus != 4){
    if(measure3.RangeMilliMeter <= 100){
    back = 'c';
    }else{
      back = 'x';
    }
  }
}
