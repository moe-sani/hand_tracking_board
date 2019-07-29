/**************************************************************************/
/*
    IMU SERIAL CODE

    History:

    Ver 01: just sends sensor A and B in a bit mode
    Ver_02: send a jason code and read it on computer: works for one sensor, speed:20hz
    Ver_03: wow, this appearantly works on 100 hertz!
    Ver_04: adding more i2c sensors....... this works with 50Hz appearantly!
    Ver_05: adding EEPROM >> storage function works well
    Ver_07: adding strain gauge:works well
    Ver_08: reading 4 set of sensors...
    Ver_09: this version can read up to 4 channels
    Ver_10: trying to have memory part inside the class:tested with Person
    Ver_13 : until now, one sensor calibration store and send works fine. now I gonna try all sensors... there is a problem, after all of the calibrtions, program stops!
    ver_14: restore calibration from predefined data:done, send all sensors with something like 33Hz
    Ver_16: so far so good
    
*/
/**************************************************************************/




#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HX711.h"



//#include<EEPROM.h>

typedef struct {
  boolean valid;
  char name[100];
  char surname[100];
} Person;

struct sensorOutputs {
//    int ox;
//    int oy;
//    int oz;
    int x;
    int y;
    int z;
    int w;
};




// The two BNO055 modules, bnoB has the ADR pin wired to 3.3v to change its i2c address
// Both are wired: SCL to analog 5, SDA to analog 4, VIN to 5v, GRN to ground

/* This code uses I2C Protocol  */

/**************************************************************************/
/*
    GLOBAL VARIABLES
*/
/**************************************************************************/

#define BNO055_SAMPLERATE_DELAY_MS (1)
// Include EEPROM-like API for FlashStorage
//#include <FlashAsEEPROM.h>
#include <FlashStorage.h>
//FlashStorage(my_flash_store, adafruit_bno055_offsets_t);
//FlashStorage(my_flash_store0, Person);
FlashStorage(my_flash_store0, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store1, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store2, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store3, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store4, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store5, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store6, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store7, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store8, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store9, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store10, adafruit_bno055_offsets_t);
FlashStorage(my_flash_store11, adafruit_bno055_offsets_t);

//__attribute__((__aligned__(256))) \
//static const uint8_t PPCAT(_data,my_flash_store2)[(sizeof(Person)+255)/256*256] = { }; \
//FlashStorageClass<Person> my_flash_store2(PPCAT(_data,my_flash_store2));
// pattern is : accel , mag, gyro, accel radious, mag radious
static const adafruit_bno055_offsets_t Predefined_Calibration[]= {{21975, 0, 3928, 8192, 3, 0 ,  127, 0, 21517, 0,2 },  //0
                                                                    {21975, 0, 3928, 8192, 3, 0 ,  127, 0, 21517, 0,2 },//1
                                                                    {18, -18, 12, -150, -307, -470 , -1, 0, -1 , 1000, 742},//2
                                                                    { 24, 39, 10 , -761, -438, 305 , 2, 1, 3 , 1000, 947},//3
                                                                    { 5, 35, 10 , -387, -216, -525 , -2, -2, 1 , 1000, 573},//4
                                                                    {-7, 24, 10 , 813, 403, -29 , -2, -1, -1 , 1000, 570},//5
                                                                    {16, 26, 15 , -52, -77, -578 , 0, 0, 0 , 1000, 556},//6
                                                                    {-7, 56, 1 , 263, 150, -383 , -3, -4, 0 , 1000, 541},//7
                                                                    {14, -6, 17 , -1055, 257, 742 , -1, -1, -1 , 1000, 576},//8
                                                                    {14, -6, 17 , -1051, 263, 740, -1, -1, -1 , 1000, 596},//9
                                                                    {11, 21, 35 , -86, 122, -872 , -2, -2, 0 , 1000, 646},//10
                                                                    {11, 21, 35 , -79, 114, -865 , -2, -2, 0 , 1000, 704}};//11

//Adafruit_BNO055 bno0A = Adafruit_BNO055(55, 0x28); /* A Sensor Address */
//Adafruit_BNO055 bno0B = Adafruit_BNO055(56, 0x29); /* B Sensor Address */



int val=0;

float sensorA_OX;
float sensorA_OY;
float sensorA_OZ;
float sensorB_OX;
float sensorB_OY;
float sensorB_OZ;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
HX711 scale;
long scale_data;
unsigned int i=0;
#define TCA9548A_I2C_ADDRESS  0x70
#define PIN_S0 0
#define PIN_S1 1
#define PIN_S2 2
#define LED_SHIELD_1 7
#define LED_SHIELD_2 6
#define LED_SHIELD_3 A6


/**************************************************************************/
/*
    FUNCTION LIST
*/
/**************************************************************************/
void serial_send(float data);
void Init_bno();
void displaySensorDetails(void);
void displayCalStatusA(void);
void displayCalStatusB(void);
void displaySensorOffsetsA(const adafruit_bno055_offsets_t &calibDataA);
void displaySensorOffsetsB(const adafruit_bno055_offsets_t &calibDataB);

boolean enableMuxPort(byte portNumber);
boolean disableMuxPort(byte portNumber);


/**************************************************************************/
/*
    class for sensors
*/
/**************************************************************************/
class imu_sensor {
    private:
        byte pinLED;
        uint8_t address;
        uint8_t channel;
        uint8_t sensor_id;
        Adafruit_BNO055 bno;
        bool isConnected;

    public:
        imu_sensor(uint8_t sensor_id,uint8_t channel, uint8_t address)
        {
            this->channel=channel;
            this->address=address;
            this-> sensor_id=sensor_id;
            bno = Adafruit_BNO055(sensor_id, address); /* A Sensor Address */
        }

        void init()
        {
            byte value = 0;
            Serial.println("==========================");
            Serial.print("Initalizing sensor ");
            Serial.print(this-> sensor_id);
            Serial.print("on port:");
            Serial.print(this->channel);
            Serial.print("on address:");
            Serial.print(this->address);
            Serial.println(" ");

            enableMuxPort(this->channel);
            Serial.println("WHOAMI register reply:");
            Wire.beginTransmission(this->address);
            Wire.write((uint8_t)0x00);
            Wire.endTransmission();
            Wire.requestFrom(this->address, (byte)1);
            value = Wire.read();
            Serial.println(value);         // print the character
            delay(500);
            if(!bno.begin())
              {
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
                Serial.println(" ");
                this->isConnected= false;
              }
              else
              {
                this->isConnected= true;
                Serial.println("This sensor is alive!");
              }
            disableMuxPort(this->channel);
        }

        void calibrate (FlashStorageClass<adafruit_bno055_offsets_t> my_flash_store)
        {
            Serial.print("Calibration for sensor ");
            Serial.print(this-> sensor_id);
            Serial.print("on port:");
            Serial.print(this->channel);
            Serial.print("on address:");
            Serial.print(this->address);
            Serial.println(" ");
            enableMuxPort(this->channel);

            adafruit_bno055_offsets_t calibrationDataA;
            // Read the content of "my_flash_store" into the "owner" variable
            calibrationDataA = my_flash_store.read();
            this->displaySensorOffsets(calibrationDataA);

            delay(500);
            sensor_t sensorA;

            //bno.getSensor(&sensorA);

            /* Display some basic information on this sensor A and B*/
            this->displaySensorDetails();

            /* Optional: Display current status A and B*/
            //this->displaySensorStatus();

            bno.setExtCrystalUse(true);

            sensors_event_t eventA;

            if(calibrationDataA.accel_offset_x==0)
            {
                Serial.println("Please Calibrate Sensor: ");
                while(!bno.isFullyCalibrated()){
                  bno.getEvent(&eventA);
                  Serial.print("X'A': ");
                  Serial.print(eventA.orientation.x, 4);
                  Serial.print("\tY'A': ");
                  Serial.print(eventA.orientation.y, 4);
                  Serial.print("\tZ'A':  ");
                  Serial.print(eventA.orientation.z, 4);

                  this->displayCalStatus();
                  Serial.println("");
                  delay(BNO055_SAMPLERATE_DELAY_MS);
                }
                Serial.println("\n\nCalibration Is Done!");
                adafruit_bno055_offsets_t newCaliA;
                bno.getSensorOffsets(newCaliA);
                this->displaySensorOffsets(newCaliA);
                // Save new calibration to arduino memory
                Serial.println("\n\nStoring calibration data to EEPROM...");

                my_flash_store.write(newCaliA);
            }
            else
            {
                bno.setSensorOffsets(calibrationDataA);
                Serial.println("Calibration data restored!");
            }
            disableMuxPort(this->channel);
            delay(1000);

        }

        ///**************************************************************************/
        /*
            Displays some basic information on this sensor from the unified
            sensor API sensor_t type (see Adafruit_Sensor for more information)
        */
        /**************************************************************************/


        void displaySensorDetails(void)
        {
          //enableMuxPort(this->channel);
          sensor_t sensorA;

          bno.getSensor(&sensorA);

          Serial.println("------------------------------------");
          Serial.print  ("Sensor:       "); Serial.println(sensorA.name);
          Serial.print  ("Driver Ver:   "); Serial.println(sensorA.version);
          Serial.print  ("Unique ID:    "); Serial.println(sensorA.sensor_id);
          Serial.print  ("Max Value:    "); Serial.print(sensorA.max_value); Serial.println(" xxx");
          Serial.print  ("Min Value:    "); Serial.print(sensorA.min_value); Serial.println(" xxx");
          Serial.print  ("Resolution:   "); Serial.print(sensorA.resolution); Serial.println(" xxx");
          Serial.println("------------------------------------");
          Serial.println("");
          delay(500);
          //disableMuxPort(this->channel);
        }

        /**************************************************************************/
        /*
            Display sensor calibration status
        */
        /**************************************************************************/
        void displayCalStatus(void)
        {
          //enableMuxPort(this->channel);
          /* Get the four calibration values (0..3) */
          /* Any sensor data reporting 0 should be ignored, */
          /* 3 means 'fully calibrated" */

          uint8_t systemA, gyroA, accelA, magA;
          systemA = gyroA = accelA = magA = 0;
          bno.getCalibration(&systemA, &gyroA, &accelA, &magA);

          /* The data should be ignored until the system calibration is > 0 */
          Serial.print("\t");
          if (!systemA)
          {
            Serial.print("! ");
          }

          /* Display the individual values */
          Serial.print("Sys'A':");
          Serial.print(systemA, DEC);
          Serial.print(" G'A':");
          Serial.print(gyroA, DEC);
          Serial.print(" A'A':");
          Serial.print(accelA, DEC);
          Serial.print(" M'A':");
          Serial.print(magA, DEC);
          //disableMuxPort(this->channel);
        }

         void displaySensorOffsets(const adafruit_bno055_offsets_t &calibDataA)
        {
            //enableMuxPort(this->channel);
            Serial.print("Accelerometer: ");
            Serial.print(calibDataA.accel_offset_x); Serial.print(" ");
            Serial.print(calibDataA.accel_offset_y); Serial.print(" ");
            Serial.print(calibDataA.accel_offset_z); Serial.print(" ");

            Serial.print("\nGyro: ");
            Serial.print(calibDataA.gyro_offset_x); Serial.print(" ");
            Serial.print(calibDataA.gyro_offset_y); Serial.print(" ");
            Serial.print(calibDataA.gyro_offset_z); Serial.print(" ");

            Serial.print("\nMag: ");
            Serial.print(calibDataA.mag_offset_x); Serial.print(" ");
            Serial.print(calibDataA.mag_offset_y); Serial.print(" ");
            Serial.print(calibDataA.mag_offset_z); Serial.print(" ");

            Serial.print("\nAccel Radius: ");
            Serial.print(calibDataA.accel_radius);

            Serial.print("\nMag Radius: ");
            Serial.print(calibDataA.mag_radius);
            //disableMuxPort(this->channel);
        }

        sensorOutputs read_calibration_status()
        {
         sensorOutputs newOutputs;
         enableMuxPort(this->channel);
          /* Get the four calibration values (0..3) */
          /* Any sensor data reporting 0 should be ignored, */
          /* 3 means 'fully calibrated" */

          uint8_t systemA, gyroA, accelA, magA;
          systemA = gyroA = accelA = magA = 0;
          bno.getCalibration(&systemA, &gyroA, &accelA, &magA);

          /* Display the individual values */
//          Serial.print("Sys'A':");
//          Serial.print(systemA, DEC);
//          Serial.print(" G'A':");
//          Serial.print(gyroA, DEC);
//          Serial.print(" A'A':");
//          Serial.print(accelA, DEC);
//          Serial.print(" M'A':");
//          Serial.print(magA, DEC);
           newOutputs.x=systemA;
           newOutputs.y=gyroA;
           newOutputs.z=accelA;
           newOutputs.w=magA;
          disableMuxPort(this->channel);
          return newOutputs;
        }

        sensorOutputs read()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {
              enableMuxPort(this->channel);

//            sensors_event_t eventA;
//            bno.getEvent(&eventA);


//            this is for test
            //imu::Quaternion quat;
//            Serial.print("Sensor: ");
//            Serial.println(this-> sensor_id);
            imu::Quaternion quat=bno.getQuat();
//            Serial.print(" quaternion w: ");
//            Serial.print(quat.w());
//            Serial.print(" quaternion x: ");
//            Serial.print(quat.x());
//            Serial.print(" quaternion y: ");
//            Serial.print(quat.y());
//            Serial.print(" quaternion z: ");
//            Serial.println(quat.z());
//
//            Serial.print(" orientation x: ");
//            Serial.print(eventA.orientation.x);
//            Serial.print(" orientation y: ");
//            Serial.print(eventA.orientation.y);
//            Serial.print(" orientation z: ");
//            Serial.println(eventA.orientation.z);
//
//
//
//            newOutputs.oz=(int)(eventA.orientation.x*10);//Note that I changed this to make it consistent with datasheet cordinate system
//            newOutputs.oy=(int)(eventA.orientation.y*10);
//            newOutputs.ox=(int)(eventA.orientation.z*10);
              newOutputs.x=(int)(quat.x()*10000);
              newOutputs.y=(int)(quat.y()*10000);
              newOutputs.z=(int)(quat.z()*10000);
              newOutputs.w=(int)(quat.w()*10000);

            disableMuxPort(this->channel);
            }
            else
            {
//                Serial.println("no sensor:");
              newOutputs.x=0;
              newOutputs.y=0;
              newOutputs.z=0;
              newOutputs.w=10000;
            }

            return newOutputs;
        }

        void restorePredefinedCalibration()
        {
            Serial.println("Restoring Predefined Calibration for sensor: ");
            Serial.print(this-> sensor_id);
            Serial.print("on port:");
            Serial.print(this->channel);
            Serial.print("on address:");
            Serial.print(this->address);
            Serial.println(" ");
            enableMuxPort(this->channel);
            enableMuxPort(this->channel);
            adafruit_bno055_offsets_t calibrationDataA = Predefined_Calibration[this->sensor_id];
            bno.setSensorOffsets(calibrationDataA);
            this->displaySensorOffsets(calibrationDataA);
            Serial.println("Predefined Calibration data restored!");
            disableMuxPort(this->channel);

        }


        //void store(Person owner,FlashStorageClass<adafruit_bno055_offsets_t> my_flash_store)
        //{

          //my_flash_store.write(owner);
          //delay(5000);



        //}
        //void restore(FlashStorageClass<Person> my_flash_store)
        //{
          //Serial.println("restoring values for sensor:");
          //Serial.print(this-> sensor_id);
          //Serial.println(" ");
          //Person owner;
          //owner = my_flash_store.read();
          //Serial.print("name: ");
//          Serial.print(owner.name);
//          Serial.print("surname:");
//          Serial.print(owner.surname);

        //}

};


imu_sensor imu_sensor0=imu_sensor(0,0,0x28);
imu_sensor imu_sensor1=imu_sensor(1,0,0x29);
imu_sensor imu_sensor2=imu_sensor(2,1,0x28);
imu_sensor imu_sensor3=imu_sensor(3,1,0x29);
imu_sensor imu_sensor4=imu_sensor(4,2,0x28);
imu_sensor imu_sensor5=imu_sensor(5,2,0x29);
imu_sensor imu_sensor6=imu_sensor(6,3,0x28);
imu_sensor imu_sensor7=imu_sensor(7,3,0x29);
imu_sensor imu_sensor8=imu_sensor(8,4,0x28);
imu_sensor imu_sensor9=imu_sensor(9,4,0x29);
imu_sensor imu_sensor10=imu_sensor(10,5,0x28);
imu_sensor imu_sensor11=imu_sensor(11,5,0x29);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(interruptPin, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN,0);
    pinMode(PIN_S0, OUTPUT);
    pinMode(PIN_S1, OUTPUT);
    pinMode(PIN_S2, OUTPUT);
    pinMode(LED_SHIELD_1, OUTPUT);
    pinMode(LED_SHIELD_2, OUTPUT);
    pinMode(LED_SHIELD_3, OUTPUT);
    digitalWrite(PIN_S0, LOW);
    digitalWrite(PIN_S1, LOW);
    digitalWrite(PIN_S2, LOW);

  Wire.begin();
  Serial.begin(230400);
  Serial.println("Dual BNO055 Test");  Serial.println("");
  Serial.println("hellowwwwwwwwwwwwwwwwwwwwwwwwwww");
  //test serial port
  delay(1000);
  Serial.println("5");
    delay(1000);
  Serial.println("4");
    delay(1000);
  Serial.println("3");
    delay(1000);
  Serial.println("2");
    delay(1000);
  Serial.println("1");
  
//  enableMuxPort(0);
//
//  delay(500);
//  byte value = 0;
//  //Serial.println("loop:");
//  Serial.println("Wire0x28 port0:============");
//  Wire.beginTransmission(0x28);
//  Wire.write((uint8_t)0x00);
//  Wire.endTransmission();
//  Wire.requestFrom(0x28, (byte)1);
//  value = Wire.read();
//  Serial.println(value);         // print the character
//  delay(1000);
//  Serial.println("Wire0x29 port0:============");
//  Wire.beginTransmission(0x29);
//  Wire.write((uint8_t)0x00);
//  Wire.endTransmission();
//  Wire.requestFrom(0x29, (byte)1);
//  value = Wire.read();
//  Serial.println(value);         // print the character
//
//  disableMuxPort(0);
  delay(1000);
  
  //Init_bno();
  //attachInterrupt(digitalPinToInterrupt(interruptPin), send_data_ISR, RISING );
  imu_sensor0.init();
  imu_sensor1.init();
  imu_sensor2.init();
  imu_sensor3.init();
  imu_sensor4.init();
  imu_sensor5.init();
  imu_sensor6.init();
  imu_sensor7.init();
  imu_sensor8.init();
  imu_sensor9.init();
  imu_sensor10.init();
  imu_sensor11.init();
  delay(1000);

  imu_sensor0.restorePredefinedCalibration();
  imu_sensor1.restorePredefinedCalibration();
  imu_sensor2.restorePredefinedCalibration();
  imu_sensor3.restorePredefinedCalibration();
  imu_sensor4.restorePredefinedCalibration();
  imu_sensor5.restorePredefinedCalibration();
  imu_sensor6.restorePredefinedCalibration();
  imu_sensor7.restorePredefinedCalibration();
  imu_sensor8.restorePredefinedCalibration();
  imu_sensor9.restorePredefinedCalibration();
  imu_sensor10.restorePredefinedCalibration();
  imu_sensor11.restorePredefinedCalibration();


//  imu_sensor0.calibrate(my_flash_store0);
//  delay(1000);
//  imu_sensor1.calibrate(my_flash_store1);
//  delay(1000);
//  imu_sensor2.calibrate(my_flash_store2);
//  delay(1000);
//  imu_sensor3.calibrate(my_flash_store3);
//  delay(1000);
//  imu_sensor4.calibrate(my_flash_store4);
//  delay(1000);
//  imu_sensor5.calibrate(my_flash_store5);
//  delay(1000);
//  imu_sensor6.calibrate(my_flash_store6);
//  delay(1000);
//  imu_sensor7.calibrate(my_flash_store7);
//  delay(1000);
//  imu_sensor8.calibrate(my_flash_store8);
//  delay(1000);
//  imu_sensor9.calibrate(my_flash_store9);
//  delay(1000);
//  imu_sensor10.calibrate(my_flash_store10);
//  delay(1000);
//  imu_sensor11.calibrate(my_flash_store11);
//  delay(1000);




  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}
/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
 void loop(void)
{
  
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  digitalWrite(LED_SHIELD_1,!digitalRead(LED_SHIELD_1));
  digitalWrite(LED_SHIELD_2,!digitalRead(LED_SHIELD_2));
  digitalWrite(LED_SHIELD_3,!digitalRead(LED_SHIELD_3));
  //Serial.println("main loop");
  sensorOutputs sensor0;
  sensorOutputs sensor1;
  sensorOutputs sensor2;
  sensorOutputs sensor3;
  sensorOutputs sensor4;
  sensorOutputs sensor5;
  sensorOutputs sensor6;
  sensorOutputs sensor7;
  sensorOutputs sensor8;
  sensorOutputs sensor9;
  sensorOutputs sensor10;
  sensorOutputs sensor11;
  sensor0=imu_sensor0.read();
  sensor1=imu_sensor1.read();
  sensor2=imu_sensor2.read();
  sensor3=imu_sensor3.read();
  sensor4=imu_sensor4.read();
  sensor5=imu_sensor5.read();
  sensor6=imu_sensor6.read();
  sensor7=imu_sensor7.read();
  sensor8=imu_sensor8.read();
  sensor9=imu_sensor9.read();
  sensor10=imu_sensor10.read();
  sensor11=imu_sensor11.read();

  if (scale.is_ready()) {
  scale_data = scale.read();
  //Serial.print("HX711 reading: ");
  //Serial.println(reading);
  } else {
    //Serial.println("HX711 not found.");
  }

  char buffer[300];
  sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d,%d], \"S2\":[%d,%d,%d,%d], \"S3\":[%d,%d,%d,%d], \"S4\":[%d,%d,%d,%d], \"S5\":[%d,%d,%d,%d], \"S6\":[%d,%d,%d,%d], \"S7\":[%d,%d,%d,%d], \"S8\":[%d,%d,%d,%d], \"S9\":[%d,%d,%d,%d], \"S10\":[%d,%d,%d,%d], \"S11\":[%d,%d,%d,%d]} ",scale_data,sensor0.x,sensor0.y,sensor0.z,sensor0.w, sensor1.x,sensor1.y,sensor1.z,sensor1.w,sensor2.x,sensor2.y,sensor2.z,sensor2.w,sensor3.x,sensor3.y,sensor3.z,sensor3.w,sensor4.x,sensor4.y,sensor4.z,sensor4.w,sensor5.x,sensor5.y,sensor5.z,sensor5.w,sensor6.x,sensor6.y,sensor6.z,sensor6.w,sensor7.x,sensor7.y,sensor7.z,sensor7.w,sensor8.x,sensor8.y,sensor8.z,sensor8.w,sensor9.x,sensor9.y,sensor9.z,sensor9.w,sensor10.x,sensor10.y,sensor10.z,sensor10.w,sensor11.x,sensor11.y,sensor11.z,sensor11.w);
//  sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d], \"S2\":[%d,%d,%d], \"S3\":[%d,%d,%d], \"S4\":[%d,%d,%d], \"S5\":[%d,%d,%d], \"S6\":[%d,%d,%d], \"S7\":[%d,%d,%d], \"S8\":[%d,%d,%d], \"S9\":[%d,%d,%d], \"S10\":[%d,%d,%d], \"S11\":[%d,%d,%d]} ",scale_data,sensor0.ox,sensor0.oy,sensor0.oz,sensor1.ox,sensor1.oy,sensor1.oz,sensor2.ox,sensor2.oy,sensor2.oz,sensor3.ox,sensor3.oy,sensor3.oz,sensor4.ox,sensor4.oy,sensor4.oz,sensor5.ox,sensor5.oy,sensor5.oz,sensor6.ox,sensor6.oy,sensor6.oz,sensor7.ox,sensor7.oy,sensor7.oz,sensor8.ox,sensor8.oy,sensor8.oz,sensor9.ox,sensor9.oy,sensor9.oz,sensor10.ox,sensor10.oy,sensor10.oz,sensor11.ox,sensor11.oy,sensor11.oz);
//  sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d,%d]} ",scale_data,sensor0.x,sensor0.y,sensor0.z,sensor0.w,sensor1.x,sensor1.y,sensor1.z,sensor1.w);
  //sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d], \"S1\":[%d,%d,%d], \"S2\":[%d,%d,%d], \"S3\":[%d,%d,%d], \"S4\":[%d,%d,%d], \"S5\":[%d,%d,%d], \"S6\":[%d,%d,%d], \"S7\":[%d,%d,%d], \"S8\":[%d,%d,%d], \"S9\":[%d,%d,%d], \"S10\":[%d,%d,%d], \"S11\":[%d,%d,%d], \"ST\":[%d,%d,%d]} ",
  //                  scale_data, i++,101,102,sensor0.ox,sensor0.oy,sensor0.oz,120,121,122,130,131,132,140,141,142,150,151,152,160,161,162,170,171,172,180,181,182,190,191,192,200,201,202,210,211,212,220,221,222);

  Serial.println(buffer);
  //Serial.write(0x0D);
  //Serial.write(0x0A);
  sensorOutputs cal9;
  sensorOutputs cal8;
  cal9=imu_sensor8.read_calibration_status();
  cal8=imu_sensor8.read_calibration_status();

  char buffer_cal[300];
  sprintf(buffer_cal, "{ \"C8\":[%d,%d,%d,%d], \"C9\":[%d,%d,%d,%d]} ",cal8.x,cal8.y,cal8.z,cal8.w, cal9.x,cal9.y,cal9.z,cal9.w);
  Serial.println(buffer_cal);
  

delay(BNO055_SAMPLERATE_DELAY_MS);
    
}

/**************************************************************************/
/*
    init the sensors
*/
/**************************************************************************/

//Enables a specific port number
boolean enableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(TCA9548A_I2C_ADDRESS, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}

//Disables a specific port number
boolean disableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(TCA9548A_I2C_ADDRESS, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}
