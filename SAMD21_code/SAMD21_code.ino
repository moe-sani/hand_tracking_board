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
#include "SparkFun_BNO080_Arduino_Library.h"
#include "HX711.h"


static const unsigned int SENSOR_UNDEFINED=0;
static const unsigned int SENSOR_BNO055=1;
static const unsigned int SENSOR_BNO080=2;


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
        uint8_t addressbno055;
        uint8_t addressbno080;
        uint8_t channel;
        uint8_t sensor_id;
        BNO080 myIMU;
        Adafruit_BNO055 bno;
        bool isConnected;
        unsigned int sensor_type;

    public:
        //constructor
        imu_sensor(uint8_t sensor_id,uint8_t channel, uint8_t addressbno055, uint8_t addressbno080)
        {
            this->channel=channel;
            this->addressbno055=addressbno055;
            this->addressbno080=addressbno080;
            this-> sensor_id=sensor_id;
            bno = Adafruit_BNO055(sensor_id, this->addressbno055); /* A Sensor Address */
            this-> sensor_type= SENSOR_UNDEFINED;
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
            if(this->myIMU.begin(this->addressbno080,Wire) == true)
              {
                Serial.println("BNO080 is detected at this node!");
                this->isConnected= true;
                this->sensor_type=SENSOR_BNO080;
              }
              else if(bno.begin())
              {
                Serial.println("BNO055 is detected at this node!");
                this->isConnected= true;
                this->sensor_type=SENSOR_BNO055;
              }
              else
              {

                Serial.print("Ooops, no BNO detected ... Check your wiring or I2C ADDR!");
                Serial.println(" ");
                this->isConnected= false;
                this->sensor_type=SENSOR_UNDEFINED;
              }
            disableMuxPort(this->channel);
        }

//        void calibrate (FlashStorageClass<adafruit_bno055_offsets_t> my_flash_store)
        void calibrate ()
        {
            if( this->isConnected)
            {

            Serial.print("Calibration for sensor ");
            Serial.print(this-> sensor_id);
            Serial.print("on port:");
            Serial.print(this->channel);
            Serial.print("on address:");
            Serial.print(this->address);
            Serial.println(" ");
            enableMuxPort(this->channel);

            if (this->sensor_type == SENSOR_BNO080)
            {
                //Enable dynamic calibration for accel, gyro, and mag
                this->myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

                //Enable Game Rotation Vector output
                this->myIMU.enableGameRotationVector(100); //Send data update every 100ms

                //Enable Magnetic Field output
                this->myIMU.enableMagnetometer(100); //Send data update every 100ms

                //Once magnetic field is 2 or 3, run the Save DCD Now command
                //  Serial.println(F("Calibrating. Press 's' to save to flash"));
                //  Serial.println(F("Output in form x, y, z, in uTesla"));
                this->calibrationProcess();
            }

            if(this->sensor_type == SENSOR_BNO055)
            {
                this->restorePredefinedCalibration();

            }

            Serial.println("\n\nCalibration Is Done!");
            disableMuxPort(this->channel);
            delay(500);
            }
        }
        //Given a accuracy number, print what it means
        void printAccuracyLevel(byte accuracyNumber)
        {
          if (accuracyNumber == 0) Serial.print(F("Unreliable"));
          else if (accuracyNumber == 1) Serial.print(F("Low"));
          else if (accuracyNumber == 2) Serial.print(F("Medium"));
          else if (accuracyNumber == 3) Serial.print(F("High"));
        }
        void calibrationProcess()
        {
            bool stillCalibrating=true;
            while(stillCalibrating==true)
            {


          if(Serial.available())
          {
            byte incoming = Serial.read();

            if(incoming == 's')
            {
              this->myIMU.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
              this->myIMU.requestCalibrationStatus(); //Sends command to get the latest calibration status

              //Wait for calibration response, timeout if no response
              int counter = 100;
              while(1)
              {
                if(--counter == 0) break;
                if(this->myIMU.dataAvailable() == true)
                {
                  //The IMU can report many different things. We must wait
                  //for the ME Calibration Response Status byte to go to zero
                  if(this->myIMU.calibrationComplete() == true)
                  {
                    Serial.println("Calibration data successfully stored");
                    delay(1000);
                    stillCalibrating=false;
                    break;
                  }
                }

                delay(1);
              }
              if(counter == 0)
              {
                Serial.println("Calibration data failed to store. Please try again.");
              }

              //myIMU.endCalibration(); //Turns off all calibration
              //In general, calibration should be left on at all times. The BNO080
              //auto-calibrates and auto-records cal data roughly every 5 minutes
            }
            if (incoming=='c')
            {
                    Serial.println("Calibration canceled");
                    delay(1000);
                    stillCalibrating=false;
            }
          }

          //Look for reports from the IMU
          if (this->myIMU.dataAvailable() == true)
          {
            float x = this->myIMU.getMagX();
            float y = this->myIMU.getMagY();
            float z = this->myIMU.getMagZ();
            byte accuracy = this->myIMU.getMagAccuracy();

            float quatI = this->myIMU.getQuatI();
            float quatJ = this->myIMU.getQuatJ();
            float quatK = this->myIMU.getQuatK();
            float quatReal = this->myIMU.getQuatReal();
            byte sensorAccuracy = this->myIMU.getQuatAccuracy();

            Serial.print(x, 2);
            Serial.print(F(","));
            Serial.print(y, 2);
            Serial.print(F(","));
            Serial.print(z, 2);
            Serial.print(F(","));
            this->printAccuracyLevel(accuracy);
            Serial.print(F(","));

            Serial.print("\t");

            Serial.print(quatI, 2);
            Serial.print(F(","));
            Serial.print(quatJ, 2);
            Serial.print(F(","));
            Serial.print(quatK, 2);
            Serial.print(F(","));
            Serial.print(quatReal, 2);
            Serial.print(F(","));
            this->printAccuracyLevel(sensorAccuracy);
            Serial.print(F(","));

            Serial.println();
          }
          }
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

          //bno.getSensor(&sensorA);

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
          //bno.getCalibration(&systemA, &gyroA, &accelA, &magA);

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
//          bno.setSensorOffsets(calibrationDataA);
            this->displaySensorOffsets(calibrationDataA);
            Serial.println("Predefined Calibration data restored!");
            disableMuxPort(this->channel);

        }

        sensorOutputs read()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {
                enableMuxPort(this->channel);
                if(this->sensor_type==SENSOR_BNO080)
                {
                    newOutputs=read_bno080();
                }
                if(this->sensor_type==SENSOR_BNO055)
                {
                    newOutputs=read_bno055();
                }
                disableMuxPort(this->channel);
            }
            else
            {
              newOutputs.x=0;
              newOutputs.y=0;
              newOutputs.z=0;
              newOutputs.w=10000;
            }

            return newOutputs;
        }

        sensorOutputs read_bno080()
        {
            sensorOutputs newOutputs;

            while (this->myIMU.dataAvailable() == false);

            float quatI = myIMU.getQuatI();
            float quatJ = myIMU.getQuatJ();
            float quatK = myIMU.getQuatK();
            float quatReal = myIMU.getQuatReal();
            float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

            newOutputs.x=(int)(quatI*10000);
            newOutputs.y=(int)(quatJ*10000);
            newOutputs.z=(int)(quatK*10000);
            newOutputs.w=(int)(quatReal*10000);

            return newOutputs;
        }

        sensorOutputs read_bno055()
        {
            sensorOutputs newOutputs;

            imu::Quaternion quat=bno.getQuat();

            newOutputs.x=(int)(quat.x()*10000);
            newOutputs.y=(int)(quat.y()*10000);
            newOutputs.z=(int)(quat.z()*10000);
            newOutputs.w=(int)(quat.w()*10000);

            return newOutputs;
        }


        sensorOutputs read_calib()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {
                enableMuxPort(this->channel);
                if(this->sensor_type==SENSOR_BNO080)
                {
                    newOutputs=read_calib_bno080();
                }
                if(this->sensor_type==SENSOR_BNO055)
                {
                    newOutputs=read_calib_bno055();
                }
                disableMuxPort(this->channel);
            }
            else
            {
              newOutputs.x=0;
              newOutputs.y=0;
              newOutputs.z=0;
              newOutputs.w=0;
            }

            return newOutputs;
        }

        sensorOutputs read_calib_bno080()
        {
            sensorOutputs newOutputs;
            newOutputs.x=this->myIMU.getMagAccuracy();
            newOutputs.y=this->myIMU.getMagAccuracy();
            newOutputs.z=this->myIMU.getQuatRadianAccuracy();
            newOutputs.w=this->myIMU.getQuatRadianAccuracy();
            return newOutputs;
        }

        sensorOutputs read_calib_bno055()
        {
            sensorOutputs newOutputs;
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
            return newOutputs;
        }

};


imu_sensor imu_sensor0=imu_sensor(0,0,0x28,0x4B);
imu_sensor imu_sensor1=imu_sensor(1,0,0x29,0x4A);
imu_sensor imu_sensor2=imu_sensor(2,1,0x28,0x4B);
imu_sensor imu_sensor3=imu_sensor(3,1,0x29,0x4A);
imu_sensor imu_sensor4=imu_sensor(4,2,0x28,0x4B);
imu_sensor imu_sensor5=imu_sensor(5,2,0x29,0x4A);
imu_sensor imu_sensor6=imu_sensor(6,3,0x28,0x4B);
imu_sensor imu_sensor7=imu_sensor(7,3,0x29,0x4A);
imu_sensor imu_sensor8=imu_sensor(8,4,0x28,0x4B);
imu_sensor imu_sensor9=imu_sensor(9,4,0x29,0x4A);
imu_sensor imu_sensor10=imu_sensor(10,5,0x28,0x4B);
imu_sensor imu_sensor11=imu_sensor(11,5,0x29,0x4A);

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
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
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
  
  delay(1000);

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

  imu_sensor0.calibrate();
  imu_sensor1.calibrate();
  imu_sensor2.calibrate();
  imu_sensor3.calibrate();
  imu_sensor4.calibrate();
  imu_sensor5.calibrate();
  imu_sensor6.calibrate();
  imu_sensor7.calibrate();
  imu_sensor8.calibrate();
  imu_sensor9.calibrate();
  imu_sensor10.calibrate();
  imu_sensor11.calibrate();

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
  }
  else
  {
        //Serial.println("HX711 not found.");
  }

  char my_buffer[300];
  sprintf(my_buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d,%d], \"S2\":[%d,%d,%d,%d], \"S3\":[%d,%d,%d,%d], \"S4\":[%d,%d,%d,%d], \"S5\":[%d,%d,%d,%d], \"S6\":[%d,%d,%d,%d], \"S7\":[%d,%d,%d,%d], \"S8\":[%d,%d,%d,%d], \"S9\":[%d,%d,%d,%d], \"S10\":[%d,%d,%d,%d], \"S11\":[%d,%d,%d,%d]} ",scale_data,sensor0.x,sensor0.y,sensor0.z,sensor0.w, sensor1.x,sensor1.y,sensor1.z,sensor1.w,sensor2.x,sensor2.y,sensor2.z,sensor2.w,sensor3.x,sensor3.y,sensor3.z,sensor3.w,sensor4.x,sensor4.y,sensor4.z,sensor4.w,sensor5.x,sensor5.y,sensor5.z,sensor5.w,sensor6.x,sensor6.y,sensor6.z,sensor6.w,sensor7.x,sensor7.y,sensor7.z,sensor7.w,sensor8.x,sensor8.y,sensor8.z,sensor8.w,sensor9.x,sensor9.y,sensor9.z,sensor9.w,sensor10.x,sensor10.y,sensor10.z,sensor10.w,sensor11.x,sensor11.y,sensor11.z,sensor11.w);
//  sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d], \"S2\":[%d,%d,%d], \"S3\":[%d,%d,%d], \"S4\":[%d,%d,%d], \"S5\":[%d,%d,%d], \"S6\":[%d,%d,%d], \"S7\":[%d,%d,%d], \"S8\":[%d,%d,%d], \"S9\":[%d,%d,%d], \"S10\":[%d,%d,%d], \"S11\":[%d,%d,%d]} ",scale_data,sensor0.ox,sensor0.oy,sensor0.oz,sensor1.ox,sensor1.oy,sensor1.oz,sensor2.ox,sensor2.oy,sensor2.oz,sensor3.ox,sensor3.oy,sensor3.oz,sensor4.ox,sensor4.oy,sensor4.oz,sensor5.ox,sensor5.oy,sensor5.oz,sensor6.ox,sensor6.oy,sensor6.oz,sensor7.ox,sensor7.oy,sensor7.oz,sensor8.ox,sensor8.oy,sensor8.oz,sensor9.ox,sensor9.oy,sensor9.oz,sensor10.ox,sensor10.oy,sensor10.oz,sensor11.ox,sensor11.oy,sensor11.oz);
//  sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d,%d]} ",scale_data,sensor0.x,sensor0.y,sensor0.z,sensor0.w,sensor1.x,sensor1.y,sensor1.z,sensor1.w);
  //sprintf(buffer, "{\"SC\":[%d], \"S0\":[%d,%d,%d], \"S1\":[%d,%d,%d], \"S2\":[%d,%d,%d], \"S3\":[%d,%d,%d], \"S4\":[%d,%d,%d], \"S5\":[%d,%d,%d], \"S6\":[%d,%d,%d], \"S7\":[%d,%d,%d], \"S8\":[%d,%d,%d], \"S9\":[%d,%d,%d], \"S10\":[%d,%d,%d], \"S11\":[%d,%d,%d], \"ST\":[%d,%d,%d]} ",
  //                  scale_data, i++,101,102,sensor0.ox,sensor0.oy,sensor0.oz,120,121,122,130,131,132,140,141,142,150,151,152,160,161,162,170,171,172,180,181,182,190,191,192,200,201,202,210,211,212,220,221,222);

  Serial.println(my_buffer);
  //Serial.write(0x0D);
  //Serial.write(0x0A);
  sensorOutputs cal9;
  sensorOutputs cal8;
  sensorOutputs cal7;
  sensorOutputs cal6;
  //cal9=imu_sensor9.read_calib();
  //cal8=imu_sensor8.read_calib();
  //cal7=imu_sensor7.read_calib();
  //cal6=imu_sensor6.read_calib();

  //char buffer_cal[300];
  //sprintf(buffer_cal, "{ \"C6\":[%d,%d,%d,%d], \"C7\":[%d,%d,%d,%d], \"C8\":[%d,%d,%d,%d], \"C9\":[%d,%d,%d,%d]} ",cal6.x,cal6.y,cal6.z,cal6.w, cal7.x,cal7.y,cal7.z,cal7.w, cal8.x,cal8.y,cal8.z,cal8.w, cal9.x,cal9.y,cal9.z,cal9.w);
  //Serial.println(buffer_cal);
  

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
