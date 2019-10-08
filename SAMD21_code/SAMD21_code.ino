/**************************************************************************/
/*
    IMU SerialUSB CODE

*/
/**************************************************************************/




#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "wiring_private.h"

///#include <Adafruit_Sensor.h>
//#in/clude <utility/imumaths.h>
//#include "SparkFun_BNO080_Arduino_Library.h"



struct sensorOutputs {
//    int ox;
//    int oy;
//    int oz;
    int x;
    int y;
    int z;
    int w;
};



#define SAMPLERATE_DELAY_MS (1)

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

float sensorA_OX;
float sensorA_OY;
float sensorA_OZ;
float sensorB_OX;
float sensorB_OY;
float sensorB_OZ;

#define PIN_LED 43
#define PIN_SDA2 11
#define PIN_SCL2 13

TwoWire Wire2(&sercom1, PIN_SDA2, PIN_SCL2);   //(&PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL);


/**************************************************************************/
/*
    FUNCTION LIST
*/
/**************************************************************************/

/**************************************************************************/
/*
    class for sensors
*/
/**************************************************************************/
class imu_sensor {
    private:
        uint8_t address;
        uint8_t channel;
        uint8_t sensor_id;
        //TwoWire myWire;
        TwoWire *myWire;
//        BNO080 myIMU;
        bool isConnected;
        Adafruit_BNO055 bno;

    public:
        //constructor
//        imu_sensor(uint8_t sensor_id, TwoWire *theWire, uint8_t address )
        imu_sensor(uint8_t sensor_id, uint8_t channel, uint8_t address )
        {
            this->channel=channel;
            this->address=address;
            //this->myWire=Wire;
            this-> sensor_id=sensor_id;
            if (channel==2)
            {
                this->bno = Adafruit_BNO055(sensor_id, address, &Wire2); /* A Sensor Address */
                this->myWire = &Wire2;
            }
            else
            {
                this->bno = Adafruit_BNO055(sensor_id, address, &Wire); /* A Sensor Address */
                this->myWire = &Wire;
            }

        }

        void init()
        {
            byte value = 0;
            SerialUSB.println("==========================");
            SerialUSB.print("Initalizing sensor ");
            SerialUSB.print(this-> sensor_id);
            SerialUSB.print("on address:");
            SerialUSB.print(this->address);
            SerialUSB.println(" ");
            SerialUSB.println("WHOAMI register reply:");
            this->myWire->beginTransmission(this->address);
            this->myWire->write((uint8_t)0x00);
            this->myWire->endTransmission();
            this->myWire->requestFrom(this->address, (byte)1);
            value = this->myWire->read();
            SerialUSB.println(value);         // print the character
            delay(500);
            if(!this->bno.begin())
              {
                SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
                SerialUSB.println(" ");
                this->isConnected= false;
              }
              else
              {
                this->isConnected= true;
                SerialUSB.println("This sensor is alive!");
              }
        }

        void calibrate ()
        {
            if( this->isConnected)
            {

            SerialUSB.println("Restoring Predefined Calibration for sensor: ");
            SerialUSB.print(this-> sensor_id);
            SerialUSB.print("on address:");
            SerialUSB.print(this->address);
            SerialUSB.println(" ");
            adafruit_bno055_offsets_t calibrationDataA = Predefined_Calibration[this->sensor_id];
            bno.setSensorOffsets(calibrationDataA);
            this->displaySensorOffsets(calibrationDataA);
            SerialUSB.println("Predefined Calibration data restored!");
            SerialUSB.println("\n\nCalibration Is Done!");
            delay(500);
            }
        }
        /**************************************************************************/
        /*
            Display sensor calibration status
        */
        /**************************************************************************/

        void displaySensorDetails(void)
        {
          //enableMuxPort(this->channel);
          sensor_t sensorA;

          bno.getSensor(&sensorA);

          SerialUSB.println("------------------------------------");
          SerialUSB.print  ("Sensor:       "); SerialUSB.println(sensorA.name);
          SerialUSB.print  ("Driver Ver:   "); SerialUSB.println(sensorA.version);
          SerialUSB.print  ("Unique ID:    "); SerialUSB.println(sensorA.sensor_id);
          SerialUSB.print  ("Max Value:    "); SerialUSB.print(sensorA.max_value); SerialUSB.println(" xxx");
          SerialUSB.print  ("Min Value:    "); SerialUSB.print(sensorA.min_value); SerialUSB.println(" xxx");
          SerialUSB.print  ("Resolution:   "); SerialUSB.print(sensorA.resolution); SerialUSB.println(" xxx");
          SerialUSB.println("------------------------------------");
          SerialUSB.println("");
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
          SerialUSB.print("\t");
          if (!systemA)
          {
            SerialUSB.print("! ");
          }

          /* Display the individual values */
          SerialUSB.print("Sys'A':");
          SerialUSB.print(systemA, DEC);
          SerialUSB.print(" G'A':");
          SerialUSB.print(gyroA, DEC);
          SerialUSB.print(" A'A':");
          SerialUSB.print(accelA, DEC);
          SerialUSB.print(" M'A':");
          SerialUSB.print(magA, DEC);
          //disableMuxPort(this->channel);
        }

         void displaySensorOffsets(const adafruit_bno055_offsets_t &calibDataA)
        {
            //enableMuxPort(this->channel);
            SerialUSB.print("Accelerometer: ");
            SerialUSB.print(calibDataA.accel_offset_x); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.accel_offset_y); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.accel_offset_z); SerialUSB.print(" ");

            SerialUSB.print("\nGyro: ");
            SerialUSB.print(calibDataA.gyro_offset_x); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.gyro_offset_y); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.gyro_offset_z); SerialUSB.print(" ");

            SerialUSB.print("\nMag: ");
            SerialUSB.print(calibDataA.mag_offset_x); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.mag_offset_y); SerialUSB.print(" ");
            SerialUSB.print(calibDataA.mag_offset_z); SerialUSB.print(" ");

            SerialUSB.print("\nAccel Radius: ");
            SerialUSB.print(calibDataA.accel_radius);

            SerialUSB.print("\nMag Radius: ");
            SerialUSB.print(calibDataA.mag_radius);
            //disableMuxPort(this->channel);
        }

        sensorOutputs read_calibration_status()
        {
         sensorOutputs newOutputs;
          /* Get the four calibration values (0..3) */
          /* Any sensor data reporting 0 should be ignored, */
          /* 3 means 'fully calibrated" */

          uint8_t systemA, gyroA, accelA, magA;
          systemA = gyroA = accelA = magA = 0;
          bno.getCalibration(&systemA, &gyroA, &accelA, &magA);

           newOutputs.x=systemA;
           newOutputs.y=gyroA;
           newOutputs.z=accelA;
           newOutputs.w=magA;
          return newOutputs;
        }

        sensorOutputs read()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {


            imu::Quaternion quat=bno.getQuat();

              newOutputs.x=(int)(quat.x()*10000);
              newOutputs.y=(int)(quat.y()*10000);
              newOutputs.z=(int)(quat.z()*10000);
              newOutputs.w=(int)(quat.w()*10000);

            }
            else
            {
//                SerialUSB.println("no sensor:");
              newOutputs.x=0;
              newOutputs.y=0;
              newOutputs.z=0;
              newOutputs.w=10000;
            }

            return newOutputs;
        }

};


//imu_sensor imu_sensor0=imu_sensor(0,&Wire,0x28); this worked
imu_sensor imu_sensor0=imu_sensor(0,2,0x29);
imu_sensor imu_sensor1=imu_sensor(1,2,0x28);
imu_sensor imu_sensor2=imu_sensor(2,2,0x29);
imu_sensor imu_sensor3=imu_sensor(3,2,0x28);


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,0);
  pinPeripheral(PIN_SDA2, PIO_SERCOM);
  pinPeripheral(PIN_SCL2, PIO_SERCOM);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  Wire2.begin();
  Wire2.setClock(400000); //Increase I2C data rate to 400kHz

  pinPeripheral(PIN_SDA2, PIO_SERCOM);
  pinPeripheral(PIN_SCL2, PIO_SERCOM);

  SerialUSB.begin(230400);
  SerialUSB.println("SMARTsurg ExoEskeleton board--- new version");  SerialUSB.println("");
  SerialUSB.println("hellowwwwwwwwwwwwwwwwwwwwwwwwwww");
  //test SerialUSB port
  delay(1000);
  SerialUSB.println("5");
    delay(1000);
  SerialUSB.println("4");
    delay(1000);
  SerialUSB.println("3");
    delay(1000);
  SerialUSB.println("2");
    delay(1000);
  SerialUSB.println("1");
  SerialUSB.println("test wire2");
    SerialUSB.println("WHOAMI register reply:");
    Wire2.beginTransmission(0x29);
    Wire2.write((uint8_t)0x00);
    Wire2.endTransmission();
    Wire2.requestFrom(0x29, (byte)1);
    byte value = 0;
    value = Wire2.read();
    SerialUSB.println(value);         // print the character
    SerialUSB.println("BNO test:");
  Adafruit_BNO055 bno = Adafruit_BNO055(20, 0x29, &Wire2); //next line will hang if this is on wire2, but it is fine when it is on wire!
  if(!bno.begin())
  {
    SerialUSB.print("Ooo00000000000000ps, no BNO055 detected ");
    SerialUSB.println(" ");
  }
  else
  {
    SerialUSB.println("This sensor is alive!");
  }
  
  delay(1000);

  imu_sensor0.init();
  imu_sensor1.init();
  imu_sensor2.init();
  imu_sensor3.init();

  delay(1000);
  imu_sensor0.calibrate();
  imu_sensor1.calibrate();
  imu_sensor2.calibrate();
  imu_sensor3.calibrate();

}
/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
 void loop(void)
{
  
    digitalWrite(PIN_LED,!digitalRead(PIN_LED));
    //SerialUSB.println("main loop");
    sensorOutputs sensor0;
    sensorOutputs sensor1;
    sensorOutputs sensor2;
    sensorOutputs sensor3;

    sensor0=imu_sensor0.read();
    sensor1=imu_sensor1.read();
    sensor2=imu_sensor2.read();
    sensor3=imu_sensor3.read();


    char my_buffer[200];
    sprintf(my_buffer, "{\"S0\":[%d,%d,%d,%d], \"S1\":[%d,%d,%d,%d], \"S2\":[%d,%d,%d,%d], \"S3\":[%d,%d,%d,%d] }",sensor0.x,sensor0.y,sensor0.z,sensor0.w, sensor1.x,sensor1.y,sensor1.z,sensor1.w,sensor2.x,sensor2.y,sensor2.z,sensor2.w,sensor3.x,sensor3.y,sensor3.z,sensor3.w);

    SerialUSB.println(my_buffer);
    //SerialUSB.write(0x0D);
    //SerialUSB.write(0x0A);
//    sensorOutputs cal9;
//    sensorOutputs cal8;
//    sensorOutputs cal7;
//    sensorOutputs cal6;

    //char buffer_cal[300];
    //sprintf(buffer_cal, "{ \"C6\":[%d,%d,%d,%d], \"C7\":[%d,%d,%d,%d], \"C8\":[%d,%d,%d,%d], \"C9\":[%d,%d,%d,%d]} ",cal6.x,cal6.y,cal6.z,cal6.w, cal7.x,cal7.y,cal7.z,cal7.w, cal8.x,cal8.y,cal8.z,cal8.w, cal9.x,cal9.y,cal9.z,cal9.w);
    //SerialUSB.println(buffer_cal);

    delay(SAMPLERATE_DELAY_MS);
    
}
