/**************************************************************************/
/*
    IMU SerialUSB CODE

*/
/**************************************************************************/




#include <Wire.h>
///#include <Adafruit_Sensor.h>
//#in/clude <utility/imumaths.h>
#include "SparkFun_BNO080_Arduino_Library.h"



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


float sensorA_OX;
float sensorA_OY;
float sensorA_OZ;
float sensorB_OX;
float sensorB_OY;
float sensorB_OZ;

#define PIN_LED 43



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
        uint8_t sensor_id;
        BNO080 myIMU;
        bool isConnected;

    public:
        //constructor
        imu_sensor(uint8_t sensor_id, uint8_t address)
        {
            this->address=address;
            this-> sensor_id=sensor_id;
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
            Wire.beginTransmission(this->address);
            Wire.write((uint8_t)0x00);
            Wire.endTransmission();
            Wire.requestFrom(this->address, (byte)1);
            value = Wire.read();
            SerialUSB.println(value);         // print the character
            delay(500);
            if(this->myIMU.begin(this->address,Wire) == true)
              {
                SerialUSB.println("BNO080 is detected at this node!");
                this->isConnected= true;
              }
              else
              {
                SerialUSB.print("Ooops, no BNO detected ... Check your wiring or I2C ADDR!");
                SerialUSB.println(" ");
                this->isConnected= false;
              }
        }

        void calibrate ()
        {
            if( this->isConnected)
            {

            SerialUSB.print("Calibration for sensor ");
            SerialUSB.print(this-> sensor_id);
            SerialUSB.print("on address:");
            SerialUSB.print(this->address);
            SerialUSB.println(" ");

            //Enable dynamic calibration for accel, gyro, and mag
            this->myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

            //Enable Game Rotation Vector output
            this->myIMU.enableGameRotationVector(100); //Send data update every 100ms

            //Enable Magnetic Field output
            this->myIMU.enableMagnetometer(100); //Send data update every 100ms

            //Once magnetic field is 2 or 3, run the Save DCD Now command
            //  SerialUSB.println(F("Calibrating. Press 's' to save to flash"));
            //  SerialUSB.println(F("Output in form x, y, z, in uTesla"));
            this->calibrationProcess();


            SerialUSB.println("\n\nCalibration Is Done!");
            delay(500);
            }
        }
        /**************************************************************************/
        /*
            Display sensor calibration status
        */
        /**************************************************************************/

        //Given a accuracy number, print what it means
        void printAccuracyLevel(byte accuracyNumber)
        {
          if (accuracyNumber == 0) SerialUSB.print(F("Unreliable"));
          else if (accuracyNumber == 1) SerialUSB.print(F("Low"));
          else if (accuracyNumber == 2) SerialUSB.print(F("Medium"));
          else if (accuracyNumber == 3) SerialUSB.print(F("High"));
        }
        void calibrationProcess()
        {
            bool stillCalibrating=true;
            while(stillCalibrating==true)
            {


          if(SerialUSB.available())
          {
            byte incoming = SerialUSB.read();

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
                    SerialUSB.println("Calibration data successfully stored");
                    delay(1000);
                    stillCalibrating=false;
                    break;
                  }
                }

                delay(1);
              }
              if(counter == 0)
              {
                SerialUSB.println("Calibration data failed to store. Please try again.");
              }

              //myIMU.endCalibration(); //Turns off all calibration
              //In general, calibration should be left on at all times. The BNO080
              //auto-calibrates and auto-records cal data roughly every 5 minutes
            }
            if (incoming=='c')
            {
                    SerialUSB.println("Calibration canceled");
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

            SerialUSB.print(x, 2);
            SerialUSB.print(F(","));
            SerialUSB.print(y, 2);
            SerialUSB.print(F(","));
            SerialUSB.print(z, 2);
            SerialUSB.print(F(","));
            this->printAccuracyLevel(accuracy);
            SerialUSB.print(F(","));

            SerialUSB.print("\t");

            SerialUSB.print(quatI, 2);
            SerialUSB.print(F(","));
            SerialUSB.print(quatJ, 2);
            SerialUSB.print(F(","));
            SerialUSB.print(quatK, 2);
            SerialUSB.print(F(","));
            SerialUSB.print(quatReal, 2);
            SerialUSB.print(F(","));
            this->printAccuracyLevel(sensorAccuracy);
            SerialUSB.print(F(","));

            SerialUSB.println();
          }
          }
        }

        sensorOutputs read()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {
                newOutputs=read_bno080();
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

        sensorOutputs read_calib()
        {
            sensorOutputs newOutputs;

            if (this->isConnected)
            {
                newOutputs=read_calib_bno080();
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
};


imu_sensor imu_sensor0=imu_sensor(0,0x4A);
imu_sensor imu_sensor1=imu_sensor(1,0x2A);
imu_sensor imu_sensor2=imu_sensor(2,0x45);
imu_sensor imu_sensor3=imu_sensor(3,0x25);


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,0);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
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
    sensorOutputs cal9;
    sensorOutputs cal8;
    sensorOutputs cal7;
    sensorOutputs cal6;

    //char buffer_cal[300];
    //sprintf(buffer_cal, "{ \"C6\":[%d,%d,%d,%d], \"C7\":[%d,%d,%d,%d], \"C8\":[%d,%d,%d,%d], \"C9\":[%d,%d,%d,%d]} ",cal6.x,cal6.y,cal6.z,cal6.w, cal7.x,cal7.y,cal7.z,cal7.w, cal8.x,cal8.y,cal8.z,cal8.w, cal9.x,cal9.y,cal9.z,cal9.w);
    //SerialUSB.println(buffer_cal);

    delay(SAMPLERATE_DELAY_MS);
    
}
