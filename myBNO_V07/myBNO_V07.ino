#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include<EEPROM.h>

// The two BNO055 modules, bnoB has the ADR pin wired to 3.3v to change its i2c address
// Both are wired: SCL to analog 5, SDA to analog 4, VIN to 5v, GRN to ground

/* This code uses I2C Protocol  */

/**************************************************************************/
/*
    GLOBAL VARIABLES
*/
/**************************************************************************/

#define BNO055_SAMPLERATE_DELAY_MS (100)

const byte ledPin = 13;
const byte interruptPin = 2;
const byte serialTxPin = 1;
volatile byte state = LOW;

Adafruit_BNO055 bnoA = Adafruit_BNO055(55, 0x28); /* A Sensor Address */
Adafruit_BNO055 bnoB = Adafruit_BNO055(56, 0x29); /* B Sensor Address */
int val=0;

float sensorA_OX;
float sensorA_OY;
float sensorA_OZ;
float sensorB_OX;
float sensorB_OY;
float sensorB_OZ;

/**************************************************************************/
/*
    FUNCTION LIST
*/
/**************************************************************************/
void serial_send(float data);




/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN,0);
  
  
  Serial.begin(115200);
  Serial.println("Dual BNO055 Test");  Serial.println("");
  Serial.println("hellowwwwwwwwwwwwwwwwwwwwwwwwwww");
  //test serial port
 
  
  
  if(!bnoA.begin() || !bnoB.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  int eeAddressA = 0;
  int eeAddressB = 256;
  long bnoIDA;
  long bnoIDB;
  bool foundCalibA = false;
  bool foundCalibB = false;

  EEPROM.get(eeAddressA,bnoIDA);
  EEPROM.get(eeAddressB,bnoIDB);

  adafruit_bno055_offsets_t calibrationDataA;
  adafruit_bno055_offsets_t calibrationDataB;
  sensor_t sensorA;
  sensor_t sensorB;

  bnoA.getSensor(&sensorA);
  // Look for a previous calibration in bno055 A
  if (bnoIDA != sensorA.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM.");
    delay(500);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor A in EEPROM.");
    eeAddressA += sizeof(long);
    EEPROM.get(eeAddressA,calibrationDataA);

    //displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055A..");
    bnoA.setSensorOffsets(calibrationDataA);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalibA = true;   
  }
  //delay(1000);

  bnoB.getSensor(&sensorB);
  // Look for a previous calibration in bno055 B
  if (bnoIDB != sensorB.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor B exists in EEPROM.");
    delay(500);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor B in EEPROM.");
    eeAddressB += sizeof(long);
    EEPROM.get(eeAddressB,calibrationDataB);

    //displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055..");
    bnoB.setSensorOffsets(calibrationDataB);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalibB = true;   
  }
  //delay(1000);

  /* Display some basic information on this sensor A and B*/
  //displaySensorDetails();

  /* Optional: Display current status A and B*/
  //displaySensorStatus();

  bnoA.setExtCrystalUse(true);
  bnoB.setExtCrystalUse(true);
  delay(10);
  sensors_event_t eventA;
  sensors_event_t eventB;
  
  bnoA.getEvent(&eventA);
  bnoB.getEvent(&eventB);

/* 	while(!bnoA.isFullyCalibrated()||!bnoB.isFullyCalibrated())
    {
		bnoA.getEvent(&eventA);
		bnoB.getEvent(&eventB);
      displayCalStatusA();
      displayCalStatusB();
      Serial.println("");
      //delay(BNO055_SAMPLERATE_DELAY_MS);
	  
	  //Serial.print("X'A'");
      Serial.print(eventA.orientation.x, 4);
      Serial.print(",");
      //Serial.print("\tY'A'");
      Serial.print(eventA.orientation.y, 4);
      //Serial.print("\tZ'A' ");
      Serial.print(",");
      Serial.print(eventA.orientation.z, 4);
      //Serial.print("\t\t\t");
      //Serial.print("\tX'B'");
      Serial.print(",");
      Serial.print(eventB.orientation.x, 4);
      //Serial.print("\tY'B'");
      Serial.print(",");
      Serial.print(eventB.orientation.y, 4);
      //Serial.print("\tZ'B'");
      Serial.print(",");
      Serial.print(eventB.orientation.z, 4);
      Serial.print(";");
      Serial.println("");
	  
    } */



  if(foundCalibA){
    //Serial.println("Need to recalibrate slightly");
    //while(!bnoA.isFullyCalibrated())
    //{
    //  displayCalStatusA();
    //  Serial.println("");
    //  bnoA.getEvent(&eventA);
      //delay(BNO055_SAMPLERATE_DELAY_MS);

    //}
  }
  else{
    Serial.println("Please Calibrate Sensor: ");
    while(!bnoA.isFullyCalibrated()){
      bnoA.getEvent(&eventA);
      Serial.print("X'A': ");
      Serial.print(eventA.orientation.x, 4);
      Serial.print("\tY'A': ");
      Serial.print(eventA.orientation.y, 4);
      Serial.print("\tZ'A':  ");
      Serial.print(eventA.orientation.z, 4);
      
      displayCalStatusA();
      Serial.println("");
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
	
	adafruit_bno055_offsets_t newCaliA;

	bnoA.getSensorOffsets(newCaliA);
	//displaySensorOffsets(newCali);

	// Save new calibration to arduino memory
	Serial.println("\n\nStoring calibration data to EEPROM...");

	eeAddressA = 0;
	bnoA.getSensor(&sensorA);
	bnoIDA = sensorA.sensor_id;

	EEPROM.put(eeAddressA,bnoIDA);

	eeAddressA += sizeof(long);
	EEPROM.put(eeAddressA,newCaliA);

	
  }

  if(foundCalibB){
    //Serial.println("Need to recalibrate slightly");
    //while(!bnoB.isFullyCalibrated())
    //{
    //  displayCalStatusB();
    //  Serial.println("");
    //  bnoB.getEvent(&eventB);
    //  //delay(BNO055_SAMPLERATE_DELAY_MS);
    //
    //}
  }
  else{
    Serial.println("Please Calibrate Sensor: ");
    while(!bnoB.isFullyCalibrated()){
      bnoB.getEvent(&eventB);
      Serial.print("X'B': ");
      Serial.print(eventB.orientation.x, 4);
      Serial.print("\tY'B': ");
      Serial.print(eventB.orientation.y, 4);
      Serial.print("\tZ'B':  ");
      Serial.print(eventB.orientation.z, 4);
      
      displayCalStatusB();
      Serial.println("");
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
	

	adafruit_bno055_offsets_t newCaliB;
	bnoB.getSensorOffsets(newCaliB);
	//displaySensorOffsets(newCali);

	// Save new calibration to arduino memory
	Serial.println("\n\nStoring calibration data to EEPROM...");


	eeAddressB = 256;
	bnoB.getSensor(&sensorB);
	bnoIDB = sensorB.sensor_id;

	EEPROM.put(eeAddressB,bnoIDB);

	eeAddressB += sizeof(long);
	EEPROM.put(eeAddressB,newCaliB);
	
  }



Serial.println("\nDone");

delay(BNO055_SAMPLERATE_DELAY_MS);
attachInterrupt(digitalPinToInterrupt(interruptPin), send_data_ISR, RISING );
}
/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
 void loop(void)
{
  //digitalWrite(LED_BUILTIN,1);
  sensors_event_t eventA;
  bnoA.getEvent(&eventA);
  sensors_event_t eventB;
  bnoB.getEvent(&eventB);
 
  sensorA_OX=eventA.orientation.x;
  sensorA_OY=eventA.orientation.y;
  sensorA_OZ=eventA.orientation.z;
  sensorB_OX=eventB.orientation.x;
  sensorB_OY=eventB.orientation.y;
  sensorB_OZ=eventB.orientation.z;
  
	digitalWrite(LED_BUILTIN,0);  
}


void send_data_ISR()
{
	Serial.begin(115200);
	digitalWrite(LED_BUILTIN,1);
	Serial.write(0xA0);
	serial_send(sensorA_OX);
	serial_send(sensorA_OY);
	serial_send(sensorA_OZ);
	serial_send(sensorB_OX);
	serial_send(sensorB_OY);
	serial_send(sensorB_OZ);
  Serial.write(0x0D);
  Serial.write(0x0A);

//      Serial.write(0xB0);
//      Serial.print(sensorA_OX, 4);
//      Serial.print(",");
//      Serial.print(sensorA_OY, 4);
//      Serial.print(",");
//      Serial.print(sensorA_OZ, 4);
//      Serial.print(",");
//      Serial.print(sensorB_OX, 4);
//      Serial.print(",");
//      Serial.print(sensorB_OY, 4);
//      Serial.print(",");
//      Serial.print(sensorB_OZ, 4);
//      Serial.print(";");
//  Serial.write(0x0D);
//  Serial.write(0x0A);
          
  Serial.end();
	pinMode(serialTxPin, INPUT_PULLUP);

}

void serial_send(float data)
{
	int data_int=(int) (data*10);//100
	//Serial.println(data_int);
	//delay(10);
	unsigned char data_high = highByte(data_int);
	unsigned char data_low = lowByte(data_int);
	
	//Serial.write(data_int);// wrong as it just shows low byte
	//delay(10);
  if (data_high==0x0A)
  {
    data_high=data_high-1;
  }
  if (data_low==0x0A)
  {
    data_low=data_low-1;
  }
	Serial.write(data_high);
	Serial.write(data_low);
	//delay(10);
}



/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/


void displaySensorDetails(void)
{
  sensor_t sensorA;
  sensor_t sensorB;
  
  bnoA.getSensor(&sensorA);
  bnoB.getSensor(&sensorB);
  
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensorA.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensorA.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensorA.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensorA.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensorA.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensorA.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensorB.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensorB.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensorB.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensorB.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensorB.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensorB.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatusA(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */

  uint8_t systemA, gyroA, accelA, magA;
  systemA = gyroA = accelA = magA = 0;
  bnoA.getCalibration(&systemA, &gyroA, &accelA, &magA);

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
}

void displayCalStatusB(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */

  uint8_t systemB, gyroB, accelB, magB;
  systemB = gyroB = accelB = magB = 0;
  bnoB.getCalibration(&systemB, &gyroB, &accelB, &magB);


  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!systemB)
  {
    Serial.print("! ");
  }
  
  /* Display the individual values */
  Serial.print("Sys'B':");
  Serial.print(systemB, DEC);
  Serial.print(" G'B':");
  Serial.print(gyroB, DEC);
  Serial.print(" A'B':");
  Serial.print(accelB, DEC);
  Serial.print(" M'B':");
  Serial.print(magB, DEC);
}

void displaySensorOffsetsA(const adafruit_bno055_offsets_t &calibDataA)
{
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
}

void displaySensorOffsetsB(const adafruit_bno055_offsets_t &calibDataB)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibDataB.accel_offset_x); Serial.print(" ");
    Serial.print(calibDataB.accel_offset_y); Serial.print(" ");
    Serial.print(calibDataB.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibDataB.gyro_offset_x); Serial.print(" ");
    Serial.print(calibDataB.gyro_offset_y); Serial.print(" ");
    Serial.print(calibDataB.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibDataB.mag_offset_x); Serial.print(" ");
    Serial.print(calibDataB.mag_offset_y); Serial.print(" ");
    Serial.print(calibDataB.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibDataB.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibDataB.mag_radius);
}
