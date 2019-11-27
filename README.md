# hand_tracking_board

    branch name: master
    hardware version: I2C and analogue MKR by Jwelsby 2019
    Processor: Arduino Vidor 4000
* author: Mohammad Fattahi Sani
* Email: fattahi.m91@gmail.com

----------

 This package contains the following branches: 
 * red_55 : this is the branch for teleoperating the davinci and 3 finger tools. this is for small boards
 * master: this branch was used to collect data in surgical tests using 11 sensors and old board with multiplexr
 * red_80 :  small board configured to work with BNO080 sensors (not finalized yet)
 * DEV_BNO080 :  the vidor4000 board configured to work with BNO080 sensors (not finalized yet)

----------
This is the source code for the arduino vidoor 4000 board that reads sensors and sends values to the computer.


![Alt text](board.png?raw=true "Title")


The specific address for each sensor then should be provided to the software at initializing step of the software. Following is showing how you can set diffrent sensor addresses to the board:

    imu_sensor imu_sensor0=imu_sensor(0,0,0x28);
    imu_sensor imu_sensor1=imu_sensor(1,0,0x29);

## How To Program:

1. you must have arduino program installed on your computer.
2. you must have Arduino Vidor4000 driver installed as follows:
    



### Required libraries in Arduino:
Please install the following libraries from Arduino Library manager:

    Adafruit BNO055 (version 1.1.8)
    Adafruit Unified Sensor (version 1.0.3)


after you followed the steps, you can simply connect the usb to the board, select the correct usb in the arduino and upload the program.



in the command window, you will first see the initialization steps, for a few seconds, and then the sensor information will be streaming.

here is the sample of data you will see repeatedly:

    > {"S0":[0,0,0,10000], "S1":[3377,-247,-9177,-2073], "S2":[0,0,0,10000], "S3":[0,0,0,10000] }
    > { "C0":[3,3,3,3], "C1":[3,3,3,1], "C2":[3,3,3,3], "C3":[3,3,3,3]} 

the first line is actuall sensory information and the second line is the calibration values (0~3) . 

in this example, it can be noticed that only S1/C1 is connected to the board and the other values are not valid.

    ("S0":[0,0,0,10000]) means that the sensor is not connected). 

