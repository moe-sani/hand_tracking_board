# hand_tracking_board

    branch name: red_80
    hardware version: RT Exo Data v1.0
    Processor: Arduino M0 Pro (Native USB)

* author: Mohammad Fattahi Sani
* Email: fattahi.m91@gmail.com

----------

 This package contains the following branches: 
 * red_55 : this is the branch for teleoperating the davinci and 3 finger tools. this is for small boards
 * master: this branch was used to collect data in surgical tests using 11 sensors and old board with multiplexr
 * red_80 :  small board configured to work with BNO080 sensors (not finalized yet)
 * DEV_BNO080 :  the vidor4000 board configured to work with BNO080 sensors (not finalized yet)

----------


This version is customized for reading  BNO80 sensors. The software only reads I2C zero wich is connected to the connectors on the bottom left of the board ( when holding it with usb on the right side)


## Sensors:

the sensors must have I2C address translator chip on them. The address on the chip can be selected using the provided resistor values.


The specific address for each sensor board then should be provided to the software at initializing step of the software. Following is showing how you can set diffrent sensor addresses to the board:

    imu_sensor imu_sensor0=imu_sensor(0,0,0x48);
    imu_sensor imu_sensor1=imu_sensor(1,0,0x28);
    imu_sensor imu_sensor2=imu_sensor(2,0,0x47);
    imu_sensor imu_sensor3=imu_sensor(3,0,0x27);
    imu_sensor imu_sensor4=imu_sensor(4,0,0x2C);
    imu_sensor imu_sensor5=imu_sensor(5,0,0x21);
    imu_sensor imu_sensor6=imu_sensor(6,0,0x68);

This means that for example sensor0 has the address of 0x48. So that specific sensor has be located on the hand on the location defined for sensor0.
## How To Program:

1. you must have arduino program installed on your computer.
2. you must have Arduino M0 Pro (Native USB) driver installed as follows:
    
    navigate to Tools> Boards > Board Manager> install:Arduino SAMD Boards (32-bits ARM Cortex-M0+) by arduino.

3. select the following board: Arduino M0 Pro (Native USB) 


### Required libraries in Arduino:
Please install the following libraries from Arduino Library manager:


after you followed the steps, you can simply connect the usb to the board, select the correct usb in the arduino and upload the program.



in the command window, you will first see the initialization steps, for a few seconds, and then the sensor information will be streaming.

here is the sample of data you will see repeatedly:

    > {"S0":[0,0,0,10000], "S1":[3377,-247,-9177,-2073], "S2":[0,0,0,10000], "S3":[0,0,0,10000] }
    > { "C0":[3,3,3,3], "C1":[3,3,3,1], "C2":[3,3,3,3], "C3":[3,3,3,3]} 

the first line is actuall sensory information and the second line is the calibration values (0~3) . 

in this example, it can be noticed that only S1/C1 is connected to the board and the other values are not valid.

    ("S0":[0,0,0,10000]) means that the sensor is not connected). 




 
