# hand_tracking_board

    branch name: red_55
    hardware version: RT Exo Data v1.0
    Processor: Arduino M0 Pro (Native USB)
    
This version is customized for reading 4 BNO55 sensors. The software only reads I2C zero wich is connected to the connectors on the bottom left of the board ( when holding it with usb on the right side)

![Alt text](red_55.jpg?raw=true "Title")

the sensors must have I2C address translator chip on them. The address on the chip can be selected using the provided resistor values.

The specific address for each sensor board then should be provided to the software at initializing step of the software.

## How To Program:

1. you must have arduino program installed on your computer.
2. you must have Arduino M0 Pro (Native USB) driver installed as follows:
    
    navigate to Tools> Boards > Board Manager> install:Arduino SAMD Boards (32-bits ARM Cortex-M0+) by arduino.

3. select the following board: Arduino M0 Pro (Native USB) 


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




 
