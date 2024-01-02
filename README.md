# Arduino Rotary Encoder

This project is a simple optical rotary encoder with 180 points per rotation and bi directional detection. This Arduino Rotary Encoder is intended to work with an Arduino (UNO, Leonardo, Micro, 328p etc) microcontroller but it should work with just about anything with interrupt pins and ADC conversion.

## Description

The device consists of 3 3D printed pieces, 8 electronic components and 9 nuts/blots.

3D printed parts:
- Emitter block
- Encoder disk
- Detector block

Components:
- 4x IR LEDs
- 2x 220 ohm resistors
- 2x 3M ohm resistors
- 5x M2x16 bolts
- 4x M2 nuts

The rotary encoder works by using an encoder disk that has 90 slots cut out around the edge of the disk. These slots block and unblock IR light from passing through the disk when it is rotated.
2 IR LEDs (IROUT_1 and IROUT_2) shine IR light through 2 tiny slits on the emitter block through the disk and into the detector block, which has a matching 2 tiny slits and 2 IR LEDs wired reversely to act as IR light detectors (IRIN_1 and (IRIN_2).

These tiny slits are offset by half a slot so that if IRIN_1 goes high, IRIN_2 will be low (beneath the detection threshold) if the wheel is turning clockwise, if IRIN_2 is high, then that means the wheel is turning anti-clockwise. Repeat and invert this for when IRIN_1 goes low, then we double the point count from 90 to 180.

IRIN_1 is attached to 2 interrupts on the Arduino, so that the Arduino can detect if IRIN_1 is going high or low and check IRIN_2 as soon as either interupt is triggered.

## Assembly

3D print the parts, and assemble the encoder as shown below.

![Assembly Diagram](https://i.imgur.com/xdQJD8J.png)
(Please note that this diagram shows the encoder disk with 180 slots, this was changed to 90 in the current version)

To wire the encoder together, follow the circuit diagram below.

![Circuit Diagram](https://i.imgur.com/L6RtXXO.png)

## Code

The code is very straight forward and simple, but a calibration will need to be done to find the correct threshold.
```
//Definitions to save memory
//IRIN_2 pin
#define IRIN_2 A0

//IRIN_1 Interrupt pins
#define IRIN_1_I1 2
#define IRIN_1_I2 3

//To find out what value you need to set threshold to, set debug mode to true and turn the wheel clockwise and record the value
//    then turn the wheel anti-clockwise and record the value. You should set threshold to halfway between the results 
#define DEBUG_MODE false
#define THRESHOLD 100

//This variable stores the positio
int position = 0;

//Setup
void setup() {
  //Set pinmodes for IRIN
  pinMode(IRIN_1_I1, INPUT);
  pinMode(IRIN_1_I2, INPUT);
  pinMode(IRIN_2, INPUT);

  //Begin Serial Interface
  Serial.begin(9600);

  //Attach interrupts to IRIN_1
  attachInterrupt(digitalPinToInterrupt(IRIN_1_I1), IRIN_1_goingHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(IRIN_1_I2), IRIN_1_goingLow, FALLING);
}

void loop() {
}

//When IRIN_1 goes high
void IRIN_1_goingHigh()
{
  //Get value of IRIN_2
  int irin_2_val = analogRead(IRIN_2);

  //DEBUG MODE
  if (DEBUG_MODE)
  {
    Serial.println(irin_2_val);
    return;
  }

  //Check value against threshold
  if (irin_2_val > THRESHOLD)
  {
    //Going clockwise
    position++;
  }
  else
  {
    //Going anticlockwise
    position--;
  }

  //Return value to serial monitor
  Serial.println(position);
}

//When IRIN_2 goes low
void IRIN_1_goingLow()
{
  //DEBUG MODE
  if (DEBUG_MODE)
  {
    return;
  }

  //Get value of IRIN_2
  int irin_2_val = analogRead(IRIN_2);

  //Check value against threshold
  if (irin_2_val < THRESHOLD)
  {
    //Going clockwise
    position++;
  }
  else
  {
    //Going anticlockwise
    position--;
  }

  //Return value to serial monitor
  Serial.println(position);
}
```

## Version History

* 1.0
    * Initial Release
