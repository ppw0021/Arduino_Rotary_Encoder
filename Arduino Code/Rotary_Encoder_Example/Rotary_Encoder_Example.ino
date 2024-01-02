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

//This variable stores the position
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