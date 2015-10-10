#include <Servo.h>

// Define the pins to which the servo and sensor are connected.
const int pinServo1 = 2;
const int pinServo2 = 3;
const int joyXPin = 0;
const int joyYPin = 1;

int bufferX[10];
int bufferIdxX = 0;
int bufferY[10];
int bufferIdxY = 0;

// Use a Servo object to represent and control the servo.
Servo servo1;
Servo servo2;

void setup()
{
    for(int i=0; i<10; i++)
    {
      bufferX[i] = 0;
      bufferY[i] = 0;
    }  
    // Tell the Servo object which pin to use to control the servo.
    servo1.attach(pinServo1);
    servo2.attach(pinServo2);

    // Configure the joystick sensor's pin for input signals.
    pinMode(joyXPin, INPUT);
    pinMode(joyYPin, INPUT);
  
    Serial.begin(57600);
    
    while(!Serial);
    Serial.println("Starting setup");    
}

void readPots()
{
    // Read the value of the joystick X sensor.
    bufferX[bufferIdxX] = analogRead(joyXPin);
    bufferIdxX++;
    if(bufferIdxX > 9) {
      bufferIdxX = 0;
    }

    // Read the value of the joystick Y sensor.
    bufferY[bufferIdxY] = analogRead(joyYPin);
    bufferIdxY++;
    if(bufferIdxY > 9) {
      bufferIdxY = 0;
    }
    
}

void getRollingAverages(int &avgX, int &avgY)
{
  int totalX = 0, totalY = 0;
  for(int i=0; i<10; i++)
  {
    totalX+=bufferX[i];
    totalY+=bufferY[i];
  }

  avgX = (float) totalX / 10.0;
  avgY = (float) totalY / 10.0;
}

void loop()
{
    readPots();
    int joyX=0, joyY=0;
    getRollingAverages(joyX, joyY);
    
    // The analog value from the angle sensor is between 0 and 1023, but
    // the servo only accepts values between 0 and 179; use the map()
    // function as a linear conversion between the two ranges.
    int joyXShaft = map(joyX, 0, 1023, 0, 179);
    int joyYShaft = map(joyY, 0, 1023, 0, 179);
    
    // Use the Servo object to move the servo.
    servo1.write(joyXShaft);
    servo2.write(joyYShaft);

    Serial.println(String(joyX) + ", " + String(joyXShaft) +
                   String(joyY) + ", " + String(joyYShaft));
    
    delay(15);
}

