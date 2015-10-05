#include <Servo.h>

// Define the pins to which the servo and sensor are connected.
const int pinServo = 2;
const int potentiometer = 0;

int buffer[10];
int bufferIdx = 0;

// Use a Servo object to represent and control the servo.
Servo myServo;

void setup()
{
    for(int i=0; i<10; i++)
    {
      buffer[i] = 0;
    }  
    // Tell the Servo object which pin to use to control the servo.
    myServo.attach(pinServo);

    // Configure the angle sensor's pin for input signals.
    pinMode(potentiometer, INPUT);
  
    Serial.begin(57600);
    
    while(!Serial);
    Serial.println("Starting setup");    
}

void ReadPot()
{
    // Read the value of the angle sensor.
    buffer[bufferIdx] = analogRead(potentiometer);
    bufferIdx++;
    if(bufferIdx > 9)
      bufferIdx = 0;
}

int GetRollingAverage()
{
  int total = 0;
  for(int i=0; i<10; i++)
  {
    total+=buffer[i];
  }

  float avg = (float) total / 10.0;
  return (int) avg;
}

void loop()
{
    ReadPot();
    int sensorPosition = GetRollingAverage();
    
    // The analog value from the angle sensor is between 0 and 1023, but
    // the servo only accepts values between 0 and 179; use the map()
    // function as a linear conversion between the two ranges.
    int shaftPosition = map(sensorPosition, 0, 1023, 0, 179);
    
    // Use the Servo object to move the servo.
    myServo.write(shaftPosition);

    Serial.println(String(sensorPosition) + "     " + String(shaftPosition));
    
    delay(15);
}

