char c = 0;   // for incoming serial data

void setup() {
  Serial.begin(57600);  // start serial for output
  while(!Serial);
  Serial.println("Starting setup");
  
  Serial2.begin(38400);     // opens serial port, sets data rate to 9600 bps
  while(!Serial2);

  Serial.println("Setup Finished");
}

void loop() {

  delay(2000);
  Serial.println("Sending");
  
  Serial2.print("This is from XBee #2");
  
  String s = "";
  while (Serial2.available() > 0) {
    // read the incoming byte:
    c = Serial2.read();
    s += c;
  }

  if(s.length() > 0) {
    Serial.print("Read: ");
    Serial.println(s);
  }
}
