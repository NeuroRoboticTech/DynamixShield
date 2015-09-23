char prev = 0;


void setup() {
  Serial.begin(57600);
  
  while(!Serial);
  Serial.println("Starting setup");

  Serial2.begin(9600);
  
  while(!Serial2);
  Serial.println("Started bluetooth serial link");
}

void loop() {
  if(Serial2.available()) {
    String out = "";
    
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
      
      if(c == 94 && prev == 94) {
        Serial.println("");
      }
      prev = c;
    }
  }
}
