
#define DATA_SIZE 9
char data[DATA_SIZE];

void setup() {
  Serial.begin(57600);
  while(!Serial);

  Serial2.begin(1000000);
  while(!Serial2);

  Serial.print("System core clock: ");
  Serial.println(SystemCoreClock);
  
  pinMode(2, OUTPUT);  
  digitalWrite(2, LOW); 

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0x01;
  data[3] = 0x05;
  data[4] = 0x03;
  data[5] = 30;
  data[6] = 0x8A;
  data[7] = 0x02;
  data[8] = 0x4E;
}

void writeData() {
  for(int i=0; i<DATA_SIZE; i++) {
    Serial2.write(0xAA);
    Serial2.flush();
  }
}

void loop() {
  digitalWrite(2, HIGH); 
  writeData();
  digitalWrite(2, LOW); 

  delay(5000);
}
