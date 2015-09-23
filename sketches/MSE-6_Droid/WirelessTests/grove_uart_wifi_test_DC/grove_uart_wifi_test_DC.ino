// test grove - uart wifi
// scan ap and display on Grove - OLED 0.96'
// Loovee @ 2015-7-28

char ap_buf[30][16];
int ap_cnt = 0;

bool WaitForResponse(String success, String fail, int wait = 200)
{
  delay(500);
  
  Serial.println("Waiting for Success: '" + 
  success + "', or Fail: '" + fail + "'.");

  String results;
  char c1=0;
  for(int i=0; i<wait; i++)
  {
    while(Serial2.available())
    {
      c1 = Serial2.read();
      results += c1;
      Serial.print(c1);  

      if(results.endsWith(success)) {
        Serial.println("");
        Serial.println("Found: " + success);
        return true;
      }

      if(results.endsWith(fail)) {
        Serial.println("");
        Serial.println("Found: " + fail);
        return false;
      }
    }
    delay(500);
  }
  Serial.println("");
  Serial.println("Failed to find values.");
}

void printResults(int wait = 3)
{
  delay(500);
  
  char c1=0;
  for(int i=0; i<wait; i++)
  {
    while(Serial2.available())
    {
      c1 = Serial2.read();
      Serial.print(c1);  
    }
    delay(500);
  }
  Serial.println("");
}

void setup()
{
  Serial.begin(57600);
  while(!Serial);
  Serial.println("Starting setup");

  Serial2.begin(115200);
  while(!Serial2);
  Serial.println("Starting WiFi Serial");

  ap_cnt = 0;

  Serial.println("Resetting module");
  cmd_send("AT+RST"); // reset module
  WaitForResponse("OK", "FAIL");
  printResults(40);
  
  //Serial.println("turn off server on port 80.");
  //cmd_send("AT+CIPSERVER=0");
  //printResults();

  //Serial.println("configuring for single connections.");
  //cmd_send("AT+CIPMUX=0");
  //printResults();

  Serial.println("Checking for access points.");
  cmd_send("AT+CWLAP");
  wait_result();
  display_ap();

  Serial.println("Sending connect");
  cmd_send("AT+CWJAP=\"TRENDnet812_2.4GHz_RPBS\",\"8124RF00337\"");
  if(WaitForResponse("OK", "FAIL"))
  {
    //Serial.println("Getting Status");
    //cmd_send("AT+CIPSTATUS");
    //printResults();

    //Serial.println("Configuring as access point.");
    //cmd_send("AT+CWMODE=2");
    //WaitForResponse("OK", "FAIL");
    
    Serial.println("Getting IP");
    cmd_send("AT+CIFSR");
    printResults();
    
    //Serial.println("Getting IP");
    //cmd_send("AT+CIFSR");
    //printResults();
    
    //Serial.println("configuring for multiple connections.");
    //cmd_send("AT+CIPMUX=1");
    //printResults();
    
    //Serial.println("turn on server on port 5442.");
    //cmd_send("AT+CIPSERVER=1,5442");
    //printResults();
    
    Serial.println("Setup succeeded.");
  }

  delay(3000);
}


void loop()
{
  if(Serial2.available()) // check if the esp is sending a message 
  {
    
     while(Serial2.available())
    {
      // The esp has data so display its output to the serial window 
      char c = Serial2.read(); // read the next character.
      Serial.write(c);
    } 
    
    /*
    if(Serial2.find("+IPD,"))
    {
     delay(1000);

     Serial.println("Found +IPD,");    
 
     int connectionId = Serial2.read()-48; // subtract 48 because the read() function returns 
                                           // the ASCII decimal value and 0 (the first decimal number) starts at 48
     
     String webpage = "<h1>Hello</h1><h2>World!</h2><button>LED1</button>";
 
     String cipSend = "AT+CIPSEND=";
     cipSend += connectionId;
     cipSend += ",";
     cipSend +=webpage.length();
     
     cmd_sendString(cipSend);
     cmd_sendString(webpage);
     
     webpage="<button>LED2</button>";
     
     cipSend = "AT+CIPSEND=";
     cipSend += connectionId;
     cipSend += ",";
     cipSend +=webpage.length();
     
     cmd_sendString(cipSend);
     cmd_sendString(webpage);
 
     String closeCommand = "AT+CIPCLOSE="; 
     closeCommand+=connectionId; // append connection id
     
     cmd_sendString(closeCommand);
    }
    */
  }
}

// send command
void cmd_send(char *cmd)
{
    if(NULL == cmd)return;
    Serial2.println(cmd);
}

void cmd_sendString(String cmd)
{
    Serial2.println(cmd);
    Serial.println(cmd);
}

// wait result of ap scan
// +CWLAP:(3,"360WiFi-UZ",-81,"08:57:00:01:61:ec",1)
void wait_result()
{
    while(1)
    {
LOOP1:
        char c1=0;
        if(Serial2.available()>=2)
        {
            c1 = Serial2.read();
            Serial.print(c1);
            if(c1 == 'O' && 'K' == Serial2.read()) 
            {
              Serial.println("");
              return;       // OK means over
            }
        }
        
        if('('==c1)
        {
            while(Serial2.available()<3);
            Serial2.read();
            Serial2.read();
            Serial2.read();

            int d = 0;
            while(1)
            {
                if(Serial2.available() && '"' == Serial2.read());      // find "
                {
                    while(1)
                    {
                        if(Serial2.available())
                        {
                            char c = Serial2.read();
                            ap_buf[ap_cnt][d++] = c;
                            if(c == '"' || d==16)
                            {
                                ap_buf[ap_cnt][d-1] = '\0';
                                ap_cnt++;
                                goto LOOP1;
                            }
                        }
                    }
                }
            }
        }
    }
}

// display
void display_ap()
{
    char strtmp[16];
    sprintf(strtmp, "get %d ap", ap_cnt);

    Serial.println(strtmp);        // Print the String
 
    delay(2000);
    
    int cnt = ap_cnt;
    int offset = 0;
    while(1)
    {
        if(cnt>=8)
        {
            for(int i=0; i<8; i++)
            {
                Serial.println(ap_buf[8*offset+i]);        // Print the String
            }
            cnt-=8;
            offset++;
        }
        else 
        {
            for(int i=0; i<cnt; i++)
            {
                Serial.println(ap_buf[8*offset+i]);        // Print the String
            }
            
            return;
        }
        
        delay(2000);
    }
}
