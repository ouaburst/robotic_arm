void setup() {
  
  Serial.begin(19200);
  Serial.write("This is a test"); 
}

void loop() {
//  Serial.println("Testing...");
//  delay(1000);
}


void serialEvent()
{

  char *token;
  
  while(Serial.available()){
    char ch = Serial.read();
    Serial.write(ch);
  
    token = strtok(ch, ","); 

    while (token != NULL) 
    { 
        Serial.write(token);    
        token = strtok(NULL, ","); 
    }

     
  }
}
