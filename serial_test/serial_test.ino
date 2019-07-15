void setup() {
  
  Serial.begin(19200);
  Serial.write("Ready..."); 

  char str[3];
  strcat(str, "1");
  strcat(str, "2");
  strcat(str, "3");  

  Serial.write(str);    
}

void loop() {
//  Serial.println("Testing...");
//  delay(1000);
}


void serialEvent()
{

  char str[5];
  char ch;
  
  while(Serial.available()){
    
    ch = Serial.read();
    //Serial.println(ch);

    if(isdigit(ch)){
      //Serial.println(ch);
      strcat(str, atoi(ch));      
      
    }else{
      Serial.println(str);    
      free(str);     
    }
  }
}
