  
  
char ch;
int index = 0;
char strValue[4];
  
void setup() {
  
  Serial.begin(19200);
  Serial.write("Ready..."); 

//  char strValue[4];
//  int index = 0;
//  strValue[index++] = 52;
//  strValue[index++] = 48;
//  strValue[index++] = 48;
//
//  Serial.write(strValue); 

}

void loop() {
//  Serial.println("Testing...");
//  delay(1000);

//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incomingByte = Serial.read();
//
//    // say what you got:
//    Serial.print("I received: ");
//    Serial.println(incomingByte, DEC);
//  }
}


void serialEvent()
{


  
  while(Serial.available()){

    if (Serial.available() > 0) {
      ch = Serial.read();

      Serial.print("I received: ");
     Serial.println(ch, DEC);

      //if(isdigit(ch)){
      if(ch!=44){
         strValue[index++] = ch;
      }
      else{
        //Serial.println(atoi(strValue), DEC);
        Serial.println(strValue);
        index=0;               
      }

  }
    
//    ch = Serial.read();
//    //Serial.println(ch);
//
//    if(isdigit(ch)){
//      //Serial.println(ch);
//      strcat(str, ch);      
//      
//    }
////    else{
////      Serial.println(str);    
////      free(str);     
////    }
//
//    Serial.write(str);    
    
  }
}
