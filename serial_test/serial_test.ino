  
  
char ch;
int index = 0;
char strValue[2];
 int count = 0;
   
void setup() {
  
  Serial.begin(19200);
  Serial.println("Ready..."); 

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

      //Serial.print("I received: ");
      //Serial.println(ch, DEC);
      Serial.println(count, DEC);

      count++;

//      if(ch==13 || ch==10){
//        
//      }      
//      else if(ch!=44){
//         strValue[index++] = ch;
//      }
//      else{
//        //Serial.println(atoi(strValue), DEC);
//        Serial.print("Result: ");
//        Serial.println(strValue);
//        index=0;               
//      }
    }
  }
}
