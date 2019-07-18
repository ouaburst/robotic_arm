  
  
char ch;
int index = 0;
char strValue[2];
int count = 0;
String data;
char char_array[40];
char *token;

int n;

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
    
    if(Serial.available()){
        data = Serial.readStringUntil('\n');
        //Serial.println("Result, " + data);

        strcpy(char_array, data.c_str()); 
    }

   token = strtok(char_array, ",");
   
   while(token != NULL) {

      n = 0;
      
      Serial.write(token);    
      Serial.println("");

      n = atoi(token) + 1;
     Serial.println(n, DEC);
      
      if (strstr(token, ".") != NULL) {
          Serial.println("Is float");
      }
      
      token = strtok(NULL, ",");
   }

   Serial.println("============");
    
//    if (Serial.available() > 0) {
//      ch = Serial.read();
//
//      Serial.print("I received: ");
//      Serial.println(ch, DEC);
//
//      if(ch==13 || ch==10){        
//      }      
//      else if(ch!=44){
//         strValue[index++] = ch;
//      }
//      else{
//        //Serial.println(atoi(strValue), DEC);
//        Serial.print("Result: ");
//        Serial.println(strValue);
//        
//        Serial.print("atof: ");
//        Serial.println(atof(strValue), DEC);
//        Serial.println("============");
//        index=0;               
//      }
//    }
    
  }
}
