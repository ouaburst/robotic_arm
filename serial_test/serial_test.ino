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

}

void loop() {
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
        
  }
}
