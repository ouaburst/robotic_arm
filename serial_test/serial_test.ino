char ch;
int index = 0;
char strValue[2];
int count = 0;
String data;
char char_array[40];
char *token;

int n;
int dataParam;
String serialData;
int posIndex;

typedef struct {
   int number;
   int steps;
   float speed;
   int type;            // type servo, stepper
   int dir=0;           // dir. Used only for servo,
} Motors[20];          // dir=DIR1 add (++), dir=DIR2 substract (--)

Motors motor;



void setup() {
  
  Serial.begin(19200);
  Serial.println("Ready..."); 

}

void loop() {
}


void serialEvent()
{

  while(Serial.available()){
    
    dataParam = 1;
    
    if(Serial.available()){
      serialData = Serial.readStringUntil('\n');
      strcpy(char_array, serialData.c_str()); 
    }
    
    token = strtok(char_array, ",");
   
    while(token != NULL) {
          
          posIndex++;
          
          if(dataParam == 1){
            motor[posIndex].number = atoi(token);            
            dataParam++;
            Serial.println("dataParam == 1");
          }
          else if(dataParam == 2){
            
            if (strstr(token, ".") != NULL) {
              motor[posIndex].speed = atof(token);              
            }
            else{
              motor[posIndex].speed = atoi(token);              
            }           
            dataParam++;
            Serial.println("dataParam == 2"); 
          }
          else if(dataParam == 3){
            motor[posIndex].type = atoi(token);              
            dataParam++; 
            Serial.println("dataParam == 3"); 
          }
          else if(dataParam == 4){
            motor[posIndex].steps = atoi(token);              
            dataParam++; 
            Serial.println("dataParam == 4"); 
          }
          else if(dataParam == 5){
            motor[posIndex].dir = atoi(token);              
            Serial.println("dataParam == 5"); 
          }
      
          token = strtok(NULL, ",");
     }        
  }

  Serial.println("No more data");  



  
//  while(Serial.available()){
//    
//    if(Serial.available()){
//        data = Serial.readStringUntil('\n');
//        //Serial.println("Result, " + data);
//
//        strcpy(char_array, data.c_str()); 
//    }
//
//   token = strtok(char_array, ",");
//   
//   while(token != NULL) {
//
//      n = 0;
//      
//      Serial.write(token);    
//      Serial.println("");
//
//      n = atoi(token) + 1;
//     Serial.println(n, DEC);
//      
//      if (strstr(token, ".") != NULL) {
//          Serial.println("Is float");
//      }
//      
//      token = strtok(NULL, ",");
//   }
//
//   Serial.println("============");
//        
//  }
}
