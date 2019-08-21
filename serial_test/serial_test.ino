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
int DisplayData;
int initCounter;

typedef struct {
   int number;
   int steps;
   float speed;
   int type;            // type servo, stepper
   int dir=0;           // dir. Used only for servo,
} Motors[250];          // dir=DIR1 add (++), dir=DIR2 substract (--)

Motors motor;

typedef struct {
  char bufferArray[22];
} SerialBuffers[50];

SerialBuffers serialBuffer;


void setup() {
  
  Serial.begin(19200);
  Serial.println("Ready..."); 
  DisplayData = 0;
  posIndex = 0;
  //Serial.setTimeout(5000);
  initCounter = 1;
}

void loop() {

  if(DisplayData){

      Serial.println("++++++++++++ Display Data ++++++++++++++++");

    for(int i=1; i<posIndex ; i++){

      Serial.println(motor[i].number,DEC);
      Serial.println(motor[i].speed,DEC);
      Serial.println(motor[i].type,DEC);
      Serial.println(motor[i].steps,DEC);
      Serial.println(motor[i].dir,DEC);
      Serial.println("-------------------");
    }

    DisplayData = 0;
  }
}


void serialEvent()
{
  if(initCounter){
    posIndex = 0;
    initCounter = 0;      
  }

  while(Serial.available()){
    
    dataParam = 1;
    posIndex++;
     
    if(Serial.available()){
      serialData = Serial.readStringUntil('\n');
      Serial.println(serialData);

      strcpy(serialBuffer[posIndex].bufferArray, serialData.c_str()); 
      //strcpy(char_array, serialData.c_str()); 

    }
  }

  if(serialData.equals("END")){
    Serial.println("No more data"); 
  
    for(int i=1 ; i < posIndex+1 ; i++){

        token = strtok(serialBuffer[i].bufferArray, ",");
        dataParam = 1;
        
        while(token != NULL) {
          
          if(dataParam == 1){
            motor[i].number = atoi(token);            
            dataParam++;
            Serial.println(motor[i].number,DEC);
          }
          else if(dataParam == 2){            
            if (strstr(token, ".") != NULL) {
              motor[i].speed = atof(token);     
            }
            else{
              motor[i].speed = atoi(token);              
            }           
            dataParam++;
            Serial.println(motor[i].speed,DEC);
          }
          else if(dataParam == 3){
            motor[i].type = atoi(token);              
            dataParam++; 
            Serial.println(motor[i].type,DEC);
          }
          else if(dataParam == 4){
            motor[i].steps = atoi(token);              
            dataParam++; 
            Serial.println(motor[i].steps,DEC);
          }
          else if(dataParam == 5){
            motor[i].dir = atoi(token);  
            Serial.println(motor[i].dir,DEC);
            Serial.println("============"); 
          }      
          token = strtok(NULL, ",");
        }             
     }    


     
  }
    


//    token = strtok(char_array, ",");
//   
//    while(token != NULL) {
//          
//          if(dataParam == 1){
//            motor[posIndex].number = atoi(token);            
//            dataParam++;
//            Serial.println(motor[posIndex].number,DEC);
//          }
//          else if(dataParam == 2){            
//            if (strstr(token, ".") != NULL) {
//              motor[posIndex].speed = atof(token);     
//            }
//            else{
//              motor[posIndex].speed = atoi(token);              
//            }           
//            dataParam++;
//            Serial.println(motor[posIndex].speed,DEC);
//          }
//          else if(dataParam == 3){
//            motor[posIndex].type = atoi(token);              
//            dataParam++; 
//            Serial.println(motor[posIndex].type,DEC);
//          }
//          else if(dataParam == 4){
//            motor[posIndex].steps = atoi(token);              
//            dataParam++; 
//            Serial.println(motor[posIndex].steps,DEC);
//          }
//          else if(dataParam == 5){
//            motor[posIndex].dir = atoi(token);                          
//            Serial.println(motor[posIndex].dir,DEC);
//            Serial.println("============"); 
//          }
//      
//          token = strtok(NULL, ",");
//     } 

  
  DisplayData = 1;
  //homingDone = 0;  
  //play = 1;

}
