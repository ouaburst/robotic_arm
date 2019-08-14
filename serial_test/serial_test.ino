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
  DisplayData = 0;
  posIndex = 0;
}

void loop() {

  if(DisplayData){

    for(int i=1; i<posIndex+1 ; i++){

      Serial.println(motor[i].number,DEC);
      Serial.println(motor[i].speed,DEC);
      Serial.println(motor[i].type,DEC);
      Serial.println(motor[i].steps,DEC);
      Serial.println(motor[i].dir,DEC);
      Serial.println("---------------");
    }

    DisplayData = 0;
  }
}


void serialEvent()
{

  while(Serial.available()){
    
    dataParam = 1;
    posIndex++;
     
    if(Serial.available()){
      serialData = Serial.readStringUntil('\n');
      strcpy(char_array, serialData.c_str()); 
    }
    
    token = strtok(char_array, ",");
   
    while(token != NULL) {
          
          if(dataParam == 1){
            motor[posIndex].number = atoi(token);            
            dataParam++;
            Serial.println(motor[posIndex].number,DEC);
          }
          else if(dataParam == 2){            
            if (strstr(token, ".") != NULL) {
              motor[posIndex].speed = atof(token);     
            }
            else{
              motor[posIndex].speed = atoi(token);              
            }           
            dataParam++;
            Serial.println(motor[posIndex].speed,DEC);
          }
          else if(dataParam == 3){
            motor[posIndex].type = atoi(token);              
            dataParam++; 
            Serial.println(motor[posIndex].type,DEC);
          }
          else if(dataParam == 4){
            motor[posIndex].steps = atoi(token);              
            dataParam++; 
            Serial.println(motor[posIndex].steps,DEC);
          }
          else if(dataParam == 5){
            motor[posIndex].dir = atoi(token);                          
            Serial.println(motor[posIndex].dir,DEC);
            Serial.println("============"); 
          }
      
          token = strtok(NULL, ",");
     }        
  }

  Serial.println("No more data");  

  DisplayData = 1;



  
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
