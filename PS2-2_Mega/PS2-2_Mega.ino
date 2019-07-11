#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14 Brown
#define PS2_CMD        11  //15 Orange
#define PS2_SEL        10  //16 Yellow
#define PS2_CLK        12  //17 Blue

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

# define SERVO_PIN 44
# define SERVO_PIN2 45
# define MIN_GRIPPER_POS 0
# define MAX_GRIPPER_POS 145
# define MIN_WRSIT_POS 0
# define MAX_WRIST_POS 170

#define shoulderSwich1 36  // switsh3
#define shoulderSwich2 37  // switsh4
#define elbowSwich1 40  // switsh5
#define elbowSwich2 38  // switsh6
#define wristSwich1 39  // switsh7
#define wristSwich2 41  // switsh8
#define baseSwich1 42   // switch1
#define baseSwich2 43   // switch2

#define HOMING_ACTION 1   
#define PLAY_ACTION 2   
#define STEP_MOTOR 1
#define SERVO_MOTOR 2
#define DIR1 1      // servo pos ++
#define DIR2 2      // servo pos --

// ------ stepper motor#1 -------
# define DIR_PIN_M1 6
# define STEP_PIN_M1 7
int pos1M1;
int pos2M1;
int pssLYDownM1;
int pssLYUpM1;

// ------ stepper motor#2 -------
# define DIR_PIN_M2 8
# define STEP_PIN_M2 9
int pos1M2;
int pos2M2;
int pssLYDownM2;
int pssLYUpM2;

// ------ stepper motor#3 -------
# define DIR_PIN_M3 4
# define STEP_PIN_M3 5
int pos1M3;
int pos2M3;
int pssLYDownM3;
int pssLYUpM3;

// ------ stepper motor#4 -------
# define DIR_PIN_M4 2
# define STEP_PIN_M4 3
int pos1M4;
int pos2M4;
int pssLYDownM4;
int pssLYUpM4;

int stopBasePos1;
int stopBasePos2;
int stopWristPos1;
int stopWristPos2;
int stopElbowPos1;
int stopElbowPos2;
int stopShoulderPos1;
int stopShoulderPos2;

int homingBase;
int homingShoulder;
int homingWrist;
int homingElbow;
int posIndex;
int steps;
int play;
int playSequence;
int countSteps;
int buzzer; 

// ------ servos --------

Servo myservo;
Servo myservo2;
int servoPos;
int servoPos2;
int gripperServoRotateDir1;
int gripperServoRotateDir2;
int wristServoRotateDir1;
int wristServoRotateDir2;

// --------- Struct -----------

typedef struct {
   int number;
   int steps;
   float speed;
   int type;            // type servo, stepper
   int dir=0;           // dir. Used only for servo,
} Motors[250];          // dir=DIR1 add (++), dir=DIR2 substract (--)

Motors motor;

int testLoop;

// ----------------------------

void setup(){  

  // ------ Initialize values ----------
  pos1M4 = 0;
  pos2M4 = 0;
  pssLYDownM4 = 0;
  pssLYUpM4 = 0;
  
  stopBasePos1 = 0;
  stopBasePos2 = 0;
  stopWristPos1 = 0;
  stopWristPos2 = 0;
  stopElbowPos1 = 0;
  stopElbowPos2 = 0;
  stopShoulderPos1 = 0;
  stopShoulderPos2 = 0;

  homingBase = 0;
  homingShoulder = 0;
  homingWrist = 0;
  homingElbow = 0;

  posIndex = 0;
  steps = 0;
  countSteps = 0;
  buzzer = 31; 
  
  // ---- servo ----
  servoPos = 90;
  servoPos2 = 90;
  gripperServoRotateDir1 = 0;
  gripperServoRotateDir2 = 0;
  wristServoRotateDir1 = 0;
  wristServoRotateDir2 = 0;
  // ---------------
  play = 0;
  playSequence = 0;

  //------------ PS2 ---------------   

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

  //--------------------------------------------

  pinMode(buzzer, OUTPUT); 
  noTone(buzzer); 
  
  // ------ Switch pins ---------
  pinMode(shoulderSwich1, INPUT_PULLUP); 
  pinMode(shoulderSwich2, INPUT_PULLUP); 
  pinMode(elbowSwich1, INPUT_PULLUP); 
  pinMode(elbowSwich2, INPUT_PULLUP); 
  pinMode(baseSwich1, INPUT_PULLUP); 
  pinMode(baseSwich2, INPUT_PULLUP); 
  pinMode(wristSwich1, INPUT_PULLUP); 
  pinMode(wristSwich2, INPUT_PULLUP); 

  // ------ servo motor -------
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo2.attach(SERVO_PIN2);

  // ------ stepper motor -------
  pinMode(DIR_PIN_M1, OUTPUT);
  pinMode(STEP_PIN_M1, OUTPUT);
  pinMode(DIR_PIN_M2, OUTPUT);
  pinMode(STEP_PIN_M2, OUTPUT);  
  pinMode(DIR_PIN_M3, OUTPUT);
  pinMode(STEP_PIN_M3, OUTPUT);
  pinMode(DIR_PIN_M4, OUTPUT);
  pinMode(STEP_PIN_M4, OUTPUT);
  // -------------------------

//  Serial.begin(9600);
  Serial.begin(19200);

  //----------------------------
  // --------- Homing ----------
  //----------------------------

  while(initMotors()!=1);



//        Serial.println("Initpos");
//        Serial.println(servoPos, DEC);
//
//        for(int i=0 ; i<200; i++){
//          servoPos++; 
//          myservo.write(servoPos);
//          delay(5);                                    
//        }                
//
//        Serial.println("New pos");
//        Serial.println(servoPos, DEC);
//        
//        delay(2000);         
//        for(int i=0 ; i<200; i++){
//          servoPos--; 
//          myservo.write(servoPos);
//          delay(5);                                    
//        }                
//        
//        Serial.println("End pos");        
//        Serial.println(servoPos, DEC);


      
}

void loop() {
 
  if(error == 1) //skip loop if no controller found
    return; 

    
    //---------------------------------------
    // ----------- Play sequence  -----------
    //---------------------------------------

    if(play){
        // Init motors before play sequences
        while(initMotors()!=1);
        play = 0;        
        playSequence = 1;
    }
    
    if(playSequence){
        delay(1000);         
        for(int i=1 ; i<posIndex+1 ; i++){            

            Serial.print("Servonr: ");                                 
            Serial.println(motor[i].number, DEC);                                             
            Serial.print("Steps: ");                                 
            Serial.println(motor[i].steps, DEC);                                 
            Serial.print("Speed: ");                                 
            Serial.println(motor[i].speed, DEC);                                 
            Serial.print("Type: ");                                 
            Serial.println(motor[i].type, DEC);                                                       
            Serial.println("=====================");                                 
                    
            if(motor[i].type == STEP_MOTOR){
              rotate(motor[i].steps, motor[i].speed, motor[i].number);  
            }
            
            if(motor[i].type == SERVO_MOTOR){

              if(motor[i].number == 5){                
                for(int j1=0 ; j1<motor[i].steps; j1++){
                  myservo.write(j1);
                  delay(motor[i].speed);                                    
                }                
              }

              if(motor[i].number == 6){
                for(int j2=0 ; j2<motor[i].steps; j2++){
                  myservo2.write(j2);
                  delay(motor[i].speed);                                    
                }
              }              
            }
        }
        
        playSequence = 0;        
    }
    
    // --------------------------------------

    if(!play && !playSequence){ 

      //---------------------------------------
      // ------------- Switches ---------------
      //---------------------------------------
      
      // ----------- Base -----------
      
      if(!digitalRead(baseSwich1)){
        stopBasePos1 = 1;

        if(!homingBase){
          homingBase = 1;
        }
        tone(buzzer, 1950); 
      }else{
        stopBasePos1 = 0;
        noTone(buzzer); 
      }

      if(!digitalRead(baseSwich2)){
        stopBasePos2 = 1;
        tone(buzzer, 1950); 
      }else{
        stopBasePos2 = 0;
        noTone(buzzer); 
      }

      // ----------- Wrist -----------
      
      if(!digitalRead(wristSwich1)){
        stopWristPos1 = 1;
        tone(buzzer, 1950); 
      }else{
        stopWristPos1 = 0;
        noTone(buzzer); 
      }

      if(!digitalRead(wristSwich2)){
        stopWristPos2 = 1;
        tone(buzzer, 1950); 
      }else{
        stopWristPos2 = 0;
        noTone(buzzer); 
      }

      // ----------- Elbow -----------
      
      if(!digitalRead(elbowSwich1)){
        stopElbowPos1 = 1;
        tone(buzzer, 1950); 
      }else{
        stopElbowPos1 = 0;
        noTone(buzzer); 
      }

      if(!digitalRead(elbowSwich2)){
        stopElbowPos2 = 1;
        tone(buzzer, 1950); 
      }else{
        stopElbowPos2 = 0;
        noTone(buzzer); 
      }   

      // ----------- Shoulder -----------
      
      if(!digitalRead(shoulderSwich1)){
        stopShoulderPos1 = 1;
        tone(buzzer, 1950); 
      }else{
        stopShoulderPos1 = 0;
        noTone(buzzer); 
      }

      if(!digitalRead(shoulderSwich2)){
        stopShoulderPos2 = 1;
        tone(buzzer, 1950); 
      }else{
        stopShoulderPos2 = 0;
        noTone(buzzer); 
      }         

    // --------------------------------------------
    // ------------- Servo1 gripper ---------------
    // --------------------------------------------
    
    if(ps2x.Button(PSB_PAD_RIGHT)){               
      if(servoPos > MIN_GRIPPER_POS){

        if(!gripperServoRotateDir1){
          posIndex++;
          motor[posIndex].number = 5;          
          motor[posIndex].speed = 5;    
          motor[posIndex].type = SERVO_MOTOR;   
          motor[posIndex].dir = DIR2;   
          gripperServoRotateDir1 = 1;
          gripperServoRotateDir2 = 0;

          Serial.println("");           
          Serial.println("S1");               
        }        
        servoPos -= 1;
        motor[posIndex].steps = servoPos;     

        Serial.print(":");        
        Serial.print(servoPos, DEC);
                  
        myservo.write(servoPos);
        delay(5);  
      }
    }
    if(ps2x.Button(PSB_PAD_LEFT)){     
      if(servoPos < MAX_GRIPPER_POS){

        if(!gripperServoRotateDir2){
          posIndex++;
          motor[posIndex].number = 5;          
          motor[posIndex].speed = 5;    
          motor[posIndex].type = SERVO_MOTOR; 
          motor[posIndex].dir = DIR1;               
          gripperServoRotateDir2 = 1;
          gripperServoRotateDir1 = 0;

          Serial.println("");           
          Serial.println("S1");               
        }
        servoPos += 1;
        motor[posIndex].steps = servoPos;     

        Serial.print(":");        
        Serial.print(servoPos, DEC);
                     
        myservo.write(servoPos);
        delay(5); 
      }
    }

    // ---------------------------------------------------
    // ------------- Servo2 wrist rotation ---------------
    // ---------------------------------------------------
    
    if(ps2x.Button(PSB_PAD_UP)) {      
      if(servoPos2 > MIN_WRSIT_POS){

        if(!wristServoRotateDir1){
          posIndex++;
          motor[posIndex].number = 6;          
          motor[posIndex].speed = 5;    
          motor[posIndex].type = SERVO_MOTOR;  
          motor[posIndex].dir = DIR2;              
          wristServoRotateDir1 = 1;
          wristServoRotateDir2 = 0;

          Serial.println("");           
          Serial.println("S2");               
        }        
        servoPos2 -= 1;
        motor[posIndex].steps = servoPos2;     

        Serial.print(":");        
        Serial.print(servoPos2, DEC);
                     
        myservo2.write(servoPos2);
        delay(5);  
      }
    }
    
    if(ps2x.Button(PSB_PAD_DOWN)){
      if(servoPos2 < MAX_WRIST_POS){

        if(!wristServoRotateDir2){
          posIndex++;
          motor[posIndex].number = 6;          
          motor[posIndex].speed = 5;    
          motor[posIndex].type = SERVO_MOTOR;   
          motor[posIndex].dir = DIR1;             
          wristServoRotateDir2 = 1;
          wristServoRotateDir1 = 0;

          Serial.println("");           
          Serial.println("S2");     
        }        
        servoPos2 += 1;
        motor[posIndex].steps = servoPos2;   

        Serial.print(":");        
        Serial.print(servoPos2, DEC);
                               
        myservo2.write(servoPos2);
        delay(5); 
      }
    }   

    // -----------------------------------------------
    // ------ Press button for playing sample --------
    // -----------------------------------------------
        
      if(ps2x.ButtonPressed(PSB_CIRCLE)){
            play = 1;                              
            Serial.println("Play...");            
      }       

    //------------------------------------
    // ------ stepper motor#1 Base -------
    //------------------------------------
    
    if(!stopBasePos1){
      if((ps2x.Analog(PSS_RX) == 0)){
        if(pssLYUpM2 == 0){
          pssLYUpM2 = 1;     
          posIndex++;
          motor[posIndex].number = 1;          
          motor[posIndex].speed = 0.10;    
          motor[posIndex].type = STEP_MOTOR;   
          countSteps = 0;     
          
          Serial.println("");           
          Serial.println("B");           
        }                
        if(pos2M2 > 0 && pssLYDownM2 == 1){
          pos2M2 = 0;
          pssLYDownM2 = 0;
        }                
        pos2M2=1;
        countSteps++;        
        motor[posIndex].steps = countSteps;        
        rotate(pos2M2, 0.10, 1);

        Serial.print(":");        
        Serial.print(countSteps, DEC);
      }        
    }

    if(!stopBasePos2){
      if((ps2x.Analog(PSS_RX) == 255)){
        if(pssLYDownM2 == 0){
          pssLYDownM2 = 1;
          posIndex++;
          motor[posIndex].number = 1;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;                     
          countSteps = 0; 

          Serial.println("");                     
          Serial.println("B");           
        }          
        if(pos2M2 < 0 && pssLYUpM2 == 1){
          pos2M2 = 0;
          pssLYUpM2 = 0;
        }                   
        pos2M2=-1;
        countSteps--; 
        motor[posIndex].steps = countSteps;  
        rotate(pos2M2, 0.10, 1);
        
        Serial.print(":");            
        Serial.print(countSteps, DEC);
      }     
    }

    //----------------------------------------    
    // ------ stepper motor#2 Shoulder -------
    //----------------------------------------
    
    if(!stopShoulderPos1){
      if((ps2x.Analog(PSS_RY) == 0)){
        if(pssLYUpM1 == 0){
          pssLYUpM1 = 1;  
          posIndex++;
          motor[posIndex].number = 2;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;   
          countSteps = 0; 

          Serial.println("");                     
          Serial.println("S");            
        }                   
        if(pos1M1 > 0 && pssLYDownM1 == 1){
          pos1M1 = 0;
          pssLYDownM1 = 0;
        }                
        pos1M1=1;
        countSteps++;        
        motor[posIndex].steps = countSteps;          
        rotate(pos1M1, 0.10, 2);
        
        Serial.print(":");                
        Serial.print(countSteps, DEC);
      }      
    }

    if(!stopShoulderPos2){
      if((ps2x.Analog(PSS_RY) == 255)){
        if(pssLYDownM1 == 0){
          pssLYDownM1 = 1;
          posIndex++;
          motor[posIndex].number = 2;          
          motor[posIndex].speed = 0.10;    
          motor[posIndex].type = STEP_MOTOR;   
          countSteps = 0;

          Serial.println("");                     
          Serial.println("S");           
        }                             
        if(pos2M1 < 0 && pssLYUpM1 == 1){
          pos2M1 = 0;
          pssLYUpM1 = 0;
        }                   
        pos2M1=-1;
        countSteps--;        
        motor[posIndex].steps = countSteps;                  
        rotate(pos2M1, 0.10, 2);
        
        Serial.print(":");        
        Serial.print(countSteps, DEC);
      }          
    }

    //------------------------------------
    // ------ stepper motor#4 Elbow ------
    //------------------------------------
    
    if(!stopElbowPos1){
      if((ps2x.Analog(PSS_LX) == 0)){
        if(pssLYUpM3 == 0){
          pssLYUpM3 = 1;        
          posIndex++;
          motor[posIndex].number = 3;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;    
          countSteps = 0;  

          Serial.println("");                     
          Serial.println("E");
        }
        if(pos1M3 > 0 && pssLYDownM3 == 1){
          pos1M3 = 0;
          pssLYDownM3 = 0;
        }                
        pos1M3=1;
        countSteps++;        
        motor[posIndex].steps = countSteps;                
        rotate(pos1M3, 0.10, 3);
        
        Serial.print(":");                
        Serial.print(countSteps, DEC);
      }        
    }

    if(!stopElbowPos2){
      if((ps2x.Analog(PSS_LX) == 255)){
        if(pssLYDownM3 == 0){
          pssLYDownM3 = 1;
          posIndex++;
          motor[posIndex].number = 3;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;    
          countSteps = 0;  

          Serial.println("");                        
          Serial.println("E");                                          
        }
        if(pos2M3 < 0 && pssLYUpM3 == 1){
          pos2M3 = 0;
          pssLYUpM3 = 0;
        }                   
        pos2M3=-1;
        countSteps--;        
        motor[posIndex].steps = countSteps;                        
        rotate(pos2M3, 0.10, 3);
        
        Serial.print(":");        
        Serial.print(countSteps, DEC);
      }            
    }
    
    //------------------------------------
    // ------ stepper motor#4 wrist-------
    //------------------------------------

    if(!stopWristPos2){
      if((ps2x.Analog(PSS_LY) == 0)){
        if(pssLYUpM4 == 0){
          pssLYUpM4 = 1;        
          posIndex++;
          motor[posIndex].number = 4;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;    
          countSteps = 0;   

          Serial.println("");                         
          Serial.println("W");
        }         
        if(pos1M4 > 0 && pssLYDownM4 == 1){
          pos1M4 = 0;
          pssLYDownM4 = 0;
        }                
        pos1M4=1;
        countSteps++;        
        motor[posIndex].steps = countSteps;                                
        rotate(pos1M4, 0.10, 4);
        
        Serial.print(":");                
        Serial.print(countSteps, DEC);
      }        
    }

    if(!stopWristPos1){
      if((ps2x.Analog(PSS_LY) == 255)){
        if(pssLYDownM4 == 0){
          pssLYDownM4 = 1;
          posIndex++;
          motor[posIndex].number = 4;          
          motor[posIndex].speed = 0.10;   
          motor[posIndex].type = STEP_MOTOR;    
          countSteps = 0;  

          Serial.println("");                              
          Serial.println("W");                                   
        }          
        if(pos2M4 < 0 && pssLYUpM4 == 1){
          pos2M4 = 0;
          pssLYUpM4 = 0;
        }                   
        pos2M4=-1;
        countSteps--;        
        motor[posIndex].steps = countSteps;                                        
        rotate(pos2M4, 0.10, 4);
        
        Serial.print(":");        
        Serial.print(countSteps, DEC);
      }      
    }
    }

    //==========================================================

    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");         
    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
     
       if(ps2x.Button(PSB_L1))
        Serial.println("L1 pressed");
             
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");   
  
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");    
    
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("Square just released");     
    
}

// ---------------------------------

int initMotors(){
    // --------- Homing ----------

      if(!homingWrist){
        rotate(-100, 0.10, 4);
        if(!digitalRead(wristSwich1)){          
          rotate(500, 0.10, 4);
          homingWrist = 1;  
        }
      }
      
      if(!homingElbow && homingWrist ){
        rotate(-100, 0.10, 3);
        if(!digitalRead(elbowSwich2)){          
          rotate(500, 0.10, 3);
          homingElbow = 1;  
        }
      }
         
      if(!homingShoulder && homingWrist && homingElbow){
        rotate(-100, 0.10, 2);
        if(!digitalRead(shoulderSwich2)){          
          rotate(1000, 0.10, 2);
         // rotate(1500, 0.10, 4);
          homingShoulder = 1;  
        }
      }
           
      if(!homingBase && homingShoulder && homingWrist && homingElbow){
        rotate(-100, 0.10, 1);
        if(!digitalRead(baseSwich2)){          
          rotate(1300, 0.10, 1);
          //rotate(2300, 0.10, 1);
          homingBase = 1;            
        }
      }  

      if(homingBase && homingShoulder && homingWrist && homingElbow){
        homingWrist = 0;  
        homingElbow = 0;  
        homingShoulder = 0;             
        homingBase = 0;  
        
        //---------------------------------
        // --------- Init servos ----------
        //---------------------------------
  
        for(int i=0 ; i<servoPos; i++){
          myservo.write(i);
          delay(5);                                    
        }                
  
        for(int i=0 ; i<servoPos2;  i++){
          myservo2.write(i);
          delay(5);                                    
        }        
        return 1;                   
      }
}

// ---------------------------------

void rotate(int steps, float speed, int motor) {
  //rotate a specific number of microsteps (8 microsteps per step) – (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest – Slower is stronger

  if(motor == 1){
    int dir = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
  
    digitalWrite(DIR_PIN_M1, dir);
  
    float usDelay = (1 / speed) * 70;
  
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN_M1, HIGH);
      delayMicroseconds(usDelay);
      digitalWrite(STEP_PIN_M1, LOW);
      delayMicroseconds(usDelay);
    }    
  }

  if(motor == 2){
    int dir = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
  
    digitalWrite(DIR_PIN_M2, dir);
  
    float usDelay = (2 / speed) * 70;
  
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN_M2, HIGH);
      delayMicroseconds(usDelay);
      digitalWrite(STEP_PIN_M2, LOW);
      delayMicroseconds(usDelay);
    }    
  }  
  
  if(motor == 3){
    int dir = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
  
    digitalWrite(DIR_PIN_M3, dir);
  
    float usDelay = (1 / speed) * 70;
  
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN_M3, HIGH);
      delayMicroseconds(usDelay);
      digitalWrite(STEP_PIN_M3, LOW);
      delayMicroseconds(usDelay);
    }    
  }

  if(motor == 4){
    int dir = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
  
    digitalWrite(DIR_PIN_M4, dir);
  
    float usDelay = (1 / speed) * 70;
  
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN_M4, HIGH);
      delayMicroseconds(usDelay);
      digitalWrite(STEP_PIN_M4, LOW);
      delayMicroseconds(usDelay);
    }    
  }
  
}
