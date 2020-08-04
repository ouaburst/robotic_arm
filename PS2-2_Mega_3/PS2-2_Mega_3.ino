#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <AccelStepper.h>
//#include "PinChangeInterrupt.h"

#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

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

AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);    // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);    // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);    // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);    // Wrist

int basePos;
int wristPos;
int elbowPos;
int shoulderPos;

// ------ Robot geometry ------

#define h 23.5  // Height cm
#define L 5.5     // Shoulder
#define M 11   // Elbow
#define N 14.5  // Wrist

// ----- IK angles -----
// d = angle base
// a = angle shoulder
// b = angle elbow
// c = angle wrist

float a, b, c, d;   // Joint variables that will be calculated

// EE position
float dx, dy, dz;

// ------ servos --------

# define SERVO_PIN 44
# define SERVO_PIN2 45

# define MIN_GRIPPER_POS 0
# define MAX_GRIPPER_POS 145
# define MIN_WRSIT_POS 0
# define MAX_WRIST_POS 170
#define SERVO_MOTOR 2
#define DIR1 1      // servo pos ++
#define DIR2 2      // servo pos --

Servo myservo;
Servo myservo2;

int servoPos;
int servoPos2;
int gripperServoRotateDir1;
int gripperServoRotateDir2;
int wristServoRotateDir1;
int wristServoRotateDir2;
int servoSteps;

// ----------------------
#define HOMING_ACTION 1   
#define PLAY_ACTION 2   
#define STEP_MOTOR 1

// --------- Struct -----------

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

// ---- Serial ----
String serialData;
char char_array[256];
char *token;

// --- Switches ---
#define shoulderSwich1 36  // switsh3
#define shoulderSwich2 37  // switsh4
#define elbowSwich1 40  // switsh5
#define elbowSwich2 38  // switsh6
#define wristSwich1 39  // switsh7
#define wristSwich2 41  // switsh8
#define baseSwich1 42   // switch1
#define baseSwich2 43   // switch2

// --- Limit positions ---
int stopBasePos1;
int stopBasePos2;
int stopWristPos1;
int stopWristPos2;
int stopElbowPos1;
int stopElbowPos2;
int stopShoulderPos1;
int stopShoulderPos2;

// ------ Stepper motor speed -------
# define STEP_MOTOR_SPEED_BASE 400
# define STEP_MOTOR_SPEED_SHOULDER 400
# define STEP_MOTOR_SPEED_ELBOW 400
# define STEP_MOTOR_SPEED_WRIST 400

// --- Misc ---
int buzzer; 
int homingDone;

int homingBase;
int homingShoulder;
int homingWrist;
int homingElbow;

int endPositionBase;
int endPositionWrist;
int endPositionElbow;
int endPositionShoulder;

int pos;
int posIndex;
int shoulderDir1;
int shoulderDir2;
int elbowDir1;
int elbowDir2;
int wristDir1;
int wristDir2;
int baseDir1;
int baseDir2;

int stepperSteps;
int DisplayData;

void setup(){  
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

  Serial.begin(19200);

  stepper1.setMaxSpeed(1000);         // Base
  stepper1.setAcceleration(500);
  stepper4.setMaxSpeed(1000);         // Wrist
  stepper4.setAcceleration(500);
  stepper3.setMaxSpeed(1000);         // Elbow
  stepper3.setAcceleration(500);
  stepper2.setMaxSpeed(1000);         // Shoulder
  stepper2.setAcceleration(500);
  
  // ------ servo motors -------
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  myservo.attach(SERVO_PIN);
  //myservo2.attach(SERVO_PIN2);

  servoPos = MAX_GRIPPER_POS;
  servoPos2 = 90;
  gripperServoRotateDir1 = 0;
  gripperServoRotateDir2 = 0;
  wristServoRotateDir1 = 0;
  wristServoRotateDir2 = 0;

  // ------ Step motors positions ------
  basePos = 0;
  wristPos = 0;
  elbowPos = 0;
  posIndex = 0;
  
  // ------ Switch pins ---------
  pinMode(shoulderSwich1, INPUT_PULLUP); 
  pinMode(shoulderSwich2, INPUT_PULLUP); 
  pinMode(elbowSwich1, INPUT_PULLUP); 
  pinMode(elbowSwich2, INPUT_PULLUP); 
  pinMode(baseSwich1, INPUT_PULLUP); 
  pinMode(baseSwich2, INPUT_PULLUP); 
  pinMode(wristSwich1, INPUT_PULLUP); 
  pinMode(wristSwich2, INPUT_PULLUP); 

  //attachPCINT(digitalPinToPCINT(wristSwich2), homeWrist, CHANGE);
  
  // ------ Misc ---------
  buzzer = 31; 
  pinMode(buzzer, OUTPUT); 
  noTone(buzzer); 
  homingDone = 0;
  
  // ------ Limit values -------
  stopBasePos1 = 0;
  stopBasePos2 = 0;
  stopWristPos1 = 0;
  stopWristPos2 = 0;
  stopElbowPos1 = 0;
  stopElbowPos2 = 0;
  stopShoulderPos1 = 0;
  stopShoulderPos2 = 0;

  // ---------- For homing ----------------
  endPositionBase = 1200;
  endPositionWrist = 1200;
  endPositionElbow = 1200;
  endPositionShoulder = 1200;

  // --------- Stepper directions -----------
  shoulderDir1 = 0;
  shoulderDir2 = 0;
  elbowDir1 = 0;
  elbowDir2 = 0;
  wristDir1 = 0;
  wristDir2 = 0;
  baseDir1 = 0;
  baseDir2 = 0;

  homingBase = 0;
  homingShoulder = 0;
  homingWrist = 0;
  homingElbow = 0;
  pos = 0;

  stepperSteps = 0;
  DisplayData = 0;

  //homingDone = 1;
  
  while(!homingDone){
    initMotors();
  }

  // --------------------------------------

//  dx = 25.05;
//  dy = 10.0;
//  dz = 10;
//
//  // d = angle base
//  // a = angle shoulder
//  // b = angle elbow
//  // c = angle wrist
//
//  inverseKinematics(dx,dy,dz);
//
//  // Initialize angles @90 degrees = postion 0
//  d = round(rad2Deg(d))-90;
//  a = round(rad2Deg(a))-90;
//  b = round(rad2Deg(b))-90;
//  c = round(rad2Deg(c))-90;
//  
//  Serial.print("Angle base: ");                  
//  Serial.println(d, DEC);                  
//  Serial.print("Angle shoulder: ");                  
//  Serial.println(a, DEC);                    
//  Serial.print("Angle elbow: ");                  
//  Serial.println(b, DEC);                  
//  Serial.print("Angle wrist: ");                  
//  Serial.println(c, DEC);                  
//
//  // Target postion base
//
//  int targetPosBase = 0;
//  int targetPosShoulder = 0;
//    
//  targetPosBase = (d*1200/90);
//  Serial.print("targetPosBase: ");                  
//  Serial.println(targetPosBase, DEC);                  
//
//  targetPosShoulder = (a*1200/90);
//  Serial.print("targetPosShoulder: ");                  
//  Serial.println(targetPosShoulder, DEC);                  
//
//  stepper1.moveTo(targetPosBase);
//  stepper1.runToPosition();
//  stepper1.setCurrentPosition(0); 
//
//  stepper2.moveTo(targetPosShoulder);
//  stepper2.runToPosition();
//  stepper2.setCurrentPosition(0); 
//  
}

void loop() {
 
  if(error == 1) //skip loop if no controller found
    return; 
    
  // -----------------------------------
  // ----------- Base limits -----------
  // -----------------------------------
  
  if(!digitalRead(baseSwich1)){
    stopBasePos1 = 1;   
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

  // -----------------------------------
  // ----------- Wrist limits ----------
  // -----------------------------------
  
  if(!digitalRead(wristSwich1)){

    Serial.println("wristSwich1");           
        
    stopWristPos1 = 1;   
    tone(buzzer, 1950); 
  }else{
    stopWristPos1 = 0;
    noTone(buzzer); 
  }

  if(!digitalRead(wristSwich2)){

    Serial.println("wristSwich2");           
    
    stopWristPos2 = 1;
    tone(buzzer, 1950); 
  }else{
    stopWristPos2 = 0;
    noTone(buzzer); 
  }  

  // -----------------------------------
  // ----------- Elbow limits ----------
  // -----------------------------------
  
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

  // --------------------------------------
  // ----------- Shoulder limits ----------
  // --------------------------------------
  
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
        servoSteps = 0;
      }        
      servoPos -= 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;                     
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
        servoSteps = 0;
      }
      servoPos += 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;     
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
        servoSteps = 0;
      }        
      servoPos2 -= 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;     
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
        servoSteps = 0;
      }        
      servoPos2 += 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;   
      myservo2.write(servoPos2);
      delay(5); 
    }
  }   
  
  //-----------------------------------------------------------
  // ------ Move stepper motor#1 Base with PS2 joystick -------
  //-----------------------------------------------------------
  
//  if(!stopBasePos2){
//    if((ps2x.Analog(PSS_RX) == 255)){  
//      if(baseDir1==0){
//        baseDir1=1;
//        posIndex++;
//        motor[posIndex].number = 1;          
//        motor[posIndex].speed = STEP_MOTOR_SPEED_BASE;    
//        motor[posIndex].type = STEP_MOTOR;   
//        stepperSteps = 0;
//
//        Serial.print("--> Posindex: ");           
//        Serial.println(posIndex, DEC);                   
//      }
//      if(baseDir2==1){
//        baseDir2=0;
//      }                          
//      basePos = basePos+1;          
//      motor[posIndex].steps = basePos; 
//      Serial.println(basePos, DEC);                  
//    }                      
//  }

//  if(!stopBasePos1){
//    if((ps2x.Analog(PSS_RX) == 0)){
//      if(baseDir2==0){
//        baseDir2=1;
//        posIndex++;
//        motor[posIndex].number = 1;          
//        motor[posIndex].speed = STEP_MOTOR_SPEED_BASE;    
//        motor[posIndex].type = STEP_MOTOR;  
//
//        Serial.print("--> Posindex: ");           
//        Serial.println(posIndex, DEC);           
//         
//      }
//      if(baseDir1==1){
//        baseDir1=0;
//      }                          
//      basePos = basePos-1;  
//      motor[posIndex].steps = basePos;               
//      Serial.println(basePos, DEC);                  
//    }      
//  }

  if(!stopBasePos2){
    if((ps2x.Analog(PSS_RX) == 255)){  
      basePos = basePos+1;
    }
  }

  if(!stopBasePos1){
    if((ps2x.Analog(PSS_RX) == 0)){
      basePos = basePos-1;  
    }      
  }

  stepper1.moveTo(basePos);
  stepper1.setSpeed(STEP_MOTOR_SPEED_BASE);    
    
  while (stepper1.distanceToGo() !=0) {
    stepper1.runSpeedToPosition();
  }
      
  //------------------------------------------------------------
  // ------ Move stepper motor#4 Wrist with PS2 joystick -------
  //------------------------------------------------------------
  
  if(!stopWristPos2){
    if((ps2x.Analog(PSS_LY) == 255)){                      
        wristPos = wristPos+1;          
        //Serial.println(wristPos, DEC);                  
    }                      
  }

  if(!stopWristPos1){
    if((ps2x.Analog(PSS_LY) == 0)){
        wristPos = wristPos-1;          
        //Serial.println(wristPos, DEC);                  
    }      
  }
  
  stepper4.moveTo(wristPos);
  stepper4.setSpeed(STEP_MOTOR_SPEED_WRIST);    
  
  while (stepper4.distanceToGo() !=0) {
    stepper4.runSpeedToPosition();
  }

  //------------------------------------------------------------
  // ------ Move stepper motor#3 Elbow with PS2 joystick -------
  //------------------------------------------------------------
  
  if(!stopElbowPos2){
    if((ps2x.Analog(PSS_LX) == 255)){                      
        elbowPos = elbowPos-1;          
        //Serial.println(elbowPos, DEC);                  
    }                      
  }

  if(!stopElbowPos1){
    if((ps2x.Analog(PSS_LX) == 0)){
        elbowPos = elbowPos+1;          
        //Serial.println(elbowPos, DEC);                  
    }      
  }
      
  stepper3.moveTo(elbowPos);
  stepper3.setSpeed(STEP_MOTOR_SPEED_ELBOW);    

  while (stepper3.distanceToGo() !=0) {
    stepper3.runSpeedToPosition();
  }

  //---------------------------------------------------------------
  // ------ Move stepper motor#2 Shoulder with PS2 joystick -------
  //---------------------------------------------------------------
  
  if(!stopShoulderPos1){
    if((ps2x.Analog(PSS_RY) == 255)){                      
        shoulderPos = shoulderPos-1;          
        //Serial.println(shoulderPos, DEC);                  
    }                      
  }

  if(!stopShoulderPos2){
    if((ps2x.Analog(PSS_RY) == 0)){
        shoulderPos = shoulderPos+1
        
        
        
        ;          
        //Serial.println(shoulderPos, DEC);                  
    }      
  }
      
  stepper2.moveTo(shoulderPos);
  stepper2.setSpeed(STEP_MOTOR_SPEED_SHOULDER);    
  
  while (stepper2.distanceToGo() !=0) {
    stepper2.runSpeedToPosition();
  }

  // -----------------------------------------------
  // ------ Press button for playing sample --------
  // -----------------------------------------------
      
    if(ps2x.ButtonPressed(PSB_CIRCLE)){
      DisplayData = 1;
    }       

  if(DisplayData){

     Serial.println("++++++++++++ Display Data ++++++++++++++++");

    for(int i=1; i<posIndex+1 ; i++){

      Serial.println(motor[i].number,DEC);
      Serial.println(motor[i].speed,DEC);
      Serial.println(motor[i].type,DEC);
      Serial.println(motor[i].steps,DEC);
      Serial.println(motor[i].dir,DEC);
      Serial.println("-------------------");
    }

    DisplayData = 0;
  }


//----------------------------------------   

  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
   
}

// -------------------------
// ------ initMotors -------
// -------------------------

int hitWristSwich1 = 0;
int hitbaseSwich1 = 0;
int hitelbowSwich1 = 0;
int hitshoulderSwich1 = 0;

void initMotors(){

  // --------- Homing Base ----------

homingBase = 1;
homingElbow = 1;
homingShoulder = 1;

  if(!homingBase){

    pos = pos-1;
    stepper1.moveTo(pos);
    stepper1.setSpeed(400);    
    
    while(stepper1.distanceToGo() !=0 && digitalRead(baseSwich1)){
      stepper1.runSpeedToPosition();

      if(!digitalRead(baseSwich1)){   
        stepper1.stop();
        hitbaseSwich1 = 1;
        break;
      }      
    }
    
    if(hitbaseSwich1){   
      stepper1.setCurrentPosition(0); 
      stepper1.setMaxSpeed(1000);
      stepper1.setAcceleration(500);
      stepper1.moveTo(endPositionBase);
      stepper1.runToPosition();
      stepper1.setCurrentPosition(0); 
      homingBase = 1;
      pos = 0;
    }
  }

  // --------- Homing Wrist ----------
  
  if(homingBase && !homingWrist){
   
    pos = pos-1;
    stepper4.moveTo(pos);
    stepper4.setSpeed(400);    
    
    while(stepper4.distanceToGo()){
      if(!digitalRead(wristSwich1)){   
        stepper4.stop();
        hitWristSwich1 = 1;
        break;
      }      
      else{
        stepper4.runSpeedToPosition();  
      }
    }

    if(hitWristSwich1){         
      stepper4.setCurrentPosition(0); 
      stepper4.setMaxSpeed(1000);
      stepper4.setAcceleration(500);
      stepper4.moveTo(endPositionWrist);
      stepper4.runToPosition();
      stepper4.setCurrentPosition(0); 
      homingWrist = 1;
      pos = 0;
    }
  }

  // --------- Homing Elbow ----------
  
  if(homingBase && homingWrist && !homingElbow){

    pos = pos+1;
    stepper3.moveTo(pos);
    stepper3.setSpeed(400);    
    
    while(stepper3.distanceToGo() !=0 && digitalRead(elbowSwich1)){
      stepper3.runSpeedToPosition();

      if(!digitalRead(elbowSwich1)){   
        stepper3.stop();
        hitelbowSwich1 = 1;
        break;
      }            
    }

    //Serial.println("Move to switch "); 
    
    if(hitelbowSwich1){   
      stepper3.setCurrentPosition(0); 
      stepper3.setMaxSpeed(1000);
      stepper3.setAcceleration(500);
      stepper3.moveTo(-endPositionElbow);
      stepper3.runToPosition();
      stepper3.setCurrentPosition(0); 
      homingElbow = 1;
      pos = 0;
    }
  }

  // --------- Homing Shoulder ----------
  
  if(homingBase && homingWrist && homingElbow && !homingShoulder){

    pos = pos-1;
    stepper2.moveTo(pos);
    stepper2.setSpeed(400);    
    
    while(stepper2.distanceToGo() !=0 && digitalRead(shoulderSwich1)){
      stepper2.runSpeedToPosition();

      if(!digitalRead(shoulderSwich1)){   
        stepper2.stop();        
        hitshoulderSwich1 = 1;
        break;
      }            
    }

    //Serial.println("Move to switch "); 
    
    if(hitshoulderSwich1){   
      stepper2.setCurrentPosition(0); 
      stepper2.setMaxSpeed(1000);
      stepper2.setAcceleration(500);
      stepper2.moveTo(endPositionShoulder);
      stepper2.runToPosition();
      stepper2.setCurrentPosition(0); 
      homingShoulder = 1;
      pos = 0;
    }
  }

  // --------- Homing Servos ----------
  
  if(homingBase && homingShoulder && homingWrist && homingElbow){
    homingWrist = 0;  
    homingElbow = 0;  
    homingShoulder = 0;             
    homingBase = 0;  
    
    servoPos = 10;
    servoPos2 = 90;
   
    for(int i=0 ; i<servoPos; i++){
      myservo.write(i);
      delay(5);                                    
    }                

    for(int i=0 ; i<servoPos2;  i++){
      myservo2.write(i);
      delay(5);                                    
    }        
    homingDone = 1;
  }
}

// Calculate angles
void inverseKinematics(float dx ,float dy ,float dz){

  float R,S,Q,f,g;
  
  R = sqrt(pow(dx,2) + pow(dy,2));  // The radius from the axis of rotation of the base to dx,dy
  S = R - N;                  // 
  Q = sqrt(pow(S, 2) + pow((dz), 2));
  f = atan2(dz, S);
  
  g = acos((pow(L,2)+pow(Q,2)-pow(M,2))/(2*L*Q));
  
  d = atan2(dx, dy);
  a = f + g;
  b = acos((pow(M, 2) + pow(L, 2) - pow(Q, 2)) / (2 * L * M));
  c = -b - a + 2 * M_PI;
  
}
