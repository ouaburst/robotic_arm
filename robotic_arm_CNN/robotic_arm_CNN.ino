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
float dx, dy, dz, dy2;

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

// a = angle shoulder
// b = angle elbow
// c = angle wrist
// d = angle base

typedef struct {
   float a;
   float b;
   float c;
   float d;
} Angles[10];

Angles angle;


// ---- Serial ----
String serialData;
char char_array[256];
char *token;

// --- Switches ---
// Move all the switch to one port
// Move to switch 43

#define pinSwich 43  // switsh

// --- Limit positions ---
int pinSwichOn;

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

long pos;
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
int moveStepper;

int baseStepMove1,baseStepMove2;
int shoulerStepMove1,shoulerStepMove2;
int elbowStepMove1,elbowStepMove2;
int wristStepMove1,wristStepMove2;

int targetPosBase;
int targetPosShoulder;
int targetPosElbow;
int targetPosWrist;

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
  pinMode(pinSwich, INPUT_PULLUP); 

  //attachPCINT(digitalPinToPCINT(wristSwich2), homeWrist, CHANGE);
  
  // ------ Misc ---------
  buzzer = 31; 
  pinMode(buzzer, OUTPUT); 
  noTone(buzzer); 
  homingDone = 0;
  
  // ------ Limit values -------
  pinSwichOn = 0;

  baseStepMove1 = 1;
  baseStepMove2 = 1;
  shoulerStepMove1 = 1;
  shoulerStepMove2 = 1;
  elbowStepMove1 = 1;
  elbowStepMove2 = 1;
  wristStepMove1 = 1;
  wristStepMove2 = 1;

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

  // -------- Homing -------  

  //homingDone = 1;
  
  while(!homingDone){
    initMotors();
  }

  while(1){

    // --------------------------------------
    // -------------- Sequence1 -------------
    // --------------------------------------

    Serial.println("Sequence1");
        
    dx = 22;
    dy = 0.0;
    dz = -14.58;
    dy2 = 10;
  
    // a = angle shoulder
    // b = angle elbow
    // c = angle wrist
    // d = angle base
  
    inverseKinematics(dx,dy,dz);
  
    // Initialize angles @90 degrees = postion 0
    d = round(rad2Deg(atan2(dx,dy2)));
    a = round(rad2Deg(a));
    b = round(rad2Deg(b));
    c = round(rad2Deg(c));
  
    angle[0].a = a;
    angle[0].b = b;
    angle[0].c = c;
    angle[0].d = d;
          
    stepper1.setMaxSpeed(700);         // Base
    stepper1.setAcceleration(500);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(1500);
    stepper3.setMaxSpeed(700);         // Elbow
    stepper3.setAcceleration(500);
    stepper2.setMaxSpeed(700);         // Shoulder
    stepper2.setAcceleration(500);
  
    targetPosBase = 0;
    targetPosShoulder = 0;
    targetPosElbow = 0;
    targetPosWrist = 0;
        
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = ((180-b)*1200/90);
    targetPosWrist = ((c-180)*1200/90);
    
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(-targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    // -------- First wrist ---------
    
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){      
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;     
//      }
//    }
//  
    // -------- Shoulder + Elbow ---------
    
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      //stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         ){
        
        //stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;     
      }
    }
  
    delay(1000);
  
    // Close gripper  
    servoPos = 55;
  
    for(int i=0 ; i<servoPos; i++){
      myservo.write(i);
      delay(5);
    }                
  
    delay(1000);
  
    // --------------------------------------
    // -------------- Sequence2 -------------
    // --------------------------------------

    Serial.println("Sequence2");
        
    dx = 24;
    dy = 0.0;
    dz = 13;
    dy2 = 10;
  
    // d = angle base
    // a = angle shoulder
    // b = angle elbow
    // c = angle wrist
  
    inverseKinematics(dx,dy,dz);
  
    // Initialize angles @90 degrees = postion 0
    d = round(rad2Deg(atan2(dx,dy2)));
    a = round(rad2Deg(a));
    b = round(rad2Deg(b));
    c = round(rad2Deg(c));
  
    angle[1].a = a;
    angle[1].b = b;
    angle[1].c = c;
    angle[1].d = d;
  
    a -= angle[0].a;
    b -= angle[0].b;
    c -= angle[0].c;
    
    stepper1.setMaxSpeed(2500);         // Base
    stepper1.setAcceleration(1500);
    stepper4.setMaxSpeed(500);         // Wrist
    stepper4.setAcceleration(300);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(1500);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(1500);
  
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);
  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    //stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      //stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         ){
        
        //stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);
    
    // --------------------------------------
    // -------------- Sequence3 -------------
    // --------------------------------------

    Serial.println("Sequence3");
    
    dx = 31;
    dy = 0.0;
    dz = 0;
    dy2 = 0;
  
    // d = angle base
    // a = angle shoulder
    // b = angle elbow
    // c = angle wrist
  
    inverseKinematics(dx,dy,dz);
  
    // Initialize angles @90 degrees = postion 0
    d = round(rad2Deg(atan2(dx,dy2)));
    a = round(rad2Deg(a));
    b = round(rad2Deg(b));
    c = round(rad2Deg(c));
  
    angle[2].a = a;
    angle[2].b = b;
    angle[2].c = c;
    angle[2].d = d;
  
    a -= angle[1].a;  // Shoulder
    b -= angle[1].b;  // Elbow
    c -= angle[1].c;  // Wrist
    
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(1500);         // Wrist
    stepper4.setAcceleration(1200);
    stepper3.setMaxSpeed(1500);         // Elbow
    stepper3.setAcceleration(1000);
    stepper2.setMaxSpeed(1500);         // Shoulder
    stepper2.setAcceleration(1000);
  
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);
  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    //stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      //stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         ){
        
        //stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }
  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }  
  
      // Open gripper
      
      servoPos = 1;
    
      for(int i=55 ; i>servoPos; i--){
        myservo.write(i);
        delay(5);
      }                
  
    delay(3000);
    
  
  }










  
//    // --------------------------------------
//    // -------------- Sequence3 -------------
//    // --------------------------------------
//
//    Serial.println("Sequence3");
//        
//    dx = 22;
//    dy = 0.0;
//    dz = -14.58;
//    dy2 = 10;     // Base
//  
//    // d = angle base
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[2].a = a;
//    angle[2].b = b;
//    angle[2].c = c;
//    angle[2].d = d;  
//  
//    a -= angle[1].a;
//    b -= angle[1].b;
//    c -= angle[1].c;
//      
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//  
//    stepper4.moveTo(-targetPosWrist);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(targetPosElbow);
//    stepper1.moveTo(-targetPosBase);
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      stepper1.run();   // Base
//  
//      if(stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         && stepper4.distanceToGo()==0
//         && stepper1.distanceToGo()==0       
//         ){
//        
//        stepper1.setCurrentPosition(0);    
//        stepper2.setCurrentPosition(0);
//        stepper3.setCurrentPosition(0);
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//  
//    delay(1000);
//    
//    // Open gripper
//    
//    servoPos = 1;
//  
//    for(int i=55 ; i>servoPos; i--){
//      myservo.write(i);
//      delay(5);
//    }                
//
//    delay(1000);
//    
//    // --------------------------------------
//    // -------------- Sequence4 -------------
//    // --------------------------------------
//
//    Serial.println("Sequence4");
//    
//    dx = 31;
//    dy = 0.0;
//    dz = 0;
//    dy2 = 0;
//  
//    // d = angle base
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[1].a = a;
//    angle[1].b = b;
//    angle[1].c = c;
//    angle[1].d = d;
//  
//    a -= angle[0].a;
//    b -= angle[0].b;
//    c -= angle[0].c;
//    
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//  
//    stepper4.moveTo(-targetPosWrist);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(targetPosElbow);
//    stepper1.moveTo(targetPosBase);
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      //stepper1.run();   // Base
//  
//      if(stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         && stepper4.distanceToGo()==0
//         ){
//        
//        //stepper1.setCurrentPosition(0);    
//        stepper2.setCurrentPosition(0);
//        stepper3.setCurrentPosition(0);
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//
//    delay(1000);
//    
//    // --------------------------------------
//    // -------------- Sequence5 -------------
//    // --------------------------------------
//
//    Serial.println("Sequence5");
//        
//    dx = 22;
//    dy = 0.0;
//    dz = -14.58;
//    dy2 = 0;
//  
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//    // d = angle base
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[0].a = a;
//    angle[0].b = b;
//    angle[0].c = c;
//    angle[0].d = d;
//          
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = 0;
//    targetPosShoulder = 0;
//    targetPosElbow = 0;
//    targetPosWrist = 0;
//        
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = ((180-b)*1200/90);
//    targetPosWrist = ((c-180)*1200/90);
//    
//    stepper4.moveTo(-targetPosWrist);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(-targetPosElbow);
//    stepper1.moveTo(targetPosBase);
//  
//    // -------- First wrist ---------
//    
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){      
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;     
//      }
//    }
//  
//    // -------- Shoulder + Elbow ---------
//    
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      //stepper1.run();   // Base
//  
//      if(stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         && stepper4.distanceToGo()==0
//         ){
//        
//        //stepper1.setCurrentPosition(0);    
//        stepper2.setCurrentPosition(0);
//        stepper3.setCurrentPosition(0);
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;     
//      }
//    }
//  
//    delay(1000);
//  
//    // Close gripper  
//    servoPos = 55;
//  
//    for(int i=0 ; i<servoPos; i++){
//      myservo.write(i);
//      delay(5);
//    }                
//  
//    delay(1000);
//  
//    // --------------------------------------
//    // -------------- Sequence6-------------
//    // --------------------------------------
//
//    Serial.println("Sequence6");
//        
//    dx = 24;
//    dy = 0.0;
//    dz = 13;
//    dy2 = 10;
//  
//    // d = angle base
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[1].a = a;
//    angle[1].b = b;
//    angle[1].c = c;
//    angle[1].d = d;
//  
//    a -= angle[0].a;
//    b -= angle[0].b;
//    c -= angle[0].c;
//    
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//  
//    stepper4.moveTo(-targetPosWrist);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(targetPosElbow);
//    stepper1.moveTo(targetPosBase);
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      //stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      //stepper1.run();   // Base
//  
//      if(stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         //&& stepper4.distanceToGo()==0
//         ){
//        
//        //stepper1.setCurrentPosition(0);    
//        stepper2.setCurrentPosition(0);
//        stepper3.setCurrentPosition(0);
//        //stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//
//
//    // --------------------------------------
//    // -------------- Sequence7 -------------
//    // --------------------------------------
//
//    Serial.println("Sequence7");
//        
//    dx = 22;
//    dy = 0.0;
//    dz = -14.58;
//    dy2 = 10;     // Base
//  
//    // d = angle base
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[2].a = a;
//    angle[2].b = b;
//    angle[2].c = c;
//    angle[2].d = d;  
//  
//    a -= angle[1].a;
//    b -= angle[1].b;
//    c -= angle[1].c;
//      
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//  
//    stepper4.moveTo(-targetPosWrist);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(targetPosElbow);
//    stepper1.moveTo(targetPosBase);
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      stepper1.run();   // Base
//  
//      if(stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         && stepper4.distanceToGo()==0
//         && stepper1.distanceToGo()==0       
//         ){
//        
//        stepper1.setCurrentPosition(0);    
//        stepper2.setCurrentPosition(0);
//        stepper3.setCurrentPosition(0);
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//  
//    delay(1000);
//    
//    // Open gripper
//    
//    servoPos = 1;
//  
//    for(int i=55 ; i>servoPos; i--){
//      myservo.write(i);
//      delay(5);
//    }    
//
//    delay(1000);
//    
    // --------------------------------------
    // -------------- Sequence8 -------------
    // --------------------------------------

//    Serial.println("Sequence8");
//    
//    dx = 31;
//    dy = 0.0;
//    dz = 0;
//    dy2 = 0;
//  
//    // d = angle base
//    // a = angle shoulder
//    // b = angle elbow
//    // c = angle wrist
//  
//    inverseKinematics(dx,dy,dz);
//  
//    // Initialize angles @90 degrees = postion 0
//    d = round(rad2Deg(atan2(dx,dy2)));
//    a = round(rad2Deg(a));
//    b = round(rad2Deg(b));
//    c = round(rad2Deg(c));
//  
//    angle[1].a = a;
//    angle[1].b = b;
//    angle[1].c = c;
//    angle[1].d = d;
//  
//    a += angle[0].a;  // Shoulder
//    b -= angle[0].b;  // Elbow
//    c -= angle[0].c;  // Wrist
//    
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(1500);         // Wrist
//    stepper4.setAcceleration(1200);
//    stepper3.setMaxSpeed(1500);         // Elbow
//    stepper3.setAcceleration(1000);
//    stepper2.setMaxSpeed(1500);         // Shoulder
//    stepper2.setAcceleration(1000);
//  
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//  
//    stepper4.moveTo(targetPosWrist);
//    //stepper2.moveTo(-targetPosShoulder);
//    //stepper3.moveTo(targetPosElbow);
//    //stepper1.moveTo(targetPosBase);
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      //stepper2.run();     
//      //stepper3.run();    
//      //stepper1.run();   // Base
//  
//      if(//stepper2.distanceToGo()==0
//         //&& stepper3.distanceToGo()==0 
//         stepper4.distanceToGo()==0
//         ){
//        
//        //stepper1.setCurrentPosition(0);    
//        //stepper2.setCurrentPosition(0);
//        //stepper3.setCurrentPosition(0);
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
//    }
//  
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//  
//      if(stepper4.distanceToGo()==0){
//        stepper4.setCurrentPosition(0);
//        moveStepper = 0;            
//      }
    //}

  //}
         
}

void loop() {
 
  if(error == 1) //skip loop if no controller found
    return; 
    
  // ------------------------------
  // ----------- Limits -----------
  // ------------------------------

  if(!digitalRead(pinSwich)){
    //Serial.println("pinSwich ON");
    pinSwichOn = 1;
    tone(buzzer, 1950); 
  }else{
    pinSwichOn = 0;
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

  if((ps2x.Analog(PSS_RX) == 255)){  
    
    if(pinSwichOn){
     baseStepMove1 = 0;     
    }else{
      baseStepMove1 = 1;     
      baseStepMove2 = 1;           
    }
    
    if(!pinSwichOn || !baseStepMove2){      
      basePos = basePos+1;
    }    
  }

  if((ps2x.Analog(PSS_RX) == 0)){

    if(pinSwichOn){
      baseStepMove2 = 0;     
    }else{
      baseStepMove2 = 1;     
      baseStepMove1 = 1;           
    }

    if(!pinSwichOn || !baseStepMove1){            
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
  
  if((ps2x.Analog(PSS_LY) == 255)){
    if(pinSwichOn){
      wristStepMove1 = 0;     
    }else{
      wristStepMove1 = 1;     
      wristStepMove2 = 1;           
    }
    
    if(!pinSwichOn || !wristStepMove2){      
      wristPos = wristPos+1;          
    }    
  }

  if((ps2x.Analog(PSS_LY) == 0)){
    if(pinSwichOn){
      wristStepMove2 = 0;     
    }else{
      wristStepMove2 = 1;     
      wristStepMove1 = 1;           
    }
    
    if(!pinSwichOn || !wristStepMove1){            
      wristPos = wristPos-1;          
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
  
  if((ps2x.Analog(PSS_LX) == 255)){
    if(pinSwichOn){
      elbowStepMove1 = 0;     
    }else{
      elbowStepMove1 = 1;     
      elbowStepMove2 = 1;           
    }
    
    if(!pinSwichOn || !elbowStepMove2){      
      elbowPos = elbowPos-1;          
    }    
  }                      

  if((ps2x.Analog(PSS_LX) == 0)){
    if(pinSwichOn){
      elbowStepMove2 = 0;     
    }else{
      elbowStepMove2 = 1;     
      elbowStepMove1 = 1;           
    }
    
    if(!pinSwichOn || !elbowStepMove1){            
      elbowPos = elbowPos+1;          
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
  
  if((ps2x.Analog(PSS_RY) == 255)){
    if(pinSwichOn){
      shoulerStepMove1 = 0;     
    }else{
      shoulerStepMove1 = 1;     
      shoulerStepMove2 = 1;           
    }
    
    if(!pinSwichOn || !shoulerStepMove2){      
      shoulderPos = shoulderPos+1;          
    }    
  }                      

  if((ps2x.Analog(PSS_RY) == 0)){
    if(pinSwichOn){
      shoulerStepMove2 = 0;     
    }else{
      shoulerStepMove2 = 1;     
      shoulerStepMove1 = 1;           
    }
    
    if(!pinSwichOn || !shoulerStepMove1){            
      shoulderPos = shoulderPos-1;          
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

      Serial.print("pinSwichOn: ");                  
      Serial.println(pinSwichOn,DEC);                  
      Serial.print("baseStepMove1: ");                  
      Serial.println(baseStepMove1,DEC);      
      Serial.print("pinSwichOn: ");                  
      Serial.println(pinSwichOn,DEC);                  
      Serial.print("baseStepMove2: ");                  
      Serial.println(baseStepMove2,DEC);                  
      Serial.println(basePos,DEC);                  
                  
      Serial.println(basePos,DEC);                  

//    for(int i=1; i<posIndex+1 ; i++){
//
//      Serial.println(motor[i].number,DEC);
//      Serial.println(motor[i].speed,DEC);
//      Serial.println(motor[i].type,DEC);
//      Serial.println(motor[i].steps,DEC);
//      Serial.println(motor[i].dir,DEC);
//      Serial.println("-------------------");
//    }
//
    DisplayData = 0;
  }


//----------------------------------------   

  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
   
}

// -------------------------
// ------ initMotors -------
// -------------------------

void initMotors(){

  // --------- Homing Base ----------

//homingBase = 1;
//homingElbow = 1;
//homingShoulder = 1;
//homingWrist = 1;

  if(!homingBase){

    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(1000);
  
    while(digitalRead(pinSwich)){
        stepper1.moveTo(pos);
        pos--;
        stepper1.run();
        delay(2);
    }

      stepper1.setCurrentPosition(0); 
      stepper1.setMaxSpeed(1000);
      stepper1.setAcceleration(500);
      stepper1.moveTo(endPositionBase);
      stepper1.runToPosition();
      stepper1.setCurrentPosition(0); 
      homingBase = 1;
      pos = 0;
  }

  // --------- Homing Wrist ----------
  
  if(homingBase && !homingWrist){

    stepper4.setMaxSpeed(1500);
    stepper4.setAcceleration(1200);
  
    while(digitalRead(pinSwich)){
        stepper4.moveTo(pos);
        pos--;
        stepper4.run();
        delay(1);
    }
  
    stepper4.setCurrentPosition(0); 
    stepper4.setMaxSpeed(1500);
    stepper4.setAcceleration(1200);
    stepper4.moveTo(endPositionWrist);
    stepper4.runToPosition();
    stepper4.setCurrentPosition(0); 
    homingWrist = 1;
    pos = 0;
  } 


  // --------- Homing Elbow ----------
  
  if(homingBase && homingWrist && !homingElbow){

    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(1000);
  
    while(digitalRead(pinSwich)){
        stepper3.moveTo(pos);
        pos++;
        stepper3.run();
        delay(1);
    }
    
    stepper3.setCurrentPosition(0); 
    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(500);
    stepper3.moveTo(-endPositionElbow);
    stepper3.runToPosition();
    stepper3.setCurrentPosition(0); 
    homingElbow = 1;
    pos = 0;
  }

  // --------- Homing Shoulder ----------
  
  if(homingBase && homingWrist && homingElbow && !homingShoulder){

    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(1000);
  
    while(digitalRead(pinSwich)){
        stepper2.moveTo(pos);
        pos--;
        stepper2.run();
        delay(1);
    }

    stepper2.setCurrentPosition(0); 
    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(500);
    stepper2.moveTo(endPositionShoulder);
    stepper2.runToPosition();
    stepper2.setCurrentPosition(0); 
    homingShoulder = 1;
    pos = 0;
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

// ------------------------------------------------
// -- Inverse Kinematics
// -- From https://www.robotshop.com
// Inparameters EE x,y,z coordinates
// Return angles for base,shoulder, elbow and wrist
// ------------------------------------------------

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
