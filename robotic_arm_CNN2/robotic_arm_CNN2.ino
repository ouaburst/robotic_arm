/*
 * MIT License
 * Copyright (c) 2020 Oualid Burstrom
 */

/*
 * Headers
 */
 
#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
/* 
 * Arduino code to send and receive I2C data
 * Tested on Adafruit Feather M0+ Express and Raspberry Pi Model 4
 * 
 * SDA <--> SDA
 * SCL <--> SCL
 * GND <--> GND
 * 
 * Sets built-in LED (1 = on, 0 = off) on Feather when requested 
 * and responds with data received
 */

#define SLAVE_ADDRESS 0x04       // I2C address for Arduino
int i2cData;                     // the I2C data received
int sequenceState;               // 1 = sequence is running, 
                                 // 0 = sequence is not running

/******************************************************************
 * Set pins connected to PS2 controller:
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
#define pressures   false
#define rumble      false

/*
 * Right now, the library does NOT support hot pluggable controllers, meaning 
 * you must always either restart your Arduino after you connect the controller, 
 * or call config_gamepad(pins) again after connecting the controller.
 */

/*
 * Create PS2 Controller Class
 */

PS2X ps2x; 

int error = 0;
byte type = 0;
byte vibrate = 0;

/*
 * Initiate stepper mortors using AccelStepper
 */

AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);    // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);    // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);    // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);    // Wrist

/*
 * Global variables for stepper motors pos
 */

int basePos;
int wristPos;
int elbowPos;
int shoulderPos;

/*
 * Inverse kinematics parameters
 */

/*
 * Inverse Kinematics
 */

#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

/*
 * Robot geometry in cm used in Inverse Kinematics
 */

#define h 23.5    // Height
#define L 5.5     // Shoulder
#define M 11      // Elbow
#define N 14.5    // Wrist

/* IK angles
 * d = angle base
 * a = angle shoulder
 * b = angle elbow
 * c = angle wrist
 */
 
float a, b, c, d;   // Joint variables that will be calculated

/*
 * End Effector position
 */
 
float dx, dy, dz, dy2;

/*
 * An array of a struct to store the angles in IK. 
 * a = angle shoulder
 * b = angle elbow
 * c = angle wrist
 * d = angle base
 * 
 */
 
typedef struct {
   float a;
   float b;
   float c;
   float d;
} Angles[10];

Angles angle;

/*
 * Use GeoGebra to calculate the angles
 * instead of Inverse kinematics function
 * a = angle shoulder
 * b = angle elbow
 * c = angle wrist
 * d = angle base
 * 
 */

float angleaData[] = {-52,-52,58.34,-52 ,-52,-52,-52,58.34,-52,-52,0};
float anglebData[] = {160.62,271.46,162.54, 160.62 ,271.46,160.62,271.46,162.54,160.62,271.46,180};
float anglecData[] = {250.08,139.24,139.24, 250.08,139.24,250.08,139.24,139.24,250.08,139.24,180};
float angledData[] = {0,0,0,-70,-70,-70,-70,0,0,0,0};

//float angleaData[] = {-52,-52,0};
//float anglebData[] = {160.62,271.46,180};
//float anglecData[] = {250.08,139.24,180};
//float angledData[] = {0,0,0};

/*
 * Servos
 */

# define SERVO_PIN 44
# define SERVO_PIN2 45

# define MIN_GRIPPER_POS 0
# define MAX_GRIPPER_POS 145
# define MIN_WRSIT_POS 0    // Not used!
# define MAX_WRIST_POS 170  // Not used!
#define SERVO_MOTOR 2
#define DIR1 1      // servo pos ++
#define DIR2 2      // servo pos --, not used

Servo myservo;
Servo myservo2;

int servoPos;
int servoPos2;
int gripperServoRotateDir1;
int gripperServoRotateDir2;
int wristServoRotateDir1;
int wristServoRotateDir2;
int servoSteps;

/*
 * Global variables
 */
 
#define HOMING_ACTION 1   
#define PLAY_ACTION 2   
#define STEP_MOTOR 1

/*
 * This struct is used to store the positions of the
 * stepper motors and servo motors.
 * It is used when creating sequences.
 */

typedef struct {
   int number;
   int steps;
   float speed;
   int type;            // type servo, stepper
   int dir=0;           // dir. Used only for servo,
} Motors[250];          // dir=DIR1 add (++), dir=DIR2 substract (--)

Motors motor;

/*
 * Serial comm
 * Struct to store serial information from Arduino.
 */
 
typedef struct {
  char bufferArray[22];
} SerialBuffers[50];

SerialBuffers serialBuffer;
String serialData;
char char_array[256];
char *token;

/*
 * Switches
 * Are used by stepper motors: 
 * For homing
 * To prevent exceed the limits 
 * 
 * All switches are connected to pin 43 
 */
 
#define pinSwich 43  // switsh

/*
 * State of pin 43 
 */
 
int pinSwichOn;

/*
 * Define stepper motor speed 
 */
 
# define STEP_MOTOR_SPEED_BASE 400
# define STEP_MOTOR_SPEED_SHOULDER 400
# define STEP_MOTOR_SPEED_ELBOW 400
# define STEP_MOTOR_SPEED_WRIST 400

/*
 * Misc 
 */
 
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

/*
 * Setup function used to initialize things...
 */
 
void setup(){  
  
  /*
   *  PS2 controller 
   */
   
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

  /*
   *  IC2 
   */

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  i2cData = 0;
  sequenceState = 0;

  /*
   * Serial
   */
  
  Serial.begin(19200);
  
  /*
   * Servos
   */
  
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);  // Not used!
  myservo.attach(SERVO_PIN);
  //myservo2.attach(SERVO_PIN2);

  servoPos = MAX_GRIPPER_POS;
  servoPos2 = 90;
  gripperServoRotateDir1 = 0;
  gripperServoRotateDir2 = 0;
  wristServoRotateDir1 = 0;
  wristServoRotateDir2 = 0;

  /*
   * Stepper motors positions
   */
   
  basePos = 0;
  wristPos = 0;
  elbowPos = 0;
  posIndex = 0;
  
  /*
   * Switch pin
   */
   
  pinMode(pinSwich, INPUT_PULLUP); 

  /*
   * Misc
   */
  
  buzzer = 31; 
  pinMode(buzzer, OUTPUT); 
  noTone(buzzer); 
  homingDone = 0;
  
  /*
   * Limit values
   */
   
  pinSwichOn = 0;
  baseStepMove1 = 1;
  baseStepMove2 = 1;
  shoulerStepMove1 = 1;
  shoulerStepMove2 = 1;
  elbowStepMove1 = 1;
  elbowStepMove2 = 1;
  wristStepMove1 = 1;
  wristStepMove2 = 1;

  /*
   * For homing
   */

  endPositionBase = 1200;
  endPositionWrist = 1200;
  endPositionElbow = 1200;
  endPositionShoulder = 1200;

  /*
   * Stepper motor directions
   */

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

  /*
   * Stepper motor directions
   */
  
  targetPosBase = 0;
  targetPosShoulder = 0;
  targetPosElbow = 0;
  targetPosWrist = 0;
  

  /*
   * Homing
   */

  //homingDone = 1;
  
  while(!homingDone){
    initMotors();
  }

}

void loop() {
 
  if(error == 1) //skip loop if no controller found
    return; 

  /*
   * Limits
   */
   
  if(!digitalRead(pinSwich)){
    //Serial.println("pinSwich ON");
    pinSwichOn = 1;
    tone(buzzer, 1950); 
  }else{
    pinSwichOn = 0;
    noTone(buzzer); 
  }

  /*
   * Servo1
   */
  
  if(ps2x.Button(PSB_PAD_RIGHT)){               
    if(servoPos > MIN_GRIPPER_POS){        
      servoPos -= 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;                     
      myservo.write(servoPos);
      delay(5);  
    }
  }
  
  if(ps2x.Button(PSB_PAD_LEFT)){     
    if(servoPos < MAX_GRIPPER_POS){
      servoPos += 1;
      servoSteps++;
      motor[posIndex].steps = servoSteps;     
      myservo.write(servoPos);
      delay(5); 
    }
  }

  /*
   * Move stepper motor#1 = Base
   * with PS2 joystick
   */

  /*
   * PPS_RX
   */

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

  /*
   * Use AccelStepper
   */

  stepper1.moveTo(basePos);
  stepper1.setSpeed(STEP_MOTOR_SPEED_BASE);    
    
  while (stepper1.distanceToGo() !=0) {
    stepper1.runSpeedToPosition();
  }

  /*
   * Move stepper motor#4 = Wrist
   * with PS2 joystick
   */

  /*
   * PSS_LY
   */
      
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

  /*
   * Use AccelStepper
   */
  
  stepper4.moveTo(wristPos);
  stepper4.setSpeed(STEP_MOTOR_SPEED_WRIST);    
  
  while (stepper4.distanceToGo() !=0) {
    stepper4.runSpeedToPosition();
  }

  /*
   * Move stepper motor#3 = Elbow 
   * with PS2 joystick
   */

  /*
   * PSS_LX
   */

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

  /*
   * Use AccelStepper
   */
      
  stepper3.moveTo(elbowPos);
  stepper3.setSpeed(STEP_MOTOR_SPEED_ELBOW);    

  while (stepper3.distanceToGo() !=0) {
    stepper3.runSpeedToPosition();
  }

  /*
   * Move stepper motor#2 = Elbow 
   * with PS2 joystick
   */

  /*
   * PSS_RY
   */

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

  /*
   * Use AccelStepper
   */
      
  stepper2.moveTo(shoulderPos);
  stepper2.setSpeed(STEP_MOTOR_SPEED_SHOULDER);    
  
  while (stepper2.distanceToGo() !=0) {
    stepper2.runSpeedToPosition();
  }

  /*
   * Press button PSB_CIRCLE
   * with PS2 joystick
   * 
   * Execute sequence 1 or 2 depending on 
   * the value sended by Raspberry pi via I2C
   */
      
  if(ps2x.ButtonPressed(PSB_CIRCLE)){

    // 1 = turnLeft
    // 2 = turnRight

    startSequence(1);

//    if(i2cData == 1)
//      startSequence(1);
//
//    if(i2cData == 2)
//      startSequence(2);
  }       

  /*
   * Misc
   */

  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
   
}

  /*
   * initMotors 
   * Homing
   * This is done when the system is started
   */

void initMotors(){

  /*
   * Homing Base
   */

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

  /*
   * Homing Wrist
   */
  
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

  /*
   * Homing Elbow
   */

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

  /*
   * Homing Shoulder
   */

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

  /*
   * Homing Servo
   */
  
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

  /*
   * Handle reception of incoming I2C data
   * Using interrupt routine
   */

void receiveData(int byteCount) {
  while (Wire.available()) {
    i2cData = Wire.read();

    Serial.print("Recived: ");   
    Serial.println(i2cData);       
  }
}

  /*
   * Handle request to send I2C data
   * Used for debugging
   */

void sendData() { 
  //Wire.write(sequenceState);
  Wire.write(0);
}

  /*
   * startSequence
   * Input: int, 1=turn left, 2=turn right
   * Using Inverse Kinematics
   */

void startSequence(int leftRight){

    // a = angle shoulder
    // b = angle elbow
    // c = angle wrist
    // d = angle base

  /**********************************************************************
   * STEP1
   * Down: Shoulder + Elbow + Wrist
   **********************************************************************/

    Serial.println("STEP1");

    // ---- Using Geobebra -----------
    a = round(angleaData[0]);
    b = round(anglebData[0]);
    c = round(anglecData[0]);
    d = round(angledData[0]);
    
    angle[0].a = a;
    angle[0].b = b;
    angle[0].c = c;
    angle[0].d = d;

    /*
     * AccelStepper
     */    

    stepper1.setMaxSpeed(700);         // Base
    stepper1.setAcceleration(500);
    stepper3.setMaxSpeed(700);         // Elbow
    stepper3.setAcceleration(500);
    stepper2.setMaxSpeed(700);         // Shoulder
    stepper2.setAcceleration(500);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(1500);
  
   //move();
    
    /*
     * Convert angle to stepper motor steps
     */            
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = ((180-b)*1200/90);
    targetPosWrist = ((c-180)*1200/90);

    /*
     * Move!
     */
     
    stepper1.moveTo(targetPosBase);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(-targetPosElbow);
    stepper4.moveTo(-targetPosWrist);    
    
    moveStepper = 1;
  
    while(moveStepper){
      stepper1.run();   // Base
      stepper2.run();     
      stepper3.run();    
      stepper4.run();          

      if(stepper1.distanceToGo()==0
         && stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;     
      }
    }
  
    delay(1000);

    /*
     * Close gripper  
     */    

    servoPos = 55;
  
    for(int i=0 ; i<servoPos; i++){
      myservo.write(i);
      delay(5);
    }                
  
    delay(1000);
  
  /**********************************************************************
   * STEP2
   * Up: Shoulder + Elbow + Wrist
   **********************************************************************/

   Serial.println("STEP2");

    // ---- Using Geobebra -----------    
    a = round(angleaData[1]);
    b = round(anglebData[1]);
    c = round(anglecData[1]);
    d = round(angledData[1]);

    angle[1].a = a;
    angle[1].b = b;
    angle[1].c = c;
    angle[1].d = d;

    a -= angle[0].a;    //Shoulder
    b -= angle[0].b;    //Elbow
    c -= angle[0].c;    //Wrist
    d -= angle[0].d;    //Base
    
    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(2500);         // Base
    stepper1.setAcceleration(1500);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(1500);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);
    
     /*
     * Move!
     */
    stepper1.moveTo(targetPosBase);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper4.moveTo(-targetPosWrist);
      
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper1.distanceToGo()==0
         && stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);

  /**********************************************************************
   * STEP3
   * Up: Shoulder + Elbow + Wrist + Base
   **********************************************************************/
    
    Serial.println("STEP3");

    a = round(angleaData[2]);
    b = round(anglebData[2]);
    c = round(anglecData[2]);
    d = round(angledData[2]);
    
    angle[2].a = a;
    angle[2].b = b;
    angle[2].c = c;
    angle[2].d = d;
  
    a -= angle[1].a;
    b -= angle[1].b;
    c -= angle[1].c;
    d -= angle[1].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);
    
  /**********************************************************************
   * STEP4
   * 
   **********************************************************************/
    
    Serial.println("STEP4");

    a = round(angleaData[3]);
    b = round(anglebData[3]);
    c = round(anglecData[3]);
    d = round(angledData[3]);
    
    angle[3].a = a;
    angle[3].b = b;
    angle[3].c = c;
    angle[3].d = d;
  
    a -= angle[2].a;
    b -= angle[2].b;
    c -= angle[2].c;
    d -= angle[2].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);
    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);

    /*
     * open gripper  
     */    
      
      servoPos = 1;
    
      for(int i=55 ; i>servoPos; i--){
        myservo.write(i);
        delay(5);
      }             

    delay(1000);    
    
  /**********************************************************************
   * STEP5
   * 
   **********************************************************************/
    
    Serial.println("STEP5");

    a = round(angleaData[4]);
    b = round(anglebData[4]);
    c = round(anglecData[4]);
    d = round(angledData[4]);
    
    angle[4].a = a;
    angle[4].b = b;
    angle[4].c = c;
    angle[4].d = d;
  
    a -= angle[3].a;
    b -= angle[3].b;
    c -= angle[3].c;
    d -= angle[3].d;

    /*
     * AccelStepper
     */        
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(2500);         // Wrist
//    stepper4.setAcceleration(2000);
//    stepper3.setMaxSpeed(2500);         // Elbow
//    stepper3.setAcceleration(2000);
//    stepper2.setMaxSpeed(2500);         // Shoulder
//    stepper2.setAcceleration(2000);

    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper2.setMaxSpeed(700);         // Shoulder
    stepper2.setAcceleration(500);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper4.setMaxSpeed(1000);         // Wrist
    stepper4.setAcceleration(800);


    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);  

  /**********************************************************************
   * STEP6
   * 
   **********************************************************************/
    
    Serial.println("STEP6");

    a = round(angleaData[5]);
    b = round(anglebData[5]);
    c = round(anglecData[5]);
    d = round(angledData[5]);
    
    angle[5].a = a;
    angle[5].b = b;
    angle[5].c = c;
    angle[5].d = d;
  
    a -= angle[4].a;
    b -= angle[4].b;
    c -= angle[4].c;
    d -= angle[4].d;

    /*
     * AccelStepper
     */        
//    stepper1.setMaxSpeed(700);         // Base
//    stepper1.setAcceleration(500);
//    stepper3.setMaxSpeed(700);         // Elbow
//    stepper3.setAcceleration(500);
//    stepper2.setMaxSpeed(700);         // Shoulder
//    stepper2.setAcceleration(500);
//    stepper4.setMaxSpeed(2500);         // Wrist
//    stepper4.setAcceleration(1500);

    stepper1.setMaxSpeed(700);         // Base
    stepper1.setAcceleration(500);
    stepper3.setMaxSpeed(700);         // Elbow
    stepper3.setAcceleration(500);
    stepper2.setMaxSpeed(700);         // Shoulder
    stepper2.setAcceleration(500);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(1500);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);

    /*
     * Close gripper  
     */    

    servoPos = 55;
  
    for(int i=0 ; i<servoPos; i++){
      myservo.write(i);
      delay(5);
    }                
  
    delay(1000);

    
  /**********************************************************************
   * STEP7
   * 
   **********************************************************************/
    
    Serial.println("STEP7");

    a = round(angleaData[6]);
    b = round(anglebData[6]);
    c = round(anglecData[6]);
    d = round(angledData[6]);
    
    angle[6].a = a;
    angle[6].b = b;
    angle[6].c = c;
    angle[6].d = d;
  
    a -= angle[5].a;
    b -= angle[5].b;
    c -= angle[5].c;
    d -= angle[5].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }

    delay(1000);
    
  /**********************************************************************
   * STEP8
   * 
   **********************************************************************/
    
    Serial.println("STEP8");

    a = round(angleaData[7]);
    b = round(anglebData[7]);
    c = round(anglecData[7]);
    d = round(angledData[7]);
    
    angle[7].a = a;
    angle[7].b = b;
    angle[7].c = c;
    angle[7].d = d;
  
    a -= angle[6].a;
    b -= angle[6].b;
    c -= angle[6].c;
    d -= angle[6].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }    

    delay(1000);
    
  /**********************************************************************
   * STEP9
   * 
   **********************************************************************/
    
    Serial.println("STEP9");

    a = round(angleaData[8]);
    b = round(anglebData[8]);
    c = round(anglecData[8]);
    d = round(angledData[8]);
    
    angle[8].a = a;
    angle[8].b = b;
    angle[8].c = c;
    angle[8].d = d;
  
    a -= angle[7].a;
    b -= angle[7].b;
    c -= angle[7].c;
    d -= angle[7].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }  

    delay(1000);

    /*
     * open gripper  
     */    
      
      servoPos = 1;
    
      for(int i=55 ; i>servoPos; i--){
        myservo.write(i);
        delay(5);
      }             

    delay(1000);    
    
  /**********************************************************************
   * STEP10
   * 
   **********************************************************************/
    
    Serial.println("STEP10");

    a = round(angleaData[9]);
    b = round(anglebData[9]);
    c = round(anglecData[9]);
    d = round(angledData[9]);
    
    angle[9].a = a;
    angle[9].b = b;
    angle[9].c = c;
    angle[9].d = d;
  
    a -= angle[8].a;
    b -= angle[8].b;
    c -= angle[8].c;
    d -= angle[8].d;

    /*
     * AccelStepper
     */        
//    stepper1.setMaxSpeed(1500);         // Base
//    stepper1.setAcceleration(1000);
//    stepper4.setMaxSpeed(2500);         // Wrist
//    stepper4.setAcceleration(2000);
//    stepper3.setMaxSpeed(2500);         // Elbow
//    stepper3.setAcceleration(2000);
//    stepper2.setMaxSpeed(2500);         // Shoulder
//    stepper2.setAcceleration(2000);

    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper2.setMaxSpeed(700);         // Shoulder
    stepper2.setAcceleration(500);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper4.setMaxSpeed(1000);         // Wrist
    stepper4.setAcceleration(800);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }  
    delay(1000);
    
  /**********************************************************************
   * STEP11
   * 
   **********************************************************************/
    
    Serial.println("STEP11");

    a = round(angleaData[10]);
    b = round(anglebData[10]);
    c = round(anglecData[10]);
    d = round(angledData[10]);
    
    angle[10].a = a;
    angle[10].b = b;
    angle[10].c = c;
    angle[10].d = d;
  
    a -= angle[9].a;
    b -= angle[9].b;
    c -= angle[9].c;
    d -= angle[9].d;

    /*
     * AccelStepper
     */        
    stepper1.setMaxSpeed(1500);         // Base
    stepper1.setAcceleration(1000);
    stepper4.setMaxSpeed(2500);         // Wrist
    stepper4.setAcceleration(2000);
    stepper3.setMaxSpeed(2500);         // Elbow
    stepper3.setAcceleration(2000);
    stepper2.setMaxSpeed(2500);         // Shoulder
    stepper2.setAcceleration(2000);

    //move();

    /*
     * Convert angle to stepper motor steps
     */      
    targetPosBase = (d*1200/90);
    targetPosShoulder = (a*1200/90);
    targetPosElbow = (b*1200/90);
    targetPosWrist = (c*1200/90);

    /*
     * Move!
     */  
    stepper4.moveTo(-targetPosWrist);
    stepper2.moveTo(-targetPosShoulder);
    stepper3.moveTo(targetPosElbow);
    stepper1.moveTo(targetPosBase);
  
    moveStepper = 1;
  
    while(moveStepper){
      stepper4.run();    
      stepper2.run();     
      stepper3.run();    
      stepper1.run();   // Base
  
      if(stepper2.distanceToGo()==0
         && stepper3.distanceToGo()==0 
         && stepper4.distanceToGo()==0
         && stepper1.distanceToGo()==0
         ){
        
        stepper1.setCurrentPosition(0);    
        stepper2.setCurrentPosition(0);
        stepper3.setCurrentPosition(0);
        stepper4.setCurrentPosition(0);
        moveStepper = 0;            
      }
    }  
    
        
    delay(1000);
}

//void move(){
//
//    /*
//     * Convert angle to stepper motor steps
//     */      
//    targetPosBase = (d*1200/90);
//    targetPosShoulder = (a*1200/90);
//    targetPosElbow = (b*1200/90);
//    targetPosWrist = (c*1200/90);
//    
//     /*
//     * Move!
//     */
//    stepper1.moveTo(targetPosBase);
//    stepper2.moveTo(-targetPosShoulder);
//    stepper3.moveTo(targetPosElbow);
//    stepper4.moveTo(-targetPosWrist);
//      
//    moveStepper = 1;
//  
//    while(moveStepper){
//      stepper4.run();    
//      stepper2.run();     
//      stepper3.run();    
//      stepper1.run();   // Base
//  
//      if(stepper1.distanceToGo()==0
//         && stepper2.distanceToGo()==0
//         && stepper3.distanceToGo()==0 
//         && stepper4.distanceToGo()==0
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
//}


/*
 * Inverse Kinematics
 * Inparameters EE x,y,z coordinates
 * Return angles for base,shoulder, elbow and wrist
 * For explanation please take a look to the document kinematics_4DOF.pdf
 */

void inverseKinematics(float dx ,float dy ,float dz){

  float R,S,Q,f,g;
  
  R = sqrt(pow(dx,2) + pow(dy,2));  // The radius from the axis of rotation of the base to dx,dy
  S = R - N;                  // 
  Q = sqrt(pow(S, 2) + pow((dz), 2));
  f = atan2(dz, S);
  
  g = acos((pow(L,2)+pow(Q,2)-pow(M,2))/(2*L*Q));
  
  //d = atan2(dx, dy);
  a = f + g;
  b = acos((pow(M, 2) + pow(L, 2) - pow(Q, 2)) / (2 * L * M));
  c = -b - a + 2 * M_PI;
  
}
