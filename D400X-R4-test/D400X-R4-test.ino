#include <AccelStepper.h>
#include <Servo.h>

// --- Pin Definitions ---
#define pinSwich 43        // Shared input pin for all limit switches
#define buzzer 31          // Buzzer pin for auditory feedback
#define SERVO_PIN 44       // Main gripper servo pin
#define pinZHat 53         // External trigger button (e.g., for homing sequence)

// --- Gripper Range ---
#define MIN_GRIPPER_POS 0
#define MAX_GRIPPER_POS 145

// --- Potentiometer Pins (Analog Inputs) ---
const int potSpeedPin0 = A0;  // Shoulder
const int potSpeedPin1 = A1;  // Base
const int potSpeedPin2 = A2;  // Elbow
const int potSpeedPin3 = A3;  // Wrist
const int potSpeedPin4 = A4;  // Gripper

// --- Thresholds ---
const int deadZoneMin = 420;
const int deadZoneMax = 570;
const int minEffectiveSpeed = 100;
const int fixedAcceleration = 300;

// --- Stepper Motors ---
AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);  // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);  // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);  // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);  // Wrist

// --- Servo ---
Servo myservo;
Servo myservo2;  // Optional second gripper servo
int servoPos = 0;
int servoPos2 = 0;
bool gripperActive = false;

// --- Limit Switch Logic ---
int pinSwichOn = 0;
bool baseStepMove1 = 1, baseStepMove2 = 1;
bool shoulderStepMove1 = 1, shoulderStepMove2 = 1;
bool elbowStepMove1 = 1, elbowStepMove2 = 1;
bool wristStepMove1 = 1, wristStepMove2 = 1;

// --- Buzzer Control ---
static unsigned long lastBeepTime = 0;
static int beepStep = 0;

// --- Homing State Flags ---
bool homingBase = 0;
bool homingWrist = 0;
bool homingElbow = 0;
bool homingShoulder = 0;
bool homingDone = 0;

// --- Position Control ---
int pos = 0;

int endPositionBase;
int endPositionWrist;
int endPositionElbow;
int endPositionShoulder;

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

int targetPosBase;
int targetPosShoulder;
int targetPosElbow;
int targetPosWrist;

int moveStepper;

// Global flag to track whether initialization has been done
bool motorsInitialized = false;


void setup() {
  Serial.begin(9600);

  // Initialize all steppers
  setupStepper(stepper1);
  setupStepper(stepper2);
  setupStepper(stepper3);
  setupStepper(stepper4);

  // Servo setup
  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo2.attach(45);  // Optional second servo
  myservo.write(servoPos);

  // Input and output pins
  pinMode(pinSwich, INPUT_PULLUP);
  pinMode(pinZHat, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);

  // For homing

  endPositionBase = 1200;
  endPositionWrist = 1200;
  endPositionElbow = 1200;
  endPositionShoulder = 1200;
  
}

void setupStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(fixedAcceleration);
}

void loop() {

  // --- ZHat edge detection ---
  static bool lastZHatState = HIGH;
  bool currentZHatState = digitalRead(pinZHat);

  if (lastZHatState == HIGH && currentZHatState == LOW) {
    Serial.println("Z Hat pressed");

    if (!motorsInitialized) {
      Serial.println("Running motor initialization...");
      initMotors();            // Run only once
      motorsInitialized = true;
    }

    Serial.println("Starting sequence...");
    startSequence(1);          // Always run this
  }

  lastZHatState = currentZHatState;
  
  // --- Read potentiometer inputs ---
  int rawSpeed0 = analogRead(potSpeedPin0);  // Shoulder
  int rawSpeed1 = analogRead(potSpeedPin1);  // Base
  int rawSpeed2 = analogRead(potSpeedPin2);  // Elbow
  int rawSpeed3 = analogRead(potSpeedPin3);  // Wrist
  int rawSpeed4 = analogRead(potSpeedPin4);  // Gripper

  // --- Read limit switch ---
  pinSwichOn = !digitalRead(pinSwich);  // LOW = pressed

  // --- Buzzer logic ---
  if (pinSwichOn) {
    unsigned long now = millis();
    if (now - lastBeepTime > 120) {
      switch (beepStep) {
        case 0: tone(buzzer, 880); break;
        case 1: tone(buzzer, 988); break;
        case 2: tone(buzzer, 1047); break;
      }
      beepStep = (beepStep + 1) % 3;
      lastBeepTime = now;
    }
  } else {
    noTone(buzzer);
    beepStep = 0;
  }

  // --- Gripper control overrides stepper ---
  if (rawSpeed4 > deadZoneMax || rawSpeed4 < deadZoneMin) {
    gripperActive = true;

    if (rawSpeed4 > deadZoneMax && servoPos < MAX_GRIPPER_POS) {
      servoPos++;
      myservo.write(servoPos);
      delay(5);
    }

    if (rawSpeed4 < deadZoneMin && servoPos > MIN_GRIPPER_POS) {
      servoPos--;
      myservo.write(servoPos);
      delay(5);
    }

    return;
  } else {
    gripperActive = false;
  }

  // --- Stepper control for each joint ---
  handleJointControl(stepper1, rawSpeed1, baseStepMove1, baseStepMove2);     // Base
  handleJointControl(stepper2, rawSpeed0, shoulderStepMove1, shoulderStepMove2); // Shoulder
  handleJointControl(stepper3, rawSpeed2, elbowStepMove1, elbowStepMove2);   // Elbow
  handleJointControl(stepper4, rawSpeed3, wristStepMove1, wristStepMove2);   // Wrist
}

void handleJointControl(AccelStepper &stepper, int potValue, bool &moveDir1, bool &moveDir2) {
  long speed = 0;

  if (potValue > deadZoneMax) {
    if (pinSwichOn) moveDir1 = false;
    else { moveDir1 = true; moveDir2 = true; }

    if (!pinSwichOn || !moveDir2)
      speed = map(potValue, deadZoneMax, 1023, minEffectiveSpeed, 1000);

  } else if (potValue < deadZoneMin) {
    if (pinSwichOn) moveDir2 = false;
    else { moveDir2 = true; moveDir1 = true; }

    if (!pinSwichOn || !moveDir1)
      speed = map(potValue, deadZoneMin, 0, -minEffectiveSpeed, -1000);

  } else {
    speed = 0;
  }

  stepper.setSpeed(speed);
  stepper.run();
}


// --- Homing Sequence ---
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
