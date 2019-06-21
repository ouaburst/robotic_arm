#include <PS2X_lib.h>  //for v1.6

#include <Servo.h>
#include <PinChangeInterrupt.h>


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

Servo myservo;
Servo myservo2;
int pos = 90;
int pos2 = 90;
int stopBasePos1 = 0;
int stopBasePos2 = 0;

// ------ stepper motor#1 -------
# define DIR_PIN_M1 6
# define STEP_PIN_M1 7
int pos1M1 = 0;
int pos2M1 = 0;
int pssLYDownM1 = 0;
int pssLYUpM1 = 0;

// ------ stepper motor#2 -------
# define DIR_PIN_M2 8
# define STEP_PIN_M2 9
int pos1M2 = 0;
int pos2M2 = 0;
int pssLYDownM2 = 0;
int pssLYUpM2 = 0;

// ------ stepper motor#3 -------
# define DIR_PIN_M3 4
# define STEP_PIN_M3 5
int pos1M3 = 0;
int pos2M3 = 0;
int pssLYDownM3 = 0;
int pssLYUpM3 = 0;

// ------ stepper motor#4 -------
# define DIR_PIN_M4 2
# define STEP_PIN_M4 3
int pos1M4 = 0;
int pos2M4 = 0;
int pssLYDownM4 = 0;
int pssLYUpM4 = 0;

# define SERVO_PIN 44
# define SERVO_PIN2 45
# define MIN_GRIPPER_POS 0
# define MAX_GRIPPER_POS 145
# define MIN_WRSIT_POS 0
# define MAX_WRIST_POS 170

void setup(){

// ------ Pin interrupt -------

 pinMode(IN_INTERRUPT_42, INPUT_PULLUP); 
 pinMode(IN_INTERRUPT_43, INPUT_PULLUP); 

 attachPCINT(digitalPinToPCINT(IN_INTERRUPT_42), stopMoveBasePos1, CHANGE);
 attachPCINT(digitalPinToPCINT(IN_INTERRUPT_43), stopMoveBasePos2, CHANGE);
// ----------------------------

// ------ servo motor -------
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo2.attach(SERVO_PIN2);
// -------------------------

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
  
  Serial.begin(9600);

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
}

void stopMoveBasePos1(void) {
  stopBasePos1 = 1;
}

void stopMoveBasePos2(void) {
  stopBasePos2 = 1;
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  
  if(error == 1) //skip loop if no controller found
    return; 
     
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");     

    // ------------- Init stepper motor ----------

    //rotate(100, 0.10, 1);

    // ------------- Servo1 gripper ---------------
    
    if(ps2x.Button(PSB_PAD_RIGHT)){               
      if(pos > MIN_GRIPPER_POS){
        pos -= 1;
        myservo.write(pos);
        delay(5);  
      }
    }
    if(ps2x.Button(PSB_PAD_LEFT)){     
      if(pos < MAX_GRIPPER_POS){
        pos += 1;
        myservo.write(pos);
        delay(5); 
      }
    }

    // ------------- Servo2 wrist rotation ---------------

    if(ps2x.Button(PSB_PAD_UP)) {      
      if(pos2 > MIN_WRSIT_POS){
        pos2 -= 1;
        myservo2.write(pos2);
        delay(5);  
      }
    }
    
    if(ps2x.Button(PSB_PAD_DOWN)){
      if(pos2 < MAX_WRIST_POS){
        pos2 += 1;
        myservo2.write(pos2);
        delay(5); 
      }
    }   

    // ------------------ End servo ----------------------

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

// ------------------------------------------

//    if((ps2x.Analog(PSS_LY) == 0))
//      Serial.println("Stick Values PSS_LY: 0");
//    if((ps2x.Analog(PSS_LY) == 255))
//      Serial.println("Stick Values PSS_LY: 255");
//
//    if((ps2x.Analog(PSS_LX) == 0))
//      Serial.println("Stick Values PSS_LX: 0");
//    if((ps2x.Analog(PSS_LX) == 255))
//      Serial.println("Stick Values PSS_LX: 255");
//      
//    if((ps2x.Analog(PSS_RY) == 0))
//      Serial.println("Stick Values PSS_RY: 0");
//    if((ps2x.Analog(PSS_RY) == 255))
//      Serial.println("Stick Values PSS_RY: 255");
//
//    if((ps2x.Analog(PSS_RX) == 0))
//      Serial.println("Stick Values PSS_RX: 0");
//    if((ps2x.Analog(PSS_RX) == 255))
//      Serial.println("Stick Values PSS_RX: 255");

// ------------------------------------------



// ------ stepper motor#1 -------

    if((ps2x.Analog(PSS_RX) == 0)){
      if(pssLYUpM2 == 0)
        pssLYUpM2 = 2;        
      if(pos2M2 > 0 && pssLYDownM2 == 2){
        pos2M2 = 0;
        pssLYDownM2 = 0;
      }                
      pos2M2=2;
      rotate(pos2M2, 0.10, 1);
      //Serial.println(pos2M2, DEC);
    }  
    
    if((ps2x.Analog(PSS_RX) == 255)){
      if(pssLYDownM2 == 0)
        pssLYDownM2 = 2;
      if(pos2M2 < 0 && pssLYUpM2 == 2){
        pos2M2 = 0;
        pssLYUpM2 = 0;
      }                   
      pos2M2=-2;
      rotate(pos2M2, 0.10, 1);
      //Serial.println(pos2M2, DEC);
    }     

// ------ stepper motor#2 -------

    if((ps2x.Analog(PSS_RY) == 0)){
      if(pssLYUpM1 == 0)
        pssLYUpM1 = 1;        
      if(pos1M1 > 0 && pssLYDownM1 == 1){
        pos1M1 = 0;
        pssLYDownM1 = 0;
      }                
      pos1M1=1;
      rotate(pos1M1, 0.20, 2);
      //Serial.println(pos1M1, DEC);
    }  
    
    if((ps2x.Analog(PSS_RY) == 255)){
      if(pssLYDownM1 == 0)
        pssLYDownM1 = 1;
      if(pos2M1 < 0 && pssLYUpM1 == 1){
        pos2M1 = 0;
        pssLYUpM1 = 0;
      }                   
      pos2M1=-1;
      rotate(pos2M1, 0.20, 2);
      //Serial.println(pos2M1, DEC);
    }    

// ------ stepper motor#3 -------

    if((ps2x.Analog(PSS_LX) == 0)){
      if(pssLYUpM3 == 0)
        pssLYUpM3 = 1;        
      if(pos1M3 > 0 && pssLYDownM3 == 1){
        pos1M3 = 0;
        pssLYDownM3 = 0;
      }                
      pos1M3=1;
      rotate(pos1M3, 0.20, 3);
      //Serial.println(pos1M3, DEC);
    }  
    
    if((ps2x.Analog(PSS_LX) == 255)){
      if(pssLYDownM3 == 0)
        pssLYDownM3 = 1;
      if(pos2M3 < 0 && pssLYUpM3 == 1){
        pos2M3 = 0;
        pssLYUpM3 = 0;
      }                   
      pos2M3=-1;
      rotate(pos2M3, 0.20, 3);
      //Serial.println(pos2M3, DEC);
    }      

// ------ stepper motor#4 -------

    if((ps2x.Analog(PSS_LY) == 0)){
      if(pssLYUpM4 == 0)
        pssLYUpM4 = 1;        
      if(pos1M4 > 0 && pssLYDownM4 == 1){
        pos1M4 = 0;
        pssLYDownM4 = 0;
      }                
      pos1M4=1;
      rotate(pos1M4, 0.20, 4);
      //Serial.println(pos1M4, DEC);
    }  
    
    if((ps2x.Analog(PSS_LY) == 255)){
      if(pssLYDownM4 == 0)
        pssLYDownM4 = 1;
      if(pos2M4 < 0 && pssLYUpM4 == 1){
        pos2M4 = 0;
        pssLYUpM4 = 0;
      }                   
      pos2M4=-1;
      rotate(pos2M4, 0.20, 4);
      //Serial.println(pos2M4, DEC);
    }
    
}

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
