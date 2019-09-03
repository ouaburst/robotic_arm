

#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <AccelStepper.h>

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

AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

int basePos;

// ----------------------------

# define SERVO_PIN 44
# define SERVO_PIN2 45

// ------ servos --------

Servo myservo;
Servo myservo2;

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

// --- Misc ---
int buzzer; 
int homingDone;

int homingBase;
int homingShoulder;
int homingWrist;
int homingElbow;

int endPositionBase;

int pos;

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

  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(5000);

  // ------ servo motor -------
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo2.attach(SERVO_PIN2);

  // ------ Step motors positions ------
  basePos = 0;

  // ------ Switch pins ---------
  pinMode(shoulderSwich1, INPUT_PULLUP); 
  pinMode(shoulderSwich2, INPUT_PULLUP); 
  pinMode(elbowSwich1, INPUT_PULLUP); 
  pinMode(elbowSwich2, INPUT_PULLUP); 
  pinMode(baseSwich1, INPUT_PULLUP); 
  pinMode(baseSwich2, INPUT_PULLUP); 
  pinMode(wristSwich1, INPUT_PULLUP); 
  pinMode(wristSwich2, INPUT_PULLUP); 
  
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

  // --------------------------
  endPositionBase = 1200;

  // --------------------------
  
  homingBase = 0;
  homingShoulder = 0;
  homingWrist = 0;
  homingElbow = 0;
  pos = 0;

  while(!homingDone){
    initMotors();
  }
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


  //-----------------------------------------------------------
  // ------ Move stepper motor#1 Base with PS2 joystick -------
  //-----------------------------------------------------------
  
    if(!stopBasePos2){
      if((ps2x.Analog(PSS_RX) == 255)){                      
          basePos = basePos+10;          
          Serial.println(basePos, DEC);                  
      }                      
    }

    if(!stopBasePos1){
      if((ps2x.Analog(PSS_RX) == 0)){
          basePos = basePos-10;          
          Serial.println(basePos, DEC);                  
      }      
    }

    stepper.moveTo(basePos);

    while (stepper.distanceToGo() !=0) {
      stepper.runSpeedToPosition();
    }
      
    //==========================================================     

    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
  
}

// ---------------------------------

void initMotors(){

    // --------- Homing ----------

    if(!homingBase){

        pos = pos-10;
        stepper.moveTo(pos);
    
        while (stepper.distanceToGo() !=0) {
          stepper.runSpeedToPosition();
        }

        Serial.println("Move to switch "); 
        
        if(!digitalRead(baseSwich1)){   
          stepper.setCurrentPosition(0);          
          stepper.setMaxSpeed(5000);
          stepper.setAcceleration(5000);
          pos = 0;                             

          Serial.println("Move back "); 

          while(pos <= endPositionBase){
            pos = pos+20;
            stepper.moveTo(pos);

            while (stepper.distanceToGo()!=0) {
              stepper.runSpeedToPosition();
            }
          }
   
          Serial.println("Finished..."); 
          
          stepper.setCurrentPosition(0); 
          homingBase = 1;
          homingDone = 1; 
        }

    }

}




  
