#include <AccelStepper.h>
#include <Servo.h>

// --- Pin Definitions ---
#define pinSwich 43      // Shared input pin for all limit switches
#define buzzer 31        // Buzzer pin for auditory feedback
#define SERVO_PIN 44     // Servo pin controlling the gripper

// --- Gripper Range ---
#define MIN_GRIPPER_POS 0    // Minimum position for gripper servo
#define MAX_GRIPPER_POS 145  // Maximum position for gripper servo

// --- Potentiometer Pins (Analog Inputs) ---
const int potSpeedPin0 = A0;  // Shoulder joint control
const int potSpeedPin1 = A1;  // Base rotation control
const int potSpeedPin2 = A2;  // Elbow joint control
const int potSpeedPin3 = A3;  // Wrist joint control
const int potSpeedPin4 = A4;  // Gripper control

// --- Control Thresholds ---
const int deadZoneMin = 420;          // Lower threshold for joystick dead zone
const int deadZoneMax = 570;          // Upper threshold for joystick dead zone
const int minEffectiveSpeed = 100;    // Minimum speed for reliable movement
const int fixedAcceleration = 300;    // Constant acceleration for all steppers

// --- Stepper Motors Setup ---
AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);  // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);  // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);  // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);  // Wrist

// --- Servo (Gripper) Setup ---
Servo myservo;
int servoPos = 0;           // Initial servo position
bool gripperActive = false; // Flag to disable stepper control while using gripper

// --- Limit Switch Logic Flags ---
int pinSwichOn = 0;  // Stores the debounced state of the shared switch pin
bool baseStepMove1 = 1, baseStepMove2 = 1;
bool shoulderStepMove1 = 1, shoulderStepMove2 = 1;
bool elbowStepMove1 = 1, elbowStepMove2 = 1;
bool wristStepMove1 = 1, wristStepMove2 = 1;

// --- Buzzer State Tracking ---
static unsigned long lastBeepTime = 0;  // Last time the buzzer was updated
static int beepStep = 0;                // Current note index for buzzer sequence

// --- Z Hat Input Pin ---
#define pinZHat 53  // Digital input pin (e.g., external control or trigger)

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Configure all steppers with standard acceleration and max speed
  setupStepper(stepper1);
  setupStepper(stepper2);
  setupStepper(stepper3);
  setupStepper(stepper4);

  // Configure gripper (servo motor)
  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo.write(servoPos);  // Move servo to initial position

  // Configure limit switch input and buzzer output
  pinMode(pinSwich, INPUT_PULLUP);  // Shared pull-up input
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);                   // Ensure buzzer is silent on start

  // Configure Z Hat digital input
  pinMode(pinZHat, INPUT_PULLUP);
}

void setupStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(1000);                // Set maximum speed
  stepper.setAcceleration(fixedAcceleration);  // Set constant acceleration
}

void loop() {
  // --- Read Potentiometer Values ---
  int rawSpeed0 = analogRead(potSpeedPin0);  // Shoulder
  int rawSpeed1 = analogRead(potSpeedPin1);  // Base
  int rawSpeed2 = analogRead(potSpeedPin2);  // Elbow
  int rawSpeed3 = analogRead(potSpeedPin3);  // Wrist
  int rawSpeed4 = analogRead(potSpeedPin4);  // Gripper

  // --- Read Shared Limit Switch State ---
  pinSwichOn = !digitalRead(pinSwich);  // Active LOW (pressed = 1)

  // --- Check Z Hat Button ---
  if (!digitalRead(pinZHat)) {  // Active LOW
    Serial.println("Z Hat pressed");
  }  

  // --- Buzzer Feedback Logic (Tri-tone loop while switch is active) ---
  if (pinSwichOn) {
    unsigned long now = millis();
    if (now - lastBeepTime > 120) {  // Time between notes
      switch (beepStep) {
        case 0: tone(buzzer, 880); break;   // A5
        case 1: tone(buzzer, 988); break;   // B5
        case 2: tone(buzzer, 1047); break;  // C6
      }
      beepStep = (beepStep + 1) % 3;
      lastBeepTime = now;
    }
  } else {
    noTone(buzzer);  // Silence the buzzer
    beepStep = 0;    // Reset buzzer note index
  }

  // --- Gripper Control (Overrides Stepper Motion) ---
  if (rawSpeed4 > deadZoneMax || rawSpeed4 < deadZoneMin) {
    gripperActive = true;

    if (rawSpeed4 > deadZoneMax && servoPos < MAX_GRIPPER_POS) {
      servoPos += 1;
      myservo.write(servoPos);
      delay(5);  // Smooth movement
    }

    if (rawSpeed4 < deadZoneMin && servoPos > MIN_GRIPPER_POS) {
      servoPos -= 1;
      myservo.write(servoPos);
      delay(5);  // Smooth movement
    }

    return;  // Skip stepper logic if gripper is in use
  } else {
    gripperActive = false;
  }

  // --- Stepper Joint Control (if gripper is inactive) ---
  handleJointControl(stepper1, rawSpeed1, baseStepMove1, baseStepMove2);         // Base
  handleJointControl(stepper2, rawSpeed0, shoulderStepMove1, shoulderStepMove2); // Shoulder
  handleJointControl(stepper3, rawSpeed2, elbowStepMove1, elbowStepMove2);       // Elbow
  handleJointControl(stepper4, rawSpeed3, wristStepMove1, wristStepMove2);       // Wrist
}

/**
 * Controls a joint using potentiometer input and respects movement limits.
 * @param stepper Reference to the stepper motor.
 * @param potValue Analog input from potentiometer.
 * @param moveDir1 Flag for allowed movement in positive direction.
 * @param moveDir2 Flag for allowed movement in negative direction.
 */
void handleJointControl(AccelStepper &stepper, int potValue, bool &moveDir1, bool &moveDir2) {
  long speed = 0;

  if (potValue > deadZoneMax) {
    // Moving in positive direction
    if (pinSwichOn) {
      moveDir1 = false;  // Disable movement in this direction if limit hit
    } else {
      moveDir1 = true;
      moveDir2 = true;   // Reset movement permissions
    }

    if (!pinSwichOn || !moveDir2) {
      // Only allow if not blocked
      speed = map(potValue, deadZoneMax, 1023, minEffectiveSpeed, 1000);
    }

  } else if (potValue < deadZoneMin) {
    // Moving in negative direction
    if (pinSwichOn) {
      moveDir2 = false;  // Disable movement in this direction if limit hit
    } else {
      moveDir2 = true;
      moveDir1 = true;   // Reset movement permissions
    }

    if (!pinSwichOn || !moveDir1) {
      speed = map(potValue, deadZoneMin, 0, -minEffectiveSpeed, -1000);
    }

  } else {
    speed = 0;  // Inside dead zone — no movement
  }

  stepper.setSpeed(speed);  // Update target speed
  stepper.run();            // Apply speed using acceleration constraints
}
