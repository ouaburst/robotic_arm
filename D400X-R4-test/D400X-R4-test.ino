#include <AccelStepper.h>
#include <Servo.h>

// --- Pin Definitions ---
#define pinSwich 43
#define buzzer 31
#define SERVO_PIN 44

// --- Gripper Range ---
#define MIN_GRIPPER_POS 0
#define MAX_GRIPPER_POS 145

// --- Potentiometer Pins ---
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

// --- Servo (Gripper) ---
Servo myservo;
int servoPos = 0;
bool gripperActive = false;

// --- Limit Switch Logic ---
int pinSwichOn = 0;
bool baseStepMove1 = 1, baseStepMove2 = 1;
bool shoulderStepMove1 = 1, shoulderStepMove2 = 1;
bool elbowStepMove1 = 1, elbowStepMove2 = 1;
bool wristStepMove1 = 1, wristStepMove2 = 1;

static unsigned long lastBeepTime = 0;
static int beepStep = 0;

void setup() {
  Serial.begin(9600);

  // Setup all steppers
  setupStepper(stepper1);
  setupStepper(stepper2);
  setupStepper(stepper3);
  setupStepper(stepper4);

  // Gripper
  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo.write(servoPos);

  // Limit switch & buzzer
  pinMode(pinSwich, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);
}

void setupStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(fixedAcceleration);
}

void loop() {
  int rawSpeed0 = analogRead(potSpeedPin0);  // Shoulder
  int rawSpeed1 = analogRead(potSpeedPin1);  // Base
  int rawSpeed2 = analogRead(potSpeedPin2);  // Elbow
  int rawSpeed3 = analogRead(potSpeedPin3);  // Wrist
  int rawSpeed4 = analogRead(potSpeedPin4);  // Gripper

  pinSwichOn = !digitalRead(pinSwich);  // LOW = pressed

  // Buzzer feedback
  if (pinSwichOn) {
    unsigned long now = millis();
    if (now - lastBeepTime > 120) {
      switch (beepStep) {
        case 0: tone(buzzer, 880); break;   // A5
        case 1: tone(buzzer, 988); break;   // B5
        case 2: tone(buzzer, 1047); break;  // C6
      }
      beepStep = (beepStep + 1) % 3;
      lastBeepTime = now;
    }
  } else {
    noTone(buzzer);
    beepStep = 0;
  }
  
  // Gripper overrides stepper control
  if (rawSpeed4 > deadZoneMax || rawSpeed4 < deadZoneMin) {
    gripperActive = true;

    if (rawSpeed4 > deadZoneMax && servoPos < MAX_GRIPPER_POS) {
      servoPos += 1;
      myservo.write(servoPos);
      delay(5);
    }

    if (rawSpeed4 < deadZoneMin && servoPos > MIN_GRIPPER_POS) {
      servoPos -= 1;
      myservo.write(servoPos);
      delay(5);
    }

    return;
  } else {
    gripperActive = false;
  }

  // Stepper logic only if gripper not active
  handleJointControl(stepper1, rawSpeed1, baseStepMove1, baseStepMove2);     // Base
  handleJointControl(stepper2, rawSpeed0, shoulderStepMove1, shoulderStepMove2); // Shoulder
  handleJointControl(stepper3, rawSpeed2, elbowStepMove1, elbowStepMove2);   // Elbow
  handleJointControl(stepper4, rawSpeed3, wristStepMove1, wristStepMove2);   // Wrist
}

void handleJointControl(AccelStepper &stepper, int potValue, bool &moveDir1, bool &moveDir2) {
  long speed = 0;

  if (potValue > deadZoneMax) {
    // Positive direction
    if (pinSwichOn) {
      moveDir1 = false;
    } else {
      moveDir1 = true;
      moveDir2 = true;
    }

    if (!pinSwichOn || !moveDir2) {
      speed = map(potValue, deadZoneMax, 1023, minEffectiveSpeed, 1000);
    }

  } else if (potValue < deadZoneMin) {
    // Negative direction
    if (pinSwichOn) {
      moveDir2 = false;
    } else {
      moveDir2 = true;
      moveDir1 = true;
    }

    if (!pinSwichOn || !moveDir1) {
      speed = map(potValue, deadZoneMin, 0, -minEffectiveSpeed, -1000);
    }

  } else {
    speed = 0;  // Dead zone
  }

  stepper.setSpeed(speed);
  stepper.run();  // Uses acceleration
}
