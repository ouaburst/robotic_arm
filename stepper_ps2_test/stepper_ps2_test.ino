#include <AccelStepper.h>
#include <DigitalIO.h>
#include <PsxControllerHwSpi.h>

// --- Global Definitions ---

const byte PIN_PS2_ATT = 10;

boolean haveController = false;
uint16_t iter = 0; // Loop iteration counter for throttling updates

AccelStepper stepper(AccelStepper::DRIVER, 7, 6);    // Base


PsxControllerHwSpi<PIN_PS2_ATT> psx; // PS2 controller object using hardware SPI

/**
 * @brief Handles speed scaling based on button press.
 *
 * @param keyPressed Button word from controller
 * @param keyOnPress Whether any button is currently pressed
 */
void handleKeyPress(uint16_t keyPressed, bool keyOnPress, float &speed_scale,
                    uint16_t &keyLast) {
  if (keyOnPress) {
    if (keyPressed != keyLast) {
      keyLast = keyPressed;
      Serial.print(F("___KeyPressed: "));
      Serial.println(keyPressed);

      float new_speed = speed_scale;

      // Adjust the speed scaling via controller shoulder buttons
      switch (keyPressed) {
      case 256:
        new_speed = 0.3;
        break; // L2
      case 512:
        new_speed = 1.0;
        break; // R2
      case 1024:
        new_speed = 1.5;
        break; // L1
      case 2048:
        new_speed = 2.0;
        break; // R1
      default:
        break;
      }

      if (new_speed != speed_scale) {
        speed_scale = new_speed;
        Serial.print(F("Speed scale set to: "));
        Serial.println(speed_scale);
      }
    }
  } else {
    keyLast = 0;
  }
}

/**
 * @brief Initializes the PS2 controller, retrying until success.
 */
void initializeController() {
  while (!haveController) {
    if (psx.begin()) {
      Serial.println(F("Controller found! :)"));
      delay(300);

      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode :("));
        continue;
      }

      Serial.print(F("Controller Type is: "));
      Serial.println(F("I cut this function, ;)")); // Placeholder

      if (!psx.enableAnalogSticks()) {
        Serial.println(F("Cannot enable analog sticks :("));
        continue;
      } else {
        Serial.println(F("Analog sticks enabled :D"));
      }

      if (!psx.enableAnalogButtons()) {
        Serial.println(F("Cannot enable analog buttons :("));
        continue;
      } else {
        Serial.println(F("Buttons enabled B-)"));
      }

      if (!psx.exitConfigMode()) {
        Serial.println(F("Cannot exit config mode :/"));
        continue;
      } else {
        Serial.println(F("Exit config mode ^_^"));
      }

      haveController = true;
    } else {
      Serial.println(F("Controller not found, retrying... :("));
    }

    delay(1000);
  }
}

/**
 * @brief Enables or disables the stepper motor based on speed.
 */
void enableDisableMotor(bool &stepperStatus, float speed) {
  if (abs(speed) < 2.0) {
    if (stepperStatus) {
      stepperStatus = false;
      stepper.disableOutputs();
      Serial.println("Output disabled");
    }
  } else {
    if (!stepperStatus) {
      stepperStatus = true;
      stepper.enableOutputs();
      Serial.println("Output enabled");
    }
  }
}

/**
 * @brief Reads button state from PS2 controller and updates speed scale.
 */
void processKeyInput(float &speed_scale, uint16_t &keyLast) {
  uint16_t keyPressed = psx.getButtonWord();
  bool keyOnPress = !!keyPressed;
  if (keyOnPress) {
    handleKeyPress(keyPressed, keyOnPress, speed_scale, keyLast);
  } else {
    keyLast = 0;
  }
}

/**
 * @brief Reads analog stick position and adjusts speed/motor accordingly.
 */
void updateSpeedAndMotor(byte &slx, byte &sly, float &speed,
                         bool &stepperStatus, float speed_scale) {
  byte lx, ly;
  psx.getLeftAnalog(lx, ly);
  if (lx != slx || ly != sly) {
    slx = lx;
    sly = ly;
    speed = ((float)(128 - ly)) * speed_scale;

    // uncomment if you want stepper enabled the whole time.
    //enableDisableMotor(stepperStatus, speed);
    stepper.setSpeed(speed);

    Serial.print(F("Set speed: "));
    Serial.println(speed);
  }
}

void setup() {
  Serial.begin(57600);
  Serial.println(F("Serial Ready!"));

  stepper.setMaxSpeed(400);
  stepper.setSpeed(100);

  initializeController();
}

void loop() {
  static byte slx = 0, sly = 0;
  static float speed = 0;
  static bool stepperStatus = false;
  static uint16_t keyLast = 0;
  static float speed_scale = 1;

  // --- Recommended: run controller input every ~20ms to avoid overload ---
  if (iter == 0) {
    if (!haveController) {
      initializeController();
    } 
  
    else if (!psx.read()) {
      // --- connection of arduino-receiver, not receive-transmitter ---
      Serial.println(F("Controller lost :("));
      haveController = false;
    }
    
    else {
      processKeyInput(speed_scale, keyLast);
      updateSpeedAndMotor(slx, sly, speed, stepperStatus, speed_scale);
    }
  }

  // --- OBS: call as frequent as possible, at least once per step-interval ---
  stepper.runSpeed();

  iter = (iter + 1) % 40;
  delay(1);
}
