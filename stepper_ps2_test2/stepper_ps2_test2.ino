#include <PsxControllerHwSpi.h>
#include <AccelStepper.h>

// Pins for Hardware SPI (Mega: MOSI=51, MISO=50, SCK=52)
#define PIN_ATT 53  // Attention pin

PsxControllerHwSpi<PIN_ATT> psx;

// Stepper motor driver pins: Step = 7, Dir = 6
AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

void setup() {
    Serial.begin(115200);

    if (!psx.begin()) {
        Serial.println("Controller not found");
        while (true);  // Stop here if no controller is connected
    } else {
        Serial.println("Controller found !!");
    }

    psx.enterConfigMode();
    psx.enableAnalogSticks();
    psx.exitConfigMode();

    stepper.setMaxSpeed(1000);  // Maximum speed
}

void loop() {
    psx.read();  // Always read the controller input

    uint8_t x, y;
    psx.getLeftAnalog(x, y);  // Get analog stick position (0–255)

    // Convert Y value to signed speed
    int speed = map(y, 0, 255, 500, -500);  // Forward = low y, Backward = high y

    // Dead zone to stop jitter when near center
    if (abs(y - 128) < 10) {
        speed = 0;
    }

    //stepper.setSpeed(speed);
    stepper.setAcceleration(speed);
    stepper.setSpeed(speed);
    //stepper.runSpeed();
    stepper.run();

    delay(2);  // Prevent CPU overload
}  
