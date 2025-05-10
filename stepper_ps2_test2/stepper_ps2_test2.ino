#include <PsxControllerHwSpi.h>
#include <Servo.h>

// Pins for Hardware SPI (Uno: MOSI=11, MISO=12, SCK=13)
#define PIN_ATT 10  // Attention pin

PsxControllerHwSpi<PIN_ATT> psx;

void setup() {
    Serial.begin(115200);
    
    if (!psx.begin()) {
        Serial.println("Controller not found");
        while (true);  // Halt
    }else{
      Serial.println("Controller found !!");
    }
    
    psx.enterConfigMode();
    psx.enableAnalogSticks();
    psx.exitConfigMode();
}

void loop() {
    if (psx.read()) {
        if (psx.buttonPressed(PSB_PAD_UP)) {
            Serial.println("PSB_PAD_UP pressed");
        }
        else if (psx.buttonPressed(PSB_PAD_DOWN)) {
            Serial.println("PSB_PAD_DOWN pressed");
        }
    }
    delay(10);
}
