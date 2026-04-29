#include <Arduino.h>
#include <MeAuriga.h>

#define LEDCOUNT 12

MeLightSensor lightSensor1(11);
MeLightSensor lightSensor2(12);

MeRGBLed leds(0, LEDCOUNT);

int lightReading = 0; 
bool ledState = false; 

void powerLights(bool power){
  if (power) {
    leds.setColor(0, 255, 255, 255);
    leds.show();
  } else {
    leds.setColor(0, 0, 0, 0); 
    leds.show();
  }
  ledState = power;
}

void setup() {
  leds.setpin(44);
  leds.show();   
}

void loop() {
  int lightReading = lightSensor1.read();

  if(lightReading < 10 && !ledState) 
      powerLights(true); 
  else if(lightReading >= 10 && ledState) 
      powerLights(false);
}
