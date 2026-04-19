#include <Arduino.h>
#include <Wire.h>
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);
MeRGBLed rgbled(0, 12);
MeBuzzer buzzer;

const int THRESHOLD = 12;

void react(int val) {
  switch (val) {
    case 1:
      rgbled.setColor(0, 0, 100, 255); // blue
      buzzer.tone(800, 20);
      break;
    case 2:
      rgbled.setColor(0, 255, 20, 80); // pink
      buzzer.tone(1500, 20);
      break;
    case 3:
      rgbled.setColor(0, 255, 200, 0); // yellow
      buzzer.tone(2500, 20);
      break;
    default:
      rgbled.setColor(0, 0, 0, 0);
      buzzer.noTone();
      break;
  }
  rgbled.show();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  gyro.begin(); 

  buzzer.setpin(45);
  
  rgbled.setpin(44);
  rgbled.setColor(0, 0, 0, 0);
  rgbled.show();
}

void loop() {
  gyro.update();
  
  float lean  = gyro.getAngle(1);
  float tilt = gyro.getAngle(2);

  bool leftUp = tilt > THRESHOLD;
  bool rightUp = tilt < -THRESHOLD;
  bool frontUp = lean < -THRESHOLD;

  if (leftUp)
    react(1);
  else if (rightUp)
    react(2);
  else if (frontUp)
    react(3);
  else 
    react(0);

  delay(50);
}