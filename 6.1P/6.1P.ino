#include <Arduino.h>
#include <Wire.h>
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);
MeRGBLed rgbled(0, 12);
MeBuzzer buzzer;

float prevAngle = 0;
const int MAX_COLOUR = 255;

float scaleIntensity(float movement){
  if (movement < 10) return 0.0;
  else if (movement < 15) return 0.2;
  else if (movement < 25) return 0.5;
  else if (movement < 35) return 0.8;
  else return 1;
}

int setBuzzerFreq(float intensity){
  if (intensity <= 0.2) return 500;
  else if (intensity <= 0.5) return 1500;
  else if (intensity <= 0.8) return 2200;
  else return 3500;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  gyro.begin(); 
  prevAngle = gyro.getAngle(3);

  buzzer.setpin(45);
  
  rgbled.setpin(44);
  rgbled.setColor(0, 0, 0, 0);
  rgbled.show();
}

void loop() {
  gyro.update();
  
  float currentAngle = gyro.getAngle(3);
  float detectMovement = abs(currentAngle - prevAngle);
  float intensity = scaleIntensity(detectMovement); 

  if (intensity == 0) {
    rgbled.setColor(0, 0, 0, 0);
    rgbled.show();
    buzzer.noTone();
    return;
  }
  
  // scales colours and brightness based on intensity of the movement. 
  // The harder the push, the lower the amount of green/blue get included in LED display
  int brightness = MAX_COLOUR * intensity;
  int red = brightness;
  int green = brightness * (1.0 - intensity) * 0.4;
  int blue = brightness * (1.0 - intensity) * 0.6;

  int freq = setBuzzerFreq(intensity);
  int duration = intensity * 300;

  rgbled.setColor(0, red, green, blue); 
  rgbled.show();
  buzzer.tone(freq, duration);
  
  prevAngle = currentAngle;
  delay(50);
}