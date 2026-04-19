#include <Arduino.h>
#include <Wire.h>
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);

float startPos = 0; 
const float THRESHOLD = 4; 
const int SPEED = 1520;

void move(int s) {
  motor_r.setMotorPwm(s);
  motor_l.setMotorPwm(s);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  gyro.begin();
  startPos = gyro.getAngle(3);
}

void loop() {
  gyro.update();
  
  float currentPos = gyro.getAngle(3);
  float movementDetected = currentPos - startPos;

  if (abs(movementDetected) <= THRESHOLD) {
    move(0);
    return;
  }

  move(movementDetected < 0 ? SPEED : -SPEED);

  while (abs(gyro.getAngle(3) - startPos) > THRESHOLD) {
    gyro.update();
    delay(10);
  }

  move(0);
}