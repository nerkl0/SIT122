#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard MOTOR_R(SLOT1);
MeEncoderOnBoard MOTOR_L(SLOT2);

// Calibrate speed of each wheel to offset drift
const int LEFT_SPEED  = 110;
const int RIGHT_SPEED = 128;

void moveForward() {
    MOTOR_R.setTarPWM(-RIGHT_SPEED);
    MOTOR_L.setTarPWM(LEFT_SPEED);
}

void moveBack() {
    MOTOR_R.setTarPWM(RIGHT_SPEED);
    MOTOR_L.setTarPWM(-LEFT_SPEED);
}

void turnRight() {
    MOTOR_R.setTarPWM(RIGHT_SPEED);
    MOTOR_L.setTarPWM(LEFT_SPEED);
}

void turnLeft() {
    MOTOR_R.setTarPWM(-RIGHT_SPEED);
    MOTOR_L.setTarPWM(-LEFT_SPEED);
}

void stop() {
    MOTOR_R.setTarPWM(0);
    MOTOR_L.setTarPWM(0);
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  MOTOR_R.loop();
  MOTOR_L.loop();

  if(Serial.available()){
    char command = Serial.read();
    switch(command){
      case '1': moveForward(); break;
      case '2': moveBack(); break;
      case '3': turnRight(); break;
      case '4': turnLeft(); break;
      case '0': stop(); break;
    }
  }
}