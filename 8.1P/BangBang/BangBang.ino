#include <Arduino.h>
#include <MeAuriga.h>

MeUltrasonicSensor ultrasonic_sensor(10);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);

const int SPEED = 140;
const int FORWARD = 1;
const float CALIBRATION = 116.0 / 128.0;

const float SETPOINT = 10.0; 

enum State { OFF, ON };
State mBotState = OFF;

void isr_process_encoder1(void){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

void isr_process_encoder2(void){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

// Non-blocking delay for encoder motors
void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void stopMotors(){
  motor_l.setMotorPwm(0);
  motor_r.setMotorPwm(0);
  _delay(1);
}

// Powers wheel motors motors
void moveBot(int direction, float speed){
  motor_l.setMotorPwm(SPEED);
  motor_r.setMotorPwm(-SPEED * CALIBRATION);
}

void _loop()
{
  motor_l.loop();
  motor_r.loop();
}

void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
}

/*
  Calculates the distance held between SETPOINT and mBot.
  If > 0, move along the wall. If <= 0 robot stops, the wall is too far to detect
*/
void loop() {
  _loop();
  float error = SETPOINT - ultrasonic_sensor.distanceCm();
  
  mBotState = error > 0 ? ON : OFF;  

  switch (mBotState) {
    case ON: 
      moveBot(FORWARD, SPEED); break;
    case OFF: 
      stopMotors(); break;
  }
}
