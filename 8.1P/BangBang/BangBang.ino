#include <Arduino.h>
#include <MeAuriga.h>

MeUltrasonicSensor ultrasonic_sensor(10);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed LEDs(0, 12);

const int SPEED = 120;
const float SETPOINT = 15.0; 

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

// Powers wheel motors motors
void moveBotTo(int speed){
  motor_l.setMotorPwm(speed * 1.2);
  motor_r.setMotorPwm(-speed);
}

void moveBotAway(int speed){
  motor_l.setMotorPwm(speed);
  motor_r.setMotorPwm(-speed * 1.4);
}

void setLED(int colour){
  if (colour == 1)
    LEDs.setColor(3,255,0,0); // red
  else if (colour == 2)
    LEDs.setColor(3,0,255,0); // green
  else 
    LEDs.setColor(3,0,0,0); // cyan

  LEDs.show();
}

void _loop()
{
  motor_l.loop();
  motor_r.loop();
}

void setup() {
  LEDs.setpin(44);
  LEDs.show();

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
}

/*
  Calculates the distance held between SETPOINT and mBot.
  If > 0, move away from the wall. If <= 0 move towards the wall
*/
void loop() {
  _loop();
  float error = SETPOINT - ultrasonic_sensor.distanceCm();
  
  mBotState = error > 0 ? ON : OFF;  

  switch (mBotState) {
    case ON: 
      moveBotAway(SPEED); setLED(1); break;
    case OFF: 
      moveBotTo(SPEED); setLED(2); break;
  }
}
