#include <Arduino.h>
#include <MeAuriga.h>

MeUltrasonicSensor ultrasonic_sensor(10);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed LEDs(0, 12);

const int SPEED = 120;
const float VEER = 0.15;
const float CALIBRATION = 0.85;

const float SETPOINT = 20.0; 
const float NEUTRAL_BAND = 3.0;

enum State { OFF, ON, NEUTRAL };  
State mBotState = NEUTRAL;

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
void moveBot(int speed){
  motor_l.setMotorPwm(speed * CALIBRATION);
  motor_r.setMotorPwm(-speed);
}

// Powers wheel motors motors
void moveBotTo(int speed){
  motor_l.setMotorPwm(speed * (CALIBRATION + VEER));
  motor_r.setMotorPwm(-speed * (1.0 - VEER));
}

void moveBotAway(int speed){
  motor_l.setMotorPwm(speed * (CALIBRATION - VEER));
  motor_r.setMotorPwm(-speed * (1.0 + VEER));
}

void setLED(int colour){
  if (colour == 1)
    LEDs.setColor(3,255,154,0); // orange
  else if (colour == 2)
    LEDs.setColor(3,228,81,203); // pink
  else 
    LEDs.setColor(3,0,51,255); // blue

  LEDs.show();
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

  LEDs.setpin(44);
  LEDs.show();
    
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
}

/*
  Calculates the distance held between SETPOINT and mBot.
  If distance - neutral_band is > 0, move away from the wall. If < 0 - neutral_band move towards the wall
  Else maintain current trajectory  
*/
void loop() {
  _loop();
  float distance = ultrasonic_sensor.distanceCm();
  
  if (distance >= SETPOINT + NEUTRAL_BAND)
    mBotState = ON; 
  else if (distance <= SETPOINT - NEUTRAL_BAND)
    mBotState = OFF;
  else 
    mBotState = NEUTRAL; 

  switch (mBotState) {
    case ON: 
      moveBotTo(SPEED); setLED(1); // Orange
      break;
    case OFF: 
      moveBotAway(SPEED); setLED(2); // Pink
      break;
    case NEUTRAL:
      moveBot(SPEED); setLED(3);// Blue
    break; 
  }
}
