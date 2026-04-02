#include <Arduino.h>
#include "Movement.h"

MeLightSensor lightSensor1(11); // left
MeLightSensor lightSensor2(12); // right

const int SPEED = 120;
const int LIGHT_TOLERANCE = 30;

enum State { TURN, DRIVE, STOP };
State state = STOP;
int turnDirection = 0;
int previousLight = -1; // variable to calculate ranger to stop once at light source

// non blocking delay for motors to continue running via _loop()
void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0; 
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  Serial.begin(115200);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_r.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  _loop();

  int sensor1 = lightSensor1.read();
  int sensor2 = lightSensor2.read();
  int lightImbalance = sensor1 - sensor2;

  // check light imbalance & drive towards souce of light. 
  // Triggers DRIVE if lightImbalance is within the light_tolerance range
  // DRIVE will then check whether the light imbalance has changed triggering TURN
  if (abs(lightImbalance) <= LIGHT_TOLERANCE) {
    previousLight = sensor1 + sensor2;
    state = DRIVE;
  }

  switch (state) {
    case TURN:
      turnDirection = lightImbalance > 0 ? 4 : 3; // sets left / right turn
      move(turnDirection, SPEED*1.75);
      break;

    case DRIVE:{
      if (abs(lightImbalance) > LIGHT_TOLERANCE) {
        state = TURN;
      } else {
        move(1, SPEED);
        _delay(0.4);
        
        // rechecks sensors and compares against the previous light level
        // if detected to be moving away from the light source will stop
        int current = lightSensor1.read() + lightSensor2.read();
        if (current < previousLight)
          state = STOP;
        else
          previousLight = current;
        }
      break;
    }
    case STOP:
      stopMotors();
      _delay(0.5);
      break;
  }
}