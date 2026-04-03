#include <Arduino.h>
#include "Movement.h"

MeLightSensor lightSensor1(11); // left
MeLightSensor lightSensor2(12); // right

const int SPEED = 120;
const int TURN_SPEED = 200;
const int LIGHT_TOLERANCE = 30;

enum State { TURN, DRIVE, STOP };
State state = STOP;
int previousLight = -1; // variable to calculate ranger to stop once at light source
int sensor1, sensor2, lightImbalance;

// non blocking delay for motors to continue running via _loop()
void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0; 
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

// Handles reading and updating global sensor/lightImbalance variables
void readSensors() {
  sensor1 = lightSensor1.read();
  sensor2 = lightSensor2.read();
  lightImbalance = sensor1 - sensor2;
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
  readSensors();

  // Begins in a STOP state. Will only start moving if it detects a change in light
  if (abs(lightImbalance) > LIGHT_TOLERANCE && state != STOP)
    state = TURN; 

  switch (state) {
    // Decides which direction to turn by evaluating whether the imbalance is coming from
    // the left or right sensor. Continues turning until it's detected the source of the light
    case TURN:
      int turnDirection = lightImbalance > 0 ? 4 : 3;
      while (abs(lightImbalance) > LIGHT_TOLERANCE) {
        move(turnDirection, TURN_SPEED);
        _loop();
        readSensors();
      }
      break;

    // Handles moving towards the light. Current and previous light detect whether the light source has been passed 
    // If current light is less than the previous, it means the light source has been found and the robot stops moving
    // If an imbalance is detected, indicates the light source has moved, state set to STOP
    // Otherwise will continue to update previousLight while it's greater than current
    case DRIVE:{
      move(1, SPEED);
      _delay(0.5);
      readSensors();

      int current = sensor1 + sensor2;
      if (abs(lightImbalance) > LIGHT_TOLERANCE) state = TURN;
      else if (current < previousLight) state = STOP;
      else previousLight = current;

      break;
    }

    case STOP:
      stopMotors();
      _delay(0.5);
      
      readSensors();
      if (abs(lightImbalance) > LIGHT_TOLERANCE)
        state = TURN;
      break;
  }
}