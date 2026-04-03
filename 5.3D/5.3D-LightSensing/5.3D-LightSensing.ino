#include <Arduino.h>
#include <MeAuriga.h>

MeLightSensor lightSensor1(11); // left
MeLightSensor lightSensor2(12); // right
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor ultrasonic_sensor(10);

const float CALIBRATION = 110.0 / 128.0;
const int SPEED = 120;
const int TURN_SPEED = 200;
const int LIGHT_TOLERANCE = 30;
const int OBSTACLE_ATTEMPTS = 3; // set number of times robot tries avoid obstacle in a single iteration

int sensor1, sensor2, lightImbalance;

enum State { AVOID, TURN, DRIVE, STOP };
State state = STOP;
int previousLight = -1; // variable to calculate ranger to stop once at light source

void isr_process_encoder1()
{
  if(digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void isr_process_encoder2()
{
  if(digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

// non blocking delay for motors to continue running via _loop()
void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0; 
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void _loop() {
  motor_r.loop();
  motor_l.loop();
}

void move(int direction, int speed)
{
  int leftSpeed = 0; 
  int rightSpeed = 0;

  switch(direction){
    case 1: // forward
      leftSpeed = speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 2: // backward
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = speed;
      break;
    case 3: // right
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 4: // left
      leftSpeed = speed * CALIBRATION;
      rightSpeed = speed;
      break;
  }

  motor_r.setTarPWM(rightSpeed);
  motor_l.setTarPWM(leftSpeed);
}

void stopMotors(){
  motor_r.setTarPWM(0);
  motor_l.setTarPWM(0);
}

// Handles reading and updating global sensor/lightImbalance variables
void readSensors() {
  sensor1 = lightSensor1.read();
  sensor2 = lightSensor2.read();
  lightImbalance = sensor1 - sensor2;
}

// returns boolean value to indicate whether an obstacle has been detected
bool checkObstacle(int distance){
  if (ultrasonic_sensor.distanceCm() < distance) {
      state = AVOID;
      return true;
  }
  return false; 
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_r.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder2, RISING);
  Serial.print("SETUP STATE: ");
  Serial.println(state);
}

void loop() {
  _loop();
  readSensors();

  // Begins in a STOP state. Will only start moving if it detects a change in light
  if (abs(lightImbalance) > LIGHT_TOLERANCE && state != STOP && state != AVOID)
    state = TURN; 

  switch (state) {
    
    // Reverses slightly from the obstacle, turns one direction. If obstacle found after turning, will turn in the opposite direction
    // If it frees iteself in less than the set attempts, state set to TURN, else state set to STOP
    case AVOID:{
      int attempts = OBSTACLE_ATTEMPTS;
      while (checkObstacle(20) && attempts >= 0) {
        move(2, SPEED/2);
        _delay(1);
        move(3, TURN_SPEED);
        _delay(0.4);
        if (checkObstacle(20)) {
            move(4, TURN_SPEED);
            _delay(0.4);
        }
        attempts--; 
      }
      state = checkObstacle(20) ? STOP : TURN;
      break; 
    }
    
    // Decides which direction to turn by evaluating whether the imbalance is coming from
    // the left or right sensor. Continues turning until it's detected the source of the light
    // Will break out early if an obstacle has been detected
    // If no obstacle, then previousLight is updated and state is set to DRIVE
    case TURN:{
      int turnDirection = lightImbalance > 0 ? 4 : 3;
      
      while (abs(lightImbalance) > LIGHT_TOLERANCE) {
        move(turnDirection, TURN_SPEED);
        _loop();
        readSensors();
        
        if(checkObstacle(20))
          break;
      }
      if (state != AVOID) {
          previousLight = sensor1 + sensor2;
          state = DRIVE;
      }
      break;
    }

    // Handles moving towards the light. Current and previous light detect whether the light source has been passed 
    // If current light is less than the previous, it means the light source has been found and the robot stops moving
    // If an imbalance is detected, indicates the light source has moved, state set to STOP
    // Otherwise will continue to update previousLight while it's greater than current
    case DRIVE:{
      move(1, SPEED);
      _delay(0.5);
      readSensors();
     
      if(checkObstacle(20))
        break; 

      int current = sensor1 + sensor2;
      if (abs(lightImbalance) > LIGHT_TOLERANCE) state = TURN;
      else if (current < previousLight) state = STOP;
      else previousLight = current;

      break;
    }

    case STOP:
      stopMotors();
      _delay(1.5);

      readSensors();
      if (abs(lightImbalance) > LIGHT_TOLERANCE)
        state = TURN;
      break;
  }
}