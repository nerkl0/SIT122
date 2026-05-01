#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor sensor_front(9);
MeLineFollower lineFinder(10);
MeRGBLed LEDs(0, 12);

const int LED_PIN = 3; 
const int SPEED = 100;
const int TURN_SPEED = 80;
const float CALIBRATION = 0.8;

const float OBSTACLE_SETPOINT = 23.0; 
const float TURN_SCALE = 0.05;
unsigned long stop_search = 2000;
float scaler = 1.0;

enum RobotState { STRAIGHT, LEFT, RIGHT, STOP, AVOID };
enum SensorState { ON_LINE, RIGHT_OUT, LEFT_OUT, OFF_LINE };
enum LEDState { L_GREEN, L_RED, L_BLUE, L_PURPLE, L_YELLOW };

RobotState mBot_state = STRAIGHT;
SensorState sensor_state = OFF_LINE;

void isr_process_encoder1(void){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

void isr_process_encoder2(void){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void _loop(){
  motor_l.loop();
  motor_r.loop();
}
void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

/*
  The next four functions are purely for debugging/logging
  stateName, ledName, sensorState return string value of enums
  printDetails prints current state to the terminal 
*/
const char* stateName(){
  switch(mBot_state){
    case STRAIGHT: return "STRAIGHT";
    case LEFT: return "LEFT";
    case RIGHT:return "RIGHT";
    case STOP: return "STOP";
    case AVOID: return "AVOID";
    default: return "UNKNOWN";
  }
}
const char* sensorState(){
  switch(sensor_state){
    case ON_LINE: return "ON LINE";
    case RIGHT_OUT: return "RIGHT OUT";
    case LEFT_OUT: return "LEFT OUT";
    case OFF_LINE: return "OFF LINE";
    default: return "NONE";
  }
}
void printDetails(){
  Serial.print("Sensor "); Serial.print(sensor_state); Serial.print(": "); Serial.println(sensorState());
  Serial.print("State "); Serial.print(mBot_state); Serial.print(": "); Serial.print(stateName());
  Serial.print("\t\tScaler: "); Serial.println(scaler);
  Serial.println();
}

/*
  moveBot powers motors, if a scale multiplier is greater than 1
  it will apply it to the current states respective wheel.   
*/
void moveBot(int speed, float scale){
  int leftMotor = speed * CALIBRATION;
  int rightMotor = speed; 
  
  if (mBot_state == LEFT) leftMotor *= scale; 
  else if (mBot_state == RIGHT) rightMotor *= scale;
  
  motor_l.setMotorPwm(leftMotor);
  motor_r.setMotorPwm(-rightMotor);
}

/*
  findLine attempts to find the line again if both sensors have lost it. 
  First continues on the current state turn trajectory in the event it didn't turn hard enough 
  If after `stop_search` time has elapsed it hasn't found the line, will turn the opposite 
  direction for `stop_search * 2` time.
  If the line gets found, continues on; else mBot_state is updated to STOP 
*/
void findLine(){
  Serial.println("Searching....");
  setLED(L_YELLOW);

  // First attempt: turn in the current state direction until sensor is not off line or search timeout
  unsigned long start = millis();
  while ((SensorState)lineFinder.readSensors() == OFF_LINE && millis() - start < stop_search){
    _loop();
    moveBot(TURN_SPEED, TURN_SCALE);
  }
  if ((SensorState)lineFinder.readSensors() != OFF_LINE) // return if the line was found (not timed out)
    return;

  // Second attempt: flip direction, searching twice as long
  Serial.println("\n== Switching search direction ==\n");
  updateState(mBot_state == LEFT ? RIGHT : LEFT, L_YELLOW);

  start = millis();
  while ((SensorState)lineFinder.readSensors() == OFF_LINE && millis() - start < stop_search * 2){
    _loop();
    moveBot(TURN_SPEED, TURN_SCALE);
  }
  if ((SensorState)lineFinder.readSensors() != OFF_LINE) 
    return;

  // To Finish: Search was unsuccesful. Cease movement. 
  Serial.println("Can't locate line. Stopping");
  updateState(STOP, L_RED);
}

/*
  avoidBot spins the robot in a clockwise direction for 0.7s before calling 
  findLine() to realign itself
*/
void avoidBot(){
  Serial.println("!! Obstacle Detected !!");
  motor_l.setMotorPwm(SPEED);
  motor_r.setMotorPwm(SPEED);
  _delay(0.7);

  updateState(RIGHT, L_YELLOW);
  findLine();
}

void stopBot(){
  motor_l.setMotorPwm(0);
  motor_r.setMotorPwm(0);
}

void setLED(int colour){
  if (colour == L_PURPLE)
    LEDs.setColor(LED_PIN,160,32,240);
  else if (colour == L_BLUE)
    LEDs.setColor(LED_PIN,0,51,255);
  else if (colour == L_RED)
    LEDs.setColor(LED_PIN,255,25,0);
  else if (colour == L_GREEN)
    LEDs.setColor(LED_PIN,0,255,30);
  else if (colour == L_YELLOW)
    LEDs.setColor(LED_PIN,255,255,0);
  else 
    LEDs.setColor(LED_PIN,0,0,0);
  LEDs.show();
}

/*
  Handles updating state of mBot and LED colour
*/
void updateState(int botState, int ledState){
  mBot_state = (RobotState)botState; 
  setLED((LEDState)ledState);
  printDetails();
}

void setup() {
  Serial.begin(115200);
  
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21); 
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
  
  LEDs.setpin(44);
  LEDs.show(); 
}

void loop() {
  _loop(); 
  sensor_state = (SensorState)lineFinder.readSensors(); 

  // This if block is triggered after findLine has unsuccesfully found the line after searching
  if(sensor_state == OFF_LINE && mBot_state == STOP){
    stopBot();
    return;
  }

 // If an obstacle is detected, update state to AVOID to turn in the opposite direction
  bool obstacle_detected = sensor_front.distanceCm() < OBSTACLE_SETPOINT; 
  if (obstacle_detected){
    updateState(AVOID, L_BLUE);
    avoidBot();
    return;
  }

  // The if statements in each case were only implemented to make it easier to read terminal logs
  switch(sensor_state){
    case ON_LINE:
      if (mBot_state != STRAIGHT){
        updateState(STRAIGHT, L_GREEN);
        scaler = 1.0; 
      }
      break;

    case RIGHT_OUT:
      if (mBot_state != LEFT){
        updateState(LEFT, L_GREEN);
        scaler = TURN_SCALE;
      }
      break;

    case LEFT_OUT:
      if (mBot_state != RIGHT){
        updateState(RIGHT, L_GREEN);
        scaler = TURN_SCALE; 
      }
      break;

    case OFF_LINE:
      findLine();
      break;
  }

  moveBot(SPEED, scaler);
}
