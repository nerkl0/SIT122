#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor sensor_front(9);
MeLineFollower lineFinder(10);
MeRGBLed LEDs(0, 12);

const int LED_PIN = 3; 
const int SPEED = 100;
const float CALIBRATION = 0.8;

const float OBSTACLE_SETPOINT = 10.0; 
const float TURN_SCALE = 0.05;
unsigned long stop_search = 1500;
float scaler = 1.0;
bool obstacle_recover = false; 

/* Time outs for serial.prints */
unsigned long last_print_time = 0;

enum RobotState { STRAIGHT, LEFT, RIGHT, STOP, AVOID };
enum SensorState { ON_LINE, RIGHT_OUT, LEFT_OUT, OFF_LINE };
enum LEDState { NONE, L_RED, L_BLUE, L_PURPLE, L_GREEN, L_YELLOW };

RobotState mBot_state = STRAIGHT;
LEDState led_colour = L_RED;
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
    default: return "UNKNOWN";
  }
}
const char* ledName(){
  switch(led_colour){
    case L_RED: return "RED";
    case L_BLUE: return "BLUE";
    case L_PURPLE: return "PURPLE";
    case L_GREEN: return "GREEN";
    case L_YELLOW: return "YELLOW";
    default: return "NONE";
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
  Serial.print("LED: "); Serial.println(ledName());
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
  First continues on the current state turn trajectory in the event it wasn't turning enough 
  If after `stop_search` time has elapsed it hasn't found the line, will turn the opposite 
  direction for `stop_search * 2` time.
  If the line gets found, continues on; else mBot_state is updated to STOP 
*/
void findLine(){
  led_colour = L_YELLOW;
  setLED();
  Serial.println("Searching....");
  printDetails();
  bool reversed = false; // false if mBot has only searched in a single direction, true if it's reversed it's direction
  unsigned long searching_for_line = millis(); // to maintain a fixed search time within the loop

  // While the line is undetected or timer has elapsed, keep loop active
  while((SensorState)lineFinder.readSensors() == OFF_LINE){
    _loop();

    // If an obstacle is detected return to main loop to handle 
    bool obstacle_detected = sensor_front.distanceCm() < OBSTACLE_SETPOINT; 
    if (obstacle_detected) return;

    unsigned long now = millis();
    unsigned long elapsed = now - searching_for_line;
    // time_limit for the reversed search is double that of the initial search
    unsigned long time_limit = reversed ? stop_search * 2 : stop_search;

    moveBot(SPEED * 0.6, TURN_SCALE);

    /* 
      if the search time has elapsed for the inital search the if block is entered turning the
      search in the opposite direction.
      On it's first entry the time_limit gets multipled by 2 and reversed boolean is set to true
    */
    if (elapsed >= time_limit){
      // was not able to find the line after the second search, set state as STOP, return to main loop
      if (reversed){
        Serial.println("Can't locate line. Stopping");
        mBot_state = STOP;
        return;
      }
      Serial.println("\n== Switching search direction ==\n");
      mBot_state = mBot_state == LEFT ? RIGHT : LEFT; // flip the direction 
      searching_for_line = millis();
      reversed = true;
      printDetails(); 
    }
  }
  Serial.println("Exiting findLine");
  printDetails();
  Serial.println("============");
}

void avoidObstacle(){

}

void stopBot(){
  motor_l.setMotorPwm(0);
  motor_r.setMotorPwm(0);
}

void setLED(){
  if (led_colour == L_PURPLE)
    LEDs.setColor(LED_PIN,160,32,240);
  else if (led_colour == L_BLUE)
    LEDs.setColor(LED_PIN,0,51,255);
  else if (led_colour == L_RED)
    LEDs.setColor(LED_PIN,255,25,0);
  else if (led_colour == L_GREEN)
    LEDs.setColor(LED_PIN,0,255,30);
  else if (led_colour == L_YELLOW)
    LEDs.setColor(LED_PIN,255,255,0);
  else 
    LEDs.setColor(LED_PIN,0,0,0);
  LEDs.show();
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

  // If an obstacle is detected, stop and wait for it to be cleared
  bool obstacle_detected = sensor_front.distanceCm() < OBSTACLE_SETPOINT; 
  if (obstacle_detected){
    led_colour = L_RED;
    mBot_state = STOP;
    setLED();
    stopBot();
    return;
  }

  // This if block is triggered after findLine has unsuccesfully found the line
  if(sensor_state == OFF_LINE && mBot_state == STOP){
    led_colour = L_RED;
    setLED();
    stopBot();
    return;
  }

  // The if statements in each case were only implemented to make it easier to read terminal logs
  switch(sensor_state){
    case ON_LINE:
      if (mBot_state != STRAIGHT){
        mBot_state = STRAIGHT;
        led_colour = L_GREEN; 
        scaler = 1.0; 
      }
      break;

    case RIGHT_OUT:
      if (mBot_state != LEFT){
        mBot_state = LEFT;
        led_colour = L_BLUE; 
        scaler = TURN_SCALE; 
      }
      break;

    case LEFT_OUT:
      if (mBot_state != RIGHT){
        mBot_state = RIGHT;
        led_colour = L_PURPLE; 
        scaler = TURN_SCALE; 
      }
      break;

    case OFF_LINE:
      mBot_state = SEARCH;
      findLine();
      break;
  }
  
  moveBot(SPEED, scaler);
  setLED();
}
