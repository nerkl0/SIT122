#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeLineFollower lineFinder(10);
MeRGBLed LEDs(0, 12);

const int LED_PIN = 3; 
const int SPEED = 60;
const float CALIBRATION = 0.9;

const float TURN_SCALE = 0.13;
const float TURN_MIN = 0.1;
const float TURN_MAX = 0.02;
const float RAMP_TIME = 1500;
unsigned long turn_start = 0;
unsigned long stop_search = 1500;
unsigned long searching_for_line = 0;
float scaler = 1.0;

unsigned long last_print_time = 0;
unsigned long last_find_print = 0; 
unsigned long print_cooldown = 1000; 

enum RobotState { STRAIGHT, LEFT, RIGHT, STOP };
enum LEDState { NONE, L_RED, L_BLUE, L_PURPLE, L_GREEN, L_YELLOW };
enum SensorState { ON_LINE, RIGHT_OUT, LEFT_OUT, OFF_LINE };
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

void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void _loop(){
  motor_l.loop();
  motor_r.loop();
}

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
float calculateScaler(){
  unsigned long now = millis() - turn_start;
  float time = now / RAMP_TIME;
  if (time > 1.0) time = 1.0;
  
  return TURN_MIN + time * (TURN_MAX - TURN_MIN);
}

void moveBot(float scale){
  int leftMotor = SPEED * CALIBRATION;
  int rightMotor = SPEED; 
  
  if (mBot_state == LEFT) rightMotor *= scale; 
  else if (mBot_state == RIGHT) leftMotor *= scale;
  
  motor_l.setMotorPwm(leftMotor);
  motor_r.setMotorPwm(-rightMotor);
}

void findLine(){
  Serial.println("Searching....");
  searching_for_line = millis();
  turn_start = searching_for_line;

  RobotState search_dir = mBot_state;
  unsigned long time_limit = stop_search;
  bool flipped = false;

  while((SensorState)lineFinder.readSensors() == OFF_LINE){
    _loop();
    unsigned long now = millis();
    unsigned long elapsed = now - searching_for_line;

    mBot_state = search_dir;
    moveBot(calculateScaler());
    printTime(500, "Time: ", elapsed);

    if (elapsed >= time_limit){
      if (flipped){
        Serial.println("Can't locate line. Stopping");
        mBot_state = STOP;
        return;
      }
      Serial.println("\n== Switching search direction ==\n");
      search_dir = mBot_state == LEFT ? RIGHT : LEFT;
      searching_for_line = millis();
      turn_start = searching_for_line;
      time_limit = stop_search * 2;
      flipped = true;
    }
  }
  mBot_state = search_dir;
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

void printDetails(){
  Serial.print("Sensor "); Serial.print(sensor_state); Serial.print(": "); Serial.println(sensorState());
  Serial.print("State "); Serial.print(mBot_state); Serial.print(": "); Serial.print(stateName());
  Serial.print("\t\tScaler: "); Serial.println(scaler);
  Serial.print("LED: "); Serial.println(ledName());
  Serial.println();
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

void printTime(int cooldown, const char* str, unsigned long time){
  unsigned long now = millis();
  if (now - last_print_time >= cooldown){
    last_print_time = now;
    Serial.print(str); Serial.println(time);
  } 
}
void loop() {
  _loop(); 

  sensor_state = (SensorState)lineFinder.readSensors(); 
  if(sensor_state == OFF_LINE && mBot_state == STOP){
    led_colour = L_RED;
    setLED();
    stopBot();
    return;
  }
  switch(sensor_state){
    case ON_LINE:
      mBot_state = STRAIGHT; 
      led_colour = L_GREEN; 
      scaler = 1.0;
      break;
    case RIGHT_OUT:
      if (mBot_state != RIGHT) turn_start = millis();
      mBot_state = RIGHT;
      led_colour = L_BLUE; 
      scaler = TURN_SCALE;
      break;
    case LEFT_OUT:
      if (mBot_state != LEFT) turn_start = millis();
      mBot_state = LEFT;
      led_colour = L_PURPLE; 
      scaler = TURN_SCALE;
      break;
    case OFF_LINE:
      led_colour = L_YELLOW;
      setLED();
      findLine();
      break;
  }
  
  moveBot(scaler);
  setLED();

  unsigned long now = millis();
  if (now - last_print_time >= print_cooldown){
    last_print_time = now;
    printDetails();
  }
}
