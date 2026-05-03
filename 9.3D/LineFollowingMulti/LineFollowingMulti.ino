#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeLineFollower lineFinder(10);
MeRGBLed LEDs(0, 12);

const int SPEED = 60;
const float CALIBRATION = 0.8;
const float TURN_SCALE = 1.3;
float scaler = 1.0;

enum RobotState { FORWARD, LEFT, RIGHT };
enum SensorState { ON_LINE, RIGHT_OUT, LEFT_OUT, OFF_LINE };

RobotState mBot_state = FORWARD;
SensorState sensor_state = ON_LINE; 

const int LED_PIN = 3;
const int WHITE = 1;  
const int BLACK = 2; 
bool whiteLine = true; 

unsigned long lastCooldown = 0;
const long COOLDOWN = 500;

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
  moveBot powers motors, if a scale multiplier is greater than 1
  it will apply it to the current states respective wheel.   
*/
void moveBot(){
  int leftMotor = SPEED * CALIBRATION;
  int rightMotor = SPEED; 
  
  if (mBot_state == LEFT){
    motor_l.setMotorPwm(-leftMotor * scaler);
    motor_r.setMotorPwm(-rightMotor * scaler);
  } else if (mBot_state == RIGHT){
    motor_l.setMotorPwm(leftMotor  * scaler);
    motor_r.setMotorPwm(rightMotor * scaler);
  } else {
    motor_l.setMotorPwm(leftMotor);
    motor_r.setMotorPwm(-rightMotor);
  }
}

/*
  Reads line sensors returning the readings while adjusting for black/white line
  On a black line return readings are ON_LINE = 0, RIGHT_OUT = 1, LEFT_OUT = 2, OFF_LINE = 3
  For whiteline the value of the index is subtracted from 3 inverting the reading. 
    ON_LINE: 0 = [3-0] -> 3
    RIGHT_OUT: 1 = [3-1] -> 2 
    LEFT_OUT: 2 = [3-2] -> 1 
    OFF_LINE: 3 = [3-3] -> 0
*/
int readLightSensor(){
  int reading = lineFinder.readSensors();
  if (whiteLine) 
    reading = 3 - reading;
  return reading;
}

void setLED(int colour){
  if (colour == WHITE)
    LEDs.setColor(LED_PIN, 180, 120, 60);
  else if (colour == BLACK)
    LEDs.setColor(LED_PIN,20, 20, 20); 
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
  sensor_state = (SensorState)readLightSensor(); 

  // The if statements in each case are only implemented to make it easier to read terminal logs
  switch(sensor_state){
    case ON_LINE:
      if (mBot_state != FORWARD){
        mBot_state = FORWARD;
        scaler = 1.0; 
      }
      break;

    case RIGHT_OUT:
      if (mBot_state != LEFT){
        mBot_state = LEFT;
        scaler = TURN_SCALE;
      }
      break;

    case LEFT_OUT:
      if (mBot_state != RIGHT){
        mBot_state = RIGHT;
        scaler = TURN_SCALE; 
      } 
      break;

    case OFF_LINE:
    // Inverts whiteLine. Cooldown is applied every time whiteLine is switched to stop it retriggering on 
    // the next loop   
      if (millis() - lastCooldown > COOLDOWN) {
        lastCooldown = millis();
        whiteLine = !whiteLine;
        scaler = 1.0;

        setLED(whiteLine ? WHITE : BLACK);
      }
      break;
  }

  moveBot();
  _delay(0.06); // slight delay for smoother turning 
}