#include <Arduino.h>
#include <MeAuriga.h>

MeGyro gyro_0(0, 0x69);
MeUltrasonicSensor sensor_front(9);
MeUltrasonicSensor sensor_side(10);
MeBuzzer buzzer;
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed LEDs(0, 12);

const int SPEED = 100;
const float KP = 1.2;
const float CALIBRATION = 0.83;

const float WALL_SETPOINT = 15.0;
const float MAX_WALL_SETPOINT = 30.0;
const float OBSTACLE_DISTANCE = 20.0;
const float OBSTACLE_MIN_DISTANCE = 10.0;
const float TURN_ANGLE = 75; 
const float TURN_SPEED = SPEED * 0.5;

const float BUZZ_FREQ = 440; 

enum Direction { CLOCKWISE, ANTICLOCKWISE };
enum LEDColour { OFF, ORANGE, PINK, BLUE, RED, GREEN };
enum State { AVOID, FIND, FOLLOW, REVERSE };
State mBotState = FOLLOW;

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

// Non blocking delay
void _delay(float seconds) {
  if(seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}
\

void setLED(LEDColour colour){
  if (colour == ORANGE)
    LEDs.setColor(3,255,154,0);
  else if (colour == PINK)
    LEDs.setColor(3,228,81,203);
  else if (colour == BLUE)
    LEDs.setColor(3,0,51,255);
  else if (colour == RED)
    LEDs.setColor(3,255,25,0);
  else if (colour == GREEN)
    LEDs.setColor(3,0,255,0);
  else 
    LEDs.setColor(3,0,0,0);
  LEDs.show();
}

/*
  Function to call to stop motors
*/
void stopBot(){
  motor_l.setMotorPwm(0);
  motor_r.setMotorPwm(0);
  _delay(0.5);
}

/*
  Controls turning the mBot using the gyroscope
  direction is enum of CLOCKWISE, ANTICLOCKWISE
  targetAngle is degree to turn
*/
void turnBot(Direction direction, float targetAngle, int speed)
{
  gyro_0.update();
  float startAngle = gyro_0.getAngle(3);

  int leftPower = (direction == CLOCKWISE) ? speed * CALIBRATION : -speed  * CALIBRATION;
  int rightPower = (direction == CLOCKWISE) ? speed: -speed;
 
  motor_l.setMotorPwm(leftPower);
  motor_r.setMotorPwm(rightPower);
  
  while (abs(gyro_0.getAngle(3) - startAngle) < targetAngle) {
    gyro_0.update();
    _loop();
  }
  stopBot();
}

void setup() {
  Serial.begin(115200);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  LEDs.setpin(44);
  LEDs.show(); 
  
  gyro_0.begin();
  buzzer.setpin(45);

  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  _loop();  
  switch(mBotState){
    
    /*
      State: AVOID
      Turns robot away from the wall -> moves along the length of the obstacle -> 
      turns clockwise to be back linear with the wall. 
      Updates state to FIND 
    */
    case AVOID: {
      setLED(GREEN);
      
      // If obstacle is too close, switch to REVERSE
      if (sensor_front.distanceCm() < OBSTACLE_MIN_DISTANCE) {
        mBotState = REVERSE;
        return;
      }
      
      turnBot(ANTICLOCKWISE, TURN_ANGLE, TURN_SPEED);
      while (sensor_side.distanceCm() < MAX_WALL_SETPOINT) {
        motor_l.setMotorPwm(SPEED * CALIBRATION);
        motor_r.setMotorPwm(-SPEED);
      }

      turnBot(CLOCKWISE, TURN_ANGLE, TURN_SPEED);
  
      mBotState = FIND; 
      break;
    }

    /*
          
    */
    case FIND: {
      setLED(BLUE);
      
      while (sensor_side.distanceCm() > MAX_WALL_SETPOINT) {
        
        if (sensor_front.distanceCm() < OBSTACLE_DISTANCE){
          stopBot();
          mBotState = AVOID;
          return;
        }      

        motor_l.setMotorPwm(SPEED);
        motor_r.setMotorPwm(-SPEED * 0.7);
      }
      mBotState = FOLLOW;
      break;
    }


    case FOLLOW:{
      if (sensor_front.distanceCm() < OBSTACLE_DISTANCE){
        stopBot();
        mBotState = AVOID;
        return;
      }

      float side_sensor = sensor_side.distanceCm();
      if (side_sensor > MAX_WALL_SETPOINT){
        stopBot();
        mBotState = FIND;
        return;
      }

      float error = WALL_SETPOINT - side_sensor; 
      float correction = constrain(KP * error, -SPEED, SPEED);

      motor_l.setMotorPwm((SPEED - correction) * CALIBRATION);
      motor_r.setMotorPwm(-( SPEED + correction));

      if (error > 0)
        setLED(PINK);
      else if (error < 0)
        setLED(ORANGE);
      break;
    }

    case REVERSE: 
      setLED(RED);
      while (sensor_front.distanceCm() < OBSTACLE_DISTANCE) {
        motor_l.setMotorPwm(-SPEED * CALIBRATION);
        motor_r.setMotorPwm(SPEED);
        buzzer.tone(BUZZ_FREQ, 250);
        _delay(0.5);
      }

      mBotState = FOLLOW; 
      break;
  }
}