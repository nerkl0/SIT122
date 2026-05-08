#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor ultrasonic_sensor(9);
MeRGBLed LED(0, 12);
MeGyro gyro(0, 0x69);

const int SPEED = 100;
const int TURN_SPEED = 60;

const int FORWARD = 1;
const int BACKWARD = 2;
const int CLOCKWISE = 3; 
const int ANTICLOCKWISE = 4; 

const int SEEK_TIMEOUT = 10000; 
float object_setpoint = 0;

float target = 0; 
float lMotorPos = 0; 
float rMotorPos = 0;

struct botConfig {
  const float WHEEL_DIAMETER = 7.0;
  const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
  const float WHEEL_DISTANCE = 13.4;
  const float TURN_CIRCUMFERENCE = WHEEL_DISTANCE * PI;
  const float PULSES_PER_REV = 370;
  const float PULSES = PULSES_PER_REV / WHEEL_CIRCUMFERENCE;
  const float CALIBRATION = 0.8;
} mBot;

enum RobotState { SEEK, TRACK, LOST };
RobotState mBot_state = SEEK; 

void isr_process_motorR(void){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void isr_process_motorL(void){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

// Non-blocking delay for encoder motors
void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}
void _loop(){
  gyro.update();
  motor_r.loop(); motor_l.loop();
}

void stopBot(){
  motor_r.setMotorPwm(0); motor_l.setMotorPwm(0);
  _delay(1);
}

/*
  Handles Forward/Backwards direction of the mBot
  target: the distance in cms multiplied by the number of pulses in a single revolution
  sets current position then continuously compares current position while moving, once target has been reached, stopMotors 
*/
void moveBot(int direction, float speed){
  int leftPower = (direction == FORWARD) ? speed * mBot.CALIBRATION: -speed * mBot.CALIBRATION;
  int rightPower = (direction == FORWARD) ? -speed : speed;

  motor_r.setMotorPwm(rightPower);
  motor_l.setMotorPwm(leftPower);
}

/*
  Calculates pulses to target; sets current position of mBot motors and direction
  Triggers movement towards target
*/
void turnBot(int dir, float a, int s){
  float arc = (a / 360.0) * mBot.TURN_CIRCUMFERENCE;
  target = arc * mBot.PULSES;

  rMotorPos = motor_r.getCurPos();
  lMotorPos = motor_l.getCurPos();

  float lWheelDir = (dir == CLOCKWISE) ? s * mBot.CALIBRATION : -s * mBot.CALIBRATION;
  float rWheelDir = (dir == CLOCKWISE) ? s: -s;

  motor_r.setMotorPwm(rWheelDir);
  motor_l.setMotorPwm(lWheelDir);
}

/*
  Locates the closest object to itself within a single rotation.
  Once full rotation has been complete, turns back towards target that was closest 
  If the object is not reacquired, expands search radius retrying 3 times before stopping search 
*/
void acquireTarget(int direction, int rotation){
  ledShow(3,0,51,255);
  turnBot(direction, rotation, TURN_SPEED);

  object_setpoint = ultrasonic_sensor.distanceCm();
  float closestObjPulses = 0; 
  int anotherLed = 3; // For visual aid. Each time a closer object found, add an extra led

  // initial seek to locate object 
  while (abs(motor_r.getCurPos() - rMotorPos) < target){
    _loop();
    float dist = ultrasonic_sensor.distanceCm();
    // Update objectSetpoint if a closer target is found
    if (dist < object_setpoint && abs(dist - object_setpoint) > 1) {
      object_setpoint = ultrasonic_sensor.distanceCm();
      closestObjPulses = abs(motor_r.getCurPos() - rMotorPos);
      ledShow(anotherLed++,160,32,240); // turns on another LED when a closer object is found
    }
  }
  ledShow(0,0,0,0); // reset LEDs
  
  // Calculate degree needed to return to closest object 
  float returnDeg = ((target - closestObjPulses) / target) * rotation;
  int returnDir = direction == CLOCKWISE ? ANTICLOCKWISE : CLOCKWISE;
  _delay(0.5);
  seekBot(returnDir, returnDeg);
  ledShow(3, 255,255,0);
}

/* 
  Finds set target, if found return to main loop
  If it's not found after first turn, expand search radius by increasing setpoint by 1
  Continue for X attempts before stopping the search
*/
void seekBot(int direction, int degree){
  int attempts = 3;
  while (attempts > 0) {
    turnBot(direction, degree, TURN_SPEED);
    while (abs(motor_r.getCurPos() - rMotorPos) < target) {
      _loop();
      if (ultrasonic_sensor.distanceCm() <= (object_setpoint + 3)) {
        ledShow(6, 160,32,240); ledShow(3, 0,0,0); 
        stopBot();
        mBot_state = TRACK; 
        return;
      }
    }
    object_setpoint++; 
    attempts--;
    _delay(0.5);
  }
  
  mBot_state = LOST; // object has not been found 
  ledShow(0,0,0,0);
  stopBot();
}

void trackTarget(){
  float objThreshold = object_setpoint * 0.2; 
  while(ultrasonic_sensor.distanceCm() > object_setpoint + objThreshold) {
    turnBot(CLOCKWISE, 50, SPEED);
    float leftDist = ultrasonic_sensor.distanceCm();
    turnBot(ANTICLOCKWISE, 50, SPEED);
    float rightDist = ultrasonic_sensor.distanceCm();
  }
}

void ledShow(int num, int r, int g, int b){
  LED.setColor(num,r,g,b);
  LED.show();
}

void setup() {
  Serial.begin(115200);
  
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21); 
  attachInterrupt(motor_l.getIntNum(), isr_process_motorL, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_motorR, RISING);

  gyro.begin();
  LED.setpin(44);
  LED.show(); 
}

void loop() {
  _loop();

  switch (mBot_state) {
    case SEEK: 
      acquireTarget(CLOCKWISE, 360);
      break; 
    case TRACK: 
      trackTarget();
      break; 
    case LOST: 
      ledShow(3,255,0,0);
      stopBot(); 
      break;
  }
}
