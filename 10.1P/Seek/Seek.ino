#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor ultrasonic_sensor(9);
MeRGBLed LED(0, 12);
MeGyro gyro(0, 0x69);

const int SPEED = 100;
const int TURN_SPEED = 60;
const int CLOCKWISE = 3; 
const int ANTICLOCKWISE = 4; 

bool object404 = false; 

float target = 0; 
float lMotorPos = 0; 
float rMotorPos = 0;

// Struct to contain mBot turning configuration
struct botConfig {
  const float WHEEL_DIAMETER = 7.0;
  const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
  const float WHEEL_DISTANCE = 13.4;
  const float TURN_CIRCUMFERENCE = WHEEL_DISTANCE * PI;
  const float PULSES_PER_REV = 370;
  const float PULSES = PULSES_PER_REV / WHEEL_CIRCUMFERENCE;
  const float CALIBRATION = 0.8;
} mBot;


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
  motor_r.loop();
  motor_l.loop();
}

void stopBot()
{
  motor_r.setMotorPwm(0);
  motor_l.setMotorPwm(0);
  _delay(1);
}

/*
  Calculates pulses to target; sets current position of mBot motors and direction
  Triggers movement towards target
*/
void moveToTar(int dir,float a){
  float arc = (a / 360.0) * mBot.TURN_CIRCUMFERENCE;
  target = arc * mBot.PULSES;

  rMotorPos = motor_r.getCurPos();
  lMotorPos = motor_l.getCurPos();

  float lWheelDir = (dir == CLOCKWISE) ? TURN_SPEED * mBot.CALIBRATION : -TURN_SPEED * mBot.CALIBRATION;
  float rWheelDir = (dir == CLOCKWISE) ? TURN_SPEED: -TURN_SPEED;

  motor_r.setMotorPwm(rWheelDir);
  motor_l.setMotorPwm(lWheelDir);
}

void ledShow(int num, int r, int g, int b){
  LED.setColor(num,r,g,b);
  LED.show();
}

/*
  Locates the closest object to itself within a single rotation.
  Once full rotation has been complete, turns back towards target that was closest 
  If the object is not reacquired, expands search radius retrying 3 times before stopping search 
*/
void seekBot(int direction, int rotation){
  ledShow(3,0,51,255);

  object404 = false; // reset object not found 
  moveToTar(direction, rotation);

  float objectSetpoint = ultrasonic_sensor.distanceCm();
  float closestObjPulses = 0; 
  int anotherLed = 3; // For visual aid. Each time a closer object found, add an extra led

  // initial seek to locate object 
  while (abs(motor_r.getCurPos() - rMotorPos) < target){
    _loop();
    float dist = ultrasonic_sensor.distanceCm();
    // Update objectSetpoint if a closer target is found
    if (dist < objectSetpoint && abs(dist - objectSetpoint) > 1) {
      objectSetpoint = ultrasonic_sensor.distanceCm();
      closestObjPulses = abs(motor_r.getCurPos() - rMotorPos);
      ledShow(anotherLed++,160,32,240); // turns on another LED when a closer object is found
    }
  }
  ledShow(0,0,0,0); // reset LEDs
  
  // Calculate degree needed to return to closest object 
  float returnDeg = ((target - closestObjPulses) / target) * rotation;
  int returnDir = direction == CLOCKWISE ? ANTICLOCKWISE : CLOCKWISE;

  _delay(0.5);
  ledShow(3, 255,255,0);

  /* 
    Handles reseeking closest object; If found return to main loop
    If it's not found after first turn, expand search radius by increasing setpoint by 1
    Continue for X attempts before stopping the search
  */
  int attempts = 3; // stop searching for object after X attempts
  while (attempts > 0) {
    moveToTar(returnDir, returnDeg);
    while (abs(motor_r.getCurPos() - rMotorPos) < target) {
      _loop();
      if (ultrasonic_sensor.distanceCm() <= (objectSetpoint + 3)) {
        ledShow(6, 160,32,240);
        ledShow(3, 0,0,0);
        stopBot();
        return;
      }
    }
    returnDeg = rotation; // reset rotation
    objectSetpoint++; 
    attempts--;
    _delay(0.5);
  }
  
  object404 = true; // object has not been found 
  ledShow(0,0,0,0);
  stopBot();
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

  // if object not located after intial seek
  if(object404){
    ledShow(3,255,0,0);
    return;
  }

  seekBot(CLOCKWISE, 360);
}
