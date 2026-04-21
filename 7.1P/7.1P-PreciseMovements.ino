#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed led(0,12);

const float WHEEL_DIAMETER = 7.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float WHEEL_DISTANCE = 13.4;
const float TURN_CIRCUMFERENCE = WHEEL_DISTANCE * PI;
const float PULSES_PER_REV = 370;
const float PULSES = PULSES_PER_REV / WHEEL_CIRCUMFERENCE;

const float CALIBRATION = 116.0 / 128.0;

const int RPM = 178;
const int DEGREE = 162;

const int FORWARD = 1;
const int BACKWARD = 2;
const int CLOCKWISE = 3; 
const int ANTICLOCKWISE = 4; 

const int DISTANCE = 100;

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0) Encoder_1.pulsePosMinus();
  else Encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0) Encoder_2.pulsePosMinus();
  else Encoder_2.pulsePosPlus();
}

// Non-blocking delay for encoder motors
void _delay(float seconds)
{
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

/*
  If power is true: Randomised LED colours and the LED to light up 
  (Further implementation would store an array of already lit LEDs so random() to avoid the same LED num returning twice if already on)
  if power = false, LEDs are switched off 
*/
void setLED(bool power){
  if (power) {
    uint8_t r = random(0, 256);
    uint8_t g = random(0, 256);
    uint8_t b = random(0, 256);
    
    int l = random(1,13);
    
    led.setColor(l, r, g, b);
  } else {
    led.setColor(0, 0, 0, 0);
  }
  led.show();
}

void stopMotors()
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  _delay(1);
}

/*
  Handles Forward/Backwards direction of the mBot
  target: the distance in cms multiplied by the number of pulses in a single revolution
  sets current position then continuously compares current position while moving, once target has been reached, stopMotors 
*/
void moveBot(int direction, float dist, float speed){
  long target = dist * PULSES;

  float startL = Encoder_1.getCurPos();
  float startR = Encoder_2.getCurPos();

  int leftPower = (direction == FORWARD) ? speed * CALIBRATION: -speed * CALIBRATION;
  int rightPower = (direction == FORWARD) ? -speed : speed;

  Encoder_1.setMotorPwm(rightPower);
  Encoder_2.setMotorPwm(leftPower);

  while (abs(Encoder_1.getCurPos() - startL) < target || abs(Encoder_2.getCurPos() - startR) < target)
    _loop();

  stopMotors();
}

/*
  Handles robot turning using motorPwm.
  wheelArc: fraction of entire turning circle; pulses: number of pulses in the turn based on the degree
  direction: determines CLOCKWISE or ANTICLOCKWISE for leftPower / rightPower
  While loop triggers motor turn stopping once the current position of the robot is less than the pulses
*/
void turnBot(int direction, float targetAngle, int speed)
{
  float wheelArc = (targetAngle / 360.0) * TURN_CIRCUMFERENCE;
  float target = wheelArc * PULSES;

  float startL = Encoder_1.getCurPos();
  float startR = Encoder_2.getCurPos();

  int leftPower = (direction == CLOCKWISE) ? speed * CALIBRATION : -speed * CALIBRATION;
  int rightPower = (direction == CLOCKWISE) ? speed: -speed;
 
  Encoder_1.setMotorPwm(leftPower);
  Encoder_2.setMotorPwm(rightPower);

  while (abs(Encoder_1.getCurPos() - startL) < target || abs(Encoder_2.getCurPos() - startR) < target)
    _loop();

  stopMotors();
}

void setup()
{
  led.setpin(44);
  led.show();   
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
}

void _loop()
{
  Encoder_1.loop();
  Encoder_2.loop();
}

void loop()
{
  // moves mBot in 1m square
  for (int i=0; i<4; i++){
    setLED(true);
    moveBot(FORWARD, DISTANCE, RPM);
    turnBot(CLOCKWISE, DEGREE, RPM);
  }
  setLED(false);
  _delay(1);
}