#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed led(0,12);

// Wheel setup
const float WHEEL_DIAMETER = 6.9;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; 
const float PULSES_PER_REV = 370;
const float PULSES = PULSES_PER_REV / WHEEL_CIRCUMFERENCE;

const float CALIBRATE_F = 114.0 / 128.0; 
const float CALIBRATE_B = 110.0 / 128.0;

// Movement
const int FORWARD = 1;
const int BACKWARD = 2;

const int DISTANCE = 100;
const int RPM_FAST = 180;
const int RPM_SLOW = 100;

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void isr_process_encoder1(void)
{
  if(digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void isr_process_encoder2(void)
{
  if(digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

void stop(){
  motor_r.setTarPWM(0);
  motor_l.setTarPWM(0);
}

void moveBot(int direction, float dist, float rpm){
  long target = dist * PULSES;
  
  int leftPower = (direction == FORWARD) ?  rpm * CALIBRATE_F: -rpm;
  int rightPower = (direction == FORWARD) ? -rpm: rpm * CALIBRATE_B;

  motor_l.setTarPWM(leftPower);
  motor_r.setTarPWM(rightPower);

  // pause briefly to give a chance for motors to complete their movement
  long startPos = motor_r.getCurPos();
  while (abs(motor_r.getCurPos() - startPos) < (long)target) {
    _loop();
  }
  stop();
}

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

void cycle(int speed, int pause){
  setLED(true);
  moveBot(FORWARD, DISTANCE, speed);

  setLED(false);
  _delay(pause);

  setLED(true);
  moveBot(BACKWARD, DISTANCE, speed);

  setLED(false);
  _delay(pause);
}

void setup() {
  led.setpin(44);
  led.show();   
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_r.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  cycle(RPM_FAST, 1);
  cycle(RPM_SLOW, 1);
}

void _loop() {
  motor_r.loop();
  motor_l.loop();
}