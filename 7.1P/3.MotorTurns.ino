#include <Arduino.h>
#include <MeAuriga.h>

MeGyro gyro_0(0, 0x69);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed led(0,12);

const float CALIBRATION = 114.0 / 128.0;

const int CLOCKWISE = 3; 
const int ANTICLOCKWISE = 4; 

const int RPM_SLOW = 100;
const int RPM_FAST = 160;
const int DEGREE = 55; 

const int FAST_DEGREE = 40;

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

void _delay(float seconds)
{
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
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

void stopMotors()
{
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(1);
}

void turnBot(int direction, float targetAngle, int speed)
{
  float startAngle = gyro_0.getAngle(3);

  int leftPower = (direction == CLOCKWISE) ? speed * CALIBRATION: -speed *CALIBRATION;
  int rightPower = (direction == CLOCKWISE) ? speed: -speed;
 
  Encoder_1.setTarPWM(leftPower);
  Encoder_2.setTarPWM(rightPower);

  while (abs(gyro_0.getAngle(3) - startAngle) < targetAngle) {
    _loop();
  }
  stopMotors();
}

void setup()
{
  gyro_0.begin();
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
  gyro_0.update();
  Encoder_1.loop();
  Encoder_2.loop();
}

void loop()
{
  setLED(true);
  for(int i = 0; i < 4; i++){
    turnBot(CLOCKWISE, DEGREE, RPM_SLOW);
    setLED(true);
  }
  setLED(false);

  _delay(1);

  setLED(true);
  for(int i = 0; i < 4; i++){
    turnBot(ANTICLOCKWISE, DEGREE, RPM_SLOW);
    setLED(true);
  }
  setLED(false);

  _delay(1);

  setLED(true);
  for(int i = 0; i < 4; i++){
    turnBot(CLOCKWISE, FAST_DEGREE, RPM_FAST);
    setLED(true);
  }
  setLED(false);

  _delay(1);

  setLED(true);
  for(int i = 0; i < 4; i++){
    turnBot(ANTICLOCKWISE, FAST_DEGREE, RPM_FAST);
    setLED(true);
  }
  setLED(false);

  _delay(1);

}