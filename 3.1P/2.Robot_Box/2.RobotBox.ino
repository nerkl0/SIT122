#include <Arduino.h>
#include <MeAuriga.h>

MeUltrasonicSensor ultrasonic_sensor(10);
MeRGBLed rgb_led(0, 12);

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);

const float CALIBRATION = 110.0 / 128.0;

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

void move(int direction, int speed)
{
  int leftSpeed = 0; 
  int rightSpeed = 0;

  switch(direction){
    case 1:
      leftSpeed = speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 2:
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = speed;
      break;
    case 3:
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 4:
      leftSpeed = speed * CALIBRATION;
      rightSpeed = speed;
      break;
  }

  motor_r.setTarPWM(rightSpeed);
  motor_l.setTarPWM(leftSpeed);
}

void toggle_LED(int colour){
  if (colour == 1) 
    rgb_led.setColor(3,255,0,0);    // set to red
  else if (colour == 2) 
    rgb_led.setColor(3,50,255,0);   // set to green

  rgb_led.show();
}

void setup() {
  rgb_led.setpin(44);
  
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_r.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  _loop();
  
  if (ultrasonic_sensor.distanceCm() < 25) {
    toggle_LED(1);
    move(3, 180);
    } else {
    move(1, 150);
    toggle_LED(2);
  }
}

void _loop() {
  motor_r.loop();
  motor_l.loop();
}