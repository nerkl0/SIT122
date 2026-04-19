#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);

const float CALIBRATION = 110.0 / 128.0;

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

void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_r.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  _loop();
  
  for (int i=0; i < 4; i++){
    move(1, 130);
    _delay(2);
    move(3, 200);
    _delay(0.5);
  }
}

void _loop() {
  motor_r.loop();
  motor_l.loop();
}