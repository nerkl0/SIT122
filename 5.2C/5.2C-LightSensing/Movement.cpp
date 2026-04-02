#include "Movement.h"

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);

const float CALIBRATION = 110.0 / 128.0;

void isr_process_encoder1(){
  if(digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void isr_process_encoder2(){
  if(digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

void move(int direction, int speed){
  int leftSpeed = 0; 
  int rightSpeed = 0;

  switch(direction){
    case 1: // forward
      leftSpeed = speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 2: // backward
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = speed;
      break;
    case 3: // right
      leftSpeed = -speed * CALIBRATION;
      rightSpeed = -speed;
      break;
    case 4: // left
      leftSpeed = speed * CALIBRATION;
      rightSpeed = speed;
      break;
  }

  motor_r.setTarPWM(rightSpeed);
  motor_l.setTarPWM(leftSpeed);
}

void _loop(){
  motor_r.loop();
  motor_l.loop();
}

void stopMotors(){
  motor_r.setTarPWM(0);
  motor_l.setTarPWM(0);
}
