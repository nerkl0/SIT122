#include <Arduino.h>
#include <MeAuriga.h>

MeUltrasonicSensor ultrasonic_sensor(10);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed LEDs(0, 12);

const int SPEED = 120;
const float SETPOINT = 15.0;
const float KP = 1.2;
const float CALIBRATION = 0.85;

void isr_process_encoder1(void){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}

void isr_process_encoder2(void){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void setLED(int colour){
  if (colour == 1)
    LEDs.setColor(3,255,154,0); // orange
  else if (colour == 2)
    LEDs.setColor(3,228,81,203); // pink
  else 
    LEDs.setColor(3,0,51,255); // blue

  LEDs.show();
}

void _loop()
{
  motor_l.loop();
  motor_r.loop();
}

void setup() {
  Serial.begin(115200);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  LEDs.setpin(44);
  LEDs.show();
    
  attachInterrupt(motor_l.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_encoder2, RISING);
}

void loop() {
  _loop();
  float distance =  ultrasonic_sensor.distanceCm();
  float error = SETPOINT - distance; 
  float correction = constrain(KP * error, -SPEED, SPEED);
  
  motor_l.setMotorPwm((SPEED - correction) * CALIBRATION);
  motor_r.setMotorPwm(-(SPEED + correction));


  Serial.print("Dist: ");
  Serial.print(distance);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  Speed: ");
  Serial.println(correction);

  if (error > 0)
    setLED(2);
  else if (error < 0)
    setLED(1);
  else
    setLED(3);    
}
