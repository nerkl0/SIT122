#include <Arduino.h>
#include <MeAuriga.h>

MeLightSensor light_sensor(12);
MeUltrasonicSensor ultrasonic_sensor(10);
MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeRGBLed LEDs(0, 12);

const int SPEED = 120;
const float KP = 1.4; // Proportional Gain
const float KD = 11.0; // Derivative Gain

// variables for derivative calculation
unsigned long prev_time = 0;
float prev_error = 0; 

const float CALIBRATION = 0.85; // offest for left wheel being more powerful than right
const float MIN_SETPOINT = 15.0;
const float MAX_SETPOINT = 25.0;
float setpoint = MIN_SETPOINT;

const int BRIGHT_LIGHT = 400;

// Cooldown for switch setpoint logic
const unsigned long MIN_COOLDOWN = 1200;
unsigned long last_switch_time = 0;
bool switched = false;

void isr_process_encoder1(void){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}
void isr_process_encoder2(void){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}

void set_LED(int colour){
  if (colour == 1)
    LEDs.setColor(3,255,154,0); // orange
  else if (colour == 2)
    LEDs.setColor(3,228,81,203); // pink
  else if (colour == 3)
    LEDs.setColor(3,0,51,255); // blue
  else 
    LEDs.setColor(3,0,0,0);
  LEDs.show();
}

/*
  Adjusts the setpoint between minimum and maximum
  last_switch_time sets the cooldown length
  prev_error resets to account for the change in setpoint to prevent the derivate value 
  from spiking on the next loop
*/
void switch_setpoint(){
  setpoint = setpoint == MIN_SETPOINT ? MAX_SETPOINT : MIN_SETPOINT;
  last_switch_time = millis();
  prev_error = setpoint - ultrasonic_sensor.distanceCm();
  switched = true;
}

void _loop(){
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

  int lightSensor = light_sensor.read();

  // Calculates how long the last iteration took against current time
  // required for derivative calculation for the rate of error change
  unsigned long now = millis();
  float time = (now - prev_time) / 1000.0;
  prev_time = now;

  float distance = ultrasonic_sensor.distanceCm();
  float error = setpoint - distance;
  
  // Derivative assists with a smoother transition if a large jump in error values from 
  // the previous reading is returned. Helping maintain the steady state of the robot when 
  // shifting between setpoints
  float derivative = (error - prev_error) / time;
  float correction = KP * error + KD * derivative;
  prev_error = error; // update prev_error for comparison on next loop

  // Constrains speed of the correction to the set speed
  correction = constrain(correction, -SPEED, SPEED);

  motor_l.setMotorPwm((SPEED - correction) * CALIBRATION);
  motor_r.setMotorPwm(-( SPEED+ correction));

  // If bright light detected and not in cooldown, switch setpoint and trigger LED
  // switched flag prevents multiple triggers from a single light fluctuation
  if (lightSensor > BRIGHT_LIGHT && !switched) {
    set_LED(3);
    switch_setpoint();
  }


  // Handles LED feedback. Blue LED maintained during cooldown period after a setpoint switch
  // Once cooldown ends, switched resets to allow future light triggers
  // Otherwise when not in cooldown, updates LED based on error direction
  if (switched){
    if (millis() - last_switch_time > MIN_COOLDOWN)
      switched = false;
  } else {
    if (error > 0)
      set_LED(2);
    else if (error < 0)
      set_LED(1);
  }

}