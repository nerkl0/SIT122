#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor ultrasonic_front(9);
MeBuzzer buzzer;
MeRGBLed LEDs(0, 12);

// Pins for HC-SR04
const int TRIG_PIN = 30;
const int ECHO_PIN = 31;

const int LED_PIN = 3;
const int BUZZ_FREQ = 440; 

const int TURN_SPEED = 45; 
const int SPEED = 60;
const float CALIBRATION = 0.9;

const int OBJECT_SETPOINT = 25; // Detectable distance to object
const int TRACK_OBJECT = 15; // Distance for mBot to keep when tracking 
const int LOST_OBJECT = 40;
const int DIST_THRESHOLD = 3;
const int REAR_SETPOINT = 15; // Distance for back sensor

enum LEDState { L_GREEN, L_RED, L_BLUE, L_PURPLE, L_YELLOW };
enum RobotState { SEEK, TRACK, OBSTRUCTED };
RobotState mBot_state = SEEK;

void isr_process_motorR(){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}
void isr_process_motorL(){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}
void _loop(){ 
  motor_r.loop();
  motor_l.loop();
}
void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

/*
  The next three functions handle mBot movement. stop, turn and move
*/
void stopBot(){
  motor_r.setMotorPwm(0);
  motor_l.setMotorPwm(0);
}

void turnBot(int speed){
  motor_r.setMotorPwm(speed);
  motor_l.setMotorPwm(speed);
}

void moveBot(int speed){
  motor_r.setMotorPwm(-speed);
  motor_l.setMotorPwm(speed * CALIBRATION);
}

/*
  Returns readings for the HC-SR04 ultrasonic sensor. Mounted on the back of the mBot.
  duration * 0.01715 converts the echo from microseconds to centimeters.  
  Sets a 30ms timeout on the pulse reading, if it comes back as 0, 400cm is returned as a reading.  
*/
float readUltrasonic(){
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration ? duration * 0.01715 : 400.0;
}

void setLED(int colour){
  if (colour == L_PURPLE)
    LEDs.setColor(LED_PIN,160,32,240);
  else if (colour == L_BLUE)
    LEDs.setColor(LED_PIN,0,51,255);
  else if (colour == L_RED)
    LEDs.setColor(LED_PIN,255,25,0);
  else if (colour == L_GREEN)
    LEDs.setColor(LED_PIN,0,255,30);
  else if (colour == L_YELLOW)
    LEDs.setColor(LED_PIN,255,255,0);
  else 
    LEDs.setColor(LED_PIN,0,0,0);
  LEDs.show();
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(motor_l.getIntNum(), isr_process_motorL, RISING);
  attachInterrupt(motor_r.getIntNum(), isr_process_motorR, RISING);
    
  LEDs.setpin(44);
  LEDs.show(); 
  buzzer.setpin(45);
}

void loop() {
  _loop();

  float frontDist = ultrasonic_front.distanceCm();
  float backDist = readUltrasonic();

  switch (mBot_state) {
    
    /*
      STATE: SEEK
      Stays seeking in a clockwise direction until an object is detected
    */
    case SEEK:
      setLED(L_YELLOW);
      turnBot(TURN_SPEED);
      if (frontDist < OBJECT_SETPOINT) {
        mBot_state = TRACK;
        stopBot();
      }
      break;

    /*
      STATE: TRACK
      Tracks mBot by initially moving closer towards it to TRACK_OBJECT distance.
      Maintains that distance by following it forwards / backwards
      Changes state to SEEK if the object disappears from view [LOST_OBJECT distance away]
      Changes state to OBSTRUCTED if the rear sensor detects an obstruction 
    */
    case TRACK:
      setLED(L_BLUE);
      // If it can no longer see the object, change state to SEEK
      if (frontDist > LOST_OBJECT) {
        mBot_state = SEEK;
        break;
      }
      // Maintain distance between the object and the mBot
      if (frontDist > TRACK_OBJECT + DIST_THRESHOLD) {
        moveBot(SPEED); // Forwards
      } else if (frontDist < TRACK_OBJECT - DIST_THRESHOLD) {
          // Rear obstruction detected 
          if (backDist < REAR_SETPOINT){
            mBot_state = OBSTRUCTED;
            break;
          }
          moveBot(-SPEED); // Reverses
      } else {
        stopBot(); // If locked on to object within correct range hold position
      }
      break;

    /*
      STATE: OBSTRUCTED
      When obstruction is detected and the object is still within distance, mBot stays in position with a buzzer alert
      If the object backs away slowly move away from the wall, change state to SEEK to re-locate the object. 
    */
    case OBSTRUCTED:
      stopBot(); 
      setLED(L_RED);
      buzzer.tone(BUZZ_FREQ, 250);
      if (ultrasonic_front.distanceCm() > LOST_OBJECT) {
        while(readUltrasonic() < REAR_SETPOINT) 
          moveBot(SPEED * 0.5);
        mBot_state = SEEK; 
      }
      break;
  }
}