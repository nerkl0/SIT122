#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard motor_r(SLOT1);
MeEncoderOnBoard motor_l(SLOT2);
MeUltrasonicSensor ultrasonic_left(9);

// Pins for HC-SR04
const int TRIG_PIN = 30;
const int ECHO_PIN = 31;

const int SPEED = 50;
const int TURN_SPEED = 40;
const float CALIBRATION = 0.9;

const int OBJECT_SETPOINT = 25; // Detectable distance to object
const int TRACK_OBJECT = 15; // Distance for mBot to keep when tracking 
const int LOST_OBJECT = 40; // Max distance for which the mBot should begin seeking object 
const int DIST_THRESHOLD = 3;
const float TRACK_THRESHOLD = 8; // Value for keeping object centered when tracking

int sensorLastDetected = 0; // Variable to hold the last known sensor to be detected
const unsigned long COOLDOWN = 1500;

enum RobotState { SEEK, TRACK };
RobotState mBot_state = SEEK;

void isr_process_motorR(){
  if (digitalRead(motor_r.getPortB()) == 0) motor_r.pulsePosMinus();
  else motor_r.pulsePosPlus();
}
void isr_process_motorL(){
  if (digitalRead(motor_l.getPortB()) == 0) motor_l.pulsePosMinus();
  else motor_l.pulsePosPlus();
}
void _delay(float seconds){
  if (seconds < 0.0) seconds = 0.0;
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}
void _loop(){
  motor_r.loop();
  motor_l.loop();
}

/*
  The next three functions handle mBot movement.
  Stop, Turn, Move. Using setMotorPwm
*/
void stopBot(){
  motor_r.setMotorPwm(0);
  motor_l.setMotorPwm(0);
}
// Pass a positive integer argument for Clockwise movement. Negative integer for Anticlockwise movement
void turnBot(int speed){
  motor_r.setMotorPwm(speed);
  motor_l.setMotorPwm(speed);
}
// Pass a positive integer argument for Forward movement. Negative integer for Backward movement
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

/*
  The most recent sensor detected to assist in tracking the object. 
  If the object is only being detected by left or right sensor, as it moves out of range of the sensor,
  sensorLastDetected is used so the mBot knows which way to turn to continue tracking.
      0:  Both left and right sensors are currently detecting the object
      1:  If only the left sensor is detecting the object 
      -1: If only the right sensor is detecting the object. 
  Sets lastSeenTime for handling the COOLDOWN in TRACK state
  Returns early if both left and right sensors are not detecting the object with no update to the history
*/
void updateLastSeen(bool leftLost, bool rightLost, unsigned long &lastSeenTime) {
  if (leftLost && rightLost) return;

  if (!leftLost && !rightLost) sensorLastDetected = 0;
  else if (!leftLost) sensorLastDetected = 1;
  else sensorLastDetected = -1;

  lastSeenTime = millis();
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
}

void loop() {
  _loop();

  float leftDist = ultrasonic_left.distanceCm();
  _delay(0.05); // 5ms delay in sensor readings to account for any crossfire activity deteted from the other sensor
  float rightDist = readUltrasonic();

  switch (mBot_state) {
    /*
      STATE: SEEK
      Stays seeking in a clockwise direction until an object is detected
      Changes state to TRACK when object found 
    */
    case SEEK:
      turnBot(TURN_SPEED);
      if (leftDist < OBJECT_SETPOINT) {
        mBot_state = TRACK;
        stopBot();
      }
      break;

    /*
      STATE: TRACK
      Uses leftLost and rightLost to handle movement of the mBot. If either is true, it means that sensor does not detect the object. 
      If both are lost and the COOLDOWN has not exceeded, it uses sensorLastDetected to know which way to turn to follow the object.

    */
    case TRACK: {
      bool leftLost  = (leftDist > LOST_OBJECT);
      bool rightLost = (rightDist > LOST_OBJECT);
      
      // History tracking to keep track of the last sensor detected. 
      static unsigned long lastSeenTime = 0;
      updateLastSeen(leftLost, rightLost, lastSeenTime);

      /*
        If both sensors have recently been lost, track in the direction of the last sensor that was detected
        If COOLDOWN has been exceeded changes state to SEEK
        If the last sensor that was detected was both sensors, the object has been lost; Change state to SEEK 
      */
      if (leftLost && rightLost) {
        if (millis() - lastSeenTime >= COOLDOWN || sensorLastDetected == 0) {
          mBot_state = SEEK;
          break;
        }
        // 1 = Left sensor was last detected turn anticlockwise. -1 = Right Sensor was last detected turn clockwise
        turnBot(sensorLastDetected == 1 ? -TURN_SPEED : TURN_SPEED);
        _delay(0.3);
        break;
      }


      // Handles movement of robot when both sensors are detecting the object. 
      // Moves towards the side that the object is drifting towards to keep the object centered between sensors  
      float diff = leftDist - rightDist;
      if (diff > TRACK_THRESHOLD)
        turnBot(TURN_SPEED); // clockwise
      else if (diff < -TRACK_THRESHOLD)
        turnBot(-TURN_SPEED); // anticlockwise
      else {
        // If the object begins to move towards/away from the robot, move in respective motion to keep tracking it at a safe distance 
        float minDist = min(leftDist, rightDist); // Take the minimum distance reading as that is the closest 
        // Object moving away, move forwards
        if (minDist > TRACK_OBJECT + DIST_THRESHOLD) {
          moveBot(SPEED); 
        } 
        // Object getting to close, move back 
        else if (minDist < TRACK_OBJECT - DIST_THRESHOLD) {
          moveBot(-SPEED);
        } 
        // Object stationary and within ideal range 
        else {
          stopBot();
        }
      }
      _delay(0.3);
      break;
    }
  }
}