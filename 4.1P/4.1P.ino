#include <Arduino.h>
#include <MeAuriga.h>

MeLightSensor lightSensor(12);
MeBuzzer buzzer;
MeSoundSensor soundSensor(14);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

const int LED_COUNT = 13;
MeRGBLed rgbLED(0, LED_COUNT);
uint32_t colours[] = {
  0xFF0000,
  0x0000FF,
  0xFFFFFF
};

uint32_t coloursLength = sizeof(colours) / sizeof(colours[0]);
float sound_frequency = 0;

// Buzzer structure to (kind of) sound like the Aussie Aussie Aussie Oi Oi Oi chant
#define G4 392
#define E4 330
#define A4 440
int melody[] = { G4, A4, G4, A4, G4, A4, E4, E4, E4 };
int noteDurations[] = { 250, 100, 250, 100, 250, 100, 200, 200, 200 };
int noteDelays[] = { 10, 100, 10, 100, 10, 500, 80, 80, 80 };

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) { Encoder_1.pulsePosMinus(); }
  else { Encoder_1.pulsePosPlus(); }
}

void isr_process_encoder2(void) { 
  if (digitalRead(Encoder_2.getPortB()) == 0) { Encoder_2.pulsePosMinus(); }
  else { Encoder_2.pulsePosPlus(); }
}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

// handles non blocking delay to ensure play() and lightUpLed() doesn't block the wheels from spinning
void _delay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { _loop(); }
}

// handles LEDs. Each LED flashes one after the other. 
// repeat cycles are based on the argument
void lightUpLed(int repeat) {
  int colourTracker = 0;
  int count = 0;

  while (count < repeat) {
    for (int i = 1; i < LED_COUNT; i++) {
      rgbLED.setColor(i, colours[colourTracker]);
      rgbLED.show();

      _delay(50);

      rgbLED.setColor(i, 0x000000);
      rgbLED.show();

      // updates colourTracker back to the first element after each colour has been displayed
      colourTracker = (colourTracker + 1) % coloursLength;
    }
    count++;
  }
}

// plays the chant based on the set melody.
void play() {
  int len = sizeof(melody) / sizeof(melody[0]);
  for (int i = 0; i < len; i++) {
    buzzer.tone(melody[i], noteDurations[i]);
    _delay(noteDelays[i]);
  }
}

// controls robot spinning in a rightwards circle
void spinRight(int speed) {
  Encoder_1.setTarPWM(speed);
  Encoder_2.setTarPWM(speed);
}

void setup() {
  buzzer.setpin(45);
  rgbLED.setpin(44);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
}

void loop() {
    _loop();
  sound_frequency = soundSensor.strength();
  if (sound_frequency > 550) {
    spinRight(200);       // wheels start spinning
    lightUpLed(2);        // LEDs complete 2 full rotations
    play();               // play the chant
    lightUpLed(2);        // another 2 rotations for LEDs
    spinRight(0);         // stop the motors spinning
    delay(1000);          // pause 1 second to stop the residual motor noise from triggering the sound_frequency
  }
}