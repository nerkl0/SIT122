#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <MeAuriga.h>

extern MeEncoderOnBoard motor_r;
extern MeEncoderOnBoard motor_l;

void isr_process_encoder1(void);
void isr_process_encoder2(void);
void move(int direction, int speed);
void _loop(void);
void stopMotors(void);

#endif