#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>

// ultrasonic pins
#define TRIG_F 16
#define ECHO_F 17

#define TRIG_FL 43
#define ECHO_FL 18

#define TRIG_FR 44
#define ECHO_FR 21

extern uint32_t us_f;
extern uint32_t us_fl;
extern uint32_t us_fr;
#endif