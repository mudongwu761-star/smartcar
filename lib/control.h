#ifndef CONTROL_H
#define CONTROL_H

#include <unistd.h>

void ControlInit();
void ControlMain();
void ControlPause();
void ControlExit();
void ResetTraveledDistance();

extern double mortor_kp;
extern double mortor_ki;
extern double mortor_kd;

#endif