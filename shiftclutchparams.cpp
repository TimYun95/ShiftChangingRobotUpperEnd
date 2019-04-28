#include "shiftclutchparams.h"

int ShiftClutchParams::shiftclutchState = 0;
unsigned int ShiftClutchParams::shiftState = 0;
unsigned int ShiftClutchParams::clutchState = 0;

double ShiftClutchParams::totalTime = 0;
double ShiftClutchParams::partialTime[5] = {0, 0, 0, 0, 0};
