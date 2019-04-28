#ifndef SHIFTCLUTCHPARAMS_H
#define SHIFTCLUTCHPARAMS_H


class ShiftClutchParams
{
public:
    static int shiftclutchState;
    static unsigned int shiftState;
    static unsigned int clutchState;

    static double totalTime;
    static double partialTime[5];
};

#endif // SHIFTCLUTCHPARAMS_H
