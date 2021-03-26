#ifndef  DRIVER_H
#define  DRIVER_H



#include "turle_driver/communicate.h"
#include<PID_Beta6.h>





class Motor: public PID
{
private:
    /* data */
public:
    Motor(/* args */);
    ~Motor();

    float kp;
    float ki;
    float kd;

    uint16_t lastEncoder;
    uint16_t nowEncoder;

    uint16_t pwm;
    uint16_t speedInput;
    uint16_t speedOutput;
    uint16_t speedDesired;

    float speed2DutyCycle;
};



#endif
