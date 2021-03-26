#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>


class PID
{
    public:

        #define AUTOMATIC	1
        #define MANUAL	0
        #define P_ON_M 0
        #define P_ON_E 1

        PID(int*, int*, int*,        // * constructor.  links the PID to the Input, Output, and 
            double, double, double);     //   Setpoint.  Initial tuning parameters are also set here

        PID(int*, int*, int*,        // * constructor.  links the PID to the Input, Output, and 
            double, double, double, int);//   Setpoint.  Initial tuning parameters are also set here.
                                            //   (overload for specifying proportional mode)

        void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

        bool Compute();                       // * performs the PID calculation.  it should be
                                            //   called every time loop() cycles. ON/OFF and
                                            //   calculation frequency can be set using SetMode
                                            //   SetSampleTime respectively

        void SetOutputLimits(int, int); // * clamps the output to a specific range. 0-MAX PWM by default, but
                                                                //   it's likely the user will want to change this depending on
                                                                //   the application
    //available but not commonly used functions ********************************************************
        void SetTunings(double, double,       // * While most users will set the tunings once in the 
                        double);         	    //   constructor, this function gives the user the option
                                            //   of changing tunings during runtime for Adaptive control

        void SetTunings(double, double,       // * overload for specifying proportional mode
                        double, int);     

        void SetSampleTime(double );              // * sets the frequency, in seconds, with which 
                                            //   the PID calculation is performed.
        double GetKp();
        double GetKi();
        double GetKd();
        double GetSampleTime();
        int GetMode();
        void debuggerPID();
    private:
        void Initialize();

        double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
        double dispKi;				//   format for display purposes
        double dispKd;				//

        double kp;                  // * (P)roportional Tuning Parameter
        double ki;                  // * (I)ntegral Tuning Parameter
        double kd;                  // * (D)erivative Tuning Parameter

        int pOn;

        int *myInput;              // * Pointers to the Input, Output, and Setpoint variables
        int *myOutput;             //   This creates a hard link between the variables and the 
        int *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                        //   what these values are.  with pointers we'll just know.
                    
        double lastTime =0;
        double outputSum, lastInput;

        double SampleTime;
        double outMin, outMax;
        bool inAuto, pOnE;
};

#endif
