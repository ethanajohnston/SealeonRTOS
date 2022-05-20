
#include <stdio.h>
#include <math.h>


struct PID
{
    double Kp;
    double Ki;
    double Kd;
    double PTerm;
    double ITerm;
    double DTerm;
    double PastTime;
    double Time;
    double SampleTime;
    double PastError;
    double Error;
    double SetPoint;
    double Output;
};


double update(struct PID pid1, double feedback){
    pid1.Error = pid1.SetPoint - feedback;

    //replace this with a way to get current time which will be pid1.Time
    

    double delta_time = pid1.Time - pid1.PastTime;
    double delta_error = pid1.Error - pid1.PastError;


    if (delta_time >= pid1.SampleTime){
        pid1.PTerm = pid1.Kp * pid1.Error;
        pid1.ITerm += pid1.Error * delta_time;
        pid1.DTerm = 0.0;
        if (delta_time > 0){
            pid1.DTerm = delta_error / delta_time;
        }
        pid1.PastTime = pid1.Time;
        pid1.PastError = pid1.Error;

        pid1.Output = pid1.PTerm + (pid1.Ki * pid1.ITerm) + (pid1.Kd * pid1.DTerm);
        return pid1.Output;
    }
    else{
        return 0;
    }
}


