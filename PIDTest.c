#include <stdio.h>
#include <math.h>
#include <status_codes.h>
#include <PID.h>


enum status_code test_pid(double P, double I, double D, int END){

    struct PID pid;


    pid.SetPoint=0.0;
    pid.SampleTime = 0.01;
    double feedback = 0;

    double feedback_list[101];
    double time_list[101];
    double setpoint_list[101];

    for (int i = 0; i < END; i++){
        double output = update(pid, feedback);
        if (pid.SetPoint > 0){
            feedback += (output - (1/i));
        }
        if (i>9){
            pid.SetPoint = 1;
        }
    }
    return STATUS_OK;
}
