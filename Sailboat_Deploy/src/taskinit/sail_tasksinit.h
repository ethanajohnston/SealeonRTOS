#ifndef SAIL_TASKSINIT_H
#define SAIL_TASKSINIT_H


#include "sail_nmea.h"
#include "Sail_WEATHERSTATION.h"

enum all_tasks { eReadWeatherSensor, eUpdateCourse, eControlRudder, eRadioHandler, eLogData};
extern volatile WEATHERSENSOR_AllMsgs weathersensor_data;

static void idle_task(void);
extern enum all_tasks running_task;


#endif