
#ifndef SAIL_CTRL_H_
#define SAIL_CTRL_H_

#include "delay.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"



#include <asf.h>
#include <status_codes.h>
#include "sail_radio.h"



#define UPDATECOURSE_ON_TIME_SEC 2
#define UPDATECOURSE_LOOP_LIM  UPDATECOURSE_ON_TIME_SEC * configTICK_RATE_HZ
#define UPDATECOURSE_SLEEP_PERIOD_SEC 7

#define CONTROLRUDDER_ON_TIME_SEC 2
#define CONTROLRUDDER_LOOP_LIM CONTROLRUDDER_ON_TIME_SEC * configTICK_RATE_HZ
#define CONTROLRUDDER_SLEEP_PERIOD_SEC 7

#define RADIO_SLEEP_PERIOD_SEC 7

typedef enum Sensor_Types {
	SENSOR_GPS,
	SENSOR_WIND,
	SENSOR_COMP,
	SENSOR_COUNT
} Sensor_Type;


/* CTRL_InitSystem
 * Initialize the sail boat controller.
 *
 */
enum status_code CTRL_InitSystem(void);

/* CTRL_InitSensors
 * Initialize each sensor.
 *
 */
enum status_code CTRL_InitSensors(void);
enum status_code startup(void);
enum status_code init_tasks();

extern CTRL_Mode mode;
extern CTRL_State state;


void UpdateCourse(void);
void ControlRudder(void);
void LogData(void);
static void check_waypoint_state(void);
static void assign_weatherstation_readings(void);
static void CTRL_Sleep(unsigned time_sec);

#endif /* SAIL_CTRL_H_ */ 