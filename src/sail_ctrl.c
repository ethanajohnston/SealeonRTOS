
#include "sail_ctrl.h"

#include <math.h>
#include <inttypes.h>
#include <stdbool.h>

#include "sail_math.h"
#include "sail_debug.h"
#include "sail_radio.h"
#include "sail_wind.h"
#include "sail_eeprom.h"
#include "sail_types.h"
#include "sail_nav.h"
#include "sail_motor.h"
#include "sail_tasksinit.h"
#include "Sail_WEATHERSTATION.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"



// Time since last reset
static uint64_t t_ms;
// Time step (resolution of RTC)
static uint16_t dt_ms = 200;
// RTC timer
struct rtc_module rtc_timer;

// Task functions


static void process_wind_readings(void);
static void EnableWeatherStation(void);
static void DisableWeatherStation(void);
static void process_heading_readings(void);


/* Message handling functions:
 * HandleMessage()
 * - checks the message type and calls the appropriate function
 * ChangeMode()
 * - changes the operating mode of the controller
 * - starts and stops EEPROM loading process when required
 * ChangeState()
 * - change the state of the controller
 * ChangeLogPeriod()
 * - change the period at which log messages are sent
 * AddWayPoint()
 * - add a way point to the EEPROM
 * AdjustMotors()
 * - apply remotely controlled adjustments to the motors
 */ 

// Sensor status
static bool sensor_statuses[SENSOR_COUNT] = {
	false,
	false,
	false
};

// Sensor readings
static GPS_Reading gps;
static WIND_Reading wind, avg_wind;
static COMP_Reading comp;

// Radio messages
RADIO_GenericMsg tx_msg, rx_msg;

// EEPROM data
EEPROM_Entry wp_entry;
uint16_t wp_count;

// Current way point
EEPROM_WayPoint wp;
// Counter to track # of times distance < radius
uint16_t wp_complete_count;
// Distance between boat and way point
double wp_distance;

float course, bearing, sail_deg, rudder_deg; 
float avg_heading_deg = 0.0;




enum status_code CTRL_InitSystem(void)
{
	// Initialize SAMD20
	system_init();
	
	// Initialize debug UART
	DEBUG_Init();
	
	// Initialize the radio
	if (RADIO_Init() != STATUS_OK) {
		DEBUG_Write("Radio not initialized!\r\n");
		} else {
		DEBUG_Write("Radio initialized!\r\n");
	}
	
	// Enable the radio receiver
	if (RADIO_Enable() != STATUS_OK) {
		DEBUG_Write("Radio not enabled!\r\n");
		} else {
		DEBUG_Write("Radio enabled!\r\n");
	}	

	// Determine reset cause
	tx_msg.type = RADIO_RESET;
	switch (system_get_reset_cause()) {
		case  SYSTEM_RESET_CAUSE_WDT:
			DEBUG_Write("WDT reset detected!\r\n");
			tx_msg.fields.reset.cause = CTRL_RESET_WDT;
			break;
		case SYSTEM_RESET_CAUSE_POR:
			DEBUG_Write("Power on reset detected!\r\n");
			tx_msg.fields.reset.cause = CTRL_RESET_POWER;
			break;
		case SYSTEM_RESET_CAUSE_SOFTWARE:
			DEBUG_Write("Software reset detected!\r\n");
			tx_msg.fields.reset.cause = CTRL_RESET_SW;
			break;
		case SYSTEM_RESET_CAUSE_EXTERNAL_RESET:
			DEBUG_Write("External reset detected!\r\n");
			tx_msg.fields.reset.cause = CTRL_RESET_EXT;
			break;
		default:
			DEBUG_Write("Other type of reset detected!\r\n");
			tx_msg.fields.reset.cause = CTRL_RESET_OTHER;
			break;
	}
	RADIO_TxMsg(&tx_msg);
	
	// Initialize the EEPROM
	if (EEPROM_Init() != STATUS_OK) {
		DEBUG_Write("EEPROM not initialized!\r\n");
	}
	
	// Set default mode and state
	mode = CTRL_MODE_AUTO;
	state = CTRL_STATE_DEPLOY;
	
	// Set the time to 0 ms
	t_ms = 0;
	
	return STATUS_OK;
}


enum status_code CTRL_InitSensors(void)
{
	
	//todo: add initialization for AIS module
	
	if (WEATHERSTATION_Init() != STATUS_OK) {
		DEBUG_Write("Wind vane not initialized...\r\n");
	}
	return STATUS_OK;
}



enum status_code startup(void)
{
	// Enable wind vane
	if (WS_Enable() != STATUS_OK) {
		DEBUG_Write("WS not enabled...\r\n");
		} else {
		DEBUG_Write("WS enabled...\r\n");
	}
	
	// Get the current way point
	EEPROM_GetCurrentWayPoint(&wp);
	
	// Reset the way point complete count
	wp_complete_count = 0;
	
	DEBUG_Write("way point: lat - %d\r\n", (int)(wp.pos.lat * 1000.0));
	
	// Start the motor controller
	MOTOR_Init();

	
	return STATUS_OK;
}




/**** TIMER CALLBACKS ************************************************************/
void LogData(void)
{
	while(1) {
		running_task = eLogData;
		// Log the GPS coordinates
		tx_msg.type = RADIO_GPS;
		tx_msg.fields.gps.data = gps;
		RADIO_TxMsg(&tx_msg);
	
		// Log the wind speed and direction
		tx_msg.type = RADIO_WIND;
		tx_msg.fields.wind.data = wind;
		
		//not needed because wind is reported in relation to the vessel's center line
		/*
		// Correct wind angle with average heading
		tx_msg.fields.wind.data.angle += avg_heading_deg;
		*/
		
		RADIO_TxMsg(&tx_msg);
	
		// Log the compass data
		tx_msg.type = RADIO_COMP;
		tx_msg.fields.comp.data = comp;
		RADIO_TxMsg(&tx_msg);

		// Log the navigation data
		tx_msg.type = RADIO_NAV;
		tx_msg.fields.nav.wp = wp;
		tx_msg.fields.nav.distance = wp_distance;
		tx_msg.fields.nav.bearing = bearing;
		tx_msg.fields.nav.course = course;
		tx_msg.fields.nav.sail_angle = sail_deg;
		tx_msg.fields.nav.rudder_angle = rudder_deg;
		RADIO_TxMsg(&tx_msg);
		DEBUG_Write("Logging data...\r\n");
		
		//put thread to sleep
		CTRL_Sleep(RADIO_SLEEP_PERIOD_SEC);
		
	}
}



static void process_wind_readings(void)
{

	// Force the wind to range [0, 360)
	wind.angle = MATH_ForceAngleTo360(wind.angle);
	
	// Convert wind vector to rectangular coordinates
	float wind_x = wind.speed * cos(wind.angle * M_PI / 180.0);
	float wind_y = wind.speed * sin(wind.angle * M_PI / 180.0);
	float avg_x = avg_wind.speed * cos(avg_wind.angle * M_PI / 180.0);
	float avg_y = avg_wind.speed * sin(avg_wind.angle * M_PI / 180.0);
	
	// Update average wind
	avg_x = 0.667 * avg_x + 0.333 * wind_x;
	avg_y = 0.667 * avg_y + 0.333 * wind_y;
	avg_wind.speed = sqrt(avg_x * avg_x + avg_y * avg_y);
	avg_wind.angle = 180.0 * atan2(avg_y, avg_x) / M_PI;
	
	DEBUG_Write("cur_wind.speed = %4d cm/s | cur_wind.angle = %4d deg\r\n", (int)(wind.speed * 100), (int)wind.angle);
	DEBUG_Write("avg_wind.speed = %4d cm/s | avg_wind.angle = %4d deg\r\n", (int)(avg_wind.speed * 100), (int)avg_wind.angle);
}

static void EnableWeatherStation(void)
{
	// Return if the controller is in LOAD mode
	if (mode == CTRL_MODE_LOAD) {
		return;
	}

	// Enable the wind vane
	WS_Enable();
}

static void DisableWeatherStation(void)
{
	// Return if the controller is in LOAD mode
	if (mode == CTRL_MODE_LOAD) {
		return;
	}

	// Disable the wind vane
	WS_Disable();
}

static void process_heading_readings(void)
{
	// Update the averaged heading
	avg_heading_deg = 0.9 * avg_heading_deg + 0.1 * comp.data.heading.heading;
}




static void assign_weatherstation_readings(void) {
	//assign gps weather sensor data to gps struct
	gps.lat = weathersensor_data.msg_array[eGPGGA].fields.gpgga.lat.lat;
	gps.lon = weathersensor_data.msg_array[eWIMWV].fields.gpgga.lon.lon;
	//assign wind readings
	wind.angle = weathersensor_data.msg_array[eWIMWV].fields.wimwv.wind_dir_rel;
	wind.speed = weathersensor_data.msg_array[eWIMWV].fields.wimwv.wind_speed_ms;
	//assign compass readings
	comp.data.heading.heading = weathersensor_data.msg_array[eGPVTG].fields.gpvtg.course_over_ground;
	
	//assign distance between boat and waypoint to wp_distance
	NAV_GetDistance(wp.pos, gps, &wp_distance);
	//assign bearing
	NAV_GetBearing(wp.pos, gps, &bearing);
}



void ControlRudder(void)
{
	while(1) {
		running_task = eControlRudder;
		NAV_CalculateRudderPosition(course, comp.data.heading.heading, &rudder_deg);
		DEBUG_Write("Setting rudder - %d\r\n", (int)rudder_deg);
		MOTOR_SetRudder(rudder_deg);
		CTRL_Sleep(CONTROLRUDDER_SLEEP_PERIOD_SEC);
	}
}


static void check_waypoint_state(void) {
	
		// Check if the boat is in the zone
		if (wp_distance < wp.rad) {
			wp_complete_count++;
			DEBUG_Write("way point in range - #%d\r\n", (int)wp_complete_count);
			} else {
			// Reset the count
			wp_complete_count = 0;
		}
		
		// Check if we can move to the next way point
		if (wp_complete_count > 5) {
			DEBUG_Write("getting new way point!\r\n");
			EEPROM_CompleteCurrentWayPoint();
			EEPROM_GetCurrentWayPoint(&wp);
			wp_complete_count = 0;
		}
	
}


void UpdateCourse(void)
{

	while(1) {
		running_task = eUpdateCourse;
		// Return if the controller is not in autonomous mode
		//store weather station data into appropriate structs
		assign_weatherstation_readings();	
		//check if waypoint was reached and affect as necessary
		check_waypoint_state();
		//calculate wind parameters
		process_wind_readings();
		//calculate heading parameters
		process_heading_readings();
			
		//update course
		NAV_UpdateCourse(wp.pos, gps, avg_wind, avg_heading_deg, &course, &sail_deg);
		DEBUG_Write("course: %6d  sail: %d\r\n", (int)(course*1000.0), (int)(sail_deg*1000.0));
		MOTOR_SetSail(sail_deg);
				
		CTRL_Sleep(UPDATECOURSE_SLEEP_PERIOD_SEC);

	}
	
}

static void CTRL_Sleep(unsigned time_sec) {
	vTaskDelay(time_sec * configTICK_RATE_HZ);;
}

