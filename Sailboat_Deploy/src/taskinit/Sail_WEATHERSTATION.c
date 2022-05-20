/*
 * Sail_WEATHERSTATION.c
 *
 * Created: 2019-04-11 5:28:55 PM
 *  Author: Antho, Serge
 */ 



#include "Sail_WEATHERSTATION.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sail_tasksinit.h"
#include "usart_interrupt.h"



#include "sail_nmea.h"
#include "sail_debug.h"

#define HEADER_FMT				"DALSAIL,%03"PRIu16
#define FIELD_FMT				",%"PRIi32

#define RADIO_BUFFER_LENGTH		NMEA_BUFFER_LENGTH

		


static bool init_flag = false;
static char msg_buffer[RADIO_BUFFER_LENGTH];

//stores all the msg types of the weather sensor
volatile WEATHERSENSOR_AllMsgs weathersensor_data;



void ReadWeatherSensor(void) {

	uint16_t loop_cnt = 0;
	//set msg type sum to 0 since no messages processed yet
	weathersensor_data.msg_type_sum = 0;
	while(1) {
		WeatherStation_On();
		running_task = eReadWeatherSensor;
		WEATHERSENSOR_GenericMsg msg;
		if( WEATHERSTATION_RxMsg(&msg) == STATUS_OK) {
			//store msg into msg array at index corresponding to msg type
			weathersensor_data.msg_array[msg.type] = msg;
			//add type to msg type sum to keep track of the saved messages
			weathersensor_data.msg_type_sum += msg.type;
			if (weathersensor_data.msg_type_sum >= MSG_TYPE_TOTAL) {
				//reset fields
				weathersensor_data.msg_type_sum = 0;
				WeatherStation_Sleep_Sec(WS_SLEEP_PERIOD_SEC);
				
			}
		}
		
		loop_cnt++;
		
		if(loop_cnt > WS_LOOP_LIM) {
			weathersensor_data.msg_type_sum = 0;
			loop_cnt = 0;
			DEBUG_Write("Failed to acquire all the WS NMEA fields!");
			WeatherStation_Sleep_Sec(WS_SLEEP_PERIOD_SEC);
		}

	}
}

void WeatherStation_Sleep_Sec(unsigned time_sec) {
	WeatherStation_Disable();
	//put thread to sleep for number of ticks specified
	vTaskDelay(time_sec * configTICK_RATE_HZ);
}

void WeatherStation_On(void) {
	WeatherStation_Init();
	WeatherStation_Enable();
}

enum status_code WeatherStation_Init(void)
{
	// Return if the module has already been initialized
	if (init_flag) {
		return STATUS_ERR_ALREADY_INITIALIZED;
	}
	
	// Initialize NMEA channel
	switch (NMEA_Init(NMEA_WEATHERSTATION)) {
		case STATUS_OK:							// Initialization complete, continue
		case STATUS_ERR_ALREADY_INITIALIZED:	// Already initialized, not a problem
		break;
		default:
	//	DEBUG_Write("NMEA module could not be initialized!\r\n");
		return STATUS_ERR_DENIED;
	}
	
	// Set the initialization flag
	init_flag = true;
	
	return STATUS_OK;
}


enum status_code WeatherStation_Enable(void)
{
	// Return if the module hasn't been initialized
	if (!init_flag) {
		return STATUS_ERR_NOT_INITIALIZED;
	}
	
	// Return if the receiver cannot be started
	if (NMEA_Enable(NMEA_WEATHERSTATION != STATUS_OK)) {
	//	DEBUG_Write("NMEA receiver could not be started!\r\n");
		return STATUS_ERR_DENIED;
	}
	
	return STATUS_OK;
}


enum status_code WeatherStation_Disable(void)
{
	// Return if the module hasn't been initialized
	if (!init_flag) {
		return STATUS_ERR_NOT_INITIALIZED;
	}
	
	// Return if the receiver cannot be disabled
	if (NMEA_Disable(NMEA_WEATHERSTATION) != STATUS_OK) {
		return STATUS_ERR_DENIED;
	}
	
	return STATUS_OK;
}


/*identifies the prefix from the NMEA message and assigns it to the message type*/
bool get_NMEA_type(eWeatherstationTRX_t *type) {
		//read prefix
		char *msg_ptr = strtok(msg_buffer, ",");
		int i;
		for(i=0; i<WEATHERSENSOR_NUM_MSG_TYPES; i++) {
			//return true if prefix found in weather station type table
			if(strcmp(msg_ptr, WEATHERSTATION_TYPE_TABLE[i].WS_Prefix) == 0) {
				//assign type to matched prefix string
				*type = WEATHERSTATION_TYPE_TABLE[i].WS_id;
				return true;
			}
		}
		return false;	
}
char tmp_ptr[8];
int tmp_cntr=0;
/*request to receive message by weathersensor*/
enum status_code WEATHERSTATION_RxMsg(WEATHERSENSOR_GenericMsg *msg)
{
	// Return if a null pointer is provided
	if (msg == NULL) {
		return STATUS_ERR_BAD_ADDRESS;
	}
	
	// Check the NMEA receiver for new data
	switch (NMEA_RxString(NMEA_WEATHERSTATION, (uint8_t *)msg_buffer, NMEA_BUFFER_LENGTH)) {
		// Data was found, continue and process
		case STATUS_VALID_DATA:
		break;
		// Data was not found
		case STATUS_NO_CHANGE:
		return STATUS_NO_CHANGE;
		// An error occurred
		default:
		return STATUS_ERR_DENIED;
	}
	
	// Extract the raw data from the message
	WEATHERSENSOR_MsgRawData raw_data;
	char *msg_ptr;
	
	
	DEBUG_Write("weathersensor: %s\r\n", msg_buffer);
	
	//assign NMEA string prefix to raw_data type
	if(!get_NMEA_type(&raw_data.type)) {
		return STATUS_DATA_NOT_NEEDED;
	}
	//DEBUG_Write("Assigning data!\n");
	
	// Get each argument
	uint8_t arg_count = 0;
	
	while ((msg_ptr = strtok(NULL, ",")) != NULL) {	
		//store msg_ptr as float value into arg array ..
		//if *msg_ptr is alphabetic, store directly, else convert string to float value
		raw_data.args[arg_count] = isalpha(*msg_ptr) ? *msg_ptr : atof(msg_ptr);
			
		if(arg_count == WEATHERSENSOR_MSG_MAX_ARGS) break;
		arg_count++;
		
	}
	
	/*
	// Compare the argument count to its expected value
	if (arg_count != WEATHERSENSOR_arg_counts[raw_data.type]) {
		return STATUS_ERR_BAD_DATA;
	}
	*/
	
	// Parse the message
	if (WEATHERSENSOR_ExtractMsg(msg, &raw_data) != STATUS_OK) {
		return STATUS_ERR_BAD_DATA;
	}
	
	
	return STATUS_OK;
}


int last_type = 1337;
int vals[4];

//extern int was_here;
static enum status_code WEATHERSENSOR_ExtractMsg(WEATHERSENSOR_GenericMsg *msg, WEATHERSENSOR_MsgRawData *data) {
	msg->type = data->type;
	last_type = data->type;
	
	switch (data->type) {
		
		case eGPGGA:
			msg->fields.gpgga.lat.lat = data->args[1];
			msg->fields.gpgga.lat.ns = ((char)data->args[2] == 'N') ? north : south;
			msg->fields.gpgga.lon.lon = data->args[3];
			msg->fields.gpgga.lon.we = ((char)data->args[3] == 'W') ? west : east;
			msg->fields.gpgga.alt = data->args[8];
			vals[0] = 1;		
		break;
		
		case eGPVTG: 
			msg->fields.gpvtg.course_over_ground = data->args[0];
			break;
			
		case eWIMWV:
			msg->fields.wimwv.wind_dir_rel = data->args[0];
			msg->fields.wimwv.wind_speed_ms = data->args[2];
			break;
		
		case eYXXDR:
		//the yxxdr type we are interested in
		if(data->args[0] == 'A') {
			msg->fields.yxxdr.pitch_deg = data->args[1];
			msg->fields.yxxdr.roll_deg = data->args[5];
			vals[3] = 1;
			//	DEBUG_Write("Assigning data!\n");
		}
				
		break;	
			
		
		/*
		case eHCHDT:
			msg->fields.hchdt.bearing = data->args[0];
			vals[1] = 1;	
		break;
		
		case eWIMWD:
			msg->fields.wimwd.wind_dir_true = data->args[0];
			msg->fields.wimwd.wind_dir_mag = data->args[2];
			msg->fields.wimwd.wind_speed_knot = data->args[4];
			msg->fields.wimwd.wind_speed_ms = data->args[6];
			vals[2] = 1;	
		break;
		
		case eYXXDR:
		//the yxxdr type we are interested in
			if(data->args[0] == 'A') {
				msg->fields.yxxdr.pitch_deg = data->args[1];
				msg->fields.yxxdr.roll_deg = data->args[5];
				vals[3] = 1;
			//	DEBUG_Write("Assigning data!\n");
			}
			
		break;
		
		//the remaining message cases are listed below
		/*
		case eGPDTM:
		break;
		case eGPGLL:
		break;
		case eGPGSA:
		break;
		case eGPGSV:
		break;
		case eGPRMC:
		break;	
		case eGPVTG:
		break;
		case eGPZDA:
		break;					
		case eHCHDG:
		break;			
		case eHCTHS:
		break;
		case eTIROT:
		break;	
		case eWIMDA:
		break;	
		case eWIMWV:
		break;
		case eWIMWR:
		break;	
		case eWIMWT:
		break;
		*/
		
		
		default:
			return STATUS_ERR_BAD_DATA;
		break;	
	}
	return STATUS_OK;
}

