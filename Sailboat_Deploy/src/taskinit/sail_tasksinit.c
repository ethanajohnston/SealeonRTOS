#include "sail_tasksinit.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sail_wind.h"
#include "sail_debug.h"
#include "usart_interrupt.h"
#include "sail_nmea.h"
#include "Sail_WEATHERSTATION.h"
#include "sail_types.h"
#include "sail_nav.h"
#include "sail_ctrl.h"
#include "sail_radio.h"

enum all_tasks running_task;

enum status_code init_tasks(void) {
	
	xTaskCreate( ReadWeatherSensor, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL );
	xTaskCreate( idle_task, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	//the remaining tasks..
	//xTaskCreate( UpdateCourse, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL );
	//xTaskCreate( ControlRudder, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL );
	//xTaskCreate( RadioHandler, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
	//xTaskCreate( LogData, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
	
	//pass control to FreeRTOS kernel
	vTaskStartScheduler();
	/*should not get here.. if it does, more rtos heap needs to be allocated*/
	return STATUS_ERR_INSUFFICIENT_RTOS_HEAP;
	
}

static void idle_task(void) {
	while(1) {
		system_set_sleepmode(SYSTEM_SLEEPMODE_IDLE_0);
	}
}

