
#include "sail_ctrl.h"
#include "sail_debug.h"
#include "sail_tasksinit.h"


int main(void)
{
	CTRL_InitSystem();
	CTRL_InitSensors();
	startup(); //sensor initializations
	init_tasks();
}

