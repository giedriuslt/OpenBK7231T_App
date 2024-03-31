// freeze
#include "drv_local.h"
static int tmp_div = 0;
static int tmp_divi =10;
static int tmp_result =1; 

void Freeze_OnEverySecond() {
	while (1) {
		// freeze
		rtos_delay_milliseconds(1);
	}
}
void Freeze_RunFrame() {
	while (1) {
		rtos_delay_milliseconds(1);
	}
}
void Freeze_Init() {
	// don't freeze
}
