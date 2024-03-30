// freeze
#include "drv_local.h"
static int tmp_div = 0;
static int tmp_divi =10;
static int tmp_result =1; 

void Freeze_OnEverySecond() {
	while (1) {
		// freeze
		tmp_result = tmp_divi/tmp_div;
	}
}
void Freeze_RunFrame() {
	while (1) {
		tmp_result = tmp_divi/tmp_div;
	}
}
void Freeze_Init() {
	// don't freeze
}
