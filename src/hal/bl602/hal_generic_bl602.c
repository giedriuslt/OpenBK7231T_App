#if PLATFORM_BL602 || PLATFORM_BL616

#include "../hal_generic.h"
#include "../../new_common.h"
#if !PLATFORM_BL_NEW
#include <hal_sys.h>
#include <bl_wdt.h>
#include <bl_timer.h>
#if PLATFORM_BL602
#include <bl602_timer.h>
#include <bl_irq.h>
#include <FreeRTOS.h>
#include <task.h>
#endif
#else
#include "bflb_common.h"
#include "bflb_wdg.h"
extern int bl_sys_reset_por(void);
struct bflb_device_s* wdg;
#define hal_reboot bl_sys_reset_por
#define bl_timer_delay_us bflb_mtimer_delay_us
#endif

void HAL_RebootModule()
{
	hal_reboot();
}

void HAL_Delay_us(int delay)
{
	bl_timer_delay_us(delay);
}

#if PLATFORM_BL602 && !PLATFORM_BL_NEW
// Watchdog fires as an interrupt (not a reset), so we can dump diagnostics
// before rebooting. The ISR first re-arms the watchdog in reset mode as a
// backstop in case the dump itself hangs. Not covered: hangs with global
// interrupts disabled - the interrupt never fires and neither does the reset.
static void wdt_diag_irq(void)
{
	uint32_t tmpVal;
	uint32_t mepc;
	static char taskListBuf[1024];

	// re-arm in RESET mode (~2s backstop); WDT clock is 32k/32 = ~1ms/tick
	WDT_ResetCounterValue();
	WDT_SetCompValue(2000);
	WDT_IntMask(WDT_INT, MASK);
	WDT_ENABLE_ACCESS();
	tmpVal = BL_RD_REG(TIMER_BASE, TIMER_WICR);
	BL_WR_REG(TIMER_BASE, TIMER_WICR, BL_SET_REG_BIT(tmpVal, TIMER_WICLR));

	__asm volatile("csrr %0, mepc" : "=r"(mepc));
	printf("\r\n!!! WDT bite - no feed for 3s\r\n");
	printf("interrupted task: %s, stuck PC (mepc)=0x%08lx\r\n",
		pcTaskGetName(NULL), (unsigned long)mepc);
	printf("free heap: %u\r\n", (unsigned)xPortGetFreeHeapSize());
	vTaskList(taskListBuf);
	printf("Name\t\tState\tPrio\tStackHWM\tNum\r\n%s\r\n", taskListBuf);

	hal_reboot();
}
#endif

void HAL_Configure_WDT()
{
#if !PLATFORM_BL_NEW
	bl_wdt_init(3000);
	bl_irq_register(TIMER_WDT_IRQn, wdt_diag_irq);
	bl_irq_enable(TIMER_WDT_IRQn);
	WDT_IntMask(WDT_INT, UNMASK);
#else
	struct bflb_wdg_config_s wdg_cfg;
	wdg_cfg.clock_source = WDG_CLKSRC_32K;
	wdg_cfg.clock_div = 31;
	wdg_cfg.comp_val = 10000;
	wdg_cfg.mode = WDG_MODE_RESET;
	wdg = bflb_device_get_by_name("watchdog0");
	bflb_wdg_init(wdg, &wdg_cfg);
	bflb_wdg_start(wdg);
#endif
}

void HAL_Run_WDT()
{
#if !PLATFORM_BL_NEW
	bl_wdt_feed();
#else
	bflb_wdg_reset_countervalue(wdg);
#endif
}

#endif // PLATFORM_BL602
