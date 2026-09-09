/*
 * bl602_crash_watchdog.c
 *
 * Software watchdog for BL602 (bl_iot_sdk) built on the *standard* TIMER
 * peripheral (bl602_timer.h) rather than the dedicated hardware WDT block.
 * On expiry it captures mepc + the active FreeRTOS task name into
 * Retention RAM, forces a reset, and on the next boot the crash record is
 * printed AND persisted into the EasyFlash-backed ef_env KV store so it
 * survives a full power cycle (Retention RAM can be lost on a cold boot,
 * depending on PDS / power-domain configuration -- flash is not).
 *
 * Verified against this SDK's actual bl602_timer.h + hosal_timer.c:
 *  - Channel type is TIMER_Chan_Type (TIMER_CH0 / TIMER_CH1), not an "ID" enum.
 *  - TIMER_CFG_Type has no free-text "mode" field -- timing behavior is set
 *    via countMode (TIMER_COUNT_FREERUN / TIMER_COUNT_PRELOAD) + plTrigSrc.
 *  - There is no TIMER_ResetCounterValue() for TIMER_CH0/1 (only
 *    WDT_ResetCounterValue() for the separate dedicated WDT block), so
 *    watchdog_kick() below pushes the trip point forward instead of trying
 *    to reset the counter.
 *  - IRQ registration + enabling is bl_irq_register_with_ctx() /
 *    bl_irq_enable() from <bl_irq.h> -- confirmed straight from this SDK's
 *    hosal_timer.c, which builds its own generic timer HAL on exactly this
 *    TIMER_* + bl_irq_* pairing. This is the fix for the timer not firing:
 *    earlier NVIC_EnableIRQ()/Timer_Int_Callback_Install() calls were
 *    guesses and are NOT part of this SDK's real API surface -- removed.
 *    Our handler is a plain void(void*) callback matching bl_irq_*'s
 *    signature, NOT an __attribute__((interrupt(...))) function.
 *
 * One remaining difference from hosal_timer.c worth flagging: it configures
 * TIMER_CLKSRC_XTAL + a fixed divider (39) + TIMER_COUNT_PRELOAD /
 * TIMER_PRELOAD_TRIG_COMP0 (hardware auto-reloads the counter on every
 * match, so it's a genuinely recurring periodic interrupt). This file
 * instead uses TIMER_CLKSRC_1K + TIMER_COUNT_FREERUN /
 * TIMER_PRELOAD_TRIG_NONE (counter free-runs forever, watchdog_kick() just
 * pushes the trip point out) -- both are valid per bl602_timer.h, but only
 * the XTAL/PRELOAD combo above has been directly observed working in this
 * SDK. If the timer still doesn't fire after the bl_irq_* fix, that clock
 * config is the next thing to try.
 * GLB_SW_System_Reset() is also assumed to live in bl602_glb.h.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bl602_timer.h"   /* TIMER_* std-driver API */
#include "bl602_glb.h"     /* GLB_SW_System_Reset()  */
#include "bl_irq.h"        /* bl_irq_register_with_ctx() / bl_irq_enable() --
                             * confirmed straight from this SDK's own
                             * hosal_timer.c, which uses these same TIMER_*
                             * calls to drive its generic timer HAL. */

#include "easyflash.h"     /* ef_set_env_blob / ef_get_env_blob / ef_save_env */

/* ------------------------------------------------------------------------
 * Crash Log Definition & Retention Mapping
 * ------------------------------------------------------------------------ */

#define RETRAM_BASE_ADDR   0x40010000  /* BL602 Retention RAM base address */
#define CRASH_MAGIC_VALID  0xDEADBEEF  /* signature flag for valid crash data */

typedef struct {
    uint32_t magic;
    uint32_t instruction_pointer; /* Program Counter (mepc CSR) at hang */
    char     task_name[16];       /* active FreeRTOS task name */
} crash_log_t;

static volatile crash_log_t * const g_crash_log = (volatile crash_log_t *)RETRAM_BASE_ADDR;

/* ------------------------------------------------------------------------
 * Watchdog timer configuration
 * ------------------------------------------------------------------------ */

#define WDT_TIMER_CH        TIMER_CH0        /* TIMER_Chan_Type: TIMER_CH0 or TIMER_CH1, either is free */
#define WDT_TIMER_COMP_ID   TIMER_COMP_ID_0
#define WDT_TIMER_IRQn      TIMER_CH0_IRQn   /* confirmed real symbol (see hosal_timer.c) -- swap to
                                              * TIMER_CH1_IRQn if you switch WDT_TIMER_CH to TIMER_CH1 */
#define WDT_TIMEOUT_MS      5000

/* TIMER_CLKSRC_1K is explicitly documented as "Only for Timer not for
 * Watchdog" -- i.e. it's available precisely for this TIMER_CH0/1 use case
 * and gives an exact 1kHz tick with no XTAL-frequency math required.
 * clockDivision further divides that; 0 = no additional division, so
 * 1 tick == 1ms and ms values can be used directly as tick counts. */
#define WDT_CLOCK_DIVISION   0

/* ------------------------------------------------------------------------
 * Hardware Timer ISR & Capture
 * ------------------------------------------------------------------------ */

/* This is installed via bl_irq_register_with_ctx() below, which is what
 * actually wires a handler into the CPU's interrupt table on this SDK
 * (confirmed via hosal_timer.c) -- NOT bound directly to the trap vector
 * itself, so it's a plain callback, not a raw trap handler. Do NOT mark
 * this with __attribute__((interrupt(...))): that would emit an `mret` at
 * the end of a function that's actually reached via a normal `call`/`jal`,
 * which is not a valid return from this context.
 * The ctx parameter is required by bl_irq_register_with_ctx()'s handler
 * signature; unused here since all state is static/global.
 * mepc is untouched between the original trap entry and this callback
 * running, so reading it here still correctly reflects the instruction
 * that was interrupted when the watchdog tripped. */
static void timer_watchdog_isr(void *ctx)
{
    (void)ctx;
    uint32_t pc_val;

    /* Mask before handling, mirroring hosal_timer.c's own ISR -- avoids
     * a re-entrant fire while we're busy writing to Retention RAM. Not
     * strictly necessary since we're resetting the chip either way, but
     * cheap insurance. */
    TIMER_IntMask(WDT_TIMER_CH, TIMER_INT_ALL, MASK);

    /* Read the Machine Exception Program Counter (mepc) to capture the
     * precise instruction the CPU was executing when the watchdog expired
     * -- i.e. wherever main_task() was stuck when it stopped kicking us. */
    __asm__ volatile ("csrr %0, mepc" : "=r"(pc_val));

    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    const char *name = "ISR/Unknown";

    if (current_task != NULL) {
        /* Not an official "-FromISR" API, but we're one instruction away
         * from a hard reset either way -- acceptable here as a best-effort
         * last-gasp diagnostic read. */
        name = pcTaskGetName(current_task);
    }

    g_crash_log->instruction_pointer = pc_val;
    strncpy((char *)g_crash_log->task_name, name, sizeof(g_crash_log->task_name) - 1);
    g_crash_log->task_name[sizeof(g_crash_log->task_name) - 1] = '\0';

    /* Set magic flag last to confirm a completed payload write */
    g_crash_log->magic = CRASH_MAGIC_VALID;

    TIMER_ClearIntStatus(WDT_TIMER_CH, WDT_TIMER_COMP_ID);
    GLB_SW_System_Reset();

    /* Should not be reached -- GLB_SW_System_Reset() resets the chip.
     * Spin here as a safety net in case of pipeline/reset latency. */
    while (1) { }
}

/* ------------------------------------------------------------------------
 * Watchdog timer init / kick (standard TIMER peripheral, not the dedicated
 * hardware WDT block)
 * ------------------------------------------------------------------------ */

/* ticks == ms at the 1kHz clock source; stored so watchdog_kick() re-arms
 * relative to whatever timeout was actually passed to watchdog_timer_init(),
 * rather than assuming the WDT_TIMEOUT_MS macro was used. */
static uint32_t s_wdt_timeout_ticks = 0;

void watchdog_timer_init(uint32_t timeout_ms)
{
    s_wdt_timeout_ticks = timeout_ms;

    TIMER_CFG_Type timerCfg = {
        .timerCh       = WDT_TIMER_CH,
        .clkSrc        = TIMER_CLKSRC_1K,           /* 1kHz -> 1 tick == 1ms */
        .plTrigSrc     = TIMER_PRELOAD_TRIG_NONE,   /* free run, no auto-reload on match --
                                                      * watchdog_kick() manages the trip point instead */
        .countMode     = TIMER_COUNT_FREERUN,
        .clockDivision = WDT_CLOCK_DIVISION,
        .matchVal0     = s_wdt_timeout_ticks,       /* first trip point */
        .matchVal1     = 0,
        .matchVal2     = 0,
        .preLoadVal    = 0,
    };

    /* Defensive reset before reconfiguring -- mirrors hosal_timer_init():
     * mask everything and stop the timer before touching TIMER_Init(), so
     * no stale state from a previous config leaks through. */
    TIMER_IntMask(WDT_TIMER_CH, TIMER_INT_ALL, MASK);
    TIMER_Disable(WDT_TIMER_CH);

    TIMER_Init(&timerCfg);

    TIMER_ClearIntStatus(WDT_TIMER_CH, TIMER_COMP_ID_0);
    TIMER_ClearIntStatus(WDT_TIMER_CH, TIMER_COMP_ID_1);
    TIMER_ClearIntStatus(WDT_TIMER_CH, TIMER_COMP_ID_2);

    TIMER_IntMask(WDT_TIMER_CH, TIMER_INT_COMP_0, UNMASK);
    TIMER_IntMask(WDT_TIMER_CH, TIMER_INT_COMP_1, MASK);
    TIMER_IntMask(WDT_TIMER_CH, TIMER_INT_COMP_2, MASK);

    /* This is the fix: bl_irq_register_with_ctx() both installs the
     * handler AND wires it into the CPU's interrupt table, and
     * bl_irq_enable() is what actually unmasks it at the CPU level --
     * confirmed straight from this SDK's hosal_timer.c. */
    bl_irq_register_with_ctx(WDT_TIMER_IRQn, timer_watchdog_isr, NULL);
    bl_irq_enable(WDT_TIMER_IRQn);

    TIMER_Enable(WDT_TIMER_CH);
}

void watchdog_kick(void)
{
    /* TIMER_CH0/1 has no TIMER_ResetCounterValue() (that only exists for
     * the separate WDT_* block), and the counter free-runs continuously --
     * so instead of trying to reset it, push the comparator's trip point
     * out relative to *now*. Functionally equivalent to "petting" the dog:
     * as long as this runs at least once every timeout window, Comp0 never
     * catches up to the counter. */
    uint32_t now = TIMER_GetCounterValue(WDT_TIMER_CH);
    TIMER_SetCompValue(WDT_TIMER_CH, WDT_TIMER_COMP_ID, now + s_wdt_timeout_ticks);
}

/* ------------------------------------------------------------------------
 * ef_env persistence
 * ------------------------------------------------------------------------ */

#define EF_KEY_LAST_CRASH   "last_crash"
#define EF_KEY_CRASH_COUNT  "crash_count"

static void crash_log_save_to_flash(const crash_log_t *log)
{
    EfErrCode ef_result;
    uint32_t  crash_count = 0;
    size_t    saved_len   = 0;



    if (ef_get_env_blob(EF_KEY_CRASH_COUNT, &crash_count, sizeof(crash_count), &saved_len) != EF_NO_ERR
        || saved_len != sizeof(crash_count)) {
        crash_count = 0;
    }
    crash_count++;

    ef_set_env_blob(EF_KEY_LAST_CRASH, log, sizeof(*log));
    ef_set_env_blob(EF_KEY_CRASH_COUNT, &crash_count, sizeof(crash_count));

    /* ef_set_env_blob() only stages the change in the write cache --
     * ef_save_env() is what actually commits it to flash. */
    ef_save_env();

    printf("[INFO] Crash record saved to ef_env (hangs recorded so far: %lu)\n",
           (unsigned long)crash_count);
}

/**
 * Prints (and optionally returns) whatever crash record currently lives in
 * flash, independent of whether Retention RAM had anything this boot.
 * Handy to wire up to a debug shell / CLI command, or to report over
 * MQTT/HTTP without re-flashing.
 *
 * @param out_buf     Buffer to receive a one-line report string. Always
 *                     NUL-terminated on return (even if truncated). Pass
 *                     NULL to skip filling a buffer (printf-only mode).
 * @param out_buf_len Size of out_buf in bytes. Ignored if out_buf is NULL.
 *
 * @return true  if a stored crash record was found (and out_buf, if given,
 *               now holds it)
 *         false if there's no record in ef_env, or easyflash_init() failed
 *               (out_buf, if given, holds an explanatory message either way)
 */
bool crash_log_dump_flash_history(char *out_buf, size_t out_buf_len)
{
    crash_log_t stored;
    size_t saved_len = 0;


    if (ef_get_env_blob(EF_KEY_LAST_CRASH, &stored, sizeof(stored), &saved_len) == EF_NO_ERR
        && saved_len == sizeof(stored)) {
        printf("[FLASH HISTORY] Last recorded hang -> task=\"%s\" pc=0x%08X\n",
               stored.task_name, (unsigned int)stored.instruction_pointer);

        if (out_buf != NULL && out_buf_len > 0) {
            snprintf(out_buf, out_buf_len,
                     "Last recorded hang: task=\"%s\" pc=0x%08X",
                     stored.task_name, (unsigned int)stored.instruction_pointer);
        }
        return true;
    }

    printf("[FLASH HISTORY] No crash record stored in ef_env yet\n");
    if (out_buf != NULL && out_buf_len > 0) {
        snprintf(out_buf, out_buf_len, "No crash record stored in ef_env yet");
    }
    return false;
}

/* ------------------------------------------------------------------------
 * Boot Inspection Function
 * ------------------------------------------------------------------------ */

/**
 * Checks Retention RAM for a captured crash record, prints + persists it to
 * ef_env as before, and additionally fills a caller-supplied buffer with a
 * one-line human-readable report (e.g. to publish over MQTT, append to an
 * app-level diagnostics log, show on a display, etc).
 *
 * @param out_buf     Buffer to receive the report string. Always
 *                     NUL-terminated on return (even if truncated). Pass
 *                     NULL to skip filling a buffer (printf-only mode).
 * @param out_buf_len Size of out_buf in bytes. Ignored if out_buf is NULL.
 *
 * @return true  if a crash record was found (and out_buf, if given, now
 *               holds the crash report)
 *         false on a clean boot (out_buf, if given, holds "clean boot" text)
 */
bool read_crash_log_on_boot(char *out_buf, size_t out_buf_len)
{
    if (g_crash_log->magic == CRASH_MAGIC_VALID) {
        crash_log_t snapshot;

        /* Copy out of volatile Retention RAM into a plain stack struct so
         * we have a stable view to print, persist, and format. */
        snapshot.magic = g_crash_log->magic;
        snapshot.instruction_pointer = g_crash_log->instruction_pointer;
        memcpy(snapshot.task_name, (const void *)g_crash_log->task_name, sizeof(snapshot.task_name));

        printf("\n================ CRASH DETECTED ================\n");
        printf("Hanging Task Name  : %s\n", snapshot.task_name);
        printf("Instruction Pointer: 0x%08X\n", (unsigned int)snapshot.instruction_pointer);
        printf("================================================\n\n");

        if (out_buf != NULL && out_buf_len > 0) {
            snprintf(out_buf, out_buf_len,
                     "CRASH DETECTED: task=\"%s\" pc=0x%08X",
                     snapshot.task_name, (unsigned int)snapshot.instruction_pointer);
        }

        /* Persist into the flash-backed KV store so the record survives a
         * full power cycle, not just a warm reset. */
        crash_log_save_to_flash(&snapshot);

        /* Clear magic flag so subsequent normal reboots don't false-positive */
        g_crash_log->magic = 0;
        return true;
    }

    printf("[INFO] Clean boot. No hang logs in Retention RAM.\n");
    if (out_buf != NULL && out_buf_len > 0) {
        snprintf(out_buf, out_buf_len, "Clean boot. No hang logs in Retention RAM.");
    }
    return false;
}

/* ------------------------------------------------------------------------
 * Main Task
 * ------------------------------------------------------------------------ */

void main_task(void *pvParameters)
{
    (void)pvParameters;

    /* Check Retention RAM (and print/save any flash history) immediately
     * after system init. crash_report can be handed off to whatever else
     * needs it -- MQTT publish, app-level log, boot-screen text, etc. */
    char crash_report[96];
    bool had_crash = read_crash_log_on_boot(crash_report, sizeof(crash_report));
    if (had_crash) {
        /* crash_report now holds e.g.
         *   "CRASH DETECTED: task=\"wifi_task\" pc=0x42010ac4"
         * do something with it here if needed beyond the printf above. */
    }

    /* Arm the software watchdog: standard TIMER peripheral, 5s timeout */
    watchdog_timer_init(WDT_TIMEOUT_MS);

    while (1) {
        /* Execute main loop logic... */

        /* Reset the hardware timer to prevent ISR execution */
        watchdog_kick();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}