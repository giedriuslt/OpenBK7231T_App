// BL602 WiFi RF/TX debug commands.
// Reads the per-packet TX confirmation counters added to the SDK's bl_tx.c by
// platforms/BL602/override/sdk/OpenBL602/components/network/wifi_manager/bl60x_wifi_driver/bl_tx.c
// (copied over the SDK original by platforms/BL602/pre_build.sh).

#include "../new_common.h"
#include "../obk_config.h"

#if ENABLE_BL602_TXSTATS

#include "../logging/logging.h"
#include "cmd_local.h"

#include <wifi_mgmr_ext.h>

// counters live in the patched SDK bl_tx.c (see override file above)
extern volatile uint32_t g_obkTxCfmOk;         // frames confirmed sent
extern volatile uint32_t g_obkTxCfmOkRetried;  // sent, but hw reported retries were needed
extern volatile uint32_t g_obkTxRetryLimit;    // frames that hit the MAC retry limit (~12+ failed tx of one frame)
extern volatile uint32_t g_obkTxRequeued;      // retry-limit frames requeued for software resend
extern volatile uint32_t g_obkTxDropped;       // retry-limit frames dropped (resend holder full)

// 0 = periodic logging off, otherwise interval in seconds
static int g_txStatsInterval = 0;
static int g_txStatsCountdown = 0;
// snapshot at previous print, for deltas
static uint32_t g_prevOk, g_prevOkRetried, g_prevRetryLimit, g_prevRequeued, g_prevDropped;

static void BL602TX_PrintStats(int secondsSinceLast) {
	uint32_t ok = g_obkTxCfmOk;
	uint32_t okRetried = g_obkTxCfmOkRetried;
	uint32_t retryLimit = g_obkTxRetryLimit;
	uint32_t requeued = g_obkTxRequeued;
	uint32_t dropped = g_obkTxDropped;
	int rssi = 0, channel = 0;

	wifi_mgmr_rssi_get(&rssi);
	wifi_mgmr_channel_get(&channel);

	if (secondsSinceLast > 0) {
		ADDLOG_INFO(LOG_FEATURE_CMD,
			"WiFi TX: ok=%u(+%u) okRetried=%u(+%u) retryLimit=%u(+%u) requeued=%u(+%u) dropped=%u(+%u) rssi=%i ch=%i",
			(unsigned int)ok, (unsigned int)(ok - g_prevOk),
			(unsigned int)okRetried, (unsigned int)(okRetried - g_prevOkRetried),
			(unsigned int)retryLimit, (unsigned int)(retryLimit - g_prevRetryLimit),
			(unsigned int)requeued, (unsigned int)(requeued - g_prevRequeued),
			(unsigned int)dropped, (unsigned int)(dropped - g_prevDropped),
			rssi, channel);
	}
	else {
		ADDLOG_INFO(LOG_FEATURE_CMD,
			"WiFi TX: ok=%u okRetried=%u retryLimit=%u requeued=%u dropped=%u rssi=%i ch=%i",
			(unsigned int)ok, (unsigned int)okRetried, (unsigned int)retryLimit,
			(unsigned int)requeued, (unsigned int)dropped, rssi, channel);
	}
	g_prevOk = ok;
	g_prevOkRetried = okRetried;
	g_prevRetryLimit = retryLimit;
	g_prevRequeued = requeued;
	g_prevDropped = dropped;
}

// called from Main_OnEverySecond
void BL602TX_OnEverySecond() {
	if (g_txStatsInterval <= 0) {
		return;
	}
	g_txStatsCountdown--;
	if (g_txStatsCountdown <= 0) {
		g_txStatsCountdown = g_txStatsInterval;
		BL602TX_PrintStats(g_txStatsInterval);
	}
}

static commandResult_t CMD_WifiTxStats(const void *context, const char *cmd, const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() > 0) {
		g_txStatsInterval = Tokenizer_GetArgInteger(0);
		g_txStatsCountdown = g_txStatsInterval;
		if (g_txStatsInterval > 0) {
			ADDLOG_INFO(LOG_FEATURE_CMD, "WifiTxStats: logging every %i second(s)", g_txStatsInterval);
		}
		else {
			ADDLOG_INFO(LOG_FEATURE_CMD, "WifiTxStats: periodic logging off");
		}
	}
	BL602TX_PrintStats(0);
	return CMD_RES_OK;
}

static commandResult_t CMD_WifiTxStatsReset(const void *context, const char *cmd, const char *args, int cmdFlags) {
	g_obkTxCfmOk = 0;
	g_obkTxCfmOkRetried = 0;
	g_obkTxRetryLimit = 0;
	g_obkTxRequeued = 0;
	g_obkTxDropped = 0;
	g_prevOk = g_prevOkRetried = g_prevRetryLimit = g_prevRequeued = g_prevDropped = 0;
	ADDLOG_INFO(LOG_FEATURE_CMD, "WifiTxStats: counters reset");
	return CMD_RES_OK;
}

void BL602TX_AddCommands() {
	//cmddetail:{"name":"WifiTxStats","args":"[OptionalIntervalSeconds]",
	//cmddetail:"descr":"BL602 only - prints WiFi TX confirmation counters (frames sent OK, frames that needed retries, frames that hit the MAC retry limit, requeued/dropped frames) plus RSSI and channel. A stream of retryLimit events under load indicates a bad RF link. With an argument, also logs the stats every N seconds (0 disables periodic logging).",
	//cmddetail:"fn":"CMD_WifiTxStats","file":"cmnds/cmd_bl602_txstats.c","requires":"",
	//cmddetail:"examples":"WifiTxStats 1"}
	CMD_RegisterCommand("WifiTxStats", CMD_WifiTxStats, NULL);
	//cmddetail:{"name":"WifiTxStatsReset","args":"",
	//cmddetail:"descr":"BL602 only - resets the WiFi TX confirmation counters shown by WifiTxStats.",
	//cmddetail:"fn":"CMD_WifiTxStatsReset","file":"cmnds/cmd_bl602_txstats.c","requires":"",
	//cmddetail:"examples":"WifiTxStatsReset"}
	CMD_RegisterCommand("WifiTxStatsReset", CMD_WifiTxStatsReset, NULL);
}

#endif // ENABLE_BL602_TXSTATS
