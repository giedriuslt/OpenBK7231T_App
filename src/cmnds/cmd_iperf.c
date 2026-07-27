// iperf2-compatible TCP throughput test commands, based on lwIP's lwiperf app.
// Use "iperf -s" / "iperf -c <deviceIP>" (iperf version 2, TCP mode) on the PC side.

#include "../new_common.h"
#include "../obk_config.h"

#if ENABLE_IPERF

#include "../logging/logging.h"
#include "cmd_local.h"
#include "cmd_iperf_lwiperf.h"

#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"

static void *g_iperfServer = NULL;
static void *g_iperfClient = NULL;
// set before posting to the tcpip thread, cleared there
static volatile char g_serverStartPending = 0;
static volatile char g_clientStartPending = 0;

// request parameters, written by the command handler,
// read by the tcpip thread callback
static ip_addr_t g_reqClientAddr;
static u16_t g_reqClientPort;
static u32_t g_reqClientDuration;
static enum obk_lwiperf_client_type g_reqClientMode;
static u16_t g_reqServerPort;

static const char *Iperf_GetReportTypeString(enum obk_lwiperf_report_type report_type) {
	switch (report_type) {
	case OBK_LWIPERF_TCP_DONE_SERVER:
		return "RX done";
	case OBK_LWIPERF_TCP_DONE_CLIENT:
		return "TX done";
	case OBK_LWIPERF_TCP_ABORTED_LOCAL:
		return "aborted (local)";
	case OBK_LWIPERF_TCP_ABORTED_LOCAL_DATAERROR:
		return "aborted (data error)";
	case OBK_LWIPERF_TCP_ABORTED_LOCAL_TXERROR:
		return "aborted (tx error)";
	case OBK_LWIPERF_TCP_ABORTED_REMOTE:
		return "aborted (remote)";
	default:
		return "unknown";
	}
}

// runs in the lwIP tcpip thread when an iperf session finishes
static void Iperf_Report(void *arg, enum obk_lwiperf_report_type report_type,
	const ip_addr_t *local_addr, u16_t local_port,
	const ip_addr_t *remote_addr, u16_t remote_port,
	u32_t bytes_transferred, u32_t ms_duration, u32_t bandwidth_kbitpsec) {
	char remoteStr[IPADDR_STRLEN_MAX];

	if (remote_addr != NULL) {
		ipaddr_ntoa_r(remote_addr, remoteStr, sizeof(remoteStr));
	} else {
		strcpy(remoteStr, "?");
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "iperf %s: %s:%u - %u bytes in %u ms = %u.%02u Mbit/s",
		Iperf_GetReportTypeString(report_type),
		remoteStr, (unsigned int)remote_port,
		(unsigned int)bytes_transferred, (unsigned int)ms_duration,
		(unsigned int)(bandwidth_kbitpsec / 1000),
		(unsigned int)((bandwidth_kbitpsec % 1000) / 10));

	if (arg == &g_iperfClient) {
		// the client master session frees itself when the TX test ends or aborts;
		// forget the handle so IperfStop won't touch freed memory.
		// (OBK_LWIPERF_TCP_DONE_SERVER here is a dual/tradeoff mode RX report
		// belonging to a still-running client, so keep the handle in that case)
		if (report_type != OBK_LWIPERF_TCP_DONE_SERVER) {
			g_iperfClient = NULL;
		}
	}
}

static void Iperf_StartServer_TcpipThread(void *ctx) {
	g_iperfServer = obk_lwiperf_start_tcp_server(IP_ADDR_ANY, g_reqServerPort, Iperf_Report, &g_iperfServer);
	if (g_iperfServer != NULL) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "iperf server listening on TCP port %u", (unsigned int)g_reqServerPort);
	}
	else {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf server failed to start on port %u", (unsigned int)g_reqServerPort);
	}
	g_serverStartPending = 0;
}

static void Iperf_StartClient_TcpipThread(void *ctx) {
	g_iperfClient = obk_lwiperf_start_tcp_client(&g_reqClientAddr, g_reqClientPort,
		g_reqClientMode, g_reqClientDuration, Iperf_Report, &g_iperfClient);
	if (g_iperfClient != NULL) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "iperf client started, %u second test", (unsigned int)g_reqClientDuration);
	}
	else {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf client failed to start");
	}
	g_clientStartPending = 0;
}

static void Iperf_Stop_TcpipThread(void *ctx) {
	if (g_iperfClient != NULL) {
		obk_lwiperf_abort(g_iperfClient);
		g_iperfClient = NULL;
		ADDLOG_INFO(LOG_FEATURE_CMD, "iperf client stopped");
	}
	if (g_iperfServer != NULL) {
		obk_lwiperf_abort(g_iperfServer);
		g_iperfServer = NULL;
		ADDLOG_INFO(LOG_FEATURE_CMD, "iperf server stopped");
	}
}

static commandResult_t CMD_IperfServer(const void *context, const char *cmd, const char *args, int cmdFlags) {
	int port;

	if (g_iperfServer != NULL || g_serverStartPending) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf server is already running, use IperfStop first");
		return CMD_RES_ERROR;
	}
	Tokenizer_TokenizeString(args, 0);
	port = Tokenizer_GetArgIntegerDefault(0, OBK_LWIPERF_TCP_PORT_DEFAULT);
	if (port <= 0 || port > 65535) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: bad port %i", port);
		return CMD_RES_BAD_ARGUMENT;
	}
	g_reqServerPort = (u16_t)port;
	g_serverStartPending = 1;
	if (tcpip_callback(Iperf_StartServer_TcpipThread, NULL) != ERR_OK) {
		g_serverStartPending = 0;
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: tcpip_callback failed");
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}

static commandResult_t CMD_IperfClient(const void *context, const char *cmd, const char *args, int cmdFlags) {
	const char *host;
	int port, duration, mode;

	if (g_iperfClient != NULL || g_clientStartPending) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf client is already running, use IperfStop first");
		return CMD_RES_ERROR;
	}
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "Usage: IperfClient [targetIP] [port] [seconds] [mode 0=tx/1=dual/2=tradeoff]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	host = Tokenizer_GetArg(0);
	if (!ipaddr_aton(host, &g_reqClientAddr)) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: '%s' is not a valid IP address", host);
		return CMD_RES_BAD_ARGUMENT;
	}
	port = Tokenizer_GetArgIntegerDefault(1, OBK_LWIPERF_TCP_PORT_DEFAULT);
	duration = Tokenizer_GetArgIntegerDefault(2, 10);
	mode = Tokenizer_GetArgIntegerDefault(3, OBK_LWIPERF_CLIENT);
	if (port <= 0 || port > 65535 || duration <= 0
		|| mode < OBK_LWIPERF_CLIENT || mode > OBK_LWIPERF_TRADEOFF) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: bad argument (port %i, seconds %i, mode %i)", port, duration, mode);
		return CMD_RES_BAD_ARGUMENT;
	}
	g_reqClientPort = (u16_t)port;
	g_reqClientDuration = (u32_t)duration;
	g_reqClientMode = (enum obk_lwiperf_client_type)mode;
	g_clientStartPending = 1;
	if (tcpip_callback(Iperf_StartClient_TcpipThread, NULL) != ERR_OK) {
		g_clientStartPending = 0;
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: tcpip_callback failed");
		return CMD_RES_ERROR;
	}
	ADDLOG_INFO(LOG_FEATURE_CMD, "iperf: connecting to %s:%i", host, port);
	return CMD_RES_OK;
}

static commandResult_t CMD_IperfStop(const void *context, const char *cmd, const char *args, int cmdFlags) {
	if (tcpip_callback(Iperf_Stop_TcpipThread, NULL) != ERR_OK) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "iperf: tcpip_callback failed");
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}

void CMD_InitIperf() {
	//cmddetail:{"name":"IperfServer","args":"[OptionalPort]",
	//cmddetail:"descr":"Starts an iperf2-compatible TCP server (default port 5001). Test from a PC with 'iperf -c [deviceIP]' (iperf version 2). Results are printed to the console log.",
	//cmddetail:"fn":"CMD_IperfServer","file":"cmnds/cmd_iperf.c","requires":"",
	//cmddetail:"examples":"IperfServer"}
	CMD_RegisterCommand("IperfServer", CMD_IperfServer, NULL);
	//cmddetail:{"name":"IperfClient","args":"[TargetIP][OptionalPort][OptionalSeconds][OptionalMode]",
	//cmddetail:"descr":"Starts an iperf2-compatible TCP client that sends data to the given host (default port 5001, default 10 seconds). Run 'iperf -s' (iperf version 2) on the target first. Mode: 0 = TX only (default), 1 = dual, 2 = tradeoff. Results are printed to the console log.",
	//cmddetail:"fn":"CMD_IperfClient","file":"cmnds/cmd_iperf.c","requires":"",
	//cmddetail:"examples":"IperfClient 192.168.0.123"}
	CMD_RegisterCommand("IperfClient", CMD_IperfClient, NULL);
	//cmddetail:{"name":"IperfStop","args":"",
	//cmddetail:"descr":"Stops the running iperf server and/or client sessions.",
	//cmddetail:"fn":"CMD_IperfStop","file":"cmnds/cmd_iperf.c","requires":"",
	//cmddetail:"examples":"IperfStop"}
	CMD_RegisterCommand("IperfStop", CMD_IperfStop, NULL);
}

#endif // ENABLE_IPERF
