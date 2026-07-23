// NTP client
// Based on my previous work here:
// https://www.elektroda.pl/rtvforum/topic3712112.html
#include "../obk_config.h"
#if ENABLE_NTP
//#include <time.h>

#include "../new_common.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../hal/hal_ota.h"
#include "drv_deviceclock.h"	// for TIME_Init()
#include "../libraries/obktime/obktime.h"	// for time functions
#include "drv_ntp.h"

// Platform-agnostic tick wrappers to support both BL602 (FreeRTOS) and Windows Simulator
#if WINDOWS
#include <windows.h>
#define GET_MCU_TICKS() GetTickCount()
#define MCU_TICKS_PER_SEC 1000
#else
#include "FreeRTOS.h"
#include "task.h"
#define GET_MCU_TICKS() xTaskGetTickCount()
#ifdef configTICK_RATE_HZ
#define MCU_TICKS_PER_SEC configTICK_RATE_HZ
#else
#define MCU_TICKS_PER_SEC 1000
#endif
#endif

#define LOG_FEATURE LOG_FEATURE_NTP

typedef struct
{

  uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
                           // li.   Two bits.   Leap indicator.
                           // vn.   Three bits. Version number of the protocol.
                           // mode. Three bits. Client will pick mode 3 for client.

  uint8_t stratum;         // Eight bits. Stratum level of the local clock.
  uint8_t poll;            // Eight bits. Maximum interval between successive messages.
  uint8_t precision;       // Eight bits. Precision of the local clock.

  uint32_t rootDelay;      // 32 bits. Total round trip delay time.
  uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
  uint32_t refId;          // 32 bits. Reference clock identifier.

  uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
  uint32_t refTm_f;        // 32 bits. Reference time-stamp fraction of a second.

  uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
  uint32_t origTm_f;       // 32 bits. Originate time-stamp fraction of a second.

  uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
  uint32_t rxTm_f;         // 32 bits. Received time-stamp fraction of a second.

  uint32_t txTm_s;         // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
  uint32_t txTm_f;         // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet;              // Total: 384 bits or 48 bytes.

#define MAKE_WORD(hi, lo) hi << 8 | lo

// NTP time since 1900 to unix time (since 1970)
// Number of seconds to ad
#define NTP_OFFSET 2208988800L

static int g_ntp_socket = 0;
static struct sockaddr_in g_address;
static int adrLen;
// in seconds, before next retry
static int g_ntp_delay = 0;
static bool g_synced = false;
// time offset (time zone?) in seconds
//#define CFG_DEFAULT_TIMEOFFSETSECONDS (-8 * 60 * 60)
static int g_timeOffsetSeconds = 0;
// current time - this may be 32 or 64 bit, depending on platform
// don't use as global variable, use functions to access and manipulate "clock" in "drv_deviceclock.c"
time_t g_ntpTime;
static unsigned int g_ntp_syncinterval=60;

// --- Sub-Second Clock Drift Tracking Variables ---
static bool g_drift_initialized = false;
static uint32_t g_baseline_ntp_secs = 0;   // Baseline sync whole seconds
static float g_baseline_ntp_frac = 0.0f;   // Baseline sync fraction of a second
static uint32_t g_last_ntp_secs = 0;       // Previous sync whole seconds
static float g_last_ntp_frac = 0.0f;       // Previous sync fraction of a second
static uint32_t g_last_ticks = 0;          // MCU Ticks recorded at previous sync
static float g_accumulated_drift_seconds = 0.0f; 
static float g_drift_ppm = 0.0f;           // Parts Per Million drift rate

int NTP_GetTimesZoneOfsSeconds()
{
    return g_timeOffsetSeconds;
}

// set offset seconds directly
void NTP_SetTimesZoneOfsSeconds(int o) {
/*
*/	
	g_timeOffsetSeconds = o;		// set new offset
	TIME_setDeviceTimeOffset(g_timeOffsetSeconds);
}

void NTP_Init() {

	//cmddetail:{"name":"ntp_timeZoneOfs","args":"[Value]",
	//cmddetail:"descr":"Sets the time zone offset in hours. Also supports HH:MM syntax if you want to specify value in minutes. For negative values, use -HH:MM syntax, for example -5:30 will shift time by 5 hours and 30 minutes negative.",
	//cmddetail:"fn":"SetTimeZoneOfs","file":"driver/drv_ntp.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("ntp_timeZoneOfs",SetTimeZoneOfs, NULL);
    
    g_ntp_syncinterval = Tokenizer_GetArgIntegerDefault(1, 60);

    addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "NTP driver initialized with server=%s, offset=%d, syncing every %i seconds", CFG_GetNTPServer(), g_timeOffsetSeconds, g_ntp_syncinterval);
    g_synced = false;
    g_drift_initialized = false;
}

// if driver is stopped, we need to make sure, we don't keep NTP in state "synched"
void NTP_Stop() {
    addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "NTP driver stopped");
    g_synced = false;
    g_drift_initialized = false;
}

// just for compatibility 
unsigned int NTP_GetCurrentTime() {
    return TIME_GetCurrentTime();
}
unsigned int NTP_GetCurrentTimeWithoutOffset() {
	return TIME_GetCurrentTimeWithoutOffset();
}

void NTP_Shutdown() {
    if(g_ntp_socket != 0) {
#if WINDOWS
        closesocket(g_ntp_socket);
#else
        lwip_close(g_ntp_socket);
#endif
    }
    g_ntp_socket = 0;
    // can attempt in next 10 seconds
    g_ntp_delay = g_ntp_syncinterval-1;
}

void NTP_SendRequest(bool bBlocking) {
    byte *ptr;
	const char *adrString;
    ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    adrLen = sizeof(g_address);
    memset( &packet, 0, sizeof( ntp_packet ) );
    ptr = (byte*)&packet;
    ptr[0] = 0xE3;   // LI, Version, Mode
    ptr[1] = 0;     // Stratum, or type of clock
    ptr[2] = 6;     // Polling Interval
    ptr[3] = 0xEC;  // Peer Clock Precision
    ptr[12]  = 49;
    ptr[13]  = 0x4E;
    ptr[14]  = 49;
    ptr[15]  = 52;

    if ((g_ntp_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP )) == -1)
    {
        g_ntp_socket = 0;
        addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP_SendRequest: failed to create socket");
        return;
    }

    memset((char *) &g_address, 0, sizeof(g_address));

	adrString = CFG_GetNTPServer();
	if (adrString == 0 || adrString[0] == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "NTP_SendRequest: somehow ntp server in config was empty, setting non-empty");
		CFG_SetNTPServer(DEFAULT_NTP_SERVER);
		adrString = CFG_GetNTPServer();
	}

    g_address.sin_family = AF_INET;
    g_address.sin_addr.s_addr = inet_addr(adrString);
    g_address.sin_port = htons(123);

    if(sendto(g_ntp_socket, &packet, sizeof(packet), 0,
         (struct sockaddr*)&g_address, adrLen) < 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP_SendRequest: Unable to send message");
        NTP_Shutdown();
		if (g_secondsElapsed < 60) {
			g_ntp_delay = 0;
		}
        return;
    }

    if(bBlocking == false) {
#if WINDOWS
#else
        if(fcntl(g_ntp_socket, F_SETFL, O_NONBLOCK)) {
            addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP_SendRequest: failed to make socket non-blocking!");
        }
#endif
    }

    g_ntp_delay = 10;
}

void NTP_CheckForReceive() {
    byte *ptr;
    int i, recv_len;
    unsigned short highWord;
    unsigned short lowWord;
    unsigned int secsSince1900;
    ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    ptr = (byte*)&packet;

    i = sizeof(packet);
#if 0
    recv_len = recvfrom(g_ntp_socket, ptr, i, 0,
         (struct sockaddr*)&g_address, &adrLen);
#else
    recv_len = recv(g_ntp_socket, ptr, i, 0);
#endif

    if(recv_len < 0){
        addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP_CheckForReceive: Error while receiving server's msg");
        return;
    }
    // Must be at least 48 bytes to contain both transmit seconds (40-43) and fraction (44-47)
    if(recv_len < 48){
        addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP_CheckForReceive: response too short for millisecond parsing (%d bytes)", recv_len);
        return;
    }

    // Extract seconds (Bytes 40-43)
    highWord = MAKE_WORD(ptr[40], ptr[41]);
    lowWord = MAKE_WORD(ptr[42], ptr[43]);
    secsSince1900 = highWord << 16 | lowWord;

    // Extract fractional part of the second (Bytes 44-47)
    unsigned short fracHighWord = MAKE_WORD(ptr[44], ptr[45]);
    unsigned short fracLowWord = MAKE_WORD(ptr[46], ptr[47]);
    uint32_t rawFraction = ((uint32_t)fracHighWord << 16) | fracLowWord;
    
    // Scale fraction to structural float percentage (0.0 to 1.0 seconds)
    float current_ntp_frac = (float)rawFraction / 4294967296.0f;

    addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"NTP Time: %u.%.3f", secsSince1900, current_ntp_frac);

    // ---- Enhanced Sub-Second Drift Calculation Engine ----
    {
        uint32_t current_ntp_secs = secsSince1900 - NTP_OFFSET;
        uint32_t current_ticks = GET_MCU_TICKS();

        if (!g_drift_initialized) {
            g_baseline_ntp_secs = current_ntp_secs;
            g_baseline_ntp_frac = current_ntp_frac;
            g_last_ntp_secs = current_ntp_secs;
            g_last_ntp_frac = current_ntp_frac;
            g_last_ticks = current_ticks;
            g_accumulated_drift_seconds = 0.0f;
            g_drift_ppm = 0.0f;
            g_drift_initialized = true;
            addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "High-resolution drift baseline tracking established.");
        } else {
            uint32_t ticks_delta = current_ticks - g_last_ticks;
            uint32_t ntp_delta_secs = current_ntp_secs - g_last_ntp_secs;
            float ntp_delta_frac = current_ntp_frac - g_last_ntp_frac;
            
            // Combine delta components directly to bypass float resolution truncation limits
            float ntp_delta_total = (float)ntp_delta_secs + ntp_delta_frac;

            if (ntp_delta_total > 0.0f) {
                float local_seconds_delta = (float)ticks_delta / (float)MCU_TICKS_PER_SEC;
                float interval_drift = local_seconds_delta - ntp_delta_total;
                
                g_accumulated_drift_seconds += interval_drift;
                
                // Compute absolute duration from baseline baseline for tracking stability
                uint32_t total_ntp_secs = current_ntp_secs - g_baseline_ntp_secs;
                float total_ntp_frac = current_ntp_frac - g_baseline_ntp_frac;
                float total_ntp_elapsed = (float)total_ntp_secs + total_ntp_frac;
                
                if (total_ntp_elapsed > 0.0f) {
                    g_drift_ppm = (g_accumulated_drift_seconds / total_ntp_elapsed) * 1000000.0f;
                }

                addLogAdv(LOG_INFO, LOG_FEATURE_NTP, "Drift Update: Int=%.4fs, Cumul=%.4fs, Rate=%.2f PPM", 
                          interval_drift, g_accumulated_drift_seconds, g_drift_ppm);
                
                g_last_ntp_secs = current_ntp_secs;
                g_last_ntp_frac = current_ntp_frac;
                g_last_ticks = current_ticks;
            }
        }
    }
    // ------------------------------------------------------

    TIME_setDeviceTime((uint32_t) (secsSince1900 - NTP_OFFSET) );
    addLogAdv(LOG_INFO, LOG_FEATURE_NTP,"Unix time: %u - local Time %s",(uint32_t) (secsSince1900 - NTP_OFFSET),TS2STR(TIME_GetCurrentTime(),TIME_FORMAT_LONG));

	if (g_synced == false) {
		EventHandlers_FireEvent(CMD_EVENT_NTP_STATE, 1);
	}
    g_synced = true;

    NTP_Shutdown();
}

void NTP_SendRequest_BlockingMode() {
    NTP_Shutdown();
    NTP_SendRequest(true);
    NTP_CheckForReceive();
}

void NTP_OnEverySecond()
{
    if(Main_IsConnectedToWiFi()==0)
    {
        return;
    }
#if WINDOWS
	if (b_ntp_simulatedTime) {
		return;
	}
#endif
    if (OTA_GetProgress() != -1)
    {
        return;
    }
    if(g_ntp_socket == 0) {
        if(g_ntp_delay > 0) {
            g_ntp_delay--;
            return;
        }
        NTP_SendRequest(false);
    } else {
        NTP_CheckForReceive();
        if(g_ntp_delay > 0) {
            g_ntp_delay--;
            if(g_ntp_delay<=0) {
                NTP_Shutdown();
            }
        }
    }
}

void NTP_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if (bPreState)
		return;

    if (g_synced != true) {
        hprintf255(request, "<h5>NTP: Syncing with %s....</h5>",CFG_GetNTPServer());
    } else {
        if (g_drift_initialized) {
            hprintf255(request, "<h5>NTP: Synced. Clock Drift: %.4fs (~%.1f PPM)</h5>", g_accumulated_drift_seconds, g_drift_ppm);
        } else {
            hprintf255(request, "<h5>NTP: Synced.</h5>");
        }
    }
}

bool NTP_IsTimeSynced()
{
    return g_synced;
}

#endif // #if ENABLE_NTP