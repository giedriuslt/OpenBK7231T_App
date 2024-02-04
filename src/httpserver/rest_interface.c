#include "../obk_config.h"

#include "../new_common.h"
#include "../logging/logging.h"
#include "../httpserver/new_http.h"
#include "../new_pins.h"
#include "../jsmn/jsmn_h.h"
#include "../ota/ota.h"
#include "../hal/hal_wifi.h"
#include "../hal/hal_flashVars.h"
#include "../littlefs/our_lfs.h"
#include "lwip/sockets.h"

#if PLATFORM_XR809

#include <image/flash.h>

uint32_t flash_read(uint32_t flash, uint32_t addr, void* buf, uint32_t size);
#define FLASH_INDEX_XR809 0

#elif PLATFORM_BL602
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <lwip/mem.h>
#include <lwip/memp.h>
#include <lwip/dhcp.h>
#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

#include <cli.h>
#include <hal_boot2.h>
#include <hal_sys.h>
#include <utils_sha256.h>
#include <bl_sys_ota.h>
#include <bl_mtd.h>
#include <bl_flash.h>
#elif PLATFORM_W600

#include "wm_socket_fwup.h"
#include "wm_fwup.h"

#elif PLATFORM_W800

#elif PLATFORM_LN882H

#else

extern UINT32 flash_read(char* user_buf, UINT32 count, UINT32 address);

#endif

#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"

#ifndef OBK_DISABLE_ALL_DRIVERS
#include "../driver/drv_local.h"
#endif

#define MAX_JSON_VALUE_LENGTH   128


static int http_rest_error(http_request_t* request, int code, char* msg);

static int http_rest_get(http_request_t* request);
static int http_rest_post(http_request_t* request);
static int http_rest_app(http_request_t* request);

static int http_rest_post_pins(http_request_t* request);
static int http_rest_get_pins(http_request_t* request);

static int http_rest_get_channelTypes(http_request_t* request);
static int http_rest_post_channelTypes(http_request_t* request);

static int http_rest_get_seriallog(http_request_t* request);

static int http_rest_post_logconfig(http_request_t* request);
static int http_rest_get_logconfig(http_request_t* request);

#if ENABLE_LITTLEFS
static int http_rest_get_lfs_delete(http_request_t* request);
static int http_rest_get_lfs_file(http_request_t* request);
static int http_rest_post_lfs_file(http_request_t* request);
#endif

static int http_rest_post_reboot(http_request_t* request);
static int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr);
static int http_rest_post_dry(http_request_t* request, int startaddr, int maxaddr);
static int http_rest_get_flash(http_request_t* request, int startaddr, int len);
static int http_rest_get_flash_advanced(http_request_t* request);
static int http_rest_post_flash_advanced(http_request_t* request);

static int http_rest_get_info(http_request_t* request);

static int http_rest_get_dumpconfig(http_request_t* request);
static int http_rest_get_testconfig(http_request_t* request);

static int http_rest_post_channels(http_request_t* request);
static int http_rest_get_channels(http_request_t* request);

static int http_rest_get_flash_vars_test(http_request_t* request);

static int http_rest_post_cmd(http_request_t* request);


void init_rest() {
	HTTP_RegisterCallback("/api/", HTTP_GET, http_rest_get, 1);
	HTTP_RegisterCallback("/api/", HTTP_POST, http_rest_post, 1);
	HTTP_RegisterCallback("/app", HTTP_GET, http_rest_app, 1);
}

/* Extracts string token value into outBuffer (128 char). Returns true if the operation was successful. */
bool tryGetTokenString(const char* json, jsmntok_t* tok, char* outBuffer) {
	int length;
	if (tok == NULL || tok->type != JSMN_STRING) {
		return false;
	}

	length = tok->end - tok->start;

	//Don't have enough buffer
	if (length > MAX_JSON_VALUE_LENGTH) {
		return false;
	}

	memset(outBuffer, '\0', MAX_JSON_VALUE_LENGTH); //Wipe previous value
	strncpy(outBuffer, json + tok->start, length);
	return true;
}

static int http_rest_get(http_request_t* request) {
	ADDLOG_DEBUG(LOG_FEATURE_API, "GET of %s", request->url);

	if (!strcmp(request->url, "api/channels")) {
		return http_rest_get_channels(request);
	}

	if (!strcmp(request->url, "api/pins")) {
		return http_rest_get_pins(request);
	}
	if (!strcmp(request->url, "api/channelTypes")) {
		return http_rest_get_channelTypes(request);
	}
	if (!strcmp(request->url, "api/logconfig")) {
		return http_rest_get_logconfig(request);
	}

	if (!strncmp(request->url, "api/seriallog", 13)) {
		return http_rest_get_seriallog(request);
	}

#if ENABLE_LITTLEFS
	if (!strcmp(request->url, "api/fsblock")) {
		uint32_t newsize = CFG_GetLFS_Size();
		uint32_t newstart = (LFS_BLOCKS_END - newsize);

		newsize = (newsize / LFS_BLOCK_SIZE) * LFS_BLOCK_SIZE;

		// double check again that we're within bounds - don't want
		// boot overwrite or anything nasty....
		if (newstart < LFS_BLOCKS_START_MIN) {
			return http_rest_error(request, 400, "LFS Size mismatch");
		}
		if ((newstart + newsize > LFS_BLOCKS_END) ||
			(newstart + newsize < LFS_BLOCKS_START_MIN)) {
			return http_rest_error(request, 400, "LFS Size mismatch");
		}

		return http_rest_get_flash(request, newstart, newsize);
	}
#endif

#if ENABLE_LITTLEFS
	if (!strncmp(request->url, "api/lfs/", 8)) {
		return http_rest_get_lfs_file(request);
	}
	if (!strncmp(request->url, "api/del/", 8)) {
		return http_rest_get_lfs_delete(request);
	}
#endif

	if (!strcmp(request->url, "api/info")) {
		return http_rest_get_info(request);
	}

	if (!strncmp(request->url, "api/flash/", 10)) {
		return http_rest_get_flash_advanced(request);
	}

	if (!strcmp(request->url, "api/dumpconfig")) {
		return http_rest_get_dumpconfig(request);
	}

	if (!strcmp(request->url, "api/testconfig")) {
		return http_rest_get_testconfig(request);
	}

	if (!strncmp(request->url, "api/testflashvars", 17)) {
		return http_rest_get_flash_vars_test(request);
	}

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "GET REST API");
	poststr(request, "GET of ");
	poststr(request, request->url);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

static int http_rest_post(http_request_t* request) {
	char tmp[20];
	ADDLOG_DEBUG(LOG_FEATURE_API, "POST to %s", request->url);

	if (!strcmp(request->url, "api/channels")) {
		return http_rest_post_channels(request);
	}
	
	if (!strcmp(request->url, "api/pins")) {
		return http_rest_post_pins(request);
	}
	if (!strcmp(request->url, "api/channelTypes")) {
		return http_rest_post_channelTypes(request);
	}
	if (!strcmp(request->url, "api/logconfig")) {
		return http_rest_post_logconfig(request);
	}

	if (!strcmp(request->url, "api/reboot")) {
		return http_rest_post_reboot(request);
	}
	if (!strcmp(request->url, "api/ota")) {
#if PLATFORM_BK7231T
		return http_rest_post_flash(request, START_ADR_OF_BK_PARTITION_OTA, LFS_BLOCKS_END);
#elif PLATFORM_BK7231N
		return http_rest_post_flash(request, START_ADR_OF_BK_PARTITION_OTA, LFS_BLOCKS_END);
#elif PLATFORM_W600
		return http_rest_post_flash(request, -1, -1);
#elif PLATFORM_BL602
		return http_rest_post_flash(request, -1, -1);
#elif PLATFORM_LN882H
		return http_rest_post_flash(request, -1, -1);
#else
		// TODO
		ADDLOG_DEBUG(LOG_FEATURE_API, "No OTA");
#endif
	}
	if (!strcmp(request->url, "api/ot2")) {
#if PLATFORM_BK7231T
		return http_rest_post_dry(request, START_ADR_OF_BK_PARTITION_OTA, LFS_BLOCKS_END);
#elif PLATFORM_BK7231N
		return http_rest_post_dry(request, START_ADR_OF_BK_PARTITION_OTA, LFS_BLOCKS_END);
#elif PLATFORM_W600
		return http_rest_post_dry(request, -1, -1);
#elif PLATFORM_BL602
		return http_rest_post_dry(request, -1, -1);
#elif PLATFORM_LN882H
		return http_rest_post_dry(request, -1, -1);
#else
		// TODO
		ADDLOG_DEBUG(LOG_FEATURE_API, "No OTA");
#endif
	}
	if (!strncmp(request->url, "api/flash/", 10)) {
		return http_rest_post_flash_advanced(request);
	}

	if (!strcmp(request->url, "api/cmnd")) {
		return http_rest_post_cmd(request);
	}


#if ENABLE_LITTLEFS
	if (!strcmp(request->url, "api/fsblock")) {
		if (lfs_present()) {
			release_lfs();
		}
		uint32_t newsize = CFG_GetLFS_Size();
		uint32_t newstart = (LFS_BLOCKS_END - newsize);

		newsize = (newsize / LFS_BLOCK_SIZE) * LFS_BLOCK_SIZE;

		// double check again that we're within bounds - don't want
		// boot overwrite or anything nasty....
		if (newstart < LFS_BLOCKS_START_MIN) {
			return http_rest_error(request, 400, "LFS Size mismatch");
		}
		if ((newstart + newsize > LFS_BLOCKS_END) ||
			(newstart + newsize < LFS_BLOCKS_START_MIN)) {
			return http_rest_error(request, 400, "LFS Size mismatch");
		}

		// we are writing the lfs block
		int res = http_rest_post_flash(request, newstart, LFS_BLOCKS_END);
		// initialise the filesystem, it should be there now.
		// don't create if it does not mount
		init_lfs(0);
		return res;
	}
	if (!strncmp(request->url, "api/lfs/", 8)) {
		return http_rest_post_lfs_file(request);
	}
#endif

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "POST REST API");
	poststr(request, "POST to ");
	poststr(request, request->url);
	poststr(request, "<br/>Content Length:");
	sprintf(tmp, "%d", request->contentLength);
	poststr(request, tmp);
	poststr(request, "<br/>Content:[");
	poststr(request, request->bodystart);
	poststr(request, "]<br/>");
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

static int http_rest_app(http_request_t* request) {
	const char* webhost = CFG_GetWebappRoot();
	const char* ourip = HAL_GetMyIPString(); //CFG_GetOurIP();
	http_setup(request, httpMimeTypeHTML);
	if (webhost && ourip) {
		poststr(request, htmlDoctype);

		poststr(request, "<head><title>");
		poststr(request, CFG_GetDeviceName());
		poststr(request, "</title>");

		poststr(request, htmlShortcutIcon);
		poststr(request, htmlHeadMeta);
		hprintf255(request, "<script>var root='%s',device='http://%s';</script>", webhost, ourip);
		hprintf255(request, "<script src='%s/startup.js'></script>", webhost);
		poststr(request, "</head><body></body></html>");
	}
	else {
		http_html_start(request, "Not available");
		poststr(request, htmlFooterReturnToMenu);
		poststr(request, "no APP available<br/>");
		http_html_end(request);
	}
	poststr(request, NULL);
	return 0;
}

#if ENABLE_LITTLEFS

int EndsWith(const char* str, const char* suffix)
{
	if (!str || !suffix)
		return 0;
	size_t lenstr = strlen(str);
	size_t lensuffix = strlen(suffix);
	if (lensuffix > lenstr)
		return 0;
	return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

static int http_rest_get_lfs_file(http_request_t* request) {
	char* fpath;
	char* buff;
	int len;
	int lfsres;
	int total = 0;
	lfs_file_t* file;
	char *args;

	// don't start LFS just because we're trying to read a file -
	// it won't exist anyway
	if (!lfs_present()) {
		request->responseCode = HTTP_RESPONSE_NOT_FOUND;
		http_setup(request, httpMimeTypeText);
		poststr(request, NULL);
		return 0;
	}

	fpath = os_malloc(strlen(request->url) - strlen("api/lfs/") + 1);

	buff = os_malloc(1024);
	file = os_malloc(sizeof(lfs_file_t));
	memset(file, 0, sizeof(lfs_file_t));

	strcpy(fpath, request->url + strlen("api/lfs/"));

	// strip HTTP args with ?
	args = strchr(fpath, '?');
	if (args) {
		*args = 0;
	}

	ADDLOG_DEBUG(LOG_FEATURE_API, "LFS read of %s", fpath);
	lfsres = lfs_file_open(&lfs, file, fpath, LFS_O_RDONLY);

	if (lfsres == -21) {
		lfs_dir_t* dir;
		ADDLOG_DEBUG(LOG_FEATURE_API, "%s is a folder", fpath);
		dir = os_malloc(sizeof(lfs_dir_t));
		os_memset(dir, 0, sizeof(*dir));
		// if the thing is a folder.
		lfsres = lfs_dir_open(&lfs, dir, fpath);

		if (lfsres >= 0) {
			// this is needed during iteration...?
			struct lfs_info info;
			int count = 0;
			http_setup(request, httpMimeTypeJson);
			ADDLOG_DEBUG(LOG_FEATURE_API, "opened folder %s lfs result %d", fpath, lfsres);
			hprintf255(request, "{\"dir\":\"%s\",\"content\":[", fpath);
			do {
				// Read an entry in the directory
				//
				// Fills out the info structure, based on the specified file or directory.
				// Returns a positive value on success, 0 at the end of directory,
				// or a negative error code on failure.
				lfsres = lfs_dir_read(&lfs, dir, &info);
				if (lfsres > 0) {
					if (count) poststr(request, ",");
					hprintf255(request, "{\"name\":\"%s\",\"type\":%d,\"size\":%d}",
						info.name, info.type, info.size);
				}
				else {
					if (lfsres < 0) {
						if (count) poststr(request, ",");
						hprintf255(request, "{\"error\":%d}", lfsres);
					}
				}
				count++;
			} while (lfsres > 0);

			hprintf255(request, "]}");

			lfs_dir_close(&lfs, dir);
			if (dir) os_free(dir);
			dir = NULL;
		}
		else {
			if (dir) os_free(dir);
			dir = NULL;
			request->responseCode = HTTP_RESPONSE_NOT_FOUND;
			http_setup(request, httpMimeTypeJson);
			ADDLOG_DEBUG(LOG_FEATURE_API, "failed to open %s lfs result %d", fpath, lfsres);
			hprintf255(request, "{\"fname\":\"%s\",\"error\":%d}", fpath, lfsres);
		}
	}
	else {
		ADDLOG_DEBUG(LOG_FEATURE_API, "LFS open [%s] gives %d", fpath, lfsres);
		if (lfsres >= 0) {
			const char* mimetype = httpMimeTypeBinary;
			do {
				if (EndsWith(fpath, ".ico")) {
					mimetype = "image/x-icon";
					break;
				}
				if (EndsWith(fpath, ".js")) {
					mimetype = "text/javascript";
					break;
				}
				if (EndsWith(fpath, ".json")) {
					mimetype = httpMimeTypeJson;
					break;
				}
				if (EndsWith(fpath, ".html")) {
					mimetype = "text/html";
					break;
				}
				if (EndsWith(fpath, ".vue")) {
					mimetype = "application/javascript";
					break;
				}
				break;
			} while (0);

			http_setup(request, mimetype);
			do {
				len = lfs_file_read(&lfs, file, buff, 1024);
				total += len;
				if (len) {
					//ADDLOG_DEBUG(LOG_FEATURE_API, "%d bytes read", len);
					postany(request, buff, len);
				}
			} while (len > 0);
			lfs_file_close(&lfs, file);
			ADDLOG_DEBUG(LOG_FEATURE_API, "%d total bytes read", total);
		}
		else {
			request->responseCode = HTTP_RESPONSE_NOT_FOUND;
			http_setup(request, httpMimeTypeJson);
			ADDLOG_DEBUG(LOG_FEATURE_API, "failed to open %s lfs result %d", fpath, lfsres);
			hprintf255(request, "{\"fname\":\"%s\",\"error\":%d}", fpath, lfsres);
		}
	}
	poststr(request, NULL);
	if (fpath) os_free(fpath);
	if (file) os_free(file);
	if (buff) os_free(buff);
	return 0;
}

static int http_rest_get_lfs_delete(http_request_t* request) {
	char* fpath;
	int lfsres;

	// don't start LFS just because we're trying to read a file -
	// it won't exist anyway
	if (!lfs_present()) {
		request->responseCode = HTTP_RESPONSE_NOT_FOUND;
		http_setup(request, httpMimeTypeText);
		poststr(request, "Not found");
		poststr(request, NULL);
		return 0;
	}

	fpath = os_malloc(strlen(request->url) - strlen("api/del/") + 1);

	strcpy(fpath, request->url + strlen("api/del/"));

	ADDLOG_DEBUG(LOG_FEATURE_API, "LFS delete of %s", fpath);
	lfsres = lfs_remove(&lfs, fpath);

	if (lfsres == LFS_ERR_OK) {
		ADDLOG_DEBUG(LOG_FEATURE_API, "LFS delete of %s OK", fpath);

		poststr(request, "OK");
	}
	else {
		ADDLOG_DEBUG(LOG_FEATURE_API, "LFS delete of %s error %i", fpath, lfsres);
		poststr(request, "Error");
	}
	poststr(request, NULL);
	if (fpath) os_free(fpath);
	return 0;
}

static int http_rest_post_lfs_file(http_request_t* request) {
	int len;
	int lfsres;
	int total = 0;

	// allocated variables
	lfs_file_t* file;
	char* fpath;
	char* folder;

	// create if it does not exist
	init_lfs(1);

	fpath = os_malloc(strlen(request->url) - strlen("api/lfs/") + 1);
	file = os_malloc(sizeof(lfs_file_t));
	memset(file, 0, sizeof(lfs_file_t));

	strcpy(fpath, request->url + strlen("api/lfs/"));
	ADDLOG_DEBUG(LOG_FEATURE_API, "LFS write of %s len %d", fpath, request->contentLength);

	folder = strchr(fpath, '/');
	if (folder) {
		int folderlen = folder - fpath;
		folder = os_malloc(folderlen + 1);
		strncpy(folder, fpath, folderlen);
		folder[folderlen] = 0;
		ADDLOG_DEBUG(LOG_FEATURE_API, "file is in folder %s try to create", folder);
		lfsres = lfs_mkdir(&lfs, folder);
		if (lfsres < 0) {
			ADDLOG_DEBUG(LOG_FEATURE_API, "mkdir error %d", lfsres);
		}
	}

	//ADDLOG_DEBUG(LOG_FEATURE_API, "LFS write of %s len %d", fpath, request->contentLength);

	lfsres = lfs_file_open(&lfs, file, fpath, LFS_O_RDWR | LFS_O_CREAT);
	if (lfsres >= 0) {
		//ADDLOG_DEBUG(LOG_FEATURE_API, "opened %s");
		int towrite = request->bodylen;
		char* writebuf = request->bodystart;
		int writelen = request->bodylen;
		if (request->contentLength >= 0) {
			towrite = request->contentLength;
		}
		//ADDLOG_DEBUG(LOG_FEATURE_API, "bodylen %d, contentlen %d", request->bodylen, request->contentLength);

		if (writelen < 0) {
			ADDLOG_DEBUG(LOG_FEATURE_API, "ABORTED: %d bytes to write", writelen);
			lfs_file_close(&lfs, file);
			request->responseCode = HTTP_RESPONSE_SERVER_ERROR;
			http_setup(request, httpMimeTypeJson);
			hprintf255(request, "{\"fname\":\"%s\",\"error\":%d}", fpath, -20);
			goto exit;
		}

		do {
			//ADDLOG_DEBUG(LOG_FEATURE_API, "%d bytes to write", writelen);
			len = lfs_file_write(&lfs, file, writebuf, writelen);
			if (len < 0) {
				ADDLOG_ERROR(LOG_FEATURE_API, "Failed to write to %s with error %i", fpath,len);
				break;
			}
			total += len;
			if (len > 0) {
				//ADDLOG_DEBUG(LOG_FEATURE_API, "%d bytes written", len);
			}
			towrite -= len;
			if (towrite > 0) {
				writebuf = request->received;
				writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
				if (writelen < 0) {
					ADDLOG_DEBUG(LOG_FEATURE_API, "recv returned %d - end of data - remaining %d", writelen, towrite);
				}
			}
		} while ((towrite > 0) && (writelen >= 0));

		// no more data
		lfs_file_truncate(&lfs, file, total);

		//ADDLOG_DEBUG(LOG_FEATURE_API, "closing %s", fpath);
		lfs_file_close(&lfs, file);
		ADDLOG_DEBUG(LOG_FEATURE_API, "%d total bytes written", total);
		http_setup(request, httpMimeTypeJson);
		hprintf255(request, "{\"fname\":\"%s\",\"size\":%d}", fpath, total);
	}
	else {
		request->responseCode = HTTP_RESPONSE_SERVER_ERROR;
		http_setup(request, httpMimeTypeJson);
		ADDLOG_DEBUG(LOG_FEATURE_API, "failed to open %s err %d", fpath, lfsres);
		hprintf255(request, "{\"fname\":\"%s\",\"error\":%d}", fpath, lfsres);
	}
exit:
	poststr(request, NULL);
	if (folder) os_free(folder);
	if (file) os_free(file);
	if (fpath) os_free(fpath);
	return 0;
}

// static int http_favicon(http_request_t* request) {
// 	request->url = "api/lfs/favicon.ico";
// 	return http_rest_get_lfs_file(request);
// }

#else
// static int http_favicon(http_request_t* request) {
// 	request->responseCode = HTTP_RESPONSE_NOT_FOUND;
// 	http_setup(request, httpMimeTypeHTML);
// 	poststr(request, NULL);
// 	return 0;
// }
#endif



static int http_rest_get_seriallog(http_request_t* request) {
	if (request->url[strlen(request->url) - 1] == '1') {
		direct_serial_log = 1;
	}
	else {
		direct_serial_log = 0;
	}
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "Direct serial logging set to %d", direct_serial_log);
	poststr(request, NULL);
	return 0;
}



static int http_rest_get_pins(http_request_t* request) {
	int i;
	http_setup(request, httpMimeTypeJson);
	poststr(request, "{\"rolenames\":[");
	for (i = 0; i < IOR_Total_Options; i++) {
		if (i) {
			hprintf255(request, ",");
		}
		hprintf255(request, "\"%s\"", htmlPinRoleNames[i]);
	}
	poststr(request, "],\"roles\":[");

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (i) {
			hprintf255(request, ",");
		}
		hprintf255(request, "%d", g_cfg.pins.roles[i]);
	}
	// TODO: maybe we should cull futher channels that are not used?
	// I support many channels because I plan to use 16x relays module with I2C MCP23017 driver
	poststr(request, "],\"channels\":[");
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (i) {
			hprintf255(request, ",");
		}
		hprintf255(request, "%d", g_cfg.pins.channels[i]);
	}
	poststr(request, "],\"states\":[");
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (i) {
			hprintf255(request, ",");
		}
		hprintf255(request, "%d", CHANNEL_Get(g_cfg.pins.channels[i]));
	}
	poststr(request, "]}");
	poststr(request, NULL);
	return 0;
}


static int http_rest_get_channelTypes(http_request_t* request) {
	int i;
	int maxToPrint;

	maxToPrint = 32;

	http_setup(request, httpMimeTypeJson);
	poststr(request, "{\"typenames\":[");
	for (i = 0; i < ChType_Max; i++) {
		if (i) {
			hprintf255(request, ",\"%s\"", g_channelTypeNames[i]);
		}
		else {
			hprintf255(request, "\"%s\"", g_channelTypeNames[i]);
		}
	}
	poststr(request, "],\"types\":[");

	for (i = 0; i < maxToPrint; i++) {
		if (i) {
			hprintf255(request, ",%d", g_cfg.pins.channelTypes[i]);
		}
		else {
			hprintf255(request, "%d", g_cfg.pins.channelTypes[i]);
		}
	}
	poststr(request, "]}");
	poststr(request, NULL);
	return 0;
}



////////////////////////////
// log config
static int http_rest_get_logconfig(http_request_t* request) {
	int i;
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"level\":%d,", g_loglevel);
	hprintf255(request, "\"features\":%d,", logfeatures);
	poststr(request, "\"levelnames\":[");
	for (i = 0; i < LOG_MAX; i++) {
		if (i) {
			hprintf255(request, ",\"%s\"", loglevelnames[i]);
		}
		else {
			hprintf255(request, "\"%s\"", loglevelnames[i]);
		}
	}
	poststr(request, "],\"featurenames\":[");
	for (i = 0; i < LOG_FEATURE_MAX; i++) {
		if (i) {
			hprintf255(request, ",\"%s\"", logfeaturenames[i]);
		}
		else {
			hprintf255(request, "\"%s\"", logfeaturenames[i]);
		}
	}
	poststr(request, "]}");
	poststr(request, NULL);
	return 0;
}

static int http_rest_post_logconfig(http_request_t* request) {
	int i;
	int r;
	char tmp[64];

	//https://github.com/zserge/jsmn/blob/master/example/simple.c
	//jsmn_parser p;
	jsmn_parser* p = os_malloc(sizeof(jsmn_parser));
	//jsmntok_t t[128]; /* We expect no more than 128 tokens */
#define TOKEN_COUNT 128
	jsmntok_t* t = os_malloc(sizeof(jsmntok_t) * TOKEN_COUNT);
	char* json_str = request->bodystart;
	int json_len = strlen(json_str);

	http_setup(request, httpMimeTypeText);
	memset(p, 0, sizeof(jsmn_parser));
	memset(t, 0, sizeof(jsmntok_t) * 128);

	jsmn_init(p);
	r = jsmn_parse(p, json_str, json_len, t, TOKEN_COUNT);
	if (r < 0) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Failed to parse JSON: %d", r);
		poststr(request, NULL);
		os_free(p);
		os_free(t);
		return 0;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Object expected", r);
		poststr(request, NULL);
		os_free(p);
		os_free(t);
		return 0;
	}

	//sprintf(tmp,"parsed JSON: %s\n", json_str);
	//poststr(request, tmp);
	//poststr(request, NULL);

		/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		if (jsoneq(json_str, &t[i], "level") == 0) {
			if (t[i + 1].type != JSMN_PRIMITIVE) {
				continue; /* We expect groups to be an array of strings */
			}
			g_loglevel = atoi(json_str + t[i + 1].start);
			i += t[i + 1].size + 1;
		}
		else if (jsoneq(json_str, &t[i], "features") == 0) {
			if (t[i + 1].type != JSMN_PRIMITIVE) {
				continue; /* We expect groups to be an array of strings */
			}
			logfeatures = atoi(json_str + t[i + 1].start);;
			i += t[i + 1].size + 1;
		}
		else {
			ADDLOG_ERROR(LOG_FEATURE_API, "Unexpected key: %.*s", t[i].end - t[i].start,
				json_str + t[i].start);
			snprintf(tmp, sizeof(tmp), "Unexpected key: %.*s\n", t[i].end - t[i].start,
				json_str + t[i].start);
			poststr(request, tmp);
		}
	}

	poststr(request, NULL);
	os_free(p);
	os_free(t);
	return 0;
}

/////////////////////////////////////////////////


static int http_rest_get_info(http_request_t* request) {
	char macstr[3 * 6 + 1];
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"uptime_s\":%d,", g_secondsElapsed);
	hprintf255(request, "\"build\":\"%s\",", g_build_str);
	hprintf255(request, "\"ip\":\"%s\",", HAL_GetMyIPString());
	hprintf255(request, "\"mac\":\"%s\",", HAL_GetMACStr(macstr));
	hprintf255(request, "\"flags\":\"%ld\",", *((long int*)&g_cfg.genericFlags));
	hprintf255(request, "\"mqtthost\":\"%s:%d\",", CFG_GetMQTTHost(), CFG_GetMQTTPort());
	hprintf255(request, "\"mqtttopic\":\"%s\",", CFG_GetMQTTClientId());
	hprintf255(request, "\"chipset\":\"%s\",", PLATFORM_MCU_NAME);
	hprintf255(request, "\"webapp\":\"%s\",", CFG_GetWebappRoot());
	hprintf255(request, "\"shortName\":\"%s\",", CFG_GetShortDeviceName());
	
	hprintf255(request, "\"startcmd\":\"%s\",", CFG_GetShortStartupCommand());
#ifndef OBK_DISABLE_ALL_DRIVERS
	hprintf255(request, "\"supportsSSDP\":%d,", DRV_IsRunning("SSDP") ? 1 : 0);
#else
	hprintf255(request, "\"supportsSSDP\":0,");
#endif

	hprintf255(request, "\"supportsClientDeviceDB\":true}");

	poststr(request, NULL);
	return 0;
}

static int http_rest_post_pins(http_request_t* request) {
	int i;
	int r;
	char tmp[64];
	int iChanged = 0;
	char tokenStrValue[MAX_JSON_VALUE_LENGTH + 1];

	//https://github.com/zserge/jsmn/blob/master/example/simple.c
	//jsmn_parser p;
	jsmn_parser* p = os_malloc(sizeof(jsmn_parser));
	//jsmntok_t t[128]; /* We expect no more than 128 tokens */
#define TOKEN_COUNT 128
	jsmntok_t* t = os_malloc(sizeof(jsmntok_t) * TOKEN_COUNT);
	char* json_str = request->bodystart;
	int json_len = strlen(json_str);

	memset(p, 0, sizeof(jsmn_parser));
	memset(t, 0, sizeof(jsmntok_t) * TOKEN_COUNT);

	jsmn_init(p);
	r = jsmn_parse(p, json_str, json_len, t, TOKEN_COUNT);
	if (r < 0) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Failed to parse JSON: %d", r);
		sprintf(tmp, "Failed to parse JSON: %d\n", r);
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Object expected", r);
		sprintf(tmp, "Object expected\n");
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		if (tryGetTokenString(json_str, &t[i], tokenStrValue) != true) {
			ADDLOG_DEBUG(LOG_FEATURE_API, "Parsing failed");
			continue;
		}
		//ADDLOG_DEBUG(LOG_FEATURE_API, "parsed %s", tokenStrValue);

		if (strcmp(tokenStrValue, "roles") == 0) {
			int j;
			if (t[i + 1].type != JSMN_ARRAY) {
				continue; /* We expect groups to be an array of strings */
			}
			for (j = 0; j < t[i + 1].size; j++) {
				int roleval, pr;
				jsmntok_t* g = &t[i + j + 2];
				roleval = atoi(json_str + g->start);
				pr = PIN_GetPinRoleForPinIndex(j);
				if (pr != roleval) {
					PIN_SetPinRoleForPinIndex(j, roleval);
					iChanged++;
				}
			}
			i += t[i + 1].size + 1;
		}
		else if (strcmp(tokenStrValue, "channels") == 0) {
			int j;
			if (t[i + 1].type != JSMN_ARRAY) {
				continue; /* We expect groups to be an array of strings */
			}
			for (j = 0; j < t[i + 1].size; j++) {
				int chanval, pr;
				jsmntok_t* g = &t[i + j + 2];
				chanval = atoi(json_str + g->start);
				pr = PIN_GetPinChannelForPinIndex(j);
				if (pr != chanval) {
					PIN_SetPinChannelForPinIndex(j, chanval);
					iChanged++;
				}
			}
			i += t[i + 1].size + 1;
		}
		else if (strcmp(tokenStrValue, "deviceFlag") == 0) {
			int flag;
			jsmntok_t* flagTok = &t[i + 1];
			if (flagTok == NULL || flagTok->type != JSMN_PRIMITIVE) {
				continue;
			}

			flag = atoi(json_str + flagTok->start);
			ADDLOG_DEBUG(LOG_FEATURE_API, "received deviceFlag %d", flag);

			if (flag >= 0 && flag <= 10) {
				CFG_SetFlag(flag, true);
				iChanged++;
			}

			i += t[i + 1].size + 1;
		}
		else if (strcmp(tokenStrValue, "deviceCommand") == 0) {
			if (tryGetTokenString(json_str, &t[i + 1], tokenStrValue) == true) {
				ADDLOG_DEBUG(LOG_FEATURE_API, "received deviceCommand %s", tokenStrValue);
				CFG_SetShortStartupCommand_AndExecuteNow(tokenStrValue);
				iChanged++;
			}

			i += t[i + 1].size + 1;
		}
		else {
			ADDLOG_ERROR(LOG_FEATURE_API, "Unexpected key: %.*s", t[i].end - t[i].start,
				json_str + t[i].start);
		}
	}
	if (iChanged) {
		CFG_Save_SetupTimer();
		ADDLOG_DEBUG(LOG_FEATURE_API, "Changed %d - saved to flash", iChanged);
	}

	os_free(p);
	os_free(t);
	return http_rest_error(request, 200, "OK");
}

static int http_rest_post_channelTypes(http_request_t* request) {
	int i;
	int r;
	char tmp[64];
	int iChanged = 0;
	char tokenStrValue[MAX_JSON_VALUE_LENGTH + 1];

	//https://github.com/zserge/jsmn/blob/master/example/simple.c
	//jsmn_parser p;
	jsmn_parser* p = os_malloc(sizeof(jsmn_parser));
	//jsmntok_t t[128]; /* We expect no more than 128 tokens */
#define TOKEN_COUNT 128
	jsmntok_t* t = os_malloc(sizeof(jsmntok_t) * TOKEN_COUNT);
	char* json_str = request->bodystart;
	int json_len = strlen(json_str);

	memset(p, 0, sizeof(jsmn_parser));
	memset(t, 0, sizeof(jsmntok_t) * TOKEN_COUNT);

	jsmn_init(p);
	r = jsmn_parse(p, json_str, json_len, t, TOKEN_COUNT);
	if (r < 0) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Failed to parse JSON: %d", r);
		sprintf(tmp, "Failed to parse JSON: %d\n", r);
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Object expected", r);
		sprintf(tmp, "Object expected\n");
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		if (tryGetTokenString(json_str, &t[i], tokenStrValue) != true) {
			ADDLOG_DEBUG(LOG_FEATURE_API, "Parsing failed");
			continue;
		}
		//ADDLOG_DEBUG(LOG_FEATURE_API, "parsed %s", tokenStrValue);

		if (strcmp(tokenStrValue, "types") == 0) {
			int j;
			if (t[i + 1].type != JSMN_ARRAY) {
				continue; /* We expect groups to be an array of strings */
			}
			for (j = 0; j < t[i + 1].size; j++) {
				int typeval, pr;
				jsmntok_t* g = &t[i + j + 2];
				typeval = atoi(json_str + g->start);
				pr = CHANNEL_GetType(j);
				if (pr != typeval) {
					CHANNEL_SetType(j, typeval);
					iChanged++;
				}
			}
			i += t[i + 1].size + 1;
		}
		else {
			ADDLOG_ERROR(LOG_FEATURE_API, "Unexpected key: %.*s", t[i].end - t[i].start,
				json_str + t[i].start);
		}
	}
	if (iChanged) {
		CFG_Save_SetupTimer();
		ADDLOG_DEBUG(LOG_FEATURE_API, "Changed %d - saved to flash", iChanged);
	}

	os_free(p);
	os_free(t);
	return http_rest_error(request, 200, "OK");
}

static int http_rest_error(http_request_t* request, int code, char* msg) {
	request->responseCode = code;
	http_setup(request, httpMimeTypeJson);
	if (code != 200) {
		hprintf255(request, "{\"error\":%d, \"msg\":\"%s\"}", code, msg);
	}
	else {
		hprintf255(request, "{\"success\":%d, \"msg\":\"%s\"}", code, msg);
	}
	poststr(request, NULL);
	return 0;
}

#if PLATFORM_BL602

typedef struct ota_header {
	union {
		struct {
			uint8_t header[16];

			uint8_t type[4];//RAW XZ
			uint32_t len;//body len
			uint8_t pad0[8];

			uint8_t ver_hardware[16];
			uint8_t ver_software[16];

			uint8_t sha256[32];
} s;
		uint8_t _pad[512];
	} u;
} ota_header_t;
#define OTA_HEADER_SIZE (sizeof(ota_header_t))

static int _check_ota_header(ota_header_t *ota_header, uint32_t *ota_len, int *use_xz)
{
	char str[33];//assume max segment size
	int i;

	memcpy(str, ota_header->u.s.header, sizeof(ota_header->u.s.header));
	str[sizeof(ota_header->u.s.header)] = '\0';
	puts("[OTA] [HEADER] ota header is ");
	puts(str);
	puts("\r\n");

	memcpy(str, ota_header->u.s.type, sizeof(ota_header->u.s.type));
	str[sizeof(ota_header->u.s.type)] = '\0';
	puts("[OTA] [HEADER] file type is ");
	puts(str);
	puts("\r\n");
	if (strstr(str, "XZ")) {
		*use_xz = 1;
	}
	else {
		*use_xz = 0;
	}

	memcpy(ota_len, &(ota_header->u.s.len), 4);
	printf("[OTA] [HEADER] file length (exclude ota header) is %lu\r\n", *ota_len);

	memcpy(str, ota_header->u.s.ver_hardware, sizeof(ota_header->u.s.ver_hardware));
	str[sizeof(ota_header->u.s.ver_hardware)] = '\0';
	puts("[OTA] [HEADER] ver_hardware is ");
	puts(str);
	puts("\r\n");

	memcpy(str, ota_header->u.s.ver_software, sizeof(ota_header->u.s.ver_software));
	str[sizeof(ota_header->u.s.ver_software)] = '\0';
	puts("[OTA] [HEADER] ver_software is ");
	puts(str);
	puts("\r\n");

	memcpy(str, ota_header->u.s.sha256, sizeof(ota_header->u.s.sha256));
	str[sizeof(ota_header->u.s.sha256)] = '\0';
	puts("[OTA] [HEADER] sha256 is ");
	for (i = 0; i < sizeof(ota_header->u.s.sha256); i++) {
		printf("%02X", str[i]);
	}
	puts("\r\n");

	return 0;
}
#endif

#if PLATFORM_LN882H
#include "ota_port.h"
#include "ota_image.h"
#include "ota_types.h"
#include "hal/hal_flash.h"
#include "netif/ethernetif.h"
#include "flash_partition_table.h"


#define KV_OTA_UPG_STATE           ("kv_ota_upg_state")
#define HTTP_OTA_DEMO_STACK_SIZE   (1024 * 16)

#define SECTOR_SIZE_4KB            (1024 * 4)

static char g_http_uri_buff[512] = "http://192.168.122.48:9090/ota-images/otaimage-v1.3.bin";

// a block to save http data.
static char *temp4K_buf = NULL;
static int   temp4k_offset = 0;

// where to save OTA data in flash.
static int32_t flash_ota_start_addr = OTA_SPACE_OFFSET;
static int32_t flash_ota_offset = 0;
static uint8_t is_persistent_started = LN_FALSE;
static uint8_t is_ready_to_verify = LN_FALSE;
static uint8_t is_precheck_ok = LN_FALSE;
static uint8_t httpc_ota_started = 0;

/**
 * @brief Pre-check the image file to be downloaded.
 *
 * @attention None
 *
 * @param[in]  app_offset  The offset of the APP partition in Flash.
 * @param[in]  ota_hdr     pointer to ota partition info struct.
 *
 * @return  whether the check is successful.
 * @retval  #LN_TRUE     successful.
 * @retval  #LN_FALSE    failed.
 */
static int ota_download_precheck(uint32_t app_offset, image_hdr_t * ota_hdr)
{

	image_hdr_t *app_hdr = NULL;
	if (NULL == (app_hdr = OS_Malloc(sizeof(image_hdr_t)))) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "[%s:%d] malloc failed.\r\n", __func__, __LINE__);
		return LN_FALSE;
	}

	if (OTA_ERR_NONE != image_header_fast_read(app_offset, app_hdr)) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "failed to read app header.\r\n");
		goto ret_err;
	}

	if ((ota_hdr->image_type == IMAGE_TYPE_ORIGINAL) || \
		(ota_hdr->image_type == IMAGE_TYPE_ORIGINAL_XZ))
	{
		// check version
		if (((ota_hdr->ver.ver_major << 8) + ota_hdr->ver.ver_minor) == \
			((app_hdr->ver.ver_major << 8) + app_hdr->ver.ver_minor)) {
			ADDLOG_DEBUG(LOG_FEATURE_OTA, "[%s:%d] same version, do not upgrade!\r\n",
				__func__, __LINE__);
		}

		// check file size
		if (((ota_hdr->img_size_orig + sizeof(image_hdr_t)) > APP_SPACE_SIZE) || \
			((ota_hdr->img_size_orig_xz + sizeof(image_hdr_t)) > OTA_SPACE_SIZE)) {
			ADDLOG_DEBUG(LOG_FEATURE_OTA, "[%s:%d] size check failed.\r\n", __func__, __LINE__);
			goto ret_err;
		}
	}
	else {
		//image type not support!
		goto ret_err;
	}

	OS_Free(app_hdr);
	return LN_TRUE;

ret_err:
	OS_Free(app_hdr);
	return LN_FALSE;
}

static int ota_persistent_start(void)
{
	if (NULL == temp4K_buf) {
		temp4K_buf = OS_Malloc(SECTOR_SIZE_4KB);
		if (NULL == temp4K_buf) {
			LOG(LOG_LVL_INFO,"failed to alloc 4KB!!!\r\n");
			return LN_FALSE;
		}
		memset(temp4K_buf, 0, SECTOR_SIZE_4KB);
	}

	temp4k_offset = 0;
	flash_ota_start_addr = OTA_SPACE_OFFSET;
	flash_ota_offset = 0;
	is_persistent_started = LN_TRUE;
	return LN_TRUE;
}

/**
 * @brief Save block to flash.
 *
 * @param buf
 * @param buf_len
 * @return return LN_TRUE on success, LN_FALSE on failure.
 */
static int ota_persistent_write(const char *buf, const int32_t buf_len)
{
	int part_len = 0; // [0, 1, 2, ..., 4K-1], 0, 1, 2, ..., (part_len-1)

	if (!is_persistent_started) {
		return LN_TRUE;
	}

	if (temp4k_offset + buf_len < SECTOR_SIZE_4KB) {
		// just copy all buf data to temp4K_buf
		memcpy(temp4K_buf + temp4k_offset, buf, buf_len);
		temp4k_offset += buf_len;
		part_len = 0;
	}
	else {
		// just copy part of buf to temp4K_buf
		part_len = temp4k_offset + buf_len - SECTOR_SIZE_4KB;
		memcpy(temp4K_buf + temp4k_offset, buf, buf_len - part_len);
		temp4k_offset += buf_len - part_len;
	}
	if (temp4k_offset >= (SECTOR_SIZE_4KB - 1)) {
		// write to flash
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "write at flash: 0x%08x\r\n", flash_ota_start_addr + flash_ota_offset);

		if (flash_ota_offset == 0) {
			if (LN_TRUE != ota_download_precheck(APP_SPACE_OFFSET, (image_hdr_t *)temp4K_buf)) 
			{
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "ota download precheck failed!\r\n");
				is_precheck_ok = LN_FALSE;
				return LN_FALSE;
			}
		is_precheck_ok = LN_TRUE;
		}

		hal_flash_erase(flash_ota_start_addr + flash_ota_offset, SECTOR_SIZE_4KB);
		hal_flash_program(flash_ota_start_addr + flash_ota_offset, SECTOR_SIZE_4KB, (uint8_t *)temp4K_buf);

		flash_ota_offset += SECTOR_SIZE_4KB;
		memset(temp4K_buf, 0, SECTOR_SIZE_4KB);
		temp4k_offset = 0;
	}

	if (part_len > 0) {
		memcpy(temp4K_buf + temp4k_offset, buf + (buf_len - part_len), part_len);
		temp4k_offset += part_len;
	}

	return LN_TRUE;
}

/**
 * @brief save last block and clear flags.
 * @return return LN_TRUE on success, LN_FALSE on failure.
 */
static int ota_persistent_finish(void)
{
	if (!is_persistent_started) {
		return LN_FALSE;
	}

	// write to flash
	ADDLOG_DEBUG(LOG_FEATURE_OTA, "write at flash: 0x%08x\r\n", flash_ota_start_addr + flash_ota_offset);
	hal_flash_erase(flash_ota_start_addr + flash_ota_offset, SECTOR_SIZE_4KB);
	hal_flash_program(flash_ota_start_addr + flash_ota_offset, SECTOR_SIZE_4KB, (uint8_t *)temp4K_buf);

	OS_Free(temp4K_buf);
	temp4K_buf = NULL;
	temp4k_offset = 0;

	flash_ota_offset = 0;
	is_persistent_started = LN_FALSE;
	return LN_TRUE;
}

static int update_ota_state(void)
{
	upg_state_t state = UPG_STATE_DOWNLOAD_OK;
	ln_nvds_set_ota_upg_state(state);
	return LN_TRUE;
}
/**
 * @brief check ota image header, body.
 * @return return LN_TRUE on success, LN_FALSE on failure.
 */
static int ota_verify_download(void)
{
	image_hdr_t ota_header;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "Succeed to verify OTA image content.\r\n");
	if (OTA_ERR_NONE != image_header_fast_read(OTA_SPACE_OFFSET, &ota_header)) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "failed to read ota header.\r\n");
		return LN_FALSE;
	}

	if (OTA_ERR_NONE != image_header_verify(&ota_header)) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "failed to verify ota header.\r\n");
		return LN_FALSE;
	}

	if (OTA_ERR_NONE != image_body_verify(OTA_SPACE_OFFSET, &ota_header)) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "failed to verify ota body.\r\n");
		return LN_FALSE;
	}

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "Succeed to verify OTA image content.\r\n");
	return LN_TRUE;
}
#endif

static int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr) {

#if PLATFORM_XR809 || PLATFORM_W800
	return 0;	//Operation not supported yet
#endif


	int total = 0;
	int towrite = request->bodylen;
	char* writebuf = request->bodystart;
	int writelen = request->bodylen;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "OTA post len %d", request->contentLength);

#ifdef PLATFORM_W600
	int nRetCode = 0;
	char error_message[256];

	if (writelen < 0) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "ABORTED: %d bytes to write", writelen);
		return http_rest_error(request, 400, "writelen < 0");
	}

	struct pbuf* p;

	//Data is uploaded in 1024 sized chunks, creating a bigger buffer just in case this assumption changes.
	//The code below is based on sdk\OpenW600\src\app\ota\wm_http_fwup.c
	char* Buffer = (char*)os_malloc(2048 + 3);
	memset(Buffer, 0, 2048 + 3);

	if (request->contentLength >= 0) {
		towrite = request->contentLength;
	}

	int recvLen = 0;
	int totalLen = 0;
	//printf("\ntowrite %d writelen=%d\n", towrite, writelen);

	do
	{
		if (writelen > 0) {
			//bk_printf("Copying %d from writebuf to Buffer towrite=%d\n", writelen, towrite);
			memcpy(Buffer + 3, writebuf, writelen);

			if (recvLen == 0) {
				T_BOOTER* booter = (T_BOOTER*)(Buffer + 3);
				bk_printf("magic_no=%u, img_type=%u, zip_type=%u\n", booter->magic_no, booter->img_type, booter->zip_type);

				if (TRUE == tls_fwup_img_header_check(booter))
				{
					totalLen = booter->upd_img_len + sizeof(T_BOOTER);
					OTA_ResetProgress();
					OTA_SetTotalBytes(totalLen);
				}
				else
				{
					sprintf(error_message, "Image header check failed");
					nRetCode = -19;
					break;
				}

				nRetCode = socket_fwup_accept(0, ERR_OK);
				if (nRetCode != ERR_OK) {
					sprintf(error_message, "Firmware update startup failed");
					break;
				}
			}

			p = pbuf_alloc(PBUF_TRANSPORT, writelen + 3, PBUF_REF);
			if (!p) {
				sprintf(error_message, "Unable to allocate memory for buffer");
				nRetCode = -18;
				break;
			}

			if (recvLen == 0) {
				*Buffer = SOCKET_FWUP_START;
			}
			else if (recvLen == (totalLen - writelen)) {
				*Buffer = SOCKET_FWUP_END;
			}
			else {
				*Buffer = SOCKET_FWUP_DATA;
			}

			*(Buffer + 1) = (writelen >> 8) & 0xFF;
			*(Buffer + 2) = writelen & 0xFF;
			p->payload = Buffer;
			p->len = p->tot_len = writelen + 3;

			nRetCode = socket_fwup_recv(0, p, ERR_OK);
			if (nRetCode != ERR_OK) {
				sprintf(error_message, "Firmware data processing failed");
				break;
			}
			else {
				OTA_IncrementProgress(writelen);
				recvLen += writelen;
				printf("Downloaded %d / %d\n", recvLen, totalLen);
			}

			towrite -= writelen;
		}

		if (towrite > 0) {
			writebuf = request->received;
			writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
			if (writelen < 0) {
				sprintf(error_message, "recv returned %d - end of data - remaining %d", writelen, towrite);
				nRetCode = -17;
			}
		}
	} while ((nRetCode == 0) && (towrite > 0) && (writelen >= 0));

	tls_mem_free(Buffer);

	if (nRetCode != 0) {
		ADDLOG_ERROR(LOG_FEATURE_OTA, error_message);
		socket_fwup_err(0, nRetCode);
		return http_rest_error(request, nRetCode, error_message);
	}


#elif PLATFORM_BL602
	int sockfd, i;
	int ret;
	struct hostent *hostinfo;
	uint8_t *recv_buffer;
	struct sockaddr_in dest;
	iot_sha256_context ctx;
	uint8_t sha256_result[32];
	uint8_t sha256_img[32];
	bl_mtd_handle_t handle;
	//init_ota(startaddr);


#define OTA_PROGRAM_SIZE (512)
	int ota_header_found, use_xz;
	ota_header_t *ota_header = 0;

	ret = bl_mtd_open(BL_MTD_PARTITION_NAME_FW_DEFAULT, &handle, BL_MTD_OPEN_FLAG_BACKUP);
	if (ret) {
		return http_rest_error(request, 400, "Open Default FW partition failed");
	}

	recv_buffer = pvPortMalloc(OTA_PROGRAM_SIZE);

	unsigned int buffer_offset, flash_offset, ota_addr;
	uint32_t bin_size, part_size;
	uint8_t activeID;
	HALPartition_Entry_Config ptEntry;

	activeID = hal_boot2_get_active_partition();

	printf("Starting OTA test. OTA bin addr is %p, incoming len %i\r\n", recv_buffer, writelen);

	printf("[OTA] [TEST] activeID is %u\r\n", activeID);

	if (hal_boot2_get_active_entries(BOOT2_PARTITION_TYPE_FW, &ptEntry)) {
		printf("PtTable_Get_Active_Entries fail\r\n");
		vPortFree(recv_buffer);
		bl_mtd_close(handle);
		return http_rest_error(request, 400, "PtTable_Get_Active_Entries fail");
	}
	ota_addr = ptEntry.Address[!ptEntry.activeIndex];
	bin_size = ptEntry.maxLen[!ptEntry.activeIndex];
	part_size = ptEntry.maxLen[!ptEntry.activeIndex];
	(void)part_size;
	/*XXX if you use bin_size is product env, you may want to set bin_size to the actual
	 * OTA BIN size, and also you need to splilt XIP_SFlash_Erase_With_Lock into
	 * serveral pieces. Partition size vs bin_size check is also needed
	 */
	printf("Starting OTA test. OTA size is %lu\r\n", bin_size);

	printf("[OTA] [TEST] activeIndex is %u, use OTA address=%08x\r\n", ptEntry.activeIndex, (unsigned int)ota_addr);

	printf("[OTA] [TEST] Erase flash with size %lu...", bin_size);
	hal_update_mfg_ptable();
	
	//erase in chunks, because erasing everything at once is slow and causes issues with http connection
	uint32_t erase_offset = 0;
	uint32_t erase_len = 0;
	while (erase_offset < bin_size)
	{
		erase_len = bin_size - erase_offset;
		if (erase_len > 0x10000)
		{
			erase_len = 0x10000; //erase in 64kb chunks
		}
		bl_mtd_erase(handle, erase_offset, erase_len);
		printf("[OTA] Erased:  %lu / %lu \r\n", erase_offset, erase_len);
		erase_offset += erase_len;
		rtos_delay_milliseconds(100);
	}
	printf("[OTA] Done\r\n");

	if (request->contentLength >= 0) {
		towrite = request->contentLength;
	}

	// get header
	// recv_buffer	
	//buffer_offset = 0;
	//do {
	//	int take_len;

	//	take_len = OTA_PROGRAM_SIZE - buffer_offset;

	//	memcpy(recv_buffer + buffer_offset, writebuf, writelen);
	//	buffer_offset += writelen;


	//	if (towrite > 0) {
	//		writebuf = request->received;
	//		writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
	//		if (writelen < 0) {
	//			ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
	//		}
	//	}
	//} while(true)

	buffer_offset = 0;
	flash_offset = 0;
	ota_header = 0;
	use_xz = 0;

	utils_sha256_init(&ctx);
	utils_sha256_starts(&ctx);
	memset(sha256_result, 0, sizeof(sha256_result));
	do {
		char *useBuf = writebuf;
		int useLen = writelen;

		if (ota_header == 0) {
			int take_len;

			// how much left for header?
			take_len = OTA_PROGRAM_SIZE - buffer_offset;
			// clamp to available len
			if (take_len > useLen)
				take_len = useLen;
			printf("Header takes %i. ", take_len);
			memcpy(recv_buffer + buffer_offset, writebuf, take_len);
			buffer_offset += take_len;
			useBuf = writebuf + take_len;
			useLen = writelen - take_len;

			if (buffer_offset >= OTA_PROGRAM_SIZE) {
				ota_header = (ota_header_t*)recv_buffer;
				if (strncmp((const char*)ota_header, "BL60X_OTA", 9)) {
					return http_rest_error(request, 400, "Invalid header ident");
				}
			}
		}


		if (ota_header && useLen) {


			if (flash_offset + useLen >= part_size) {
				return http_rest_error(request, 400, "Too large bin");
			}
			//ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d bytes to write", writelen);
			//add_otadata((unsigned char*)writebuf, writelen);

			printf("Flash takes %i. ", useLen);
			utils_sha256_update(&ctx, (byte*)useBuf, useLen);
			bl_mtd_write(handle, flash_offset, useLen, (byte*)useBuf);
			flash_offset += useLen;
		}

		total += writelen;
		startaddr += writelen;
		towrite -= writelen;


		if (towrite > 0) {
			writebuf = request->received;
			writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
			if (writelen < 0) {
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
			}
		}
	} while ((towrite > 0) && (writelen >= 0));

	if (ota_header == 0) {
		return http_rest_error(request, 400, "No header found");
	}
	utils_sha256_finish(&ctx, sha256_result);
	puts("\r\nCalculated SHA256 Checksum:");
	for (i = 0; i < sizeof(sha256_result); i++) {
		printf("%02X", sha256_result[i]);
	}
	puts("\r\nHeader SHA256 Checksum:");
	for (i = 0; i < sizeof(sha256_result); i++) {
		printf("%02X", ota_header->u.s.sha256[i]);
	}
	if (memcmp(ota_header->u.s.sha256, sha256_result, sizeof(sha256_img))) {
		/*Error found*/
		return http_rest_error(request, 400, "SHA256 NOT Correct");
	}
	printf("[OTA] [TCP] prepare OTA partition info\r\n");
	ptEntry.len = total;
	printf("[OTA] [TCP] Update PARTITION, partition len is %lu\r\n", ptEntry.len);
	hal_boot2_update_ptable(&ptEntry);
	printf("[OTA] [TCP] Rebooting\r\n");
	//close_ota();
	vPortFree(recv_buffer);
	utils_sha256_free(&ctx);
	bl_mtd_close(handle);

#elif PLATFORM_LN882H
	ADDLOG_DEBUG(LOG_FEATURE_OTA, "Ota start!\r\n");
	if (LN_TRUE != ota_persistent_start()) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "Ota start error, exit...\r\n");
		return 0;
	}

	if (request->contentLength >= 0) {
		towrite = request->contentLength;
	}

	do {
		//ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d bytes to write", writelen);

		if (LN_TRUE != ota_persistent_write(writebuf, writelen)) {
			//	ADDLOG_DEBUG(LOG_FEATURE_OTA, "ota write err.\r\n");
			return -1;
		}

		rtos_delay_milliseconds(10);
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "Writelen %i at %i", writelen, total);
		total += writelen;
		startaddr += writelen;
		towrite -= writelen;
		if (towrite > 0) {
			writebuf = request->received;
			writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
			if (writelen < 0) {
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
			}
		}
	} while ((towrite > 0) && (writelen >= 0));

	ota_persistent_finish();
	is_ready_to_verify = LN_TRUE;
	ADDLOG_DEBUG(LOG_FEATURE_OTA, "cb info: recv %d finished, no more data to deal with.\r\n", towrite);


	ADDLOG_DEBUG(LOG_FEATURE_OTA, "http client job done, exit...\r\n");
	if (LN_TRUE == is_precheck_ok)
	{
		if ((LN_TRUE == is_ready_to_verify) && (LN_TRUE == ota_verify_download())) {
			update_ota_state();
			//ln_chip_reboot();
		}
		else {
			ADDLOG_DEBUG(LOG_FEATURE_OTA, "Veri bad\r\n");
		}
	}
	else {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "Precheck bad\r\n");
	}

#else

	init_ota(startaddr);

	if (request->contentLength >= 0) {
		towrite = request->contentLength;
	}

	if (writelen < 0 || (startaddr + writelen > maxaddr)) {
		ADDLOG_DEBUG(LOG_FEATURE_OTA, "ABORTED: %d bytes to write", writelen);
		return http_rest_error(request, 400, "writelen < 0 or end > 0x200000");
	}

	do {
		//ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d bytes to write", writelen);
		add_otadata((unsigned char*)writebuf, writelen);
		total += writelen;
		startaddr += writelen;
		towrite -= writelen;
		if (towrite > 0) {
			writebuf = request->received;
			writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
			if (writelen < 0) {
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
			}
		}
	} while ((towrite > 0) && (writelen >= 0));
	close_ota();
#endif

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d total bytes written", total);
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	return 0;
}

static int http_rest_post_dry(http_request_t* request, int startaddr, int maxaddr) {
	int total = 0;
	int towrite = request->bodylen;
	char* writebuf = request->bodystart;
	int writelen = request->bodylen;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "OTA post len %d", request->contentLength);
#if PLATFORM_BL602

	int sockfd, i;
	int ret;
	struct hostent *hostinfo;
	uint8_t *recv_buffer;
	struct sockaddr_in dest;
	iot_sha256_context ctx;
	uint8_t sha256_result[32];
	uint8_t sha256_img[32];
	bl_mtd_handle_t handle;
	//init_ota(startaddr);


#define OTA_PROGRAM_SIZE (512)
	int ota_header_found, use_xz;
	ota_header_t *ota_header = 0;

	ret = bl_mtd_open(BL_MTD_PARTITION_NAME_FW_DEFAULT, &handle, BL_MTD_OPEN_FLAG_BACKUP);
	if (ret) {
		return http_rest_error(request, 400, "Open Default FW partition failed");
	}

	recv_buffer = pvPortMalloc(OTA_PROGRAM_SIZE);

	unsigned int buffer_offset, flash_offset, ota_addr;
	uint32_t bin_size, part_size;
	uint8_t activeID;
	HALPartition_Entry_Config ptEntry;

	activeID = hal_boot2_get_active_partition();

	printf("Starting OTA test. OTA bin addr is %p, incoming len %i\r\n", recv_buffer, writelen);

	printf("[OTA] [TEST] activeID is %u\r\n", activeID);

	if (hal_boot2_get_active_entries(BOOT2_PARTITION_TYPE_FW, &ptEntry)) {
		printf("PtTable_Get_Active_Entries fail\r\n");
		vPortFree(recv_buffer);
		bl_mtd_close(handle);
		return http_rest_error(request, 400, "PtTable_Get_Active_Entries fail");
	}
	ota_addr = ptEntry.Address[!ptEntry.activeIndex];
	bin_size = ptEntry.maxLen[!ptEntry.activeIndex];
	part_size = ptEntry.maxLen[!ptEntry.activeIndex];
	(void)part_size;
	/*XXX if you use bin_size is product env, you may want to set bin_size to the actual
	 * OTA BIN size, and also you need to splilt XIP_SFlash_Erase_With_Lock into
	 * serveral pieces. Partition size vs bin_size check is also needed
	 */
	printf("Starting OTA test. OTA size is %lu\r\n", bin_size);

	printf("[OTA] [TEST] activeIndex is %u, use OTA address=%08x\r\n", ptEntry.activeIndex, (unsigned int)ota_addr);

	printf("[OTA] [TEST] Erase flash with size %lu...", bin_size);
	//hal_update_mfg_ptable();
	// bl_mtd_erase_all(handle);
	uint32_t erase_offset = 0;
	uint32_t erase_len = 0;
	while (erase_offset < bin_size)
	{
		erase_len = bin_size - erase_offset;
		if (erase_len > 0x10000)
		{
			erase_len = 0x10000; //erase in 64kb chunks
		}
		printf("erase  %lu / %lu \r\n", erase_offset, erase_len);
		rtos_delay_milliseconds(100);
		bl_mtd_erase(handle, erase_offset, erase_len);
		printf("eraseD  %lu / %lu \r\n", erase_offset, erase_len);
		erase_offset += erase_len;
		rtos_delay_milliseconds(10);
	}	
	printf("Done\r\n");

	if (request->contentLength >= 0) {
		towrite = request->contentLength;
	}

	// get header
	// recv_buffer	
	//buffer_offset = 0;
	//do {
	//	int take_len;

	//	take_len = OTA_PROGRAM_SIZE - buffer_offset;

	//	memcpy(recv_buffer + buffer_offset, writebuf, writelen);
	//	buffer_offset += writelen;


	//	if (towrite > 0) {
	//		writebuf = request->received;
	//		writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
	//		if (writelen < 0) {
	//			ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
	//		}
	//	}
	//} while(true)

	buffer_offset = 0;
	flash_offset = 0;
	ota_header = 0;
	use_xz = 0;

	utils_sha256_init(&ctx);
	utils_sha256_starts(&ctx);
	memset(sha256_result, 0, sizeof(sha256_result));
	do {
		char *useBuf = writebuf;
		int useLen = writelen;

		if (ota_header == 0) {
			int take_len;

			// how much left for header?
			take_len = OTA_PROGRAM_SIZE - buffer_offset;
			// clamp to available len
			if (take_len > useLen)
				take_len = useLen;
			printf("Header takes %i. ", take_len);
			memcpy(recv_buffer + buffer_offset, writebuf, take_len);
			buffer_offset += take_len;
			useBuf = writebuf + take_len;
			useLen = writelen - take_len;

			if (buffer_offset >= OTA_PROGRAM_SIZE) {
				ota_header = (ota_header_t*)recv_buffer;
				if (strncmp((const char*)ota_header, "BL60X_OTA", 9)) {
					return http_rest_error(request, 400, "Invalid header ident");
				}
			}
		}


		if (ota_header && useLen) {


			if (flash_offset + useLen >= part_size) {
				return http_rest_error(request, 400, "Too large bin");
			}
			//ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d bytes to write", writelen);
			//add_otadata((unsigned char*)writebuf, writelen);

			printf("Flash takes %i. ", useLen);
			utils_sha256_update(&ctx, (byte*)useBuf, useLen);
			bl_mtd_write(handle, flash_offset, useLen, (byte*)useBuf);
			flash_offset += useLen;
		}

		total += writelen;
		startaddr += writelen;
		towrite -= writelen;


		if (towrite > 0) {
			writebuf = request->received;
			writelen = recv(request->fd, writebuf, request->receivedLenmax, 0);
			if (writelen < 0) {
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv returned %d - end of data - remaining %d", writelen, towrite);
			}
		}
	} while ((towrite > 0) && (writelen >= 0));

	if (ota_header == 0) {
		return http_rest_error(request, 400, "No header found");
	}
	utils_sha256_finish(&ctx, sha256_result);
	puts("\r\nCalculated SHA256 Checksum:");
	for (i = 0; i < sizeof(sha256_result); i++) {
		printf("%02X", sha256_result[i]);
	}
	puts("\r\nHeader SHA256 Checksum:");
	for (i = 0; i < sizeof(sha256_result); i++) {
		printf("%02X", ota_header->u.s.sha256[i]);
	}
	if (memcmp(ota_header->u.s.sha256, sha256_result, sizeof(sha256_img))) {
		/*Error found*/
		return http_rest_error(request, 400, "SHA256 NOT Correct");
	}
	printf("[OTA] [TCP] prepare OTA partition info\r\n");
	ptEntry.len = total;
	printf("[OTA] [TCP] Update PARTITION, partition len is %lu\r\n", ptEntry.len);
	hal_boot2_update_ptable(&ptEntry);
	printf("[OTA] [TCP] Rebooting\r\n");
	//close_ota();
	vPortFree(recv_buffer);
	utils_sha256_free(&ctx);
	bl_mtd_close(handle);

#endif

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d total bytes written", total);
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	return 0;
}



static int http_rest_post_reboot(http_request_t* request) {
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"reboot\":%d}", 3);
	ADDLOG_DEBUG(LOG_FEATURE_API, "Rebooting in 3 seconds...");
	RESET_ScheduleModuleReset(3);
	poststr(request, NULL);
	return 0;
}

static int http_rest_get_flash_advanced(http_request_t* request) {
	char* params = request->url + 10;
	int startaddr = 0;
	int len = 0;
	int sres;
	sres = sscanf(params, "%x-%x", &startaddr, &len);
	if (sres == 2) {
		return http_rest_get_flash(request, startaddr, len);
	}
	return http_rest_error(request, 400, "invalid url");
}

static int http_rest_post_flash_advanced(http_request_t* request) {
	char* params = request->url + 10;
	int startaddr = 0;
	int sres;
	sres = sscanf(params, "%x", &startaddr);
	if (sres == 1 && startaddr >= START_ADR_OF_BK_PARTITION_OTA) {
		// allow up to end of flash
		return http_rest_post_flash(request, startaddr, 0x200000);
	}
	return http_rest_error(request, 400, "invalid url");
}

static int http_rest_get_flash(http_request_t* request, int startaddr, int len) {
	char* buffer;
	int res;

	if (startaddr < 0 || (startaddr + len > 0x200000)) {
		return http_rest_error(request, 400, "requested flash read out of range");
	}

	buffer = os_malloc(1024);

	http_setup(request, httpMimeTypeBinary);
	while (len) {
		int readlen = len;
		if (readlen > 1024) {
			readlen = 1024;
		}
#if PLATFORM_XR809
		//uint32_t flash_read(uint32_t flash, uint32_t addr,void *buf, uint32_t size)
#define FLASH_INDEX_XR809 0
		res = flash_read(FLASH_INDEX_XR809, startaddr, buffer, readlen);
#elif PLATFORM_BL602
		res = bl_flash_read(startaddr, (uint8_t *)buffer, readlen);
#elif PLATFORM_W600 || PLATFORM_W800
		res = 0;
#elif PLATFORM_LN882H
// TODO:LN882H flash read?
        res = 0;
#else
		res = flash_read((char*)buffer, readlen, startaddr);
#endif
		startaddr += readlen;
		len -= readlen;
		postany(request, buffer, readlen);
	}
	poststr(request, NULL);
	os_free(buffer);
	return 0;
}


static int http_rest_get_dumpconfig(http_request_t* request) {



	http_setup(request, httpMimeTypeText);
	poststr(request, NULL);
	return 0;
}



#ifdef TESTCONFIG_ENABLE
// added for OpenBK7231T
typedef struct item_new_test_config
{
	INFO_ITEM_ST head;
	char somename[64];
}ITEM_NEW_TEST_CONFIG, * ITEM_NEW_TEST_CONFIG_PTR;

ITEM_NEW_TEST_CONFIG testconfig;
#endif

static int http_rest_get_testconfig(http_request_t* request) {
	return http_rest_error(request, 400, "unsupported");
	return 0;
}

static int http_rest_get_flash_vars_test(http_request_t* request) {
	//#if PLATFORM_XR809
	//    return http_rest_error(request, 400, "flash vars unsupported");
	//#elif PLATFORM_BL602
	//    return http_rest_error(request, 400, "flash vars unsupported");
	//#else
	//#ifndef DISABLE_FLASH_VARS_VARS
	//    char *params = request->url + 17;
	//    int increment = 0;
	//    int len = 0;
	//    int sres;
	//    int i;
	//    char tmp[128];
	//    FLASH_VARS_STRUCTURE data, *p;
	//
	//    p = &flash_vars;
	//
	//    sres = sscanf(params, "%x-%x", &increment, &len);
	//
	//    ADDLOG_DEBUG(LOG_FEATURE_API, "http_rest_get_flash_vars_test %d %d returned %d", increment, len, sres);
	//
	//    if (increment == 10){
	//        flash_vars_read(&data);
	//        p = &data;
	//    } else {
	//        for (i = 0; i < increment; i++){
	//            HAL_FlashVars_IncreaseBootCount();
	//        }
	//        for (i = 0; i < len; i++){
	//            HAL_FlashVars_SaveBootComplete();
	//        }
	//    }
	//
	//    sprintf(tmp, "offset %d, boot count %d, boot success %d, bootfailures %d",
	//        flash_vars_offset,
	//        p->boot_count,
	//        p->boot_success_count,
	//        p->boot_count - p->boot_success_count );
	//
	//    return http_rest_error(request, 200, tmp);
	//#else
	return http_rest_error(request, 400, "flash test unsupported");
}


static int http_rest_get_channels(http_request_t* request) {
	int i;
	int addcomma = 0;
	/*typedef struct pinsState_s {
		byte roles[32];
		byte channels[32];
	} pinsState_t;

	extern pinsState_t g_pins;
	*/
	http_setup(request, httpMimeTypeJson);
	poststr(request, "{");

	// TODO: maybe we should cull futher channels that are not used?
	// I support many channels because I plan to use 16x relays module with I2C MCP23017 driver
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		// "i" is a pin index
		// Get channel index and role
		int ch = PIN_GetPinChannelForPinIndex(i);
		int role = PIN_GetPinRoleForPinIndex(i);
		if (role) {
			if (addcomma) {
				hprintf255(request, ",");
			}
			hprintf255(request, "\"%d\":%d", ch, CHANNEL_Get(ch));
			addcomma = 1;
		}
	}
	poststr(request, "}");
	poststr(request, NULL);
	return 0;
}

// currently crashes the MCU - maybe stack overflow?
static int http_rest_post_channels(http_request_t* request) {
	int i;
	int r;
	char tmp[64];

	//https://github.com/zserge/jsmn/blob/master/example/simple.c
	//jsmn_parser p;
	jsmn_parser* p = os_malloc(sizeof(jsmn_parser));
	//jsmntok_t t[128]; /* We expect no more than 128 tokens */
#define TOKEN_COUNT 128
	jsmntok_t* t = os_malloc(sizeof(jsmntok_t) * TOKEN_COUNT);
	char* json_str = request->bodystart;
	int json_len = strlen(json_str);

	memset(p, 0, sizeof(jsmn_parser));
	memset(t, 0, sizeof(jsmntok_t) * 128);

	jsmn_init(p);
	r = jsmn_parse(p, json_str, json_len, t, TOKEN_COUNT);
	if (r < 0) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Failed to parse JSON: %d", r);
		sprintf(tmp, "Failed to parse JSON: %d\n", r);
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_ARRAY) {
		ADDLOG_ERROR(LOG_FEATURE_API, "Array expected", r);
		sprintf(tmp, "Object expected\n");
		os_free(p);
		os_free(t);
		return http_rest_error(request, 400, tmp);
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
		int chanval;
		jsmntok_t* g = &t[i];
		chanval = atoi(json_str + g->start);
		CHANNEL_Set(i - 1, chanval, 0);
		ADDLOG_DEBUG(LOG_FEATURE_API, "Set of chan %d to %d", i,
			chanval);
	}

	os_free(p);
	os_free(t);
	return http_rest_error(request, 200, "OK");
	return 0;
}



static int http_rest_post_cmd(http_request_t* request) {
	commandResult_t res;
	int code;
	const char *reply;
	const char *type;
	const char* cmd = request->bodystart;
	res = CMD_ExecuteCommand(cmd, COMMAND_FLAG_SOURCE_CONSOLE);
	reply = CMD_GetResultString(res);
	if (1) {
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "[WebApp Cmd '%s' Result] %s", cmd, reply);
	}
	if (res != CMD_RES_OK) {
		type = "error";
		if (res == CMD_RES_UNKNOWN_COMMAND) {
			code = 501;
		}
		else {
			code = 400;
		}
	}
	else {
		type = "success";
		code = 200;
	}

	request->responseCode = code;
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"%s\":%d, \"msg\":\"%s\", \"res\":", type, code, reply);
	JSON_ProcessCommandReply(cmd, skipToNextWord(cmd), request, (jsonCb_t)hprintf255, COMMAND_FLAG_SOURCE_HTTP);
	hprintf255(request, "}", code, reply);
	poststr(request, NULL);
	return 0;
}
char *c=
"`ZBulrAKYvAphuRKXsjMFSyMfPmgmRPpZIJkZyAFpVeehyAxxVitndkMpPETYHfKW`lblYrdXkiT_gOojXcWBqYwPdi_B]w]z^cLWdPBJMA_HQdNPdB_PIaHoSHkScvnLUBEndTnTUsdXYHSFRlsvyjUBTWD]`jaQMSnP`ZgQNKuQwjWKoeArDrMehCZMPRofRTLGUDasEbinpDFeAthuiiNVFIc]z[BwWOyd_PqlDInIYodxpLWetwdFpAL[DtdamXbFGkCvravmMgUz`JOfR^RV_iPfRzOJuWBN_j[TiBWhTcTq[^jUv[tPjxXXerRlrqfGE[oRcvhKGwoq`ozwIxo^RzwEKEwmijzZOadrXUBmQ`cnVmwst]IgaJoV^RrljUNEqCROVEldnkA[TRpRzdsLbTsFlOTF^PDwR^xSi]pl`sqIUBM]`lFC^^KM]HkfG^hEJqU]qRHD[B[CwWMHdy_x[OzZIOLVEFYVIELLszi[yYbDHPFudCIzjduquTcFlzVdEtsLBOdPDzCh^WBzF[DQVZ[RUxfGUMZMsyOWusVYsvkaMS_SczQ_RSjZXtgQxJjmd[fXmcLcFfXZjuECZT^Uvuv`PoyBLE[iMxwlhGazoiZgvPk^oeregUrj`FIydKWALjMmxTUTn^aN[OezEMbNpBsFNMcxcWeRZLCyaP^rJndOQ[JtIxpmVhTgjGmVLHwvBnYxGvlRjxsF]yvHoiNg]gF_iRqU]HgCqiN^BMnNM^CXoOKlLfb]tQYUiuOoFWocRqOnlXFcuA[ydij[HFIraJIaPspHxzIpi[GyzWzoazcsp[kAKnhzOxBHhh^scxeuseQJbl[kFURbaPlpHpwJSIblZyQSsHLcUiALKbYZTwutg[aYwiknB[bkC_kd[HryJiMsP`faX]qVNhEyXsVKWYbtGPIfrozLFwkyNiIWCKW_eL]U[lLcmgblHTmzXAic^eQRxGljkyHMBDPumnKGcWZYVzquwIzCnAzvLXTz]sn_UjEDsMdpczHKsvy"
"PrnbzFlNCPzbWqGG_gtAPmqEvWMsotoCQYDHjQoZdBnsfMytc_zui^BjgcssZJ[wRxnyWOsZXokCLPJOWvQQDVkaGAeLakCrSwUPL]]Uf`]umy^WYe_sKtvMMsWegthmtkPvuCtYwe^sqnI]NmvO^yICYCyeGjygpItOO_cbGw`VDWYktWuxYOubM]rDPRMgrCiutOmOTgXCcgVgauNTEjWtzgCyh_Sojt_Ai_qfefdmqry`HL^UEtbIgovNLnbpUdTjIUxBEcjYFeDHi]rBxkpRWUtdcZqEoYN^OWmNQ]KICCln`ivFnWs^YmAEYIsURLpeZGFO`gNfITRIpDkuzABUAqelJCpTRolqvJGcqrNjfsPZivcXXxVQKec]TE^ruTUcMgxIEHnRjHbKCNZXwUZ[g[`^G^mkrk_urNjw[PtN^sl]BRQqD[xzvHomHZBaUC[ydgTp_^whLxoiC]pNwgXn`zkZxoANTs[YntUxRa_Q_jJADFxMSYVCLDhdVbivZHS]VvJIrmfGs]BPaZtjDAA[EVbphTFjrcrWWheHvpUanPJjCXDwizsYHsXATU[_hvPQffttomoOg[hpLsnQQIBrjYP_PdobUqsrztDX_KHvLemgKcp[aKN_tXaNWUdKyMtpbppSoWmFBFgFfcUhoTAJcUxcSqAJawxykxEUm]fKYH^ol[bnAwWeMjNMsNzyVc`EIhVi^zuShpl[rJqrPNIeaSmAjr]`]hppcYfDFtQABExWymViWIiHZu^puVyf^VpzAvrY^nmVwIc]VWBx]nIUyhVhvqIkue^gOxVDuwKdWTLRSA[_TBIjM[BS`K^heB[SewNXAyCad`xf]EVuWeWtnZnwD^kZ^yUsshVnC_UycGg^VqocRAebqq]ZxRGGbdndXzHUGCuthmQaMBwqLykRCqsTaathAFtcMEowucgxIkJvkZVQuct]baGon`INXhgWxAxHVOEHeFhIKfqZRLF^AGf^igJVZZ`DmxkOuf_lrsxeIiivLbJWrUZZjFjh"
"L`MFXhFxDQ[yPOKfkCAgk^UttWGWCoL`pDgvgsM[clsuGLGcKojhFCxiLSLBbjvOFvJLcOyrp`DzeZmiP^LOF_[mGykTgKsspdpLBvAvprr`zxDm`^mE^hJa[WFe_fKehKKAebgU^QYNgvnIuxnJYoUI[OkfpVXKgNsonCIyqYaNOvu`dwjarBMspC`VqB_HsOaZDyVspoSAOHt`ULCtgqSfAzzl]mIptZUAU[]eaqlCiMeiIABfyf[CvvZDzeVDg^GpmsLtjYSqFQxoVQc]k^qOySFZBYewiWKFpwQoHvaSeweyehjCsSIgsqZmlsQnWUXtmiaURhdbQvTW[WkvucVN_Ny^_DOfnwP_kLvHVkKibWw^CRQ]armBgN[crmeUw]yDrkP[daJXgfHR[[NmmFv^]Ey__xaUetxZKbBBwRSUUAFJHW`sbrb^`VfRzECLrSBOTAPgllIeIIHDsRXHYhnnSZtpvkCsPG^CzrpGHtHqjxVXNcZhhunJ_NwtG[kAjQUBfFEAQA_axEUE]cTRUKUTU^lvwzhIGGZcTmH_YLXuOcbJboerXjUFAiQbSF`QTV[qemlk[osIn[FlwbTHVplWmTceaLSfHbsIk`pDtXcrLSac`fklYu^eEERJGPtd`]gsMdLC_E]XI_aBCBwvbNMDNEgOfwe^zRjghWv[hNUYF]`_PHETyIyPZoz`_osIsuJNqVUfAnTtoyl_Iy^rQ_rTZMsWVvtVM`LSShtnZS^UfWeqNAWqUQbVStXIPe`uxgOmYMEkYyA]hDcCIV`CAvypAYPlRKhy[hjVEpovlescUpLUaZGPArwlQxXoLTYNQCBATMxTaeRWCV[KMvLsEXxdlxFHXAjYLeSSoLATmjWaaju]MbPH`JtFYOWHqwautKXDIWAvsWRibDLOKeKY[QgYNWweMeqI]hVRrTqaTPIsK^UDHwMlHZc_wevzDRfmaFWpwTRUrGA^cOjBt]QYEACErVMXq^CeWWeYyOLcuXQTbBTWMyKpVTW`CRybHt^U"
"EWGMdQDeRh^hki]pBpLsSNOCzFVgPe[BtvPLuHmwu`_hqjoTpI`RifBrIDM_uWUgN]npZRvuDPxmiaMPQCrHEITOtXCquNB_Lr[Twzu^NeOYhPIbrwNs`NbEJ_vBOEApNZbrGsTRITu[uubePzxAJyxPWKYDsPIeYLOhQydwkLTHJOxQmnFCpE[ecjiuNPElq]iSxzAATDZndHMvnG]CbcHoDilQji[HbRgaeTnmFilfhA[^mZ[JbjOub_tQOkeVyfCqaVrZSjxsqGCNO^PNQ`sCmyFZ]_UEWgBydwWcBqsmWDGXrDAMVIhGJonXlzqLco[pFEdRGhM[gGgishcZ^HjNzuopIzqBzhJbcRSmZiGivNZDHF`EXXh[FMpKPoIHqL[zwEFaVpzpBBSllhDZmdcAi]WH^fayrkn[HDzyibXsZDuEj_GEhDbrzNcjFNSUOCJcyBdeEDBKHPYWYy_RnIv[tPsODdWuSFhuAMAahryJNipUqnlVZDakJoccmH_s_JRkDC[FncNdvRCJfuMpf_sCGKSLdXnaSFfDXEQnTfcrpwprmYwsujNzRPdtkPVphQpgymadZ]zwAMJV[YLufdrfmIXDDKMCIfzLofWOMNDQCZ^PqQuuBhyALeFLVYpjyD^WPd`EgKWdcFBuoNtSIHyBKcBRCFoYTSzvHAMlpNqQUYj_DRbLwWqw`VEB^ndFcLOJGFmnAH]uQhgmt]Nk^c]w]CWmwLOCdkiIJaYx`nLouvfriRIA`tbzjShDjArotZQ`z_dSEQLkXiCaW_EeDN^kJiZrsUO`wWu[fFrKipvPhrcBPHhKvMlfw`WREm[jCgnBncvRguwORODRGYblhDdjeO^FjaFwULNBDaNhLW_BQ]Ti]ELgKiCzDDW_EpbGoAtGACN`Kt`R`bZHuodeUZlfJNyAJjdVoXNPdNh[qgRJl[eOdPMxmVPNwmDNVQK^HTJ_gHJnixZ]Yn]WvFTJrerAWRuWdQABzNSH[kwIDJEYpk_ybr[WwAf^]JIVMnKX"
"wY^KUvKILFhaZLfmvolqeEybpfRcawQygWMT^MvPNkvywCx]dQyYaSU`cgiecbPpjtxpYYYbqbXUCdsdTFxfS`FJChlq_RgfGPCEIEKrWczvA^yoFQ^dd]JoomlhcD_ulswUp_LBnnvwFOCKmCYAICrdFOQVS]PTDFJHaeybF]pqbMiakbvaf_liBSObOqiR^ctAEudHDrvgMBBtajekaji`TGOVXSOn_SzThpyn]ndF]AQVA_LxkotTgdoVmg`gsiiCatOomnNuvOzGxAWnDqn_FzAsBdApOxMoROpHQWLwazYYeiXmQldCuKwasVnJmXfyCKoiKO]geqxGbcOSLO`^UJtGSOPudCKIT[VZtjbBOwXDNqzPHx_sIdsRiWvzcDcgomHMpY]g^OXmqtPznHEgQLq^ILTgCgtoWYEsDSCrUtMG[ougzov_A_aQjhNIAc_xZXXwFcovSeXhXYzCrxQWu^bTfoSqtRhIBS^ODTqvLcOgRzqkAYkagCbtpcqQQyCAwOfEukz_HVLRPuCfycp[aHyleZ]`eNaNkCUmpQZLYRrPdvaKSiIb[onhSQR_CckylCKzRLm`BViJH]A^uRMwEynvPh[`yvJqbs_bkbtGwcaFFd`qPndkVntmOOjelNbFLscOyZnTYGAYhpfHePKSZ[oylS]rPUiB[tNBQrqAPSPmYTaYD]HxWnmYoPMGQ]IlRZyoceqmYJCGKBaXlDnmMkfhHoNBfkjmJ_yVmpZgV`MkfNmjoUNjV_aoRgoWpGi[Qxg`bCzNTGrLYwb[RqV]DnzuJEFBDqOqm[ibMHmVzVPhVWULMw^PSxMJgfYxtLQQcUf]nzNKEA]h_S`njuoGSy_oN`ubjonFzEAksXMiTLR`eTGeNzWJxdqrOOFgP`jb`O`Cr]cYznMTiubEaWwBJteJHCsvllxFFbbW]olyqd_aU_KWiDSf^mVJNT[VGr[bkou]KybKDfaUDpdr]NiZRbSLYJJUVryIwp`CQt_Fl[[HpoJ^r_eemIZMIiEA"
"pNpLgXYOPaaciciqHKWdAf^qMUVoWStcp^P^fPXWvNQyCwJZcFyR[AJjzySBdrxhxnKTKLrjyNFXNsKXMUnMvRU^GofQIWnDQdAPlqPXNPSNul]GnupOEklgYgMV`bac[CHdsvgVFWygyHcVR`]Ob_zarUqEetrgikmsesOsOZIhlFhRDqEAESBvCam_FbkL`SxOUjlZRezHVRpmg`AnprcWirpDGkngixHZHYwUFej[St`llAr`axWijb`^thTarTYYJWeNdgiKHIEpRESGsNGlbZjatY[XRIcyHbVVdwtPLab]kLpVMidMgrkgDUKMQsz`FTK^xkOQM]VtHDINeBF[yloPriRPmhySaCXWARY]ZTAKq`zMwbOn`aT`ugPZz^`^DSFglMTddAfldoAIRCqFj_BErFfJC[pgY^ZTkOfK_vcvGfGHwhZn_RdctGh[aapPwW]iybGrET]a]vzzCv^IBttMplbdaKeyFSykIXIUbcjLpDGxHhXeChOD]QsNA]Qti^[JwOrfmIaZJhXmedsnAvXDvNs_vUERp]RcwAtqmruGototGcXAYGibCGBAyGqoHULFJiccBAJBQOBYyXTxEtioj`qzsrfAGfh_dnxtoXWF`amrAbzi^qXNfWWdsdkKVIvfb]pL[ZIg^dPJnzzQrx^PNZtMXTBLhMkWfGP^moNEiljCHNpteZSXfgXHdcnHXxyRss^hj_DjZb]kqFK^IaOtSlyiIuJAGgkmYJOjIJIuPiMFvXn^ddPWkXaH_ybbUCcmqjRgqUzYTed`tnkHgX]XTCAdHKKcbaMFZOLFGgIOPyXmwcfLq[ifFV]hRDjQKXqmY_FfbeAAOOHVeNg_`ylGrbzEl_oCwrEsHeUY]zdMZhWrNHDDWKlLfjur_BtDqyDWKONztOC`V`hKKIqe[kflGsMYj^tGaVdzhzuDKIIMqGCkgWLPcFSSQRBpPWVaYoJLaK[qjXa^dtBGVyq^PrxQJRiWxFnonbnBxqK]vU`L_oiXPvZQ`rQbrtCO"
"EtedmkBGAJ_oUbvRmlVdRGCf]OvQLKEowArublDgRPmu]LQVPFHgfKyqzG_pkupEksUjZXKARwAqPkADevvFwHQqtevNNvWZhKeravsvHVjJHaOGLpykIcb^qpimDAdXc`VLDzCKpmbglDgUmCGYj`cDTjGXMURi`kyilPHpvaSqzrFloUAAtLhDxRsTtfy[[sGBFxd]vQ^IMlE]eUcrTQpUCukZWnwPcrVGkDfrl_Fdh_rMNWj_DGGJDlzKNJFIUNaUWylbilGpLLh^D_MAEXMlLCyE[ecYxO[y]pPfHqSARysEHiwFDxAEdKvm]NH_XOwM`ajxVwbu[NKucTvfStRZtteTsfcKtjvswazzkHVGdJxlsuuLsEVenfhEe]Rzjza^PCfnwfclvSQkgcAkhzcpUQERsuVOqWWdc[]qOGONdi[VxXCaRkWlIyxUuBQUu`wgHLFK]bkVTkbevv]SmQFgMrS^stMNwHiJ_ipLwRDkABYXXSZy_xceZP[QtcRdU^ZLBXFNGRW[MVgLZuF`[Niy_EkOO`STAZV^bMWzNLsNbHXDdOqz]lJGo[hxhrzayCZnAIdN_GSOV[XWggfziSrPQOiG_YSVMSO_ieD[fdWYvlUDQQAMArgLK]l[^iFnwBDLtiCdKUFr[qV]UDOISJM[cS^HRzHuWctiIJbOWBflokUHPGZZxnocdbLuSKMLsUoLthtvIainbIACthZGiFF`cVUErZaTszg[valDafOtpzCtPR_jClgWigQS`qRHI[jJUbyHjV`DrMjdS[o`vqRGSPmOQZ_kTGlVQt`iSXAgMd`]whzEBlMRIcqEpjrPrpjk[USZvGM^qeujkQSc[Zicu[MhC[sdakpWtpBbUaJv_i[^iHU`WOpXHGpAHeWNdxFp^qvPlNuQmEMzjzvGTMSvYOmzAZkJihulViKRYQfAvHUCdBYfUtWfJXODvOR^kcEJ[dAoehDbRAQukWyBFhTo_GLRvO]BU]K`QsIXXqrhqoTJGUS`PPYDOQfUYjsV"
"KCNppldsObdOpfxjnBKavxlEDHQzNnzhx]ob^TUlZRifKNcSvbW`R[NQJlief[cwQUJJIPippgDYtuhgRMcIfBP_XmftGoqFjtsZJnSij]ybMlrnnxUOzofSufk]CwjhpxiABN^^uuYJDhRgvrFJ[cGvO``_CpfnmQlOCqma[qTEWPMTDbLq`MNrnmiyEOeWmwDdEF`ieOB`mQTKyN[xND^XOERrUPqlE^PQZ^GRqczpOzLVo]qohJiZfUzniNPzLnhmDu_kUgW_bAEFhsYBDvpqeE^ScdSAe[SYStxGZBnMuEUuiMaWaJkmxMPZNCDmI^x`mhJnvScBHcvakceWCwZ[k`[Fd^sXyAEXGpHADnObieVRjjeYgkME]oNMEJYZLDH^_Zv[dTPKtCvFnunOyRQsTeAuoydCpWe^rnPoJpT[ltdlSGQOeKMlqfKpuapUfYV^YYOAfyEJxpHZAPchiVXeJsgLcZudzzoS_GTusPcfGDoB_lpexHfQAkT`lCYMEJyAovuwioZml_GEfhrdPfrU_SnmnIqqVbetqAjpuiPTL^OMNGUbcoqRd]]HIIaydWKtMOexBjbV_IADb`RBOmnocM_Nhm`jVcHESANWatM[mZacr_]UdXFeCGkTwCgJ`tzO_E`EitaTpkCoHJiKsO^hNWLVgMxKodvbZREmyYfkqloRDwMmURRWSBHtKY]wV`Mp[UpFvL[cmq]btRjzKqSSdLHQwsozJo[LcdcWhYPrid_azsj]m`NuLPM`JCX[ZyVQzYiGc]YC[vbglWtwQgCvyiQTWr_VsWOfGNbbJcuPHKZQhanmJ[Jnv[SQIUkkyCKDgwa_pSnhmnooPzgqElMXNjfvQZosJQEzVdlAPxrhBIf_BIiugNqmqLMuc_sypc]kotVvponUklZaAWOQGy]DNN_Y_GgYyigYgvkHmnErZCFlftMQcsrFZ]bVxWjIUWBEBuEgG_StfdlKZhtM[IWqoSMYImpXUjFFBAjko`JD`x[qlWvP]^iLAKQiig^T"
"iv_w_fIYxIXuzSQrsWhmq`urAtFqkBCICpVeZ_SFN^RUFnMiuZqRzl]JBgveaShwFPsJxfXTEb_TN`mNgXwRLlJQTMdmdh_vpVPqvDkOB]NXMJvuhyJVWkUGbqkCfjXDc^sQwScgiQi^lDJRIrviV`ZevjTNALVmqUefCKyVV^hGCqU^gZunoihCENVMyVg]uvhXZ_OxtaTrsScFVpaaRbyIgbappmXzrLLGIxEILxuNUpKleQqIoFKTx`ksxZHHJTAgH]wYlYUUgknqWoK`vOLlAsJcMLKmpnEBo^eITYXWlqgMmuIObMuIPgPTTM`Nc_tflBsKBM]qoAKMwqRjB^ObJt`rUpzMrIISmaAsdyNVjETP]DW^uqMFQS]s]BpLMQzfrrZVpYGDPsrF`PVRZ]RBwCC[EIWcIBJZEhXedAijJpXWTMf^mjyyjopzRgC_uNjXBrrlhYoqRnNezhpJweOZFydoejBYKObiDVUuqUvHgvbLAdrsbPyPwvsOwDinXH_fvvSToCou^TYCBdeFJaczcYGZdGxmfIk`XwPEQ^E`[Emd]qNTZddr_`C]ptkiFCPVctfFT`pgD__g^wmsdTC`tBrtnPWnVZYpXAdKHztnxV[A_pRxxpjMpjrnMaQYsGdGiP^SckdZOc^arVxxZp`yjcbQlWvZdVEPqup`oeVErPyNLRwqcSyDafXGiMbLpKpt^Ojw`XJp[[GZeTF[ZEqQWAiL^jrcDJPhZFjxbARLJJdF`xUGGmkT`]]`_ZqaamRLYz`eItZnfGMXdg`N]wW`TucMXDa[edXRuGywkyNSwiYPFwsudDFpufEExXkqsTA^pFi`bRm]jDcfSHKgrb^bXPBtzSOW[div[zNSSSRdFqCZdePPfMYANiZ^hSZNSdzxasINsT`PubMcQUDzlNHPvMQoofzsVnMDClHCmswJGlHNvXxWObWrifgrYDi`X_iUIGNcvxyVKGo[JbtOC]HLqMnWFQdiKXlbDmEoJV[QcAgcqMhrK[]vX^^wTr^e"
"UPKnUNXW]EEhBqdpaKatcl_kupKaOI[SrpwSptNdpQVFVpuatiVIQdBcl[s_CrHTempLtRzhxxDSVYUFvegamRBNnUaVAiayICcAjgbiiLT`jMTFmdYKbaSlbX`kegOWFJTwmFMxEsEGhiDHNPNFHnb]JrYRHrDCxw[asWThXFj]MDtTX^VYIPSbrnugupNoBIkO^Oqvaifhszwo`ykTidohHzBdZA]gsu[DSfqIh[s[deirqGHwFuGXUDG^NN]`Lzpx`ttHbgNlTE]QJmSsabyoDjP_SvZySPepkSxfOo^NsfAzdNDJhfYrJJMUpgMPZE[h`rohelDkwXiQvFryofeLybqnYtzrAcMw_whMg]CtUGNEwZkoOoZOiyABOlmLjTKllyYfRRckqrUmS]ZPDmHY]QNiMeORHJXttjK[RJwKLjXreNjlR^IxMmPljYhCRdh[[tcRzxupxNnsWEyv]zdBZhG[jickPHuFkWJUGZSaOHP`cVfZlaRzfAmmIcQUJaqThIzueoDhDnSYbyGTwQ^bcGEVWbyrBLEaZMo[Vlow^ewshUKJxYsCPkbDe^DqLfrimzVKgxLaDyJLhCXoWEnL^JMWlNlGRyEJvVozozMW]LmYrwgBpbE^OpuxPMyk[mzECmhcobFaoByio_omFEXAgg`uB_nOgBYmTpKnENO]^iNqLJlXLeVQcEVFHhmCRSIzav[jeyBrNv]yZjjsnWCCgHKPZFgW[LKKaNQPiBmNd_ChbXfYxuOvQFTYlLP`SeXBlRYMY[KldoZHF^iCZNZawOu[tr^^mNvyGBwPyZz^j^SgLTO^MFELrF]XySawhzWt[BEbYEANvqZvYnh_PA`HqnPvZrAGZbB^wcLKPhEIFqPAfbzKfCghZPChmIHbDuoUowzpb^PcvXBA^OQrrqWuUDh`_YH`^ELBvMicKcImElCNCJ^rQjNZXZ[sRPCEIDYSqycCbPhNSeqcBZ^VuPPormdmqQu`DeATkXDaPABzGndrqslhJsoe[DUHPlzn"
"iPApPRC[pYAOmsgqruzYIstlMiPTiZL_AORqnob`pRrlr`rcX_PVlbeqWOcbCilVN]O_bqDDS[cqAIuhqgwKKqnYd[[_LA^Ga]^VQTBbLCQznqNUvkVlaLWjpuGV_sRxQ^e_W^fVizn]ZpeMKPQIUzRHfYvLwTkS^^Sdp`NjWrLcMWEIUcBpGQM^svEkKlgDJwSSdUkDYyHF_ZETZCckj^WNFFxeIzLF``D[mJ`zryXHIHyZqkRHJvUg`EN^XGuZmB`zWeaP^dskGOFxH^nXiEweLWviwBblEyUGcyDMKBe`CdaQzPWtS]kiFuozjsT]BHgItyVI[bNhZXbbqeigehYbNgBhKE^RhI_vxXWAuNZlODOhSNIEKgrTiEuNxhxWDwzRtgdYOhmXOgTJXTUxvYGAQKMt^IDMPDiRDwogdkq]t[FBllqeSA[epzAiZkvFeaHKyBG[_iTLcdGqFKHmSZtPcD]EwFMsoksTTuDkeVFegPBMsJdPcJmKhrbv`B_Z[Wsln^KHXHxUOdSBPLm]xQeyhgTHKdqfJLtJFgZHLCoqYOOuPqmx_KltDGCrzM_GiFXZroFFCpEcJrONENvmEieAHPYzdcHy_iISqpC[kguiFJkLspnmQlCz[WSbWG][iGE[ivWOKCYowuy`MHopUhngIrZ]DdK[zzFP]wZ_bpidL_]UCKzhfTYRoaEwljXYNTwvtPjKVQnsECxTxIYte]pganws]msXQWrbamoSUaaMmclydQ_nl_QUFomSs_u[ogFlWhfXqKFAbjTG_Jbs]eSRMWyi]ZpfFYF]zbvfGxaDKU^FZLUVTbRBnZKRhjUlf[uIdqHwNwm_EPHsstOQFZR_HEj`XexXdpXjZV]cEXIXpmx^SSYo[DfLJWB_Gb`cyVuOrXY]XgLhRtiUGMNELJwaJLaDUx^r`Z[sjfvtwWoryuPhgXuUWoYUEfqUI[Kiql[O]XaRKPmHiLRRWh`bJWIeB^eAF`nwdIHIoZKATCQHBRfsU_y[ud`ZcqMymaMf"
"q_`EJ^w]`NGGBckrmgaaTCAdh`xMMNnqefLmFoKBBxVVZTbN]GAufFIo^wWe`kQR[Q]WWxly`_VgeKUykkGNtK_]XOJYaRhDvIrKr_UBYGanHtmBgpzKEbFxtzicavDNhdiB]nsLlL[EyjQVWWtgR^zUDuRrpl^CTF_Ll_twnYtGAzJjTLJBXvKdTxcAVzjwCeelKfrAj^EGODvHI]`RfXuurnwXvXYJKyU_xJaSAhXG^FuhTrtZYWNkrSrvdoGfwDEadLTzz[xzoWmtODD_ylibHoGqiWXtR[aBpGkg`[YAYNw[aygakbhLV]ZoF]RcDEhTpkEHSXy_SiJtrfQjkrJknVgShvaCIqwU`cept_NScItgT`x[MbsMvxa[El[XrwWmpUPApLeqHeAyHyGqayjYbCJlHEpgmXt_LhZllhcFqvzDnTYkFp^CWmMrjhGoU[sDGZvQhFoRHTlHKhUkWlbwurIZbBEankVCvxTYIDzBMuDAFgCTWXdtwJPRpFEklTcWEPkASOhbvhwFZVvzkKVHGliCwSyvQb[dBWSSDJvSB]SMCZJFzaxux_ByWvjfeMVfpU_gXGMLJFPVqutoqQudyjDsQUgNMgWPxbMFfYc[lQlsPfI]CvoTRmgGRfWDZyz^izYPinIYi[riyKdeLKLQrDxKuFnzC_AmQNJPNtnOmWeruxIGClrPniffBOUcAJWKZFXkBTXSWw_YGQwwhxQglzcc^Oz_LOcA[XpaYFiU_sqJJNvxXfTMF^iGoH^EYnLivyiW_bOwHtIZYQTBH`vWuwjjlRcdvkgINiPN]BHlxJVpxViMlWggn`aBylfMzeAWMzLkBF_GXJYkbtlVcVs_EzViHNOkVvmTJycyKyKfsaPdGq[FAPddmSNe^cnrnk]XFacRnLeLMg^kmbRnJkGRIbigVTgWWnEqnLBPrnNEzY`op]rvQAiDeKO^swvZ_OoYchxa`CwV]BSkBtqs]kFWFAMcw^WvsoakdHMH`RWOETbcs_yFCxnIXTb[UdCf"
"A]i[bGuhiTKUrSlZdkYBXqpqncLnehKhTcDqsIRXbRWjmYRFbIreLANKhrgyuGCBI^BOPuaYA[GZFEVQGL]pa]TVsJAS][OUaPXDphbWLK`HkbnRhnvaMcBhsWlazG_ArF`oyNR_vZURnOIbNJei`iIyOjKZwAWFg`PfCyT[]WenNjteOJTpp[JxaaODscuCPfOz]`OqKMIDVl_qaKnhadqEFi]YEXeokkyDffleFzRdRFcoBY^hpMfpJtoCplnbUSen[KuYJcbSMnAGhWguwz_PSO]lxbPO_CxmxBaNcOgduScVtiVZEseVoV[MiI[bTToKpNIfpZRiTuUzfGsPLpJfXcycbU`ZoEQa_Yaf`jdfawBuA[abQLW_KInNP`EtsuPeoNaNkL^E[aDcGnxJchmWVeTWzfImiCmQMzTBC[uSgESgLxxhvjRbsXFcMqkpClWYonVH[vGHmZkRSgLBh[qylKYptkoMkFPviusXbpNzdJoYgoQ`fiIcuXgdVaOjHuXhKpcYonZrNqUcVzIE[PaBwhAQNOCBb^sjlHOrLrlrOQUEDENVYJPsTVk]oSL`kYQjxfqOlQsc^ingSvNIBhcXOAWWWlDtgLGpzfGXNydIQgUebDJtFC_DpswP[rzNzwScxhhopuCD[HbgmoTbKodhDzHJJInGeuRkCCeW]EPHAVluQ^f_XDKSf_x`_fhHvY`pqejJwFF`]LCeWOtfHcMrGXX^gqyYbA`bpvDfncSxfqZ]TJPlBzTMiiZNM^FHgx]g[p[LNsfAuubxAThihaAMVHNeYay]KQiTnC^jE_JTTSoNrPMJYYzX[EiNaoRh^jdmNKxpxeJWgsDTzmIUpDmQRsSxp^jsgpdK[eFOT[oqAobBVNogws[xTVrAA]H[^AC[I`IDGcy[QDlw[OiDvoVfryPNNTygW^o`bMigcqWrzsLRGD`NZWbMgoRasqsHeREnOLVO`CFX_gynxGKnkNzvcsmuHnlxPNhoYDuiRiWv]oHHTnvjiTtEEsfTcTXO"
"jIIRzJUuM]CqDYnkfbUrBPvTdkaXbGxQHMTgqIJYB^MrqNAZscwlDXhtMxWZo]fV_mirFHu^Yvq_CE^VjG`yJMMk_[Zsljd_rrSE_ITkhZTHaIrI[AkhxgLS`yTehBljRRgKlZqamoRktFJVdRYkzwaLgA_apiDfhepO]gXAKJuJU[Z[oxBWdBjVk[_luQkDlfDPPvtwuXapBXqLbEmshhCgu[_BIfGuYqXjXKKtDFL[tUSeSKp^sWEdWUpWsnOQgXKAHnWutobLqLAFLXqQWa[TFApU]LRNuo]KS]_nveDpsnKUkLxnzQqkgBnb`WdPyvX^vsuRiVl^e[SpeQkU[u_g[g[lLVNZ`urnqoTagEe_SFMjxGuGtv^Atr[MZ`]z^DE`[CfpxJhVQzTU[loFHaeXruyUHGGceAPAgOuEIkNIjt_GmilX_V^ZIdIgqCbmgyETn]teWyqXOHA[NVYVLFema_emodShgVDuZmyMrnIXgCt]ILNeMbaURmPrHpnfIlsEFAWM^YxNF_eIedVrDZn^aHzjUkyUl^RAYCKmEYiJcGMOei]LYoFRFa_s]grm[d]cMdkbYdmekeu[vlEfNmwwYAvr^RQXawjzdeBBxvmoB[agExlcM`hg`JMhtEHTVEouvqRjChSzHvxZ[KYU`pUWBTE[pTEqfgWmakOfB`_Ni_dMnNifmo`jYsclrujdTgPbhuZKi^PqJTo[NBGVKthGgTiI]oHqulUsUfFSOEQQtO_nRdscOR[yJuTMRhuVYvYzwGcvnGioWqmFJVLKPOgZttLkmGxoJvryq`qKBwoHcfbSHOCvjGKOyFCvrJgIufyoOd_AJS[_rOSsd_ECS[ogjs[oPJnHziAFTSzYXankkHrkDOfcGbKhSFWJkkz`S`INM_Q_feruBwykcDMvH_sGpYBNdPgkaNYLyd^_yGxkQfWGfbg_DXngBXo]DvWyGTn_nmy`hxBbXW`mfGxizM^pe_wMjCuKb^dxsrTtcHqGWLPfsBqwyhyupxtPpmqf"
"CqAtmbunnDFsVXZYPRThLxMt_jHuyDEpRLD`LVtRHDJucF_Y_[z_YIZEeGUDZApjzPbKTuNuXCaIoRQZKjV^E`XJlyuEAprHN[vS_UEeNFGWbRQTpvtkeRfZsYZVzItcKZuUWyuqelQyZAb[ibOEwH^vo[GujeEL^LIWvwrADFsBgArqzhYtpgDwz`v^ovvHLnvoZDmeXDtBeuI[cPvCMLWrIMXVBTMHKkqtEDXvun]lnBRFX[v`AsuAWDfNgWdDrlrtDPV_rzIZieEHvRyaYgHSqq]BXKO^JZuBKXBIEWhAkRmy_RUXDfKd_DMmUHhkeQPjCx_QqBzHbU_kCtstFFnjYfToVkPAPGLCCwanEbbeLxjiBSOor_KAeSK[lmyrJ]iGSEp__J_widGTYALYbBbCSOccGqrMdaLTLwkBAgRhdbVGSlnZuHvgoD]MXhRRSMDvAMEWbVPaefBNA_GZKaYPhXsOpcyzKUW^ZGGgSDTmfGJY[yXJLsIfefToMNayWQVQO^ntX[GGNgebO[nqLpoSLktonRZF]VWVksKwK^lklBYyBl`lZkmaDF^ezGFABesPOrWQqfiEEbJSunBmifmcptgngSLXfjjX`imuJcEE]X]vKkLETSWsP[VKdd_ZQYUrlmLBGcMCi_TKr]cASRWT`QOXkarEenDCeJX_KxlGTKlGtzlHwZyHx_M]weWCUyDkPZahEDqVaQzV_H^lokHKpwEGxub`bilUGAr[ZECsccbh_rypGfOeqsuTU[kK]Qnr`fP^QDRjhPbMVXbGwmZvZG_NF^GzUsWSpjItc[_CT_wbQiywseNjrrXYGBYmxSVpsPXfPh_NWcvJwwX`JGjeg[xRjmaCmFAJHiBQSgKGlP]OJFAMnMfKMp[UavxZQcmCOvHN]vdsKyio[cB`Rn^x]LqxGPISAKhkMvKtzRRCRBqm`FS]XwzoPFljQPWfaKoBENhQIaA][fVDYAxfuELsHK[cjvfblrBlsfLPVyRoZQTNrVipgGcIcF]qaqFq"
"ttdTSwIS[eiMahhtGVr`xmElZcsmIVj`tuagAuvMVUBXWbvYdjKy`tHpsujDWJFvJTFmHMG[wTbXDTCAxT_f_aTbnknFKrqLPGOC]JQWckCMRjm`rjGCfZXcA_yWZAQF^VIwEA_nDIDo^GPEyOOlnaqigByLtxcZxjwbpqKSz]WdEtOtXKmpTjpROYyacJK[UaXT^JnXQYoD[FbIjRmpedzHPaCsdkpSvO_]uWOXV_vvlrIrqqQhXfkxXMNck`hhzl`qDgpvNU[FhEVYXLBmZHuP^aEhJfhprNwzRSoKBV[CKfPTqoL_^uOrrPS]cHmcheOZIWGFXrlDhrFsRkrU[ISwzEICzSbtdrV`WpK[iFHfIKJAJZKrQXI`RR[DOEGFQr[q`]`WgUBhoouHs`HkdLjqNFVGMezCVRjT_exiRtln^e^fJGLax`OOHfSXGZKCVyS]FbdObYZj[vSdbjxFYYEpDMsRacsjjY`SAab^wy`G_oLTipXEdwu]ZdUkwBqaAl]nRbIWc][_j_hA`dlKXeFepawNmzxi`_CiaXOOF]G]VnvwTLTPfJVfBJAMJq^]qJzeRzL^wRIyzOBfAtbfhDVJWhWvNCwgdA`dvRCLImirNgBu[VLPOromnqNecQPuVVyiXjjrV^vU^vkf]sqgXlLnqEDXds]o]lRCLCf]YtUsYPv^yGoXDjEdnvbffZmfZaBJYdQYNUN^XSUaJaZ^DEkmhhPQ[AqzHjZrOlFUpVKj]aM[aBFzPbUqP]lTAyM^NWht^SY[EhhuAQG[pjZ_lAsHEAzOTBpXxLOK_aDjDeubu[pyWQ_Q_hmN_ao]TJHDrPwpCvvOgkHGQMeLArD^_tPZBgylaWt^qje^Aw[UPeoMeHNTKRSRDaSWgVBPx]FUbdPacGKEXiNuEDouYebPDbZQWOqevKXm_[eBrglDGOuUSYSdbGrSvqQAKyKTPaJMRWdYnClt^IphEyyvNDoTErhJeqt]XnaEhDqBiKnEFZXiSFzLXC]VtMdi[owJOjbk"
"zBHwNCPjNfZziRltRzlEjorMTOWrvqxhGkTErOKzMfgaKil]LPx]y^ALJudzfYoWimtyxgBCplrEHQCe^uggs[jcjIuwfKUnrQrYpm__NvhMgwDtsy]lmI^OCQOpmEWeXDMrYepkVHYxn]wnEBEgicBjndV__jlI`CpcjM[NJmWeja[aU[klAkwpC^EoPQLgWQXlhTDsar]vLWzMJpJDyAIoGzaMHgysbkOZIjsAXsKINAdf^rf[HHhVHcfhnrluteUIaE[wIgGqzbFWuiQA^lZCtR`ELNaGQPdfI^xmK_yb`IB`RsItVsvYPO`WkvGzakyZOXl^qznx[TJ^_tUXfWtkkjNiNonCVcyXDGD[WxDWExyvFBrqaiTJqNj_JDQBLkcsUl[YqtAiT^gmlSdLwwgOpapW_RThCKTUFaTbZAhifqTbwnr[EEPPwe`LeBRchARSSYfmPe^woydyNO[k[sdxvEX[AIwXiEAf`BP_HEK[bS[yBayWbaIpWTXNAKlHiFNjxxZMzvdVhJWcdZZZK`qHwLOMdB_wTClFlADHmWfxmHNwodalRxIOFnmjNcxXgCfXu^sYWSYIKKJEAPoXxLBFdMzyOUuJZORk]qCRlIUz`zEvdijyn`Z[A`nh^tKCYOjwsZcJM^gpqC`dIEUafwWtoqjwCUorySbRQHCKF`lQtedyAK`Z_eXrqIBjfGsior_[OTBBK[FLGzJMkvspUSTvFZZzB[ZYsbEnF_mq^YUJRQlqyoykODU^cVotBAjAhS`uWHMJBRwyKeohdBM]jJoMcQ^IxxYeJdxMvchmaMmX^TyaGPAnNQ^_SRFeDLEp_UAArhY_jJwBpFx_cqheWsGZud]DiUIxtISyjfbbP^Hb]]MIFVWUtyHuXaEw[BooEvweatLFtVsl_xXXTyNzmdAKKERF^fo]SmXbP^`bJQ[_FVV[Ihc]npBvm[bFcNaVZm`qHpYfpIEojrwqCRi]lwf^DDADiNxjvdvR`mDIzaSZhfmkJlLOhxAcKemUflmv"
"_UsGveQPNa]KeMqYRPuwBiCycTHuAj]OoXvSmgo_vAG[qyyoUDVrptpSu^c^VP_DOWNR^feWGiw_lVIGgfGXy^FzsILAFgCM[kgQiZu[TnHWWmNVPctxrXjL^n]PWWZMtSRepxMMwekydWxfqDcFUDBmuwfExTv^EdtZehlumyNuSFRl`VenxNcADDPbsgWDGXvUmWZuUZQDNscrTTXyRjGNBsWbXJCFcKnuKxyJwjY^x`nKYN`PXmXRhStAs^KWL[DFigVUOCbQcQ]ELFzw^qKYVd^eEIEMOHmEkEnlrKAVKGiXqpom^OtUQejEJlPGyVC_XUGSOESqoOPvFnPTSizNXnFVicGvuXoEMtY[hZPKa_W[JDxzmghDNGdOlegxXvX[LiMVpfsZohsSjv__IvHlHAympLwpdMIcEwPrcKGnNOaOR`qFHp]ogSKj[`djDqc`POngDhalAytmY_^gGjXgoXdFlWCaXjWfAyDlKB[OlniZXQwCxdTVLtqKbRQ^KdO]GX`[vviAIuAjJh[AdoVB[H^MadkKkaKZwQsWcBhfUCy[GAtqp^bosgQCNUSyqXUqPNXcEYbHjhYJCFajtUNyVaecTUhUh^eddrn]NlFAhJoNGAHocE]KV[jmnAkDtmCJoaYOmNgcHyiXhW]TNtldBZNJIBjNBCGuGeZabsBVXtDXmREW^WqABYHUSPGl_ZbFsnQCwvmUrWLHWqXqwSjeMFth^EOSlQqyRx`yACZceJfNzysQpLjxsmuc`rh[cb`MqYRYqtsHzkwO[^GmZ[fPUFnEX^buKsbFAJ`KZZvScxHpgIBlvAHGuxT_UZWQNpsVIZTaczB`SgWxxYOqyKl[LDhMfFA_CVIPZ^ihKTAFbbQFWawKv[OfHuwWgUwuBBQRNh_uangfJftdmcpFJqccH^I`aSBpWGEaRzriCBdIndATdD_rkKYiKj`zUNlqODbTRnkHCIMZo^Cg_NiyR`VtHdpIJDgVXwUYuhBpTXocxytght[G`n`ihhMiycxz"
"qCrJemqMsftknvIkJQc`qeXgkMSrkZpGGwC^CGeugHgytlDypHhWvAY[^t`VtBuZuhpMifYJyfK]lhq[XUCdmZLG^czJbOBeNMDMwrPJj^YQpvipCEx_yRg]iiGgOcjd`O_ee_aFtuOBuRbtKG]Q^gw`bkOIJ^[vihPhJKYaXZEWY[bWGMIFZFtSzCuPXvW[TTNCujAiwtEb`HHJrCxBRpPbVUKH_Qz_iwEHTUL^ZJ`BRmsvkYVyUI^Dgga`sKftatdCAp]qZargjaDbRMnsdaTnbDhifxcP^g[YJpuvv`_nQqUGcyNfWnPA`pxIoL]ycICFfY]SsxEB`^kiaGgkxskW[tTTxmHUBpihSBx`lT_pLyzSlOci_mAjYfelEIUFqkAjpCW`IGzdUuLtBciKZpPpyPgWLcyYzyQ]isC`XmXV^[IsC`gvWukuqnkJxfWevvj^c_upHtX]XdlLNSYCS^caNVgvItbmWm[ckfP_fqn^]N]NB_vfNTRYSJPNsyhOSfGfIJ[cCuW]jFDAsZudyw`C[CQbVCfhI`KrHrErEScSxpiMFPUYEDQWGEKGttNAOjUKvMtBVC`fTMFfTD]FhMcEPokDgxyLFcEN_RsWTDC_fCuy`mpWHSBHPuGqUXDU`lHrJpmdjUJ`adlqEHKJoyPCGMdKXayfyJFQmHVxCnYysbiQUflNNuabElxMcMIV^bX_DwlNGA`RK_^SplolYEd_piVXlZaPdUARmtbp]An`BKuN[tn[UdE]nzfVbTg[Php_L`vJzPYRD]enIxrgFvzP^^nNMOI_GFm]EiOUrRUUgLj^BbYWdzUTiaAimhxlGuKyjHjZi^poaedSnPMfpmuGkYo_iabJTdOmAlnuryhFK`mDwiu]SvUzENRTLlIwf[xWrRC]SnrEovLgkbzgEiBitmw^]agN`J_EphEXrAJ_EPKICecdl]rxZO]EU`AlGe`fXHqC]coE`]eecoryCDSbfbLzCSAXY[j`lvo`GJhmXwer]aDInEI]SpPr]`]E"
"HEJvCDaCYvSV`KiwT^]OreTRMMwZ`iddLiPr[w[]^FUIqHnemS_zCNM^ksiqHG^jSiKrurXDzllPLFYRgMKebymGcNdVWcfqIxwkvqHwC[N_XdXp[Q`hHLVIktJZeUIeVSI]akWDh_]B_bLeL`wSFIBlbcLxx^YHRcJEuPjt]AcjUdIi_NEXq`NRWvy`ZFKFVjwYGJRQNYcHYCMyCOYPUjDvmEzQJ[Wq]TfhjbVwnKXwGsUiA]KgulfNs^TERADxMRwibRShDTb`DHpukQQgQkvliXoMNQemHZkowfGiK^dbJGJkiZUBkWhVKAK]GZmioXdXAxEKb]nHTbbND^ovdBkmOYlfwMDDqtUiaTiQrJtuUCdYxYcfqFngrIbMYNVYjQ[MshkpSflsrruWINynYznbCiLGPsdo[LDbXaPimSLTupAOKELPbxQQgeqy_iCmIEWgqpkhiEiFo[gMiwwLhUM^unumjRJEPYKqZUYVT_a]OL]MxeQcmrFossmCuyxXveqVPqmU_bsIojOjQioFAbSYTWyabV[VaIbeUHwHrwDKzwAIeDmLJsXTRYCtrMmIH]cfHdUJmhOD_HYWVpw_dOLikOSwECQfKuD]UUAAicZ]eTpKzg^BPN`sRuutRXFkWctdEwCBpwqBpjDXYIxY_OVxVLLZZHjR^iFB[MFCd]MTX_Cbq^[IZgLINwOtlcbhN__GeCwMCuoBBC`qIddwgarq^wn_yMRBv_FsAwxobOG`rKGLWXkHzeBhhhdhOXOpdnF^whqMgoVX[SySqcIbKCQ]YKSSQ`JAQvEEdOia_[KPeMViQqfPhUZkoA^]^tWPLXJWuONTZAwjsBsLpeBZttSw]UPckMCfEuDMDyoCdRpgXkrnHSgtrwVSENu`yedNyRexxgIrpzqcEg`m]][Y`JnI`lIpEOEyibiFGuqJpDsGnOYiID]JKip^XL_Smaurt]yZfcfyOjFvCm^jHwLbp^ttce]exrD]xOtlZO]qKvyjbZehoKqVSBVcFqBFABXg"
"cyntlQt[klecuJALrwJgMHeM_IUY[VpJ_SHdflv_RVebXnivppNDrkZ[TaUdvzVvDcBXaFAfVr]Dpk[ZceHlSosgQEWScqWPfwjGtYPICLUcyQkeI]sUNJJPjVGiiC`[NXtAU`JVziGZQiATmKGxCB]n[W_RoZUIpHksrdowjzO[CpYhGHw^wbYLuGaxAra`UxdIxH]JWJOCnBkfyrLg^rMNj`ykDqvrYiGFLQjzfmHmTVYtWuzJguC^kWYYJlG`NXeEvfZhHMBkj[uUHEUIrmL_fdUprRptgc_wkTylRCZsSnQEREWxiLGYRQdqqWHEHvydnGdg`jlfTpEUmzl^itDkeug[sLDlOgU^^xmRJl[FLPDqfeVNsOyt[JoD_iuFJkrR]nkXkRPJpLPdYUXGsYO]e^d]FNmazR[XmUrtx]bypEOtLeJQnNkPcosQHveE]cqtoLgzaNuwtQh`OjR[D]nmbLpx]blCr^NOVEzyJk`SoVSHSCXeKAAoDSdWi^zsEWo[RseR]tlNZmp^qK`UYh][YKDdPgkDZkz^hAJmThHOae`ossS[H[WuTBHpJhMMAKACqzJKVbFzUMr`_glXTQwzTEKfsUcfvNWd^ytiIEaUW`HKbR^woskNAilp_SByZ^Zy[wAiriLMWew_C[QDFjhC^IcoMEKqCbvtGJkfQoEGh^rWmMmDBytjqW[RTM]n^VJojDYtkemQfbVzQMCzRHoEhLCjiYSDSRcRUDhiotTXLcEPB[AGK]WaEG^BiKRRjyTyDfN`TKYoWtFHmiOYMtqqLlPwJhGiBF]ekRyomaqRCcJLBWPAwtFpI^nSVRWgEkmRWWjZffC_ndbDunXFmHzgN^fYKKNSpsoaAnQ]O[ndkJlSWcOyyeRkzIxEAK[sRniOUmdpPt`tr^TuGEbvOULbEarqM]EzuSPWCDCpeJillrHfdJtFswCGcHKovndnp`y`NxsDBmQFf[pNYMMCcyHxPIousNur`vvLscXgRpDGgxISxVjaaKqYBPFIxLor"
"Z`k_dVqoakmuARShgcvsLQzlFmrSS^UePPMYkdic`uzqXwYjxY]VPvfIayZq^^]CWHekFuwCtTTRRJTIUHVuByVUeDrlKsEBYxZPRheCZvcUrXfQWIcBYmQGBCKsAPSsTjErh^SYc]ZmQM[GtWhtqwB^AadAQagQ^haZotpJYvT_qpcEyebYCXibqGuSHBBmoin`[boDg^GJ]jnYMcqFrYGyzciz^BXtaCBHp_^CmVVucXUWGuvZLheWbo`stpafFMdZigciEoSDLVcOSscGyhsXf`AyfJwpkoVnuiZRMUWUISVYxebwx_H_kCCHcjZeWgSXAoTdZRtdR_OSJcAAczrLhqCsfRj]vmy]AyYIQx^ge^NMnjyziYCWpHkJmxHcWi_amSKKske]LaJcqFwTjn^jbhsOOWpzVEOpigAWnpqiYudaaavVBoscLpyFZwv]ueGHSUtKE``txoQhDShi[ZyaLfdthvwruUnnn]NMglRPyfW`RBHOMB[g`vhkA]JchzdB]tbECjI]QNANTVtpciSTHOaafOHtczBVCYZCnKSSJOig^hHPnEQQhVfB^zW[AdiNysRxZmp_zBxzrtyyHrG^ilhhnY`nnrRaaieVJpAPmQgXfJdBYTJ^L_NKeKBNCkIkWZWLTcxBFOw[APHOvlU_IDUwZlzomNBIwAMnYhpVHXxcmKUCmSQVdbTEZjuryNTAuREPu]E`IBgXTiPWwB[xg_UhITinyXtDwi[GFIRpMqaggGAe[BMtDOnjbmHnQbkHc[eOJRojekpDnANiI^VhKt[hW^WKBvOkZELmJN]PSYjvbQuFHDVZXbKamnX`cpaSUvPlvhUIKvaCxwFQwnLVzgeRLjVpxg^rWbfAIK[jMO]BHcQrSOgCVxUPf[jxzSNODLbuQEHlSIkKcJdY^^iAZMVVh[^juBNywBWbNHujupeFDRG`]p_OcjHiujZQplNjweMbCvPbFlbdbQSWUxNF]zVMthg^hdtIbvJYE`^a`xIqI_GeXDwr_IMxOzhU"
"Olt]sZCgHtRo[ha`r`EVbCGehBdYiLKFrIRnEgyjZ]_mHXrchFjuRmMfPpqpexviLxRVn[LaVuIacs`emqBGKaWFHprItrbpaKFoMuI`XegDWPLCnwpNBXvQffTYDgymSU`RBpLSeIlZEYbwrD`lb_nJKJaakEUyhQEugkmHevWOKrbNmdzHNKnbWGsOecaonmnTgQGRXHgI_yTcToO[Ur[GRTQettASpZXFnuFL^hjF^eCXnPupTG[vShBTz]X_HUSFbOoZiCwgHUcgQtqdyKyiznGsmLBPKyJSxp_GRJnoxEVCGfnawVdFGhawrvWxWYUTDSI]Dh_MhpB^jEV]eBoisjDnvK[OWqbgAjDTfDthFsZfgDMm`uSVegUOUbsNQqqOV]ABCqaHGdzPXHrxhj^LBUOjER]A[oCcXQaMk]EmN_D]EqMjv`bUSCTIlSx`DHfowgAqRlwMXly^RmeOEnFvNiQUxY[Y`_iGsO`XqWYJcVxbEprzhjQBkbOWfttQ_hsCsfCMMXhVvSsljQHblpDXEIhRBK^_tEiE_LKon[RnvGXj^hVlSpnrpfo^VuLNKqCOJgKExmhQ^P]xkyoTpmMEQqPtXhZsOEdFEI`q^Qn`H]U`IqmXGsYYdxxTWQhQTpBYgWtcYyq_yhwo]kkbuCmWgKGKMZMd]MSvsmwKyIBuZwBJqgaGVQCemunXM_eu][eSasyekDeUml`EBVnf^HvPqIWrJmmdXzwWqQnbfNYPAkViWrlyWV_VY]H[xRAxmoEiKFzEBiqaIg[WvsYcbwBVsR[AhvYSNvRAMlluVtq_vPqfAbWH_KNHhpNieAomYOukK[KJH_NiG]ZcoAF`QoDiJTW_DQUEDGgVF`XNAzw_BKVeMpAjNH[qCBHneDOteCVnMWTiSPoGJIIBtq[UVpyM_CZYdcwlDuyocYq]_AvsfktsylkNn_PNXahTtCEjTDCPJTZAypUuGboIbZsrjx`flXBk]dwL_chejeRPNpjuSQztKSdLrdF^dYwXl^VT"
"XibzH_roac]HpkYGCrRQskUv]JARit_j[V^ejzlg[ofOHxitzpVtLE[gTg]x^aGgEmhZNoXCFfqEFFo_HRt_e_WvqQD^oIthyJUQOfLXSoaNWbOrx]WONH[OQ_X^U`Q]wWGv[ONGLpxI_OCL`Sle[rjFzjOhDn^DRzSXiPvIlveRIzIJzylRvPMVP^MZXYSur_njnqDq]ivEoqjGLuo`SzDMGCZljiU^SXR[VmVIvuoNUHaYnFN^enbrbclXRHoUXlW`PlZ`fdXLqsyBydqt`pZzoJdldnAswVKs_UnyyJKgejePWT^S__MbFPxQvuzdKyvcRI[OhsuWfnVE`TVQPhUYJcmOX]DEJagnByNsPefdpwSuslb`jT^d^YiWLqLoqMESiR^VHYACioTg[U`l`[MyCQcPrhYZrndrzLbOMXRT[uiFCvrR[TQmsUfAvFQkbdrAkp[Jy]PHJt_Ic^NvqHz^]qyf`aTGGalqDhUNhTxDIdzq^YXsVu[eT_tqoeXJJBcHhEuIzZIoC[TNzpBIvNkfRMjJ[WgDvnuHArIY^OswmgbtKgqzcOOvRxq[VlicmEsP`MNIgcfgVnsfeqyoah[g_zqCBhDlaPJhnSTxZiSzdca^ELq`mtCEdLkaWKGz`tFFeLrS]BVrVDwZSyqr^ygNFDccnbObS[j][ttT_Cl_LjIHnHDMQHBuZs_HOji[GPCAT[yZE]TNIBWu[og^CRLYpCcUmFoTRXvgIoTIeIooahQqIMovCwEyrb_XVESX`HwkGjmg[^su^WsHmmL`RsRUTxoaPvm`pWOZlakZZH[_DK[jWfdtmGgpbXIuMW_lTEGIYWMBmFJXmQIUqsQIgDQcdJIb_NkwnkGUSQAZcGgwPCQOgaV_OcgAp`oMDZNsYzaCGBasqa`XTTbttuysMhLDF`mnVIso_nDkM`_pfXBHd]MNTK_PAMnn]wbPEF[Xgdp`^IhUZCaQJw]CaNZLddoRAQOPyPlawd[zghkK]Jpxf^e`Z_iqfPoDp_`NuXWK"
"RzT`^Ws^OjZnqf]fipZauvhpXKII^QFnfffJEDwIqSW`UPysCbLbxeXEgkESNkzydgILATbesQkb`HIMQGwE^pBOb^UzeuGSXwhGmsjzbigup`_OvGxeDuGqHxYpNfOVdOHLnBZYWw^oIGYCyFD_xP^przoZtmv[RQaXTug`NHXRiA`WqHbkYIzdVwCey^KbTrqsefeJrYOHG_IyPDCeUksIaox`rBajxpxqJgmL^XsAtr^rnDVv[`MdziRqhFRps_KQaSodQrGxrsOiQGlogePXdUrVw^YFCIUgNQ[OmedYPlH[c`aT^JNPnhMTNUTirpjtCDqQ`uzfyUckAmWwFctuFt`Ezu_PPkXwXaa]kCyONdoTdlTyXcPvpoI^lReWjWOwgS[LuLCdehEMObCRrHIAclXyzlTs^NaWwYsPELafQHTDQQCCTY_GogkinvMQQmJN_q`pjmNgb]j[YUijiR_FiZbQhJJyPV_yF_[kMTbLs`RwQQZN^czOD`FOOI`REhRHLSpNyxxuqtQTXtBvhWQuaZLEoQHKNKloyez`GBRJiMyvlgPIMeluamWC]XJgnggFccuXwvaUzlrNNaTIByxcZqOMNxDjseWC`sTIlrLRDXvpuJovDAfoiDglTiJzrrVUohUNKXJng`BuYREIIfrycU]LboChLZMva[F]Zhsd]AC_ervWFrjLdWm[KJgVUbNSGUyLci[nNbSybktfcUjARJhfSQB]PAYpL_]wL]IPYVuLILcw]isN^RoKQaQYYQyI^nCNw[FLZcvO]]GiEAaHLRtjSgeMkqHXZCfY`j`OnBEjtvoEJjTQYTxLtTuQqsc_pZqiJVSUJMKqXr`DA]rzFEZ_MpOXu`NVFUN[aypFzozj]^YQp[Fe`jVXKwaMu[dkwR^Sc[_BtGQ[ojRpGEFWdpVyqQdYcK`LOmBLvyMAbcfdFwVNOXoV_USfv_SqGZiaEMBiGuIsEpBKmttuWJLWKgt[VQW^uJ[JxDAkT_I^f]Z_RJebX[wORWXSbSq^"
"fyuUsMjtJbrBilhNUIuGmv]j^vfgcYlMSzA__KZO[lYqcv`aiKxZXjljLqxbCQlpfjikIiKiggA]gHcVFxX[SocOQDTT]hXgAwMrq`]mklCkNpDObEsQ^qGZZFUrMzO`qfKW_PXvhbD`^DNQFuLScV^FFXX[Iv_moLWJtrDMhxdwnavVrElRnvw[HEQGsxmmCRlShEWNqzfembS`BcDKMysOwBobxUjKrt]odinh^[nQShamkfqVjbpStDkCcgtfwZvhplrToWglKcwOBqR^TPDKuaxlErbITsltwzn]aSvEHRHbbpbHcxgefqRWDRpKCOD]bEr_BmJUzCdY]K^BzBluFPRJG^BGtFDDyz]yDFpoSecMsttnnLA]IXSnhCuPHNFxaV^KBDvwvGneiwcMeNSiEaubgjRBLagE_fg^WimkQEaCdiEkZieuuXsz[w^SlukVZiFDQFqwcXAzsxyeab^vmJGmgD[KZyVTLducB[UriYSzsAcy`iLSNZg`zReBWlBeChoOCWcGNfNTrGtgflmDFlwVdkpSwXhxLLTwIFEEcLSisXxauGPrybCRgnsQuJRwJJi[I_]ghy]`dhkSsmaqHpzdIBPVMoJYuW_ji^YWddJAE^izC`YIc[oTJERGOHZtoDeNIBdVTKN_gz`fTQShWAq[bK`dLRHJQLAs[rAvDpYNQhK`gnyom^nNKfx]zJBClUVn_msYSlSLXMR^UE__JgXnXmipt^vtnVvSWBtu`]AKcqxT[Ive^tEwcWlEC`opZYBkVLxCfddKWcpwkZVzOQ]XzjAQpsAhLkJoNxPLtztQt`zaoUaFZpKH`TWwmqld]LrfI^_KKGPWM^Zk[RuOVaBOtbnOGJIgWUHufrCvPjvlTpxl]RU[iOpRdudxWvzFN`GEc`peOyygeQuEdCmpoiyI_qQlNF_Tjq_ojsJwkLjPJhTz_TUJJR`ayamOEWJ[fwomGVlspUD^OsUIxljlKK`_zNkRXhXUCPeMtbtJcUpNBSFLdRShXQWPmseZ"
"HwizNIPVs`JiqkRxveGoWEhezJpdoA`JAMs_aSsDTw[HOcfgxpxzThlsBqAESfaszAidsmWcmC[Ze`VX_JgyC`pS[A^jSnkhVcHOtBnLZVMQNSNd`Jzbp[KVJsPgPzRmDCXH^^V`hT`FImCOjNbiomhE_svvnFNy[OKDUFCn_CHGfQAFMXaljEPPkCyeZ`mcFaeubzudLoJmeaYOrqjV`]PWyvTRGXTzP`BFfG^N^wavu^eJdkndpXCdROvgfd]n[K]F`TclSsdwTQKhWekJs^YoXKrbtZnkA`dQIBRU_ZhxnZWwGIkAebH`LbuyJIskkaWOdZpTDkqdk[FwSekkWykw[pZaP_FCygu`s`RLQ_WjbqAoBr_HAFmFgRIdVnoMlamoLX[FZ`FZFAUepf_YzxCttpsaWhp^jKyADLdeWZizSzfx_nkPVMn_svV_tiljbjvCTaeBtBaPbtbpNdrtEOUxK]KQOG]kwlXcEcdyJiVUg^wzT`RQqreTVxrrVTnn_pTqykeAtkVtzQgaLwioMtEGEMHlmdqvbprKG^UohQxHOZWgLQJXFzSvPjWmdtNQu[RKZ][ibtHvwwreq[`ZYpc^vvWGcLnChNoILNiqpLrWzSsmvJTmqfjh_aDbziTtOiD]eMreydelGvc^UgB[PrmIZ_iIntp^^^leoPRw]HsrBxacx`NTgsdWvuDF_jrFoGNdKyjFVFyIzvHyGKNf]qetUiuLLYmPmCukwYyqIxbuXAmcHY_eZRQckpQotKgLsMsNkBhLW^`OkYwhHdDAOfTldHMciBOnuicLH`rFkSNwmBsRtchQGgxMtHedpE^UOlUCpl`clyPvXWycPujFUygOkWRAuOQfpsiAhp]HDE`IWBp^JBTuVYAPFGRZ[`h]heF_ixvZYcBPZ^riaortvWQXsYvb_Dh^XC[mwQoLGt]mWkqbOcsfiobRTigPCUvUv_sWCEmQzfmBt^AIbTSwKBpcJYNd_JmyHrsr[_JpbfCC`Xqi_iiBzKtiPvtg`gzD"
"mgIQHoIGdVmavmc_FqlzxyJb_f^PuKbC]eKaObttsa^vjAHDBbezODphyicJfriLK^YHYyrPbrGW[Ns_^[OT`boZp]b`ykOBsSkLbAeCjVYWwZNkpEfKcUfdjzSwWRQrdF^I]gw^pMeKJX]VOqkuEDY^gH]ciifXEamqvFNJHIVWkZS]whabWmahzVKUpHaMFcef[JdamwhCnKiKFmAFqhoMJvdsymSOp^Wqhq^xAdqseMQgAODGVXoY`akbAdZMPor]PQ]IpEyT_jSMy_PHTIyCPUwxMYdMZXwp^zmeTaTqEwpO_dURsHqpMuU`vW[muHmqW[`]^AcfMLYGjipIrH_ofARFYFXJsEKpCYHIwros`LcB]NqMbSumcxKWJZEYdZmIAcDRbhLATgP[XPdLOKbaKXNFNky]nVPGZ[BtJ`ftGtYvikzuqtJYrLycUyDqiiMxXJBwOjBj`iZQySW]lEguMqPN]Y`LpLYh^`jCfGrZsImidCbbEdCRlato[pKqOk]Bc^ornjZ`CmyG[oo]dukbVVkxfvcpwBKIglFGNknVK^PycnTfBhMUzvfMnrMsGdcAbp[jjiPtGUWrGlUrbxoPpqIlFsSW^vPbvOWnTewSYTOf_nGB]UQJoTLlgidqWsVKFpwsVb_xA]rc^MRmQtIfFnbCeGeAFw`yDajqOgvqSIRkJJ]REFyM`zBUqk[twuaR^_x^[^eySSxFRxIjBqbl^ahYvxWNlKNeMyqa[AuNC`P_P]lNtjklBWqyFi]UwjkvEqlJsWaqQ_HMtgyLIUB[dpIr[m^`QTpT[YlbVYTJukrimMB_InUj[cNvBAAeDRzLStZONenwiQbtxZFOkzW^yGsrLp`UXiGdkgWtmnQ_zXAKBUxculuEu]MLAbCYEZ_zKxIRCM]wis_aFiqrrxZgoCjO[KsLks[`tAzDLgahPxVCxfGRkLbGzsLIOuqrUQLqYyhEjnfKXmXqghQoEb[`RTaBproQqiTym`ezyBcxaWjNUbeBmmqg`EXgPsyU"
"UjYYDHCRqDz^zudTaVNgovbIGz`bRmjyLCApCQU_SLjnSheEFn]kianqPxH^j_QP_nsd_^KBiUdLnqYUE]LyMO]]MynSMAcgZ^otmracdBUxbCZ_mZNSSCQRVW`krpilwmQB`cMwdUlMI`fvTnZbVTH^cXAslgTdApmXzKNQjxQEIqleWDVHVcfo^xKlwldFs`_pIvoOUFeJKZ[ajVhQMNXKxtCzUxTLLcRvPGnPyeE]eBaQvogRHfKBmPslgpyOrOotzYJZxZiTp`VZnhYW`QKQRZ^ru_RffpfVNgQ]fMThwrMMtZxzXciaD_cavS`ZHDhzuzLHXvlSw^TZNQyaSNDGERjEjbThu`tJMSuNqbrWeaKITeUDsppetph]DhTYfCxZPAxDOkklMk[l_vWxydBkE`T^MlnuIYkcdlsSgQpIEDYIVwxHQWZBANVkh[r^xqdnzrHnCEQiotfAHKHPnfdlzjcWWIeLYSKOZeI^oKFMFkMGbUd^TbjRMroQPlWtZFQ]iWb]Cd[daXgO]oC`HsqctCbqsODnLmA[kXKmG[jGFlpLannwUUSMZtpUzgKVXqUefomEePZgIj^HMWQE[GZr[oBqbpnZQTjPXLalAjSEvgDjWhHdWFvQW^Krmv]o]zVoANkBq_DloUvnkR[uhKCKYYstAizFN_cvQOjJxGnsBK]CbWHcGSwKWbUqAH_y^PmA^cRwbXKJOwqSeb^deYQyFsHoGFfTDgMQbsgsKf_vmBHe__WaBVtr]BXc[]q]lqFcjakmvfYztUMDl]QbjFShnUoa[DXioCJvv_vxoefVwtyHoTqyLPmg^YpVlKRrNkLqLQIMbSnFZxltWhlGNZ]`Q^X^j`MdSoHLiHw_LdWsgMagvyEYKeoOfahse^BGJmfpaLOkNvdDdDdQqrnbHLIJTxCaOfaGUmSualGkjTLPAkecxXBnj[zyecJi]bU^irkijFkDrnztJcSZcqdnWtmUPFQ^bBrdu^BnrOhwCgdMylSsQjFgxbGoEWZeoN]q"
"wWyfa`cTJhkkQaV_mXNrvLfTWNi[pDHpJnQbmgRkGl]QqW__AurXzeulkfPTrH_F`ua^QomGG]lJiV]ziJ[vLkq`s`rHWxtyNXkwqAOMIlQ^tkCVIPfhhcupEBfkwfjIdmvReaJiuNn[ZSfH]yHSLRf]vspV^P^CItHpllcGwm[p[VlV^OgcpX[QoiiUSXxPYQY_SkkWAVYrpC`GZsOorZS]g_xCtmeV]`BESrL_kPbZzdlIbu[IIYyywrnmi]NlDUPBnbgkEbrs^CDMtSrJNTGDGVzXlBmTivPLkhnSnYErGFZpkapKWbqAIxUYcPyCo^Kbnr]L__VoPPwBwKgLMcxGFODmSVk^ZBCcusBsEdfrLBksATYdnqMaD`ojtIosVOMUHEInHdwFuE^KmztSugLqCqW]EVk]DTf]ggMXPpfNP[JWBextPxiDWRQhgjmsZBaYB_Fn]CFAybbhzZXqNbWwZPVvkdFtUpYcxAGrGnZsiFpjchKQUBL[K]bOkrlYzHdlJEB[T`k[I_OU[fSCYp`SdyYYsHOBAsbDa^fkxoRlRfNNnE_jzKbQHKgaC[YZHghsWFpWiskBshmOLEDkSBhzKwVrFi]WpRKz`UXzjSRSqPO^uRrWUlHGyii[jxQAVUMb]miqhEAzjVIvhrworGBDKDBrgwPlBRw]ChAHdAiQlHWd`nmsB]JPSJWZlqKkoPyYYfsAroKqXkSDgXStKXk`IhJtQbWnqxJyLtulAMRSrcrTmT_RaX`ZQk]zkPYXAZOquiMYyYW^NppbBwoCO_XEwkh[XuBlntRXTBZDHFSHBBL[uuyn[WICKtKBmAEWfkT^XOA^lvJMPDv_M`^wOQWlqVJHPCVFOlWQsw]gxPjzPmHxfoM^IIUTchCEsNj`Dx_gegPCqyybvlacBJdNIbGcBxPP[bECzvvRO[Tz_JhJJnXrHZDid]X]rvfhWAcOtwbzpOUbqeiyuwYN`YIB_WY^_ZZSofd_bYPNMXBsEsznYcv^YHxsak^_DVoCmKBJ"
"XgKAHWQwD^YjFr_oW_fgAimjJwgDBihyeBGOSXzacdgzFTV_QOWgV^WnurLkMvUqZFEkQ]z_EEWBmepaImnSAdGfTUr`oqViSIfNRnAlXHQYzlHrBfc[HBpqzZXvfZkOjiidycOUmBvtShfbMz^JOuaQToOrVEGxXCjRC^sZWNhGGw_RnqjfvbAoHYQJTXyivx]QyU_jGEmAN[KWQwE`V[CpCvJeMjGzPyKIZNTZRz_rozbgNtsWTYT[vgODLu]XxnlOQCwCdeOVwLBCT]XTqjgqfkNnzQOW^HtAOSWLwqniKx_jfMji]JaOHuc`oJDSWrrpSAnBGMWAvxfiM_Hnb`uUaesTjqWnm[RSFjXfYhcQnaSKOuTQCPlqx^OiIZRPpcNgTc]H]UtxdazRiEp^Gk^i[Ub[j^zWSKSr^vSqJ`ussUwPWZU]mjskmLyACYzV^fIWg[KJ]gwLcqcAfLSDeualScESxRola_UNMacwOLzGmhHvcxVrFvldFWEEVsMLsLeDVoEcWoZuZye[IjzwwfenlIp^bWzoObCE[NIQOjHSifXZdMIMOv`APkMzdFpY[GLXcWiqghw^D]XBtUOm`SCCokAJhJtjrbZArN^gWcNc]ogjbbL[JhbtUabygy_Dod]DWKgrcWxFZ`ADQbFyZ`JAafRg`Ljdj`iKKgLX`dbReouAbuKFtaeOZ^ktzRWgTVurqteWiYpxsWllQFeBdkIvZkiI[^`RLeQYAJlB`iKHoRbFlzcuhVDcMzXFKvKt^mNFTlHUcvaBGIJcX[vfepbhLAlfrDmzNQ_UAXO[`EtICLsOwbE[POtxfXIkWilYIanqeoczMFawZfAH]jPyRkzXbicP^ErS[zXCeHyFOnISALmIdtz[qqoYDGzjiWONZJcEscRsDwrEa]qHuLsqAJfdebAzW`WuE`mc`_AWKbmtwT]dIcoLWW[P_z[QbR]^awzAkObWKgZXmgQEXqnlK_hE]EaDkpsEOTAT]tuBXBukAslSKMKjmVYSvNJAT[fn"
"VIZfwc^vmUJdxEdSNs^fXU]xcBafkAEbPxfkSgOeg`xYLoUrdqn[wAWAfDuCrKlPcUzYtAaetyrXEmnhiBtNdePbfErdTcoVLId[UZkqMqDB_cbjQNUpQ_gxFYnoJxQRK]MvITSORDAmaNCC^HTGOyEggVwfvhPamrCWfawvF_N_uWrWlRBeKCLY`[]yVcVWlMoWAfsaWKI[qIbP^aueffVjfdrLEy`]Irnhc[VFY[NzukbWsM[pikkFqEkWoh]XfgRyVthCKoLKXWpuC[Sr_koXHhr^kzRlypUNZdAy`Xet_`i[PpUTUetustriCKSK^[MbBbY[USGFusbzXOufyXTwZV`lSwVE[WETGYhUvsASmpxAQjfwY`_CsfcMwhIXwBf_oOsza`kFXBJeX_dRZgurC[qYNkcVkKZMBqfVtpZfrxoXM`G^VkglnXIFH]UofYZyRZmfDnFYWczsgBBLrVZm[OY]PlTEqLD_qFnmZcfOVLYQZG[IToDzLqLTInBmTaXwUqvOZAXmo^FCAMqfsv^Xz_eObYTfbtuon]wcKLjTWQLhlITvxVtXCAwqMggBglLvdQP^QNhMJAiTw^ZEXiMA^mn`GW^_igeFPE[spfjuLLtzN^PJNwPDyvF_yJrxI^^WcDrDaOlzRoRsoSdGTRmWbGUyvWr]swDhVcfZYGgh_DhHofhgakdqlGoisHJXPHVkd^LgJhNfEs^ZNKPjLvHSNwOoJkay]PPwRWLI]zXCWLgORBhr[MTzC^hOW^YHOIocaIqzHMM_FwBPz^HPsGBDvxOt^tFyuHCrgktfikOApuGYOtVJPHHlhdtxNHk`KoyXTnNdKWnDFDIXFfLFpfzvGzGAiM^HPPjR^AuGI`ElThbal`Wbvek_pDQUgNvxsgnqTquEkFxPaNcZYaOIqvLf`L][ka[udHthulMufRrqg[xb[rwGynGx[IQIU[UKfDfZUu`hUU^`EVyENvztdTvqBwMl[DaLxhLdp_MaSPlePbsEJMvYaRHoXYhnMQnR"
"cWKVPiiYaUbRa[DIcOIHkHu]AGns`Cryy]uoxMkCuMjpNxV`KyIfbGPQsOcuaBRG[caNoRvXibuKgmIYSUESLWsMevidpsvcVe]oRWhMOt`UFMZ]NgMnOMcUdscSbMizcRHgdByXGx]Tr_jMKTNMioGYu[i^EuggqV^vTntMiphRjP]FJVHOz]RnKjmJSSLQq]]BnsX[A^sHoiexcTjmYhUDkkeAjtN^AtmAEV]koyyYfVAHLjHdZsIHYQDOWkHQVfnorx_xMCdjWfEUdt^JFuCIlfbnSOTEriDzcMPOLKWWPYlnMjf^MxKdfDMWlfyi`shbEmXdPyu^gtStnbdcqrt_cWwIDrZ`YhjaEMHwOtwyPxxmlSdKCVshyJzipO`X`VEyJlpa[gUwuvOeZKXlOO^DoBbuMYHzT`Zusycz]CkWAvnDvVPfqoJQsBJIKuUB_VcVUl[bYOL_HCtO_zrII_VIBro[obiireSXbadoEYaxllyQtfv]sDMEzSSugGdg^qQXaqwZNTXkyYXQs^bJgLpnYOXPOjIxouZv[akCj]P_g]FavBBI`[ASA]zRZhK_LZvEzpRn_ZKwRHvIrTMhAYUtHvyaYdCRlTMbBb[MjVBBpWjUEIcTyqKBbKsarlUuWtLxvUHPtzyfXSEkMe]PBiSuJLzrjuVyAGewLlZjBS[yr`eOGHTMOEthaQnjUPazezfQiH`[KmOjOTDeallcaIjvNwVvyjVlmUrtT`ybtCqKZNNJId_xY]PjDlme]X]wnzLkNLnBhFOEC^IhoalNoCrcKPoLwWbDUNgNCZvXlwlWBfBOewhufPHkHcayXbEQ_KjpqufVfUsQ[a`O`bzjgXFGvFZswzKssqvLXjRSPRnPZhCBjrzFbrnejvoM]iUQApcjdCIsiWXGRq]YWrMILnuADQ]]vEpaw]PeESXhjGWK[gSsZVpeMFNucgT`fxdaqKobrUE[`YEWk`WxeD`OLFjXCj]fEHDJnde]_QbBlKvoBqKCPVvcXVVk[X`HsYVX"
"VudGvuZT_^AJvWL`ZWjM[mXOtBGlZITp_ZGNEHVvbGssVb`BTcZCgKuHLsCsTbBVJLfPx[SAofTt`sRFXrjRIT_zCLE]bzF^iKwDlWHGBs`aOfzkWNKKfsCRxQvPN^MfOWriVXTBjcPQnykWMTGUpurpIeGst^guRrnRVyuJ[Ccm_z`_CuXrpT_BakRsarjmdsgtcuxHZg[oMVGHvwpzFoExTllfhBcdTqIfTgvIwPXJtjoKeJwdBkgpni`rJS[YDqWN[oPBhCqWvEdwbA^FzbcJSreGEzdpzFDNYwJVgcUhqYsevubRwSEPtNucJIwhZAWpBW_frIYWNvTtzALKXRshBoUKBbL_fgOpCDd`Vw[othByuBdIxmQHHjnMCTdPwulag^rwXfp[rDnUmVQAxCatWGhTPW^BTKszv[RWJhlRxpQEvZ[gDdStTsKap_gBInHgLoYREtrQRPwaqdXpvqyjsyfDPlbfAjIJhesHBJcJmCbEwekLGrGSpYoMIUBTSMPcGmtMlQomOpxpXbHfBW_JdqTPqgygHPqanzxJl_[qCTEA[ypbMVRwhFyQZdVESIfzyXCXdyu^vhHPmSorrXZlxushAhjHIV_tiECCvXxOoAzyj_Go^NuitBD`VWwOnUaJNahQmwZZLQItsiRIcSeHdKpCy^aAAzhZ[qkIqqCbo[^dtKggFofkYaCleROWZZLciZCyIRmPCTIhLJoqtuMTMjBPiWM_nLVBSBKkeiw]tLhFXVa[^Y]_MsVzYmPlFNryGVKyELLykZABvsTqlBViqoZfLRsYLWgkpHQAV]YMRa]YVx^uwuTLGJHSv_mmTmnTnQMobSOcIXQSBPOox]QC[dgzyFqjgt_mJeFliSdZFcymqAtP^yVQ^QTg`ocNe_wOs^JWK^FesqxreuJyLbMh^NPsRYLukVpagjtSr]FAskXgTBfeeCCSOkfOQFlRLu^AXLuBrHi]Lpnqc_pW`PGTmNXyNEaXTihpugvQHen]TaHZxofmPUmvoJNKnvnm"
"bQJ]Hxvwd_PvdzMFyBYZoIywlBkZl^LrBvDXeRAtiIkFq[KKAEouUTnIKaWvL^rBED[hwphA`kfXtFRqaYgjQ^hIvMEvEiLQjAmA]NKdHtocIkR[FEeFLSoam`BdXmwr`UisTQTSNDbWiEAKvdpzDGFWMMNxMbMVxwDnBYHolSrIkXrh`qhnwwnYORgHDWFfrMZmBMJ]]BOlOxgpUaXWtYZAJe`jmAlUMLQ_xFgShLRviCKRfqm^yjlYnZdWhZk]Pu_CgHrPtEiiG^SG[OEvNOV]Zsl_]QfN[pfn^He^y^duADSPG]vYVoOB_BcoWTmdYORs_cSdawke]Wa]YzUNsfbdP_qHfkA`pAWbtdpos^XC`MOuvmPLcckOPGnnZRzeAwb`FqGkXDnDMMwSbNkykPwMQISnpBVa_nCdeipEg_n`ogCCK[LMbBtlAUOx^SvWiN`YKTisYfRMXXdFPcUqIz]tRcP[rbTcXqbHdsrmSowjhPcwerOkba`NC`tkYYaExdDap]Xkb^UCnMBzSpATdBrRUFEzHTtOsmELCHIWqzzCLLPSrkFd`kUbMPXWXdNdGKAIpxLge]dBpGIpmHWOQiHX]zmflzYWlYEAOKCnuwvOl`XJSbeQMzTXDdfqKaveDMQWBIbcFdUTmsfwbdSKVIJmEgxe_LcTBQL_PAIKzCKdfAfKxiOaOkPuD^RUKxDuAAR]hHhbHLSyaGEBmgpxBERAQlinzCyQceiI^qBhFG`ww]XNTYQOwzwAWgvsiIIUiy]WsPqoPwxlEUyU[blkTdihhGKFpXyWcO_uq[I[ZhCyPXBiaWOaWdUUnjKIfi[_yUGZZfzZRQgakXQF]_YJYAztJHHPzolM^zuwvOHlOHESioDRuDJJCVaUNPS^kZ^`ZCl^UJNXhuRvgKVEqUvebKIYa[hzoN_hbnOmSnauwmUPZpCtOhV^txPuSwuOc[WdTtwdmCoBipqhJGefFa`JKipzrzrfYhJuEfFwGQfaKHZNIqPqICuNZfDwlmLJdAgv"
"c^Bi`oEuci^haerAWb`HzrMZsJ`jZXsJ^eepaRTxAelIVWGuKmB`wlRbpVPV[LpTMCAMLpQLJyNR^nZoPeqiwatTOkvEcaYvoITGiywkPVx_qPSbqaKbyeQ]GtxEUHbYo`^UZSGQhGZbcV^yYTOBVrwJsrzpBsVIn`jQlxI^ZutsLitlVygAtaDwcUbTRPpkQnDfMN[Lkb[yExJyNZCLUPNmhXaJIeDIuvpGIlLpIycy_WSdGsNdVBabs`pzfbZquvCOIHQQA`GOsnIBJwDQXZmtUONdvMJb`mEcTDzbwdquRiFWopi_JZ[JJuuugUoV`jGjahG[Di[qwyC`HMHW[CWSeZKIbEucp[VccWqGjO[[zeNLrVmZNXu^TnEG^xtznsAd[wlWAgMcvTzMyRMS[VX`sOoRjdmUhHdlcXYADMlhuYZEu_ghgS]dvlLI[hXusHNMWWtnRXIgsxqUMpFFhoVeAprLQaWpqpnKNiZHmtVuUwDnVnNrSoKSscVg`h^lBTpCJfH^VxToZ[PDlDq]S_^BeSeTnbEVdtAHBejfk]UWtXSRu]rBZ]qnYiT^KcVOZpbPwkYGqcCIaNbuRBsFNUKWKTdvVIatyNxDqDVKDkKq[hNKGZLIkQBEfKFcH^liCJKTQ_DOghB]JTw]GYpvwXJhMsYgymDaz]oR^utcWx]CP_vRNcihplez]GXFWQte[^h_fyM_M`zuHYhxHYVRVsKlKRBn]^AhdXoBNmcdPJQljsEY[yxucOcTvUUxYCj`RRjdgjUqlBMWtz[YxUB_CmBJMiSXOgqGmlZYJ[mkeGJaCLZwuvAEAK]`P`XyXwcFKFeI`BO]cRuvd[GAOV^SglFbOuIdDnTh_DVxaQWtg^]jQavoZWjBZPYYADufsfZoyNW`FJ[yzdgIPVRhzTKgERNPeww^WlABEilDJVjQwlCpcQJemMg_mjgzHuIMwcOdfDbdN^detkSOXgUnSlJE`SyFDC[aEzJoMxMcA_iCubBXXb[aVtLYJHFlc_KJDplJ"
"[LjHKRNCbyQbh]btLKokfErzTMtqmQNxVlZLbxwGaH[BrhuPsewYpHrfKqtOJkLAUjGp[Acqn__kAydfSdyIToIOtnAlEixpv[gIbEosBGeQcHeLMNw`mDg`brvgzqmFbioaRENoGGNJrfmIo`qmXDsnHbjJIsLU`juTEukIilfwZVjLJ^LQvTJevmKuVEgNvseHlZEuphDgXvUlr^AehuYL`PXSjqneEYBakc[jZQOP^NzXYDrDCaB[Yb]ClKzuYczuoBODQKWYKwCCXuRmvaBsAFSAxhhG]TrvNFdXoonsJcQ]`ZertMyraGtijNFd]QwqFHoRQuyGnJiXpp_CqMPVzVGLviajJVtgZXXfTazNrEWY]stmbagDgGlhFVLYXdoplWXDsjPNB^uyijDooIdipzyRfyNLnp[[bcl]]iiqXqexLnyWRXeOAFzlXGXu]D[MC[WFybSGZmRzUAGszqtWLLZ]dzEWQoUTWYulWsYkF_pOozWXz`ufJDArAu]y]wpfXxdwFV]GYJQhWLiddhWltjKSXDkWZNJiJqqW[ReCoMAnT[oMsUp`EminR_CeBkyvHzNVrNJVliE]K_EdwP^EYddXHDEvOABENEfTNYUfLZoWhMdm[ZimMt^_VpOoJNHlhAOFtkW^ustyoNkuZRpRPrTbGyXKaySCigx^tjDLrTA_jIFTUrmR[YIu^nmySTWufm_ExvYlmGX^TDWpSrMeLBcQ^CAFxEs_xFsxdAWYYA_TAfbZWwfHmieJOauvSoiNsaNKYLR[nLL]OPs[ajAYXWLWOoV[]^GLqGIEUPxlUATa^HuyiZUdoPqVqwepryQFAgIHnGRlPEWDetGtXlCI^XbQgj_STv]CD^mruTDr`KZYkwqlYFvIbcErEcyYTcUzJWeGarjFfsWqNybb]WGvtuWvVqkEYLcFYbaMesOMCwkiANFA^qrLOEPNLKByBNhnloXAPxeHtsxKiuIkkbbTXrUREyZiNNBiDuOYhzNtNDUMSR`QBQOZgTWu[GvR"
"_wtgqOGofjDkThGT^IznZIgxzbCj^UybEwVRJhVDKkfSN_GyMDXSGIrVPwhPMXQRlgXSpJpmwci^Tk[E]ZxfvEzBNAGqn[TblsifXuc`K`JswMMnIuqUFAkTSovogFpOJmycnYF`UFQWvGFBoSisdzlHUTdyotEjujJNdAjpRzzjUYLiuzTWxeKRNYFaaMLgAXvoLSzCSABnwHuEo`imQ`rAIbeUkYn^hXonGoBXuRJqLhxa]mh^tjsDoDkUicbh^G`THyz_QsyVBo[GgnNgGAnhvjRlQCLjcIibveZ_mIMBl^CzcdhvUub^SrYYgONOhOmtGsKCUjuxomwXhSIgVPViaLPEMAr^vyU]lItjZfbeAejavAkUFasXtETvdGqTsWBpX[g_YYNjilhwiJLP^YUFwchpyGTpOmdctI[TMvgTJjIWWZuweDUygjrzjZSuuZzDTc`isDclBwM[RY`LWKLXfvJhcU]TZOQJSDqUOmzmMfnOXiRuH^kObYXYsF^bGXciwRyUiqIYMfZQWYLgLbMv_BbufX]IOXfoDFRFjwLQVRBvyHCZtrV`KgJujEjQUzJozfNQHGwPVw`MAfv_VCOWCGdWvWVeMuwVHNwSulSnvGJOHxuEIKvRlmmsBqVkkRC[AJEmjdedj[vwUOx]R]CzeeUDQDWcCdEjOxsIU`aRbQQqvhbXNZpAV`jQfSaNHYsrNGRFTwwhCTDfHdyUdnDOtYJriNzE[sYyiaQSu]t_nSLff_iFOmoEnzNtw]NiccfgZDKg[Gu`[aM^LK`tpiDTLjTXWXOKCKkjAzEeucRiJO_ggYQDJcmtXSjknQDsk]Pq]^LFuyNWVJwLgkxGLA[YqRi_VuHROqZlOqJvExOd`]CI__DDSwgjCrzLphkArvI`kLwqaBBNmrtTP`AqrDzRJjjXqyGXNOldT^YScM]RuYxLyfvPhtInwqVA]`zcveA^FPzlHhOebiXkm_YLCqkTFxg^HXqCNuVQYZFifKIzf^MOZfOyGo`EthYId]ej"
"qLXgSHSTYk`sikibkYD]JswYURWkBxFI`YJZG`wSSu`WPgVJzqXLZHaxPyRAkbSeWSPbOKlJOT`JivMlL`ujYsLKHNvbkcElsUFbkUMjiyRejpdF^qGvkhuXWETwaJYgPoQTpfAqTKBZbb[SepDuIBLZiLfAPJ_`eUv`QQspQc`g[qoeBSkG_KGnFuG[VpDB]uwM`nT`KELa`ysWBzwYYBZOIZVJY[__KatcEOyKvZz`RavODUqHcDs]xXTyWmekCwUYznwkxN^b[iqt[uUGeXvGhWehvAoIpP^UL]PPXFJV`RCPaZ^X_FfBOkcIOpwYUlKKixfEi`NabWTVG^TlRKsTwmhHIPMwK`sEFlaqSp`XhUOnIUZFd[zSIlLyx`QBdWqQBkx]Yf[sPYYTAvmqbFsmwzUsUQPWrQ^m]Gd]dKyUrQn`e_UOnpld_FkceELWEAzufquRQNWH^ug_bTkmuHDmJyBtuyGmymP]stndDyaZeFdYMBRTZ]wx_I]stYWOPU`IWBiZxIqJ^BHEUcgthmT]qSxkXukfbRJP`CKSrWQfmZ]AY^Rl^b^ADSEDyCCXLbMss_taapVCqDsjzWZtEkfP]JxHTpUt[KNdItekPwlIxMOMcPDfyR^NdCjcUMAAJWserbFmieIy^RPuk_EmivJhlW`ruIujlHoqWblhxtPgU]jRmpsqHNSONKt[CNrajYNW^Z_InXopf]TOhNFFPEb^p`cLiQfjCjxvPToSQjCkZ]dkrenoIlQfrkaxqSZ]lFnw`XzTYmoaBqESwsUE^qaQYSE_mqxPazTjgLZeZWVwkdAgOByyVzu_]_MT[acRrQrkdPguZE[LThiDs_o^yubEpMIyGJsoEFZHJvemSEvu[TIq_PcCnnsWumvWGhnPLgGOOauwtCLuyge]DS[qbqBQNCNXAOfeQfjoNubxVgIkcyXOHYToNxiUfpBLT^vfXadTwqPIk^uiQiPfzPburWTwUIXtWdrk]SBUERQJKDkngUHmaUNTlaKPWchrPUtd"
"KLyguahyYkogsIhoQHpPeXlmrFijdSviqsdWrzgOvAm_XNutbI`ezm[LopsyVFpFpA`MYpPpcV]kATh[pjlmJMsrAKgBHitpLgiXih[msjBeQjLHXJfUCwcHiLNINhtg_TeQmHPyuZLOaRzKuk`N[JkFPAMdeBHIdKrqreBWVH]XsHuOyTxUOMVd_AmfCVQYDGxGHTCF_rECNiWsbyfQEntXpORqsVTArDTmKnGeRrqXHTVqetEEkAUQM`QtCMNI^ELKUeinrAzMw]Q]sMyRPPqjD`e^NcEdhtCHLzKQMjAHTkPoBaAOqNpU[RFEQeYucPcNUgAMoAD`qbjoViMBDVrgNnkJuGAbRF`EHBZUao[qKP^tbQibYWrNXwAawqShAOkCfhUPrPuB]yQrbdYYJuORgKAsYjnpiAnrwcUSavbTW[CQEOiPbSUky]TWZmnHLnx]AfAEGczhyPkycTn_NWfRGyGAvGInLLhEnEdhLjVomimUoGRAdxWRLthuDKqumECuoPrFTmhQ^UPTgHArmcNRmbKwmHQN`tzek[zqZuQLjBi`XbXDyHJrlHMFvTLZkdUfdLHJYsSBblte_LkMmbmpmyTnl^bEjPRFLmyLwR`PrpJWT[XIPgJq_sesUYuQNSSZnZlM_QB]yxowZQGZd[cHOguTTctfzd]rw`BfvwyLHU`uqvuHnrerjS_mrtKhcW^V[xAuO[^gXzuGz^XilrHyeHeAPoCmQdlqK[svAsSTxWSSefrVmzq`WWppVtzguLnODw`e]it_nBOpSWdwgzRnM[neD`GsgrDL_DaiSlQRxAcVazD]VjputACdJuawbArwxDQOqvevizpLsXDDe^xCIR_Bw`YKTkxIfX^jpf[EuLuKqklq][w^utomlMtVMkVPsCMzaUgyCsIwXTLMnbhtn[OmdlxdmEsJqeRn_lPIox[ClrUXBhceC]Yo[WpXDMxzyGmCXhBS`EpzLBlW`bE]OGUItzEU[kll]CmlcVdzaS^XgZcYfZrHlsleIYdh"
"roTE]JPznJxFwvdx][WfjAtPdBRuAyNLVt[iDbwZJffi^C`pOnIiByxqvD[[rVzpwmjC[uuu^x]DVKla]fMp_KXvdmzHbogpllT]VTtNLE]ukSuyWjmzXbRGqIunBViW`YANzGSTcSGvBIChN^KEtpTJBV_Ly[Jio^vRRgzQXZXFWFiGN`_Ld]kEqLdbGVHJYH_MkVa^uh[XOheQqFluf^KALM[VjQEQwDPbVrzAQwCSqIbTlBdM^uw^B^tFILWWHSeViUjtveMOytWTtBxP[EYISD[sxtsF`fK^_LeWRkQsii`HGaLGNHwCuCfuqvBmGjTitohXaxjn`[e_SmWhlrYiIcDIz]oPGbyLYMm_jgJMOVAYGbE_TAOReaQl_XE]v_YsfTtaeJCLO]tNCktuXPvFShbsvAzRchRkwHmnh^uJyDCIgDjo_JFfcTaxlBeeKkcbLIRuyFMtVlHOfZLsQKnYbfHcboBgtPyOw^lppYwMPrMRSSplTzxKz`BiSJEazFmsnK]hHVUVVQWQ]jrIwzQRX]C]gUWMJET_IpHeredieJlRCgVirJnc]iFLEzizEIMMDlUtj_FUhcrNlk[zziVAQlBZkw`swddx_OQS]zTEZutrrZT`jAWmLvSkuImyMXEzVQM]Jg^iCwqqHcs]MTNdMjldvaYTMUrBqubNqVC]hjPFcRPPmtK[mg_dbdOJ]iEoANHQPVxKH`z]^ZxGDwcyrVIpDePmnSLBKQUKyAXvzTMgBFHfHG`FGxeFSNfFLWoTySNycwFwLCrCvIB_IHgMfvDTqPrlVRkd_tijsICKAduDFcJJLGxEkciCOtISETatSO^C[SgzELCBzNNGi`PUgswLf[yAjGgCAlrFHisZxzPWfBOguMWoR`gVarprizVV_zAOWIyzgpWgda`dMSG]kslF_QtBlydtrgBAPlNWH]IzmzRLfWssffO`JudiGfEYCjiscMCVOTlPSFj_yiWUFlOMY]qVH`QXjwCRZdYU^dRuzMDKEjXPjTR[][GV"
"gNjDPrcJhN`FxAbHxRZPnKCpsMhWeboz]DhPuNeHeu^rKSSjlK^KAqc]LtLDJM_FYactSRMjgRfsgLf`uye[Deyg[LEBCbyXWXYdkYWZxn^iLjpzNfILbDBobwZHNLgFTN]PmBobL]QGP[kWFnXYmqSVeCozPMmGYVyN`MMqhdWalhSikIYPgaWJtIzA]SykJiu^qsbalIZldwkSQ_EUrmobDgEMqKRYqczXzLuPrjuYwYPMPhMntWSwGuZKnUkbrBuBwSTFeo^IrWeeWrxIy^_[QDRFlj]]yTav^OPwUHJFtJEjPoovWbcT^[ksANTvSAJ^pjHHcBorUYaGybpwvaGXkNXC_WvGnNmsAbuahgTVmlyEtdtZXWzmtlmbSEdLFiDHghFXhPPPMlezPSIu^]iWOAzOoSNcCIPEhrYL^]lHHkaIHnSzvjiczhsgirdUCgMDaRXtxWBkaO^ZqHAzNhS[hEAym^gsXF^]ha]uGyTgrYYVMsjcUozudnRZHZjIT[nAxIb_EHiznpfbd`exQCRdttELy_e__yfUPjHEQgQvpyqjpbplsRJRWEO]TdJKUOQhwOQuqLmfLiDvOfWjsgMoTyK[elg^GaahImbQuvg`^GiATOrxm[]cDsdQG[L[FIFI]NGiEGpxzhRNPJHWtjkfDydPJRK[kLZVBa_GHIsgf_cWbVAwTobVXZE_dq`nvUNoOvMjfAOlqEz`VNinQOGveDxbA]RdFlQRoHgJQQS`q`TZ`IVIl]gNRKKxeOf_xHlSjObGCeAxoWo_eHwrGJscECzdwi`CfocioAkQaPFH]iZvWgT]RnXnPIOgPyJTExSAnGkdXfRqDlGVdDNQVCksWoYfmEBcGmC_WYHQwjFfbj_CsCLksyCN[ezhEqSeijRjbyKvxUfxEWpEudtBqrOozRwoIcrsmBmrYp[tsPm^SFIDoumxRoq^jQCVnU^nKDdxgFIzJaFkuItAOpVvNvVvGKr]]YA`uFZqNhTxINllBoHJV``HrjphclyyIgEg"
"jwADZJAHCyTzJetRCiHwtthLPlO]iUpIFBEoEkM_XFtWq^xrhw[`HnqPpohu_YhIrqEdp^xxuY]oooZNReBYUwBRKpyQCSAyXk_zZTCJy]ejRC[AVZcuXPnupnYz_hvcX^WDP[oxbsZsRSAIUyvdjGaHweJgvCeRbBRnfXkvefS`pvc^nPf`mRtRxPGfQIB_NIRPAM^xOAzDnBSBBgsgXE`PIzqWQEednRhdT]amiRlSp_vwFeePVHIcBHAWGgKV^[zjeXiJStjnnxTuZLZAMNgJFcL]KfBHWpU_LShwx]YVoEn_hkVRMTa`p]gHOhfXHyxjG^]_OlNoQ^XxjxaXB_IKfns[opNkjXwpamaxNqCFmPREjYQGOWdqnDTeCkY^^EsbcWwXcJKgEs^bVFXxeBSyniNTKFdxciXjSt_ELIJjLpCgfBatdqOedOKPyErjKGrumvEFWpKoE^XQ_[cWjEa[rjNjPFGRZNAjLmtG]nNAEp_lny[ePSmGaWDCiUqQjsUr]hFla]uwr`VBKwQe]BH[uIoiqXHwwRL[hOFsjDnYpOBABsMY`fMrNWc`RCkEmzuNKb]MrYvYOiefHnhxLjWREUMRgabgTifqUnqZVAbHjoc`lwX_VmtDdcIpUL`QrPF`CoRTwNcLIPbQmh`kICQapcMlfBFK[QQ[jC]weEcHKivUBsChfpIoy`k[baiuIYkryHbjLdFE[JVHhskmyX_eHKSBUEVhzmxDVtRSCjETpJcmjpvvktFk]gWXJOAinJdKe^qfvvDHfBjOfO[lOsrTsNuvSojVaf`YILz[QoXKgUAViuclOBvlSQfdbkdhLhbzQG[PAZ[gg^M_MXLxH_knGraWs`LkyynAuXoI]OaQgeIWC_L]OaCPhKOutHwOkhhjLeodC[tyP]ZNYzpBzNkYllMNaOcGeMaywYKYjbZngDALkA]uRvO]xPy[YbDosmUCwRzoXSXnwlhlB]`mipixEBg[kAH_fvujxOPj]FudqjA^QjWF_BJxfFsZe`EW"
"KtHBrynmMqPcvFGVBIJAdBAdlxDsWqEwB^YxbzgSvetimaB`y^YVDnJt^PtuvVWlSbYhJnB]rFzIRrLv_ttXdbxjODDjnXzO^wZzdrpjWz_yoAbfExWC]yIleiYFkRiZFkfUYLSgEKBdCzvOXfpAxm_ohVuZjzEQEsEVMMsekxRx_jemKdSgsYIxGalqXMVLwSxGrsOKnKQAbQxkHOFgYXfkGOgBM^cGALm_vSVHtiIH]`PnGQxP_^uRVkdbjlxmXUaELLMwk``j^cejXhvw`Sc]Q[JLinYIkWjHyLbvDB`zyLomFtKBqBMFlDnlCBOxGXzGs^lMFzs_nd_FgWFocWRjKeVuXqsCmcsbLitcIyvqIhSI`iFgvZUjFQvOHqoPHqlKkwty]]gdb^RNVjlMGTNI]vmfRKgiRvHXdoHJZnlOejYwtF^vjYxb[yLrBUZomPbKplueCh`usMTibpEJnc]JpJqvSXZRZrZqmDjKssCthbmrIZ_UfzhEAAcEjkPGKAm]w^FuQuadiIbDDJyuwnsmmlDGFJImmRMbzVMvY[VEG[SrBAfaRMNhHzkKZyoGlnqWvzEixC]YofRZMFF^baE_ZAeBZgOSTdwwCwxdOkkPTTYlNtjdKSQRW`DDwaBpJSh^PCIayMpTPFhLPO`xfIATe`]ttghjvGsAlOxraDLF[xH[WsgSs^TRTtkaPgR`GYJNg[BQiWcxvtgXC]BAUNCc`iXUNQRXtaT`yMkXeCm]vVW[MS_meRKsWbixCgDYELDU`HZpmdWR[agJSxIum`mcVJW_mVxzUXkSCvEsizAusYpu`fF_If_Y[zEyCRzOYSGIwqQBDBD`mh]CT]yHQZritSrCrGpWAl]aRgyiUD[Hbygn]BeaSryzXPeyNKwLh`ALvlh_QmMykGufEaqPDIn^FuWUjHt^vuZfbaQxBCEmAlQVyuhTwdWMiqqQKafS[oL[z^za_NBEB]UdBVp[aCzRCYtkZhrZ[`wFvyXhnxicSLLNPolKmXptXbdPGqW`"
"eawY_FXYmmFRjkMiOLYCkOhHHL^XnYgGSChfODjJXTYzS_KhUvAvYToWsoQTbAeUFzzgCbZK]OfXqipdAqDsXliyORm[mWbXiDuT`^empYJiX`KzhSlxmxTQAYkpzXGhNimn]E^MQWbT_PrlbFEq[wGaC_Etw]XS][inX]do_gVVAlkbyO_SCqnmgM[`ZSUtJACpIyJyPRnakaQn]uiyFQfgwGKCDVMSLzjvYAE`qm_zLvfJmwXfvEkWIMsyvYuFE_basDudYO^GgkK_sRYpgaHOSmMHFdYrZ_GWEcts]C]qNQ]h[CGQo_XSOFrm^opfgIdn[QXaSeDRBCQk[faSWJjczlUnAj_iAbdTstzmiuxK[FRMpHSrN^P^YBojweacFVQ[YxragodM_Ht[HytXg[^RGuqFQqEUVFFornrPlLZuoRuuuPofhKYunISDBVx_GqZBEcUGdgBd^pAwKBCZhKgLBi[oOPpVh[[spg[_D_xtKf^DEQUvEYgmqDutSUWf`NpV`EVVbuefjSqCxkKz`RPniCllaLiIRTxqI_AvzTKoBC^MsvbKXJzv`bqBddBiv^XztMoMJD]MHmf_`tee_kITBQahyRVSs^f_TfHVAThnoEpK]EkWEWnPgrvO`LlGtfcOAALN^O^W[`MMl]NewWkVs]ksppcauIaK]t^sqtWpGei]Cj_]wPG_fHi]eZhoteDlKSzUHEacT`lrCWmGDExtUB_UKANWLbv]awRkkKUsj_KFWr[OgblohKWIneJGqJXmdPQFECWtkEJqrZySaykdqHoW`IxGANBaHYGaUKdLrEcAaRUfrarYnv][u`vFQs]CdjYxrmMoGYyCMVzrc[JhKHT]yfgoO_vzb`WrsWnFUDRUw[N_aE]pjps]VwyLFXab^z[hzIyMgESGpy`_YIOJCFPmguwXbzw]jrxza`YFQIf]mgWW[nxJW`XjiBEgCoFEWdFHc`rdqEmwePuOFaxnCnGK[OR`[lDDaGIaPnuwUSSn[EYHpbrRCc^vhUOS"
"IEAQtEJliqmCrioXzJsYfKbel^^fecZgeLNR]bAOVHljo[UHi^hKVt^uYfHafGDNTVM]N]uYJeC^ZspRJWwGuORkiTtqZZcOE^WZNsuMdOBlrzappXPKwTEE]ntdDyjBnZv]WEQ]j^fMatjsSAdBSEHKRUlEAd`PRjD``EZlHn[Yhr_dEBQCViijRlKBSLGyVuqqJSObsu`di]ZqIAzS]udATOntXYvSU[RwhmviB]oJFdPkLKL]nRp]v_VKZyNPOL^IVFQQBXorNf_JLOkqazqfrQzKgQDpwqUDNCpFL]ZD]yOQzMyLOWiOUrREFGekUzuXQ[vJYeeD`mIsCjK[Ssb`HGjVlkAFsjj^EqA`QpmemejcjYCVwJYXUPJl`Xie_LpDeoLjeK^Fxm]WHgTheJiRNFjbqnkXEAOZt_EdrfRzvV[cmnetDAzV]_p_EetrZFBw_C^cbjqysjWrfHoDIrTEAtxUi`_NmZapU^DkPCRUJA]mIytxpqiblxwfFItCKdMYJa^HDEnGazuAWKLkgbXwlyEWbXeHIPJaoEdc[mulDbBlDwPuwJVdLQgKG`fZfyRqTLRadPuhftKwSTfIjKyt`V^XZHTMOSXckvjkxUTtw`X]NInu^j[zXQyitH]wiYTbMXevKVrFYdpGvLyaRlcwdSfkCEMrxm^MvvAlIzSTduzhIbyw^VWhk`yultsGJAIfFdLzKMYSYOSjSyc[FPTbWqVxGzPGLKdlUc`WzmQxEO]MObnRXBtgaPOHpCENILNEjxyVCEG^CeRbzIi_MyiKQwKFTXawiDosBX^RdXOTvWewxgIgT`AK]ConntCe^JpzpYacfXuojVPtSOHgPudICeWzcj_kVeplbfH_GaAweMHnu_lVVd`gwLCazwtKYE`^ShHd`itg^qIOLhNzyAOMLuiv`bED[nWcPcoXIpgjtJFCoURitMYqSRXyLHAHodDDlgwFZbpLb^SldshKPWJNUaPyKqMBp`uPGCQHYMOSXNOoygUSZprYZpiLWyuc"
"UnVCdDeab`mDHrqHS[z_tbAipQUFl[RfxdrLLvBnluCnRcyasvyVHMVUHNUM]oRltNPGOD]hIhxuKMSnMp]v_]W]rEhhbOFlBpikRlgutZS_YZwABZPzpqwXqnPKWpGtcLsU_b]IbCEJsEYHgfHzGwLsRu^]]_DTYwMab][xoqNnsNyjrCjJytPh]MhyyfCyllvrraJLKLPiYADvofZouU_[wtFQnypKKoG]Crz]ayoiMbPIlKSLOEWMvy[tXLWE^dtbtxgxOvaqOVedHBo]zwkjPbwBBgacSHEgcbsMgOAIqWM`UrmWXtFasJRRBevbVx_HRyB`IbqkwcttUWhKuKLYB^nYPaLCSYntrFOWpMNpDUeQQYxexkgKmFFnKcQcVnX^ozBOHRUixVVht]IrdaFcmcGmA[LNwfZPxGQWj^fVxNWjYuh[YypiplMtPNgFHmSwGjmBr[uDZMoVkCc`^n^geaIUPXHq`OLv[diHgcyElPcxrJMeGcUOfRabYlZSYNtycZPZuXrK`bsQ`WYOvqjxtwPIDGjuIff_WdVXtnkikJjDZPCBTLXc^vEfKHWoixBtwUuvBAVHGAMkNLjnEiaxOCA[`jNlXwAnIpHEVGATqP]]iUYyYcjAWEftT[WRndkJAZcXEeYxhNHza_lKRPToMAXDKNkvki`M_S]PmAtnKmSIxgTpiLA_PvxzB_Mru^Mzhu[NBYQVad[hOG_uvM`hBwcV]NNoCth`SCLFoNnjIjS^SnYhwyQhRgyrs_NK]ShwuVnDSpHylxtNQsK[pGGuWrApEUMPzcNpCVcxAUUHrSvTmkaAUnUWMm`CUyzycrzJkDKvPN[[lyDEwhCToMvMNqcPpacBVW`zTmdjaflGTufOjGhwfKVwmvt[vFjeYFUZDlJDSnIosTLU`UZvFkMJCaqjOeaC]voQogjwddrBuItbiGVkPiSUIQALSUEIcZSx_uatf^S]sKPugPBIPpmncZI_f_dP`iZ[pwCn`]ElWp]yBF_DSvtmg_UFmBNH"
"tDxJloFkMNcmkBXmxK^azawdpgoQbxB_OxsSsTMNAorFSHCfHcHc_hHTHTUFzoJOME_mJEudxQaLjrQidJJzpZzbq_L_zHNLecsOcPCh_eOGweQCgqfKZnExwetVepCUaHyCFUhGRRol[tuv]SgWa[JFKfSrJRtpLmW]RAaGJ_sHyS]DjxtRJgoFyss[BqxzYzai^Xd]m`NBbBHrIPHLYbf_xQ_ZHGFTatjzmoaSUX`eYn^wlBRmimaqTBlMsJUfZu[iRHFTjpqBCAGJmciGpSeoStfkDTZKCtK_SpedwxCtv`nTvbZQ[OqHQfVYUQVVgyIqbqxxFKrPltmFSp]uIjoDDOgZLHbhh^Paf^TkSVEzr_mdTJ`oMwKFmTJ^lxcKcNeLZpmYqTjEMjuqsyCtMQSWakAaybY]oORIXtUDCCWcBaePhtz`RWxRptEL[PvNPxYlYyezrJtjv_KVQQhYw^sTqbzPbqbHhNldL_OuVa[MDcuVuFvKb^QmEEurzUXAbmChnbKlCnDWCfQOgeV]vXaKwz_HeXuAIZeRaNbTvJHo[ujZofKUzvFKLskoDCHfsCOdByiZCBalPmkCPFTyMaBMT[Fi]nQtB]vXgFYpxkSxVUuZIi_xiJYSABPatzxPlhjmqSWJh_AODrzafanPSKsEdXeqrwxnCLheHdpSUQKIxEwJPANZJAZ]riNulTpbDhLrJVZCkuBzkH_[arj^dCmKnYWvbAcEPtEzuaLjiFY^Gbi[Qmxs`EIFzcAgpttaWfrfVgQ_ouxUhgrh[QO^k`LkzARa_uoLrG[jDEoFhUkJKlRvJyUdfCgGguW[UkzuFm]bLHeWIqVoB`NjVX``L`pZUCIiUlxVqDYPjkBUyH]MRSfVIlzSZUuMgIrF]hHXtAxyGyzfVLbECdICXmaclevJMcbtwsQzFTQUgOJmTxOnpJDzohjH`fPE]puWXlYSQFxQrhk_WqYghiUC_bEPJJZZDLsrTmUb_v^jMnIHwKljNcXnrsphJJHg^MfptLdq"
"ImLeqkZxSyYkMoEWu_uUlVDib]SDzHNW[ecqBHWjrKjvQkrk]jjwwnApbHJxfPzQ^qAJ`[mIK^bgIYqfbaQYQ[henPtyYEbLYY[khOwPwomPYIYjSSDjjsMVT`iQaojFwWNjxGxSdA]]iXLeqJfGAsaLEWIagiHEgTCvjmMUlFiWkhvTDZWIcEDuh^sjplwSUXBEJjHXZmPmnJXo^MSybPHNLt^_zJWBTgqKTSyjqpRH^BNANkzVv[oBv^wNPdWHIleeJuTPflyAkJYP_mZWgXLCJrXndahkVtkdWDqgGQiqAllEgj^KAk`c[R]r]BLQPUQigHicj`H^LIGfWGuD^ktIKAvmToeO^`lXWAbobQvdBWSLqVHHn_[H^fLwPuvsAgWjKzFTOv[y`Jdda]TOHWlNwTsVVsSVzkBkv]fQossD]cUBcVmbhlduJnrHkxWdFyoHthEqryT^tJZk_Tm]`nSzXpljoUaq]Oy_oNIuRvkjYGBATGcWKtERfojDwUC^Y]BeKvwRLsLdDdbRnlVhlJfcKgsz^EywtMQvFXpYKRLSi[NBNYPLyUlOqYZ_CwnGtOkqSfMWvx^ZxTujZrmFonx_BFgxRVN_FZZkXjcLtPBrRR^^kELKiRGQMVQ``ZebQvsu[zCIJtv^wgJkcINMKCpnSJ_BYCg[fZbVYdCkCDApWcT`LaxWLXXNGFbRAjHMs^FnSzF]JHBzWkpyqsFPONCTPTBlPCTtcvkqZXNxRiOPLGiq]JrBxMe[l_zxUMHoIWzz`ZeIy`zmIJeKQnsrfiFVhsOzwGCjNq]xkNtzgfidVUmDZOSRlnudraVIBDsRFZXToqIOxgB[hD^tMtCsIlyZwtFJrlydmW_wxYPdFETFlZvnCvHBzJT[xXvkyWMhYiR`bwodZnEYti]xSGVpqbmNLABUsGhGXTldO`H^bMFubPuxYrBXrqvDTGvmPagWSUlTDfpiSzaIRtlGW[Ogj`]q`vUdVOB`ibFuJdUMLj`qVvCMKfsGYkdvsuoIRLTD"
"yAZjnrxnFC[AAf[esIZjVuHyyKBOpLvzWsVSYTiFBVPdvMZJPImdiEsldpiyuHe_nFIajGBQsljVOppfvmMmnn]Abo^zHHOnzWZTBjZgbXLHmWVAtSyVOrlzdIyucBIl^vVsVeLSaKVbVNLZ]GoIt`HeZksl_D`PZerA^eYimChgWXOumFvUcdWONwkl[o^eGHlWLwshGWqr[OB]xyCxeurAmQihwRNKCQQ^_b`NONMm_nyuOn]maALQkvZfJzKEdhYylyZW^jVheNwzRnZIpdBEzmdItEODIJrUAJPUFPqe^g^z[UVaBOIDWtmMpOdfHe]DEeQeJokMnWaSPei`ihCtBoxHWD_qfL^tJZupDlXM_vQASZwLaExZbdCPgJNFPuabthJpjswPpVF_xoxGTQbNDSaWfJu^SweViHfQH]hcAJlOwljUPywdyTpUcYEjbYxslkqqJhzqczNtaS[uZ`OTAAWURQhiYyAbMt]MusLczFBzGwoQyJPCKeslU]`tARYdyfhhpOzbUfUvyWNVWjOiyXbAlrXTScigXB[OHkKii`QxmyOJXdbKvzOFgDWbNBDgOaVmGUvzDAQrRDztJKvyG]BLIXUc[geiLc]CChbHujWocE]YDGE`mETbyt^fadWhVnQhHsR`UcRpTsTrKW`XEwHOuEJJwfUFAHNRMlwOnAX]BleAKoQJEycYfCmCjHciNMoMfCfY[RdE_fMJZIzIVLBB`ofJzt_gXiOpxYUIIsimV^uApddaHZTCKmgrXYbnxgpWXLBorz]]PgBtfUWQy_xVMeKl^khZHzXqFdYXbmQ[r`vGfxlGCRoYj[KmAPQGSDVQPKoBwziMO^^gjEGazaXqKyHeYfWRJIoKfr[IjKDccKLw_DwJjHlg`FQDJdQcWkEVjDNfYPVTNbnwiuc[ImCWHf`aXyGuMsFqMCzi^RsVSR_]HjiHNyCybq_gOZIsAjiF]rcle[KBpTBDP^XMTytDreL[COsju[QooBzrFWd^sxm]^cywlP_CpfOr"
"gHjVc[HVDMPO_aGxVVKusjxZQvyevOyJnHKygNvXpXZRTkXKIbgZ[TzxKQFg]][oNGXVIQH[nE[PgF`^nTb_iZHNoVIOuTxeDMPApAQ`zhO_h^K`IjAGNjXBPHUuzLsl^edsQ]xCC`BLcaQLcGKSqXTpOlhgV`oo]BCuSPfm`ELKWpgbDjWFhZhNPDClPog]lNib^WsT[YnmMJZYEGdgJiGDTpeOMu]kXiYIGEEhrDePfvBGedgJLATCGeNhleGavDU^kQRzBvpFDrYYo_dcNT_XVWC]rWwMwETca^VUSgHQLNyOMptMA^Sc_xHUNpIfzcIz_prQJPQaMOLIXnPB]jMYmkEdWBG]^QdEzkXydDlNlVQlzw[aHIQWkNnlOtIInCkxcDYDsDtf[LAVorWEgwWSAMUmwfjjvcZifSJv_XhEPev_MYxX]D]rgXxVjfGMTJkslRjhaPOSDVRp`^iSMrGcCNUnMsQkGuDmvijB^McUUbEtGc_BwslJO]GuPiyncwNDepMcLUtzisniSmhwiJBRdgMUompNplXGqqnSYl_rJadTLwvQqtmZwZycroZ`gKwgr]QueQY^Z[FcBcptxyseguW^K[eRzLydIgU[fWMdTw]zsApLdfjMEqAxGmjJfH_O][UOVNamOQkXWfKeZhT]m`SgWUsgDxMXO_Wa[wOYI[p`P`jH`_MZLljwHisBUujRESJsPeJ]YuUSkmWrelG_lnM]pZYmENDNk_wVqrvUjAQXWMURAWdnK`QGgDpMXYLrGbLR[ntLJBmwFYND[Dr[[ayKqjnCjxRpYQpADr`cjcA^_KaebIwUTXgBNHdlAr]Mkx^pdNAd^Pkr`ggII[aAwBdonvNncrFnmahlBUITqEgolxdrXwMo]Lp[EVkhtmliFyXHFXgFWVoSBZQKk[JbyEL`MuEyGXnCjDVyMkR``iFGW]ZeF`wYcKjBnrvLTizzbmdxDRyzIeBQzFIj[Zta[UZ`UULUpGLrjG_etanbxPyIZBhiLkuoY_NSDdjF"
"dJCYKbta^pzsFy^E[WvS^fdVRDQbRSpsriynNayrIHLVlBiZAIrgiWbxagGGQROLAGAXAjruYDJW[NePUraJ[IKGANaZrbrszVeARZRQxOfML]SCrmYkFlrw`Rz]QbS^szOCYPs^u`Ou^v_cEIGmN_iAZozgJthB[CcjAMOwXQWbmt]SLlNTRnCy[tBNVUM][]zUIfazbRKdyNNcJqhwnGkogBATNiBwJzGPtHKaKOmAcZANfYE_gQiUlLiHfMlsPOZyHxjBgHl_UjMNAnioe]PtQl_iZCuy]tkUzBFiiCr`_JABzGs_xNmzaiVZBcdMnC[zrNzhAFcejUGl]jywC[`gxUZTkOhZNSlH`CHNHL_rfHPLcdxSdhKS]oB]ngwlYIXXdyLZbkprwiAqFOkGRVqaIYflBe[OmZ]krbdX_ktrSWonLX_vq[CSqPLT[XxPUdJO_FNhHinexjpBqyqNw`YSWJucVPxGnd^jN`OFJrnHhYVH[J^]ajToKqsUKfRslgrTMFoQjkDEmQfyonIMm`DPi_Whuv_`vWAbBvUVMmqgPCuLuaX]FGPGnE^aXdpp__rueXnrnqaijXAgeLmT^B[oGunwXnxcxBHkKAa^n`jOabXWErwMrR]ECnkdq[AtxkiPPefPaxSMO_]Wsarll]jegHMptwFjVcrjGoVSbbuTKrnOOMytIIMOyfdTXpVesdrghnQDK`JJbW[oUxbgyLaMVdkKBulMQaAHb[wvjdyCgcKpzZOiggDeQECosIeYxRuuoMWGkD]xOSxIQVC[]^eo]Vq]YICay`FUhQBmCIycKGWxHu^^GJvZhGXYWuibAvyiNczTPYU`BeWzsV_e`[oeggatdWI[XcOHhk_Y^Jqlbug]AY^_Jw[[KEavuGktPtIYVPSIqFGWeCCmAEtKsdQ[HoxcJc_XYx]imNGkGViEpEqHCovpMaAfchQbSjyOajxgaUzmyID^vMeDxpaVgjKswqFEx`YWWZ^GGJbioJrD]^jWDNd]Jdbr_FkjOvpY"
"khGi^wnOsa^sHVZHiAjvvNnsgcBZrZtTYTpWNiaRqOQCDzXjRc`uF^oZTdBvuUWZgJZnaCtA[YFhxkphKSIOtyNv_WCegeW]rrWt`dJOi`[hXHdnUkcLHaiv_OKAOTpBdpGUwHaJtaT_a^XuZrRuoQjRhxAVyNZLffoRVOrMJZRbMKVBqPaXCuNSFpgyGLKfGIsuiF_KzElUyHE^bz_ihCJkYLacsFyPfERSllVYxJsUykIWHgyDyI]MtycHynFSfAMGgnxRdMa`HNq[yRohXFIyGrmnRyWnCSjvQ[t^gZmbwFfHIfAReOuEHavK_SNKlFvG_L^XQAxTbBBMfffyu]XEorIuySIH`XLAkBdZ]EAeQb^G`j`thPPPhrRhDdZysqoJqAeBMqnoNTCUOmPZPHvStjwWteCmZnTP_NvroZouCETBOKLcX_CvicuEpbMZBUZAmqBqXyhHYqecUfZqJCPQUpfAmfMNUthfSOXJVLYCUnQlQ]wmdrSKkYAVpeTP[m_IyQGNQ[xdMva`tkQJQlBUcpWn`bEnuBlcRLUiIuixos]FTJyzB^n^hQq^vW`ZB`nRYFpczdHg_oxvR[OuouAOoo_DryM_J^nThKgOUWAXVHztTvMIefG``]dallBWd]ABeOGnzaXucsgpekwmlV`rgnHiDhdfyAmfysjgFhIDOLwhpovApPfNwIUqHYkSxmvbFBqmBPgDNdr[sOlcqjV`ip_s`hxpG]PUboyJz`cbz`tzg`bUvLMaSBOIkzFYVHL]sDQbdQHn`NQetsHOlaEIBwuqpdbFXi]n]LzleEYAFkdPrsdKebHUrbThm_RyHswXEfQOzvzjAWMQbxksXqbHwusSyJaGCxshrnMTvefHfrLHWL_nZagCVPIkXwAmZcxZTdjJ__]qKZ]Fikvnemh]RpRcevyethDtlnmVPvEprvnDChBMIgvH_SH_Cc^MRlZLxxIRP^lX]yVobZOxXlAIXdP[UqNSacaJUrEjsvsJJEfawSxcU`pakQoTiixZ"
"PYqtXL[kU_F]sryTyj_BsfyBmAYDClBKxZukNlIksNNp[ThtSUyJwae[pHXnWhd^v_YwNFMaoIeYJaYS_laXCABsPQXekx]aeS`oOxEWIYVnvMQOiFOfGM[u^ahxhbqrgEnrBeDbgKOefNwDzIwfjqZYreRiPHiab_HqYZNEBgzvxpPQdex_fKHqfVzXSVS`]hCHFFeavrdDr^[CVAFfJJvWWRLCZplgTZZPmIK^zWcnbEzHvxCRQgVSmfkoubGGGxjXEKPNpbfmaHpRSEH]nZWhDH^HXWdYP]heVbDkip]iBclfqZ]p[eSgbCTxhIpZyN^EZLwkBXoyxhf[VrQyunxfJBCTOFD`zCjnFhLe]moDpUitWsbprr`LPruYIdvTOyvysViggIzuppSJwV_NiH^_^Nmn]tQsjbVnv^^tJKVXMUCFFzgGXG`rXZUHvsKcrGLNZ_OYsfE_lFSILhi]MJcB_VJVxRAn`FJofakNNmCBFRHLwOJhViecMvJnP^hm^yaMzWBHmywjaAZETxZiHTYr[ackXxykEP^HlvMaMCvWlvCME[RDzfK`jlYnRkpALfR]SSXlsFYBF`_WOS]ggEdTbBJnmvRFW`QkjJlCaL^am^dMuXCXST`ahIkROyGanSf`PngEa[GbDbDXxsUstnDAjDbY[nbExm[sDxGlyrrMEzdbhtRFlht_c``kFfFg_WdRGLTAmduCvqwkZzlb]]SQsvQ_XWgSlKihVS[`RTImrcldoQFu_KCDTYLupcWxcRlIOVG_PCLIK^BS_vcFaYMAfdBtPajixLK`rdUZKae]ztsqXsWuzRWKEl]eUMPiS]LzSuaJuAnkBmkK]WKpjgoAGHjFxRvJ]it^vX]TpWWWTjktJks_]n`KSAVnrExjsa_Drhbzi]tQYdHWWpvMYWhiIiLvCi]lYh^nLgC]JtqAtPmityWBIlQ]tpvIfxzAIrkFTLknZD_agptv]CDR[chWBtVltA^wrAlXCc`ngOjhmvMACMzdth``KUE[pytZ"
"acigDtt^QfLtAeFhUjgAtCJ[MMh_FxiVQGJNaxaTFGdCUgKjynPMYg``LrOLUXSwqct[sGWITunZGG`CxaR]YIr[dhUGRHliQF^DROFgEGt^tGYFRMNoFq_hlZrLlQrLJEIBclj[fXRPUa]PPoFtwcZapSsXuuoPhToxW]p_T[ZNYWsUv]lSNi[mJsRakdBcBl_smzeUdIATXwzDyXS_VGTPwjgcNmjpLpVGpDk]HxdThGVMtOcTQ^NuYPvCSBHqYLIbasNuhqWX]zbpUZjzKW`hc[i`HhJGpnu]yngAAhf[HpYYiGisSIfCg[zGKhnAgQcjEErbniy^kEanCPrByRnhHsq_o`qTufnqNiUeOGEdKHRbtMDuqlBV]ubMNTnNsedFRj`TvNmsMK^zaPecv^IdYUSHIWvTqk`_LQJgkzdyJISXbfyiLlxH[]sVNMmqr_VZxUz[vlvuK[QBu`]XNIEnXDjXoY_MmHemt[GQzOPABNQBcTSgQYeWRgaYfbg^NiHDp^I[ZMYzsElJmhp[FrwT^OMthfXblnTGNMELsNDAe^MuWgxmLYVloaaCvalbBXHRYD`kPNetTslOS]BhfWTYbRhgCPR]yNWv]TG]BwLks[ipndEktVAntrlspizvJWUeVTpJuLd[IhiqzqOgNkccMcCfbOhTwPa]GDZ[nupimaeLIuty[TiZolXmRKbg^Q_tGMkvWosSutqgyq]hVjtcbHhVfhIdTJK]ClEwKxbfV_uvlo[huQoXwSz_VPjfOzliTdMla^YHNGLdnZuthwYtWFGZFYohsANWmLbHbjHXshKpGWlvVpxSasSO^ngHikPMtoqbsmSyoBAivQSjgtSCDvpusTenCziIXxwR`JDd^y^_reTNxpXMJxzvi`I_wpTsQxTAJ]pj_EXRCOzXSF_skKoySd^[zChLzTVt[nyziiE`hs[`lLqR[IFJUi`BSO`CkFDve^sS`tB]jYekAyjnLWkrNM_mNwh[ICpbp_vNT_lbGdZvDoAU_lqUULiY"
"yGEdLbUhdCv]D[heJkemdjYvTvWMNdq]OMZIV`OmSAt_RCQIjK[DfVXfhsfxHpRfaMNN]oLVAQi`WCsGDlCImrtpo[]tRRKKbslpCvNWPieFiGRqmrXWnJtiTYZCRRDjG_x_RDDI_ftDMoHiJgH[toWYzC]eKExevyKIm`jnPrHQp]`RktfKdpSH``OaYgTbvZQpwaWHYgewOIJawTkRZnBFgDcNQIUd^aUWLkUK]^mwLVsWVieFvCHHXKItxIrPRAZinnVMzapvBEuRKLrb_PhYNyAb[NTg]fKwKdcTXGVMMFTQrilRPlCTxPaoUf_SO`ZjVAcPOGJSOMHfjkl]CXlcryuCoymbClxaubiIBAesAiCYJfunqYqnqrWG`QUbP^OakIODTAXHjrQsdeTroMSHnDjFcdakPRkLosY]gj]YZgGAFNvqw]nCKHnhZJkZhrYGFDlMqLEkuhGwizUhknn^hAhElD^OmymohuAJKeGQjSPZgkkJHCjtDvYEWh[nipxwvSPoyK^Fh]oVkNXASTHjOslmwn^JSbtVT^UvLw`dMaGDTJPZViNhDa]TBXzwVuuzXkD`WIKJffKfXddOjqMLHfZnycJ``hrgLDwYLtEM^DCYWqvYnlM_ijslQPbnzedyn_ZUhLCpmzhDdBXIhkmo[Tx]LKHvSRNs[EesWblbjWmiBoIPsYGOsp^[axI]DG^[mpd_X_lGzKEEbPXCsDtKzAebJSZYO_ctRwMKdrUhAOhkYtQ]l[KDdQnozxNMShlg]LQTOMZBjMKkStIQBxhoT`PAIXLfdHUS^BONcxN_BJUWGC]FpxLVAxSwvPKrAnXbrrMRfvhDkImJLtuGvGqgwzSaiRRTmV_Nmm`OqKZtPHfsKIlCUEwtxlfr_fdTdTQUd`XYTfPkhwWoU[W`aPyiZePAGhjKbFQwqQOelSijoLMAVr]EPDnTVcuzPWSgvFJbf[xEMloNaPtOXOfPKfu^RwaOVOUUACniBfVutYs^O`nGigbAkCQLfPjtiixW"
"[N_LMmtlVmnWOmJ[ajhUdxb^^VWfBLkuHoROf^ZpGzgj]oW^`ZIzQCqQxxhqVNjKBsP[[upCPrNdQrAjztGbCleEyBRhqcZsPhe_wCeeFjYoMzFwxr]HIIQNL]YcHMgXUUoKWLCqxwBLPlCMkslwlHTWXT[eC]tDA[sKvamoggKLPBvkVWWUvdMnyscMZDyLpssRJGwQFxHHVXjOgF`H`rgiHbuM[NjIQqGs^ENNyL_PTfd[symV_qfyMJJbLkqsNzr_RrkbsRJUSXV]yZT[UeZLIgTufCNVgBUnEJBNL]MgQfpohdlXUXtUXTIKWhNOwzgVAiuQRNPrXnu^r`MBQqAOIAWkArr^PxkQyZnyjFGICweYfTr[LyFyYNASJP[WlcZCd`XlOSokuFGDmRV^ACDzfS`Mng`LfjXZ]bpGwFMtJUETHw`ruwWi`[XRzKybncFKigaK]PbuqJpQNMFqGhRiTiuCK^GlHeGbOtfWVVeVsVfGEmFH[FQXrzl_zyIpnLDOBTbe[lShSrq`WEZUgzKAQrCfjrWW]aigRXEkL[hjndFzILHv`oJnBCxKRvucUvKmJKOzziILS[NglD^_kqiWueOZuZP`EVrYSpyxIvqfxbpRYNYWstF]VP`zaFPtgOFWNYN_bc`ACZZDZILqJLlwxaYxcqblRMYDzAtpQTF[vp]`XdDC]DdeRhGdzjYRIMbBE]LxUuHg]CkwXKN^ioJsnm_TnLQegImdu_TGsFII]xpYwJMpucUDjNRysKId]ATPHBsqGfdfAVpxFWdz[IlaY`KRCIAvz_zx]JpITzVPyiCTXUsaJJTOmrbrboxfhf]Wsvbgn]bj]qthP_UqVfijRNQlLXdQCax`swuF]hamKIEAiGhHTpGkJleTKuQVsQgwsTpSdPUhFw`JIQDEk]xNZNWHSbvbXKttIHURU^D[YeMM[KtrLfWTY]fxhACTmQuCfiAJinVytmAdJusWBkL`^aCf]XdUkiIkKamKkIXQXZfCnnHfSCEU`FfUhRKA"
"]JEsszTFMQboPFHWJ[dqMqXgDVeGpDrNPuPbdJoZBYB[QFZyOKMhTngmYp^UszXNjaALrpaRpWLYWLnQL^seqJt]O^JofZaCAnlkEOGZevswcNVMCNJVHnW`g^wUfmfVqIYc[CLxXYAuRxtBHwlYQLvNWLEbRlnrQ]GZpnmzBMyin]OjdvMiANFeMKLOyeffeIeoq[LUKmWENN[uwVxW_ErCWMeb^BXaSDer[sJjs_OqBXzkQSdhPG`RbYUNbmglz^uJpD]]nhgacMUENtQLXjRL^TQB_wHFr]q_^pvNpHtc[ziVDQAh]TPMig]n[xr`biGVqbXyDVQpjl^euyOvZdWJbRoIZ[VJCCtQczMicjYGC[^xIrsjvFxSwo_nO`cAHngrNCvm`TkKQHGoqAbct[HxvyfLhvTKqGvhqFvoxjJbUnCphkwOxAGhfiwOWjEezaUKLZxlEnfJR`cIwq_xzFNpzOBIJgYNf`coZsdNmLJJVwVGGoSaLGeuS_KxYpcXVX_mxnjDMQLovD[DsvCUTrC_uhuhfoxdloxXdISEOTvvqyPEHF^nND^VzrPRoWaNKJO]fOmkPF^HdDiMpa`RJVhRcYlsAQsnZhfpAcrkDxyAOWobFypBJEpxtN]ukzMkhvXtLNEibGYiRUBxRuRClPdqTDiOVpTxmPa[XhtllTcxWmg`L`cpiJyXSaaH`qSlnwBnb`dA`RYhf[aXJDqM^OdQJIrHlMuAqKvqemezzoIr`kgFzkWn_PMTkeYi]OTXYr`knWYqNjGllYlHBqH_Rui^tHLR]`o[HsKaPxzUeRaGbzLwwUN^PtabqNtPM`lJRRdaRLsEqntATcvuwPUISQLOSdjCDjHFaWoPVeflJgzaNORzSxKcrAGVOiqAHfo[xng_RddgIejOmV[kEXKux`nJzWwPeI`qKGQ[e[BTbwiWXH]SBFvku^Xz^`ugTBzEsEgFAGeVqSArXlsudO_NpUQcs[HuGgNuiS_DpNBzQUfb]aclhADF]mO^nDM`ykPJ"
"EGIEMpEWN`wjiGSsWNGdIXvDBPmB]BaF]^WX^_svgsxpnoFwtgb]I`wiMBL`RUk^VVNKkvUrVKwQ[GPNJqIpZEwxbhK]SSdSHZZXbuLySkHycgOGlyVMvERZPdCks`CbeIAgsgPeJ^jxyUvZDkUwIGLVIyRQINZKEWx^N[tSGqCYVU[vidgHKGY_xf_tUywD_sbWXMdrBqRDmAn^Vc`aSzyyZabvzxIVGVqt_QWDQJPH[KfkWzVvVl_udwrehrEhwxXJRerRJ]BqiqzWwLdymZkeRbQbZmWYFcK]mZmHEmsjhMmx[i]NFu^NnrkUQsWGclFg^e[lrqigrueU^eyEtLzhhPydiYrMVlGwGG[qNkWOMqVsNVqftSMXmYyNfqDZz]QGohOWKHqJokruGWSbLSD_iJiIMLWhwPj`PE`HZm]DcBxHT`[kroJNbF_aOxv[slRfBqKHjabGHkAtgwwEaShzBbK_qBbkaMqh^d`kEdgFzWpNDP_`nALyxBpdxKk`tVb_KMISWUWwfcAflw[b^CwaBrk]owmttHMdpQDrGLIZdOSVBKrAkJBqZA^Tg^[b^D_VWptd[Ru^xm_U[eFmJkFVKonjVZiirGKozeUWGGijwvDkoHMtUoTTYAMBhJNBdLfqbkVpPjgCBVYOkCvONoTCmrVzYSiwtgGlPMJElWdfIRRCcxQfOyopscDkUWXxAkJUPhy^tr]qKhu`SW[BDhusRshJYjVzgfYHwnuynlXxeLibKCzwvxCO]yoRU[HHq_B]Ud`]ORQYuhhybNgWDkhVs[oGhYRPUHFJLNb^^[gglqbRHTnMlnoTGIkWoqABDnb^mTgJscDBeCt^yUeEodTRpFmZmHfqgkmnjda[aRsKkS]YPMX[LvN[[QihaNXlxNVtQFnUnK^^kIb^cUJ`jKfk^RTZvstTtHN^wcIJYDgAYkjRjKFMDrBmbfWymUYvRKddvEbzLzGg_eSmbBkknhQCF[vQLKXEeljNGGVfdFADkVOxTdWkeiIdua^[IQnB"
"bCCnvEgfFHhgMRkMyxboh[vcFT`cv]TRyiSBOxuE`b[whPnOKSLNucGKqlGskcnVikdXlwjdtcuUHIfQogRBGlaFMRL^rBUwnFOxfV^NdbUShlChFpDDRmPxJ^L[lLmrOybCtieZJDX_WOY[O^XCxQfApRnIKD]Xp]DiKznXKeuNUlauFlLOojtyTaMcu]PGbAitLjPPjzrTVXgElsCYrOcGdFVemL]iVngUDgjW]w_osacvuIqBZqOTXcQGTf]^]JxblRBFJgwvjnNC]ZkeV[O`_Sw[VdCrZARjdAKcGlsnuTddUNduusIBuXCYVaj`eF]i]_ZOH^dc[fLjSGHzHwbsQoLxlNaBTJoquHqoHfmV_ZZmrjC[b`]JvqLjN_^uwbPRqUoLQTs]JLTqUZGBLF[nhfvEvGadIsYwKIv]vqKOhzzj]VVoXjrH]eLZkPJmOURnXfM^eGUvMFhgMbaklMgJapi^kDLbAZfivFjIRmVUvvWx_MIjYAChIMcxjmbvmukgCxQqc[ZRoftRaLqCsbWxai^mZ`LZcBLkvbwhsqadWTbifJalsuSOPEXcyEJuNrMdibeeuVzlvnbMGpGPz[bhiw[qKTjogfV_HxAD_vr^ubT]^KIxtvnQ_vlHo[agLEPYCEIpClDdDmGcySzsaqnjbOdCOwjQrj^DE[FSTuIsKMlpJJ[hFURaqoGaGBcSMDd`tstaMweqXBrNFnNAdlVL^OfeLnoGQsAiLdPdbHtYgjYndtSnbj[iXBfMPUZH^a`BJB_eZrw`npsBSRTiJ`zocdWSm[QIJtmZsAoJINdlrOqUNCHfKrWKNsvOHiXgWdwFbZQhKavwOwTmBEkTeyS]QPfHsuB_LFeuOgcqxAQ[`wQ`IMCyDBYRZpSoJpP_WkIjsnXH]gxBClTUcjiy`IDCSZTnhIlmDonppQZerbU[QWDz[aFaaqcEYXWTDTp_MPyw`_nYPTiajcCEWUTGCKLMMiXYuGa^mqNAYGZvngRMIBVpUSwjAiCDlkqHU_Jm";


