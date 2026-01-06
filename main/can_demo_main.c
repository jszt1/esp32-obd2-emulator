#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include <inttypes.h>

#include "CAN.h"
#include "CAN_config.h"

#include "obd.h"

#include <string.h>
#include "http_server.h"

#include <dirent.h>
#include "fs.h"

#ifndef CONFIG_ESPCAN
#error for this demo you must enable and configure ESPCan in menuconfig
#endif

#ifdef CONFIG_CAN_SPEED_100KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_100KBPS
#endif

#ifdef CONFIG_CAN_SPEED_125KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_125KBPS
#endif

#ifdef CONFIG_CAN_SPEED_250KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_250KBPS
#endif

#ifdef CONFIG_CAN_SPEED_500KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_500KBPS
#endif

#ifdef CONFIG_CAN_SPEED_800KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_800KBPS
#endif

#ifdef CONFIG_CAN_SPEED_1000KBPS
#define CONFIG_SELECTED_CAN_SPEED CAN_SPEED_1000KBPS
#endif

#ifdef CONFIG_CAN_SPEED_USER_KBPS
#define CONFIG_SELECTED_CAN_SPEED CONFIG_CAN_SPEED_USER_KBPS_VAL /* per menuconfig */
#endif

CAN_device_t CAN_cfg = {
	.speed = CONFIG_SELECTED_CAN_SPEED,		 // CAN Node baudrade
	.tx_pin_id = CONFIG_ESP_CAN_TXD_PIN_NUM, // CAN TX pin example menuconfig GPIO_NUM_5
	.rx_pin_id = CONFIG_ESP_CAN_RXD_PIN_NUM, // CAN RX pin example menuconfig GPIO_NUM_35 ( Olimex )
	.rx_queue = NULL,						 // FreeRTOS queue for RX frames
};

// Queue for CAN multi-frame packets
uint8_t can_flow_queue[5][8];

unsigned int vehicle_speed = 0;
float vehicle_rpm = 0;
float vehicle_throttle = 0;
float vehicle_coolant = 90;
float vehicle_fuel_level = 100;
char vehicle_vin[17] = "ESP32OBD2EMULATOR";

static EventGroupHandle_t wifi_event_group;

#define WIFI_SSID "ESP32-OBD2"
#define WIFI_PASS "88888888"

// Debug mode - set to 1 to enable detailed CAN frame logging
// To enable: change to 1, rebuild (idf.py build), flash, and monitor serial output
#define DEBUG_MODE 0

#if DEBUG_MODE
#define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

CAN_frame_t createOBDResponse(unsigned int mode, unsigned int pid)
{
	CAN_frame_t response;

	response.MsgID = 0x7E8; // Standard OBD2 ECU Response ID
	response.FIR.B.DLC = 8;
	response.FIR.B.FF = CAN_frame_std;
	response.FIR.B.RTR = CAN_no_RTR;
	// Length will be set by the caller based on data size
	response.data.u8[0] = 2; // Default length (Mode + PID)
	response.data.u8[1] = 0x40 + mode; // Mode (+ 0x40)
	response.data.u8[2] = pid; // PID
	// Initialize rest to padding
	memset(&response.data.u8[3], 0x00, 5); // 0x00 is standard padding

	return response;
}

int sendOBDResponse(CAN_frame_t *response)
{
	int success = CAN_write_frame(response);

	DEBUG_PRINT("TX CAN Frame:\n");
	DEBUG_PRINT("  MsgID: 0x%03" PRIx32 "\n", response->MsgID);
	DEBUG_PRINT("  DLC: %d, RTR: %d, FF: %d\n", response->FIR.B.DLC, response->FIR.B.RTR, response->FIR.B.FF);
	DEBUG_PRINT("  Data: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			   response->data.u8[0], response->data.u8[1], response->data.u8[2], response->data.u8[3],
			   response->data.u8[4], response->data.u8[5], response->data.u8[6], response->data.u8[7]);
	DEBUG_PRINT("  Status: %s\n\n", (success == 0) ? "OK" : "FAIL");
	
	return success;
}

void respondToOBD1(uint8_t pid)
{
	DEBUG_PRINT("Building Mode 1 response for PID 0x%02x\n", pid);

	CAN_frame_t response = createOBDResponse(1, pid);
	unsigned int A=0, B=0, C=0, D=0;
	int data_len = 0;

	switch (pid)
	{
		case 0x00: // Supported PIDs
			data_len = 4;
			response.data.u8[3] = 0x00; // Data byte 1
			response.data.u8[4] = 0x18; // Data byte 2
			response.data.u8[5] = 0x80; // Data byte 3
			response.data.u8[6] = 0x00; // Data byte 4
			break;
		case 0x0C: // RPM
			data_len = obdRevConvert_0C(vehicle_rpm, &A, &B, &C, &D);
			response.data.u8[3] = (uint8_t)A;
			response.data.u8[4] = (uint8_t)B;
			break;
		case 0x0D: // Speed
			data_len = obdRevConvert_0D(vehicle_speed, &A, &B, &C, &D);
			response.data.u8[3] = (uint8_t)A;
			break;
		case 0x11: // Throttle position
			data_len = obdRevConvert_11(vehicle_throttle, &A, &B, &C, &D);
			response.data.u8[3] = (uint8_t)A;
			break;
		case 0x05: // Coolant temperature
			data_len = obdRevConvert_05(vehicle_coolant, &A, &B, &C, &D);
			response.data.u8[3] = (uint8_t)A;
			break;
		case 0x2F: // Fuel level
			data_len = obdRevConvert_2F(vehicle_fuel_level, &A, &B, &C, &D);
			response.data.u8[3] = (uint8_t)A;
			break;
	}

	if (data_len > 0) {
		response.data.u8[0] = 2 + data_len; // Mode + PID + Data bytes
		sendOBDResponse(&response);
	} else {
		DEBUG_PRINT("Unsupported PID 0x%02x or conversion failed\n", pid);
	}
}

void respondToOBD9(uint8_t pid)
{
	DEBUG_PRINT("Building Mode 9 response for PID 0x%02x\n", pid);

	CAN_frame_t response = createOBDResponse(9, pid);

	switch (pid)
	{
		case 0x00: // Supported PIDs
			response.data.u8[0] = 6; // Mode + PID + 4 bytes
			response.data.u8[3] = 0x40; // Data byte 1
			response.data.u8[4] = 0x00; // Data byte 2
			response.data.u8[5] = 0x00; // Data byte 3
			response.data.u8[6] = 0x00; // Data byte 4
			sendOBDResponse(&response);
			break;
		case 0x02: // Vehicle Identification Number (VIN)
			// Initiate multi-frame message packet
			response.data.u8[0] = 0x10; // FF (First Frame, ISO_15765-2)
			response.data.u8[1] = 0x14; // Length (20 bytes)
			response.data.u8[2] = 0x49; // Mode (+ 0x40)
			response.data.u8[3] = 0x02; // PID
			response.data.u8[4] = 0x01; // Data byte 1
			response.data.u8[5] = vehicle_vin[0]; // Data byte 2
			response.data.u8[6] = vehicle_vin[1]; // Data byte 3
			response.data.u8[7] = vehicle_vin[2]; // Data byte 4
			sendOBDResponse(&response);

			// Clear flow control queue
			// memset(can_flow_queue, 0, 40);

			// Fill flow control queue
			// Part 1
			can_flow_queue[0][0] = 0x21; // CF (Consecutive Frame, ISO_15765-2), sequence number
			can_flow_queue[0][1] = vehicle_vin[3]; // Data byte 1
			can_flow_queue[0][2] = vehicle_vin[4]; // Data byte 2
			can_flow_queue[0][3] = vehicle_vin[5]; // Data byte 3
			can_flow_queue[0][4] = vehicle_vin[6]; // Data byte 4
			can_flow_queue[0][5] = vehicle_vin[7]; // Data byte 5
			can_flow_queue[0][6] = vehicle_vin[8]; // Data byte 6
			can_flow_queue[0][7] = vehicle_vin[9]; // Data byte 7
			// Part 2
			can_flow_queue[1][0] = 0x22; // CF
			can_flow_queue[1][1] = vehicle_vin[10]; // Data byte 1
			can_flow_queue[1][2] = vehicle_vin[11]; // Data byte 2
			can_flow_queue[1][3] = vehicle_vin[12]; // Data byte 3
			can_flow_queue[1][4] = vehicle_vin[13]; // Data byte 4
			can_flow_queue[1][5] = vehicle_vin[14]; // Data byte 5
			can_flow_queue[1][6] = vehicle_vin[15]; // Data byte 6
			can_flow_queue[1][7] = vehicle_vin[16]; // Data byte 7

			break;
	}
}

void task_CAN(void *pvParameters)
{
	(void)pvParameters;

	//frame buffer
	CAN_frame_t __RX_frame;

	//create CAN RX Queue BEFORE CAN_init
	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));

	//start CAN Module
	CAN_init();
	printf("CAN initialized...\n");

	// DEBUG: Send test speed frame at startup
	vehicle_speed = 85; // Set test speed to 85 km/h
	CAN_frame_t test_frame = createOBDResponse(1, 0x0D); // Speed PID
	test_frame.data.u8[0] = 3; // Data length (Mode + PID + 1 byte value)
	test_frame.data.u8[3] = 85; // Speed value
	CAN_write_frame(&test_frame);
	DEBUG_PRINT("Sent test speed frame: 85 km/h\n");

	// Track time for periodic diagnostics
	TickType_t last_diagnostic_time = xTaskGetTickCount();

	while (1)
	{
		//receive next CAN frame from queue
		if (xQueueReceive(CAN_cfg.rx_queue, &__RX_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
		{
			printf("RX ID: 0x%03" PRIx32 " Data: %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   __RX_frame.MsgID,
				   __RX_frame.data.u8[0], __RX_frame.data.u8[1], __RX_frame.data.u8[2], __RX_frame.data.u8[3],
				   __RX_frame.data.u8[4], __RX_frame.data.u8[5], __RX_frame.data.u8[6], __RX_frame.data.u8[7]);

			DEBUG_PRINT("\nRX CAN Frame:\n");
			DEBUG_PRINT("  MsgID: 0x%08" PRIx32 "\n", __RX_frame.MsgID);
			DEBUG_PRINT("  DLC: %d, RTR: %d, FF: %d\n", __RX_frame.FIR.B.DLC, __RX_frame.FIR.B.RTR, __RX_frame.FIR.B.FF);
			DEBUG_PRINT("  Data: %02x %02x %02x %02x %02x %02x %02x %02x\n",
					   __RX_frame.data.u8[0], __RX_frame.data.u8[1], __RX_frame.data.u8[2], __RX_frame.data.u8[3],
					   __RX_frame.data.u8[4], __RX_frame.data.u8[5], __RX_frame.data.u8[6], __RX_frame.data.u8[7]);

			// Check if frame is OBD query
			if (__RX_frame.MsgID == 0x7df) {
				DEBUG_PRINT("  Type: OBD QUERY\n");
				DEBUG_PRINT("  Mode: 0x%02x, PID: 0x%02x\n\n", __RX_frame.data.u8[1], __RX_frame.data.u8[2]);

				switch (__RX_frame.data.u8[1]) { // Mode
					case 1: // Show current data
						respondToOBD1(__RX_frame.data.u8[2]);
						break;
					case 9: // Vehicle information
						respondToOBD9(__RX_frame.data.u8[2]);
					break;
				default:
					DEBUG_PRINT("  Unsupported mode: 0x%02x\n\n", __RX_frame.data.u8[1]);
				}
			} else if (__RX_frame.MsgID == 0x7e0) { // Check if frame is addressed to the ECU (us)
				DEBUG_PRINT("  Type: ECU MSG\n\n");				if (__RX_frame.data.u8[0] == 0x30) { // Flow control frame (continue)
					CAN_frame_t response = createOBDResponse(0, 0);

					for (int i = 0; i < 5; i++) {
						if (can_flow_queue[i][0] == 0) { continue; }

						for (int j = 0; j < 8; j++) {
							response.data.u8[j] = can_flow_queue[i][j];
						}
						sendOBDResponse(&response);
					}

					// Clear flow control queue
					memset(can_flow_queue, 0, 40);
				}
			}
		}
	}
}

const char *get_filename_ext(const char *filename)
{
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

char * get_type_for_filename_ext(char *filename)
{
	char *ext = get_filename_ext(filename);
	if (strcmp(ext, "html") == 0) {
		return "text/html";
	} else if (strcmp(ext, "css") == 0) {
		return "text/css";
	} else if (strcmp(ext, "js") == 0) {
		return "text/javascript";
	}

	return NULL;
}

static void cb_GET_root(http_context_t http_ctx, void *ctx)
{
	const char *html = 
		"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
		"<title>ESP32 OBD-II Emulator</title><style>"
		"body{background:#222;color:#fff;font:16px sans-serif;margin:0;padding:20px}h1,h3{text-align:center;margin:10px}"
		".row{display:flex;justify-content:space-around;flex-wrap:wrap;margin:20px 0}.col{flex:1;min-width:250px;margin:15px;background:#333;padding:20px;border-radius:10px}"
		"h1{font-size:3em;margin:0}h3{font-size:1.2em;color:#aaa;margin:10px 0}.slidecontainer{margin:20px 0}"
		".slider{width:100%;height:10px;border-radius:5px;background:#555;outline:none;border:none;cursor:pointer}"
		".slider::-webkit-slider-thumb{-webkit-appearance:none;width:25px;height:25px;border-radius:50%;background:#0ae;cursor:pointer}"
		".slider::-moz-range-thumb{width:25px;height:25px;border-radius:50%;background:#0ae;cursor:pointer;border:none}"
		".info{background:#2a2a2a;padding:15px;border-radius:8px;margin:10px 15px;border-left:4px solid #0ae}"
		".label{color:#aaa;font-weight:bold;display:inline-block;min-width:120px}"
		"</style></head><body>"
		"<h3>🚗 ESP32 OBD-II EMULATOR</h3>"
		"<div style='max-width:800px;margin:0 auto'>"
		"<div class='info'><span class='label'>Status:</span> ✅ Running</div>"
		"<div class='info'><span class='label'>CAN RX:</span> GPIO 43</div>"
		"<div class='info'><span class='label'>CAN TX:</span> GPIO 44</div>"
		"<div class='info'><span class='label'>CAN Speed:</span> 500 kbps</div>"
		"<div class='info'><span class='label'>VIN:</span> ESP32OBD2EMULATOR</div>"
		"</div>"
		"<div class='row'>"
		"<div class='col'><h1 id='current-speed'>0</h1><h3>SPEED (km/h)</h3>"
		"<div class='slidecontainer'><input type='range' min='0' max='255' value='0' class='slider' id='speed'></div></div>"
		"<div class='col'><h1 id='current-rpm'>0</h1><h3>RPM</h3>"
		"<div class='slidecontainer'><input type='range' min='0' max='16654' value='0' class='slider' id='rpm'></div></div>"
		"<div class='col'><h1 id='current-throttle'>0</h1><h3>THROTTLE (%)</h3>"
		"<div class='slidecontainer'><input type='range' min='0' max='100' value='0' class='slider' id='throttle'></div></div>"
		"<div class='col'><h1 id='current-coolant'>90</h1><h3>COOLANT (°C)</h3>"
		"<div class='slidecontainer'><input type='range' min='-40' max='215' value='90' class='slider' id='coolant'></div></div>"
		"<div class='col'><h1 id='current-fuel'>100</h1><h3>FUEL (%)</h3>"
		"<div class='slidecontainer'><input type='range' min='0' max='100' value='100' class='slider' id='fuel'></div></div>"
		"</div><script>"
		"function update(n,v){var x=new XMLHttpRequest();x.open('PATCH','/api/vehicle',true);"
		"x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');x.send('name='+n+'&value='+v)}"
		"function link(s,o,n){var slider=document.getElementById(s),output=document.getElementById(o),timer;"
		"output.innerHTML=slider.value;slider.oninput=function(){"
		"output.innerHTML=this.value;clearTimeout(timer);timer=setTimeout(function(){update(n,slider.value)},100)}}"
		"link('speed','current-speed','speed');link('rpm','current-rpm','rpm');link('throttle','current-throttle','throttle');link('coolant','current-coolant','coolant');link('fuel','current-fuel','fuel');"
		"</script></body></html>";
	
	http_response_begin(http_ctx, 200, "text/html", strlen(html));
	http_buffer_t http_response = { .data = html };
	http_response_write(http_ctx, &http_response);
	http_response_end(http_ctx);
}

static void cb_GET_file(http_context_t http_ctx, void *ctx)
{
	char *file = malloc(FILE_MAX_SIZE + 1);
	size_t response_size;
	ESP_ERROR_CHECK(readFile((char*)ctx, file, &response_size));

	char *content_type = get_type_for_filename_ext(ctx);

	http_response_begin(http_ctx, 200, content_type, response_size);
	http_buffer_t http_file = {.data = file};
	http_response_write(http_ctx, &http_file);
	free(file);
	http_response_end(http_ctx);
}

static void cb_PATCH_vehicle(http_context_t http_ctx, void* ctx)
{
	const char *name = http_request_get_arg_value(http_ctx, "name");
	const char *value = http_request_get_arg_value(http_ctx, "value");
	unsigned int code = 200;

	if (name != NULL && value != NULL) {
		printf("Received %s = %s\n", name, value);

		if (strcmp(name, "speed") == 0) {
			vehicle_speed = strtol(value, NULL, 10);
		} else if (strcmp(name, "rpm") == 0) {
			vehicle_rpm = strtol(value, NULL, 10);
		} else if (strcmp(name, "throttle") == 0) {
			vehicle_throttle = strtof(value, NULL);
		} else if (strcmp(name, "coolant") == 0) {
			vehicle_coolant = strtof(value, NULL);
		} else if (strcmp(name, "fuel") == 0) {
			vehicle_fuel_level = strtof(value, NULL);
		} else if (strcmp(name, "vin") == 0) {
			strncpy(vehicle_vin, value, 17);
		}
	} else {
		printf("Invalid data received !\n");
		code = 400;
	}
	
	http_response_begin(http_ctx, code, "text/plain", HTTP_RESPONSE_SIZE_UNKNOWN);
	http_buffer_t http_response = { .data = "", .data_is_persistent = true };
	http_response_write(http_ctx, &http_response);
	http_response_end(http_ctx);
}

void wifi_init_softap()
{
	wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
		.ap = {
			.ssid = WIFI_SSID,
			.ssid_len = strlen(WIFI_SSID),
			.password = WIFI_PASS,
			.max_connection = 2,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK},
	};
	if (strlen(WIFI_PASS) == 0)
	{
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	
	ESP_ERROR_CHECK(esp_wifi_start());

	// Reduce WiFi TX power to prevent brownout
	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // 11 dBm (default is 20 dBm/80)

	printf("wifi_init_softap finished.SSID:%s password:%s\n", WIFI_SSID, WIFI_PASS);
}

void app_main()
{
	// wait for IDF logs to end
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	printf("CAN RXD PIN NUM: %d\n", CONFIG_ESP_CAN_RXD_PIN_NUM);
	printf("CAN TXD PIN NUM: %d\n", CONFIG_ESP_CAN_TXD_PIN_NUM);
	printf("CAN SPEED      : %d KBit/s\n", CONFIG_SELECTED_CAN_SPEED);
#ifdef CONFIG_CAN_SPEED_USER_KBPS
	printf("kBit/s setting was done by User\n");
#endif

	///////////////// NVS - Initialize FIRST before any tasks

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	///////////////// WIFI	

	printf("Initializing WIFI...\n");
	wifi_init_softap();

	///////////////// HTTP

	printf("Initializing HTTP server...\n");

	http_server_t server;
	http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();

	ESP_ERROR_CHECK(http_server_start(&http_options, &server));
	ESP_ERROR_CHECK(http_register_handler(server, "/", HTTP_GET, HTTP_HANDLE_RESPONSE, &cb_GET_root, NULL));
	// Full web UI requires FAT filesystem - see instructions above
	// ESP_ERROR_CHECK(http_register_handler(server, "/", HTTP_GET, HTTP_HANDLE_RESPONSE, &cb_GET_file, "/spiflash/index.html"));
	// ESP_ERROR_CHECK(http_register_handler(server, "/main.css", HTTP_GET, HTTP_HANDLE_RESPONSE, &cb_GET_file, "/spiflash/main.css"));
	// ESP_ERROR_CHECK(http_register_handler(server, "/main.js", HTTP_GET, HTTP_HANDLE_RESPONSE, &cb_GET_file, "/spiflash/main.js"));
	ESP_ERROR_CHECK(http_register_form_handler(server, "/api/vehicle", HTTP_PATCH, HTTP_HANDLE_RESPONSE, &cb_PATCH_vehicle, NULL));

	////////////////// FAT - Disabled (requires partition table reflash)
	// Close monitor first, then run: idf.py -p /dev/ttyACM0 flash
	/*
	esp_vfs_fat_mount_config_t mountConfig;
	wl_handle_t m_wl_handle;
	mountConfig.max_files = 4;
	mountConfig.format_if_mount_failed = false;
	ESP_ERROR_CHECK(esp_vfs_fat_spiflash_mount("/spiflash", "storage", &mountConfig, &m_wl_handle));
	printf("FAT filesystem mounted successfully\n");
	ESP_ERROR_CHECK(dumpDir("/spiflash"));
	*/

	printf("\n========================================\n");
	printf("ESP32-S3 OBD-II Emulator Ready!\n");
	printf("WiFi AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
	printf("Web UI: http://192.168.4.1\n");
	printf("CAN: RX=GPIO%d TX=GPIO%d @ %d kbps\n", 
		CAN_cfg.rx_pin_id, CAN_cfg.tx_pin_id, CAN_cfg.speed);
	printf("========================================\n\n");

	//Create CAN receive task - START LAST after all initialization
	// Increased stack size to 4096 bytes for debug logging
	xTaskCreate(&task_CAN, "CAN", 4096, NULL, 5, NULL);
}
