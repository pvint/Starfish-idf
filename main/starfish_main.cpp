/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "Arduino.h"
#include "fauxmoESP.h"
#include "driver/ledc.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* The examples use WiFi configuration that you can set via 'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID	  CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS	  CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "starfish";

static int s_retry_num = 0;

#define LEDC_HS_TIMER		  LEDC_TIMER_0
#define LEDC_HS_MODE		   LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO	   (18)
#define LEDC_HS_CH0_CHANNEL	LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO	   (19)
#define LEDC_HS_CH1_CHANNEL	LEDC_CHANNEL_1

#define LEDC_LS_TIMER		  LEDC_TIMER_1
#define LEDC_LS_MODE		   LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO	   (4)
#define LEDC_LS_CH2_CHANNEL	LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO	   (5)
#define LEDC_LS_CH3_CHANNEL	LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM	   (4)
#define LEDC_TEST_DUTY		 (4000)
#define LEDC_TEST_FADE_TIME	(3000)

ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel[8];

char ssid[32];
char pwd[32];

int wifiTimeout = 20;
WiFiServer server(8080);
bool wifiConnected = false;
bool hasCredentials = false;
bool connStatusChanged = false;

fauxmoESP fauxmo;

// VintLabs Module PWM GPIO Assignments
#define PWM0 12
#define PWM1 13
#define PWM2 14
#define PWM3 15
#define PWM4 16
#define PWM5 17
#define PWM6 18
#define PWM7 19

// Default PWM properties
int freq = 5000;
byte ledChannel = 0;
int ledValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte resolution = 12;
byte ledDelay = 1;
int pwmFadeTime = 2000;


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG, "got ip:%s",
				 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

	// Enable fauxmo
	ESP_LOGI(TAG,"Enable fauxmo");
	fauxmo.enable(true);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		{
			if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
				s_retry_num++;
				ESP_LOGI(TAG,"retry to connect to the AP");
			}
			ESP_LOGI(TAG,"connect to the AP fail\n");
			break;
		}
	default:
		break;
	}
	return ESP_OK;
}

void wifi_init_sta()
{
	s_wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
		.sta = {
			{.ssid = EXAMPLE_ESP_WIFI_SSID},
			{.password = EXAMPLE_ESP_WIFI_PASS}
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");
	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
			 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void connectWifi()
{
	// Connect using arduino libs
	ESP_LOGI(TAG, "Connecting to WiFi (%s)...", EXAMPLE_ESP_WIFI_SSID);
	WiFi.disconnect();

	WiFi.begin(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

	int i = 0;
	while (WiFi.status() != WL_CONNECTED)
	{
			delay(250);
			//Serial.print(".");
			i++;
			if ( i / 4 > wifiTimeout )
			{
					ESP_LOGE("Timeout connecting to %s\n", EXAMPLE_ESP_WIFI_SSID);
					return;
			}
			fauxmo.enable(true);
	}

}

extern "C" void app_main()
{
	// Init arduino crap
	initArduino();
	//pinMode(12, OUTPUT);
	//digitalWrite(12, HIGH);
	
	ledc_timer.duty_resolution = LEDC_TIMER_12_BIT; // resolution of PWM duty
	ledc_timer.freq_hz = 5000;					  // frequency of PWM signal
	ledc_timer.speed_mode = LEDC_HS_MODE;		   // timer mode
	ledc_timer.timer_num = LEDC_HS_TIMER;			// timer index
	
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	ledc_channel[0].channel = (ledc_channel_t)0;
	ledc_channel[0].duty = 0;
	ledc_channel[0].gpio_num = PWM0;
	ledc_channel[0].speed_mode = LEDC_HS_MODE;
	ledc_channel[0].hpoint = 0;
	ledc_channel[0].timer_sel = LEDC_HS_TIMER;


	// Set LED Controller with previously prepared configuration
	//for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[0]);
	//}

	// Initialize fade service.
	ledc_fade_func_install(0);


	// fauxmo testing
	ESP_LOGI(TAG, "Setting up fauxmoESP");
	fauxmo.createServer(true);
	fauxmo.setPort(80);

	char d[12];

	for (int i = 0; i < 8; i++)
	{
		ESP_LOGI(TAG, "Adding fauxmo device %c (%d)\n", i + 49, i + 49);
		sprintf(d, "Starfish %c", i + 49);
		fauxmo.addDevice(d);
	}

	fauxmo.onSetState([](unsigned char device_id, const char *device_name, bool state, unsigned char val)
	{
		unsigned int v = val << 4;
		ESP_LOGI(TAG, "device_id = %d\tval = %d\n", device_id,v);

		ledc_set_fade_with_time( ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel, v, pwmFadeTime);
		ledc_fade_start( ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel, LEDC_FADE_NO_WAIT);
	});
		


	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	//ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	//wifi_init_sta();
	connectWifi();
}
