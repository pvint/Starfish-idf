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

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "cJSON.h"

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
static const char *SPP_TAG = "starfish-SPP";

#define SPP_TAG "STARFISH_SPP"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "Starfish"
static void parseJson(cJSON *root);


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

// Array to hold pin assignments
int pwmPins[] = { PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7 };

// Default PWM properties
int freq = 5000;
byte ledChannel = 0;
int ledValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte resolution = 12;
byte ledDelay = 1;
int pwmFadeTime = 2000;

// Bluetooth
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
	{
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        // Get MAC
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        char *d = (char*) malloc(strlen(DEVICE_NAME) + 4);
        sprintf(d, "%s %02hhX", DEVICE_NAME, mac[5]);
        esp_bt_dev_set_device_name(d);
        free(d);

        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
	}
    case ESP_SPP_DISCOVERY_COMP_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
	}
    case ESP_SPP_OPEN_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
	}
    case ESP_SPP_CLOSE_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
	}
    case ESP_SPP_START_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
	}
    case ESP_SPP_CL_INIT_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
	}
    case ESP_SPP_DATA_IND_EVT:
        {
	if (param->data_ind.len == 1 && *param->data_ind.data == '.')
          return;
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);

        char *rcv = (char *)calloc(param->data_ind.len + 1, 1);
        memcpy(rcv, param->data_ind.data, param->data_ind.len);
        ESP_LOGI(SPP_TAG,"%s\n", rcv);

        // Test to see if it is valid JSON
        cJSON *json = cJSON_Parse(rcv);
        // Causes crash if not valid JSON char *prt = cJSON_Print(json);
        if ( json != NULL )
          parseJson(json);
        free(rcv);
        break;
	}
    case ESP_SPP_CONG_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
	}
    case ESP_SPP_WRITE_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
	}

    case ESP_SPP_SRV_OPEN_EVT:
        {
	ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
        break;
	}
    default:
    	{
        break;
	}
    }
}


void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}





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

static void parseJson(cJSON *root)
{
	const cJSON *ch = NULL;
	const cJSON *dc = NULL;

	ch = cJSON_GetObjectItemCaseSensitive(root, "ch");
	dc = cJSON_GetObjectItemCaseSensitive(root, "dc");

	if (cJSON_IsNumber(ch) && cJSON_IsNumber(dc))
	{
		ESP_LOGI(TAG, "Got ch=%d & dc=%d\n", ch->valueint, dc->valueint);
		ledc_set_duty(LEDC_HS_MODE, (ledc_channel_t)ch->valueint, dc->valueint);
		ledc_update_duty(LEDC_HS_MODE,(ledc_channel_t)ch->valueint);
		//setPWM(ch->valueint, dc->valueint, 0);
	}
}

void pollFauxmo( void * parameter )
{


        ESP_LOGI(TAG,"******************** Task");

	while (1)
	{
		fauxmo.handle();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
     	
	vTaskDelete( NULL );

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
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "Enabling fauxmo");
	fauxmo.enable(true);

}

extern "C" void app_main()
{

	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	//ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	//wifi_init_sta();

	// init Bluetooth

        //    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }


    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
	
	// Init arduino crap
	initArduino();
	//Serial.begin(115200);
	//Serial.print("==========================");
	//pinMode(12, OUTPUT);
	//digitalWrite(12, HIGH);
	
	ledc_timer.duty_resolution = LEDC_TIMER_12_BIT; // resolution of PWM duty
	ledc_timer.freq_hz = 5000;					  // frequency of PWM signal
	ledc_timer.speed_mode = LEDC_HS_MODE;		   // timer mode
	ledc_timer.timer_num = LEDC_HS_TIMER;			// timer index
	
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);


	for (int i =0; i < 8; i++)
	{
		ledc_channel[i].channel = (ledc_channel_t) i;
		ledc_channel[i].duty = 0;
		ledc_channel[i].gpio_num = pwmPins[i];
		ledc_channel[i].speed_mode = LEDC_HS_MODE;
		ledc_channel[i].hpoint = 0;
		ledc_channel[i].timer_sel = LEDC_HS_TIMER;
		ledc_channel_config(&ledc_channel[i]);
	}


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
		ESP_LOGI(TAG, "device_id = %d\tstate = %d\tval = %d\tv = %d\n", device_id, state, val, v);

		if (state)
		{
			ledc_set_fade_with_time( ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel, v, pwmFadeTime);
			ledc_fade_start( ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel, LEDC_FADE_NO_WAIT);
		}
		else
		{
			// turn off
			ledc_set_duty(ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel, 0);
			ledc_update_duty(ledc_channel[device_id].speed_mode, ledc_channel[device_id].channel);
		}
	});
		

	connectWifi();



	// Create the fauxmo polling task
	xTaskCreate(
                    pollFauxmo,          /* Task function. */
                    "PollFauxmo",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */


	/// MAIN LOOP
	/*
	while (1)
	{
		fauxmo.handle();

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}*/
}
