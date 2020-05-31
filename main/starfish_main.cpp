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
#include "driver/i2c.h"
#include "starfish.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sys.h"

//#include <driver/adc.h>
//#include "esp_adc_cal.h"
#include "SSD1306Wire.h"  
#include <math.h>
#include "MS5xxx.h"
#define MS5607_ADDRESS 0x77
#define TC74_ADDRESS 0x4D

#define USE_DISPLAY 0

MS5xxx pressureSensor(&Wire);


// Set the reference pressure to the current pressure to get relative altitude changes
double pressure, temperature, p_ref, t_ref, altitude;


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

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1


#define ADC_SAMPLE_INTERVAL 2000

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

float meanPressure = 0;

// default device names
char deviceBasename[] = "Elephant";
char deviceName[8][20];

#define DEBUG_FAUXMO_VERBOSE_UDP true

//#define DEFAULT_VREF    1100 
//static esp_adc_cal_characteristics_t *adc_chars;
//static const adc_channel_t adc_channel = ADC_CHANNEL_0; // ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
//static const adc_atten_t adc_atten = ADC_ATTEN_DB_0;
//static const adc_unit_t adc_unit = ADC_UNIT_1;
SSD1306Wire  display(0x3c, 22, 23);

void scan1(){
Serial.println("Scanning I2C Addresses Channel 1");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  Wire.beginTransmission(i);
  uint8_t ec=Wire.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");

}


// Bluetooth
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

uint32_t spp_handle;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
	{
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        // Get MAC
	const uint8_t *mac = esp_bt_dev_get_address();
        char *n = (char*) malloc(strlen(DEVICE_NAME) + 6);
        sprintf(n, "%s %02hhX", DEVICE_NAME, (int)mac[5]);
        ESP_LOGI(SPP_TAG, "Init Bluetooth - Device name: %s\t MAC: %02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX", n,(int)mac[0],(int)mac[1],(int)mac[2],(int)mac[3],(int)mac[4],(int)mac[5]);
	free(n);

        char *d = (char*) malloc(strlen(DEVICE_NAME) + 4);
        sprintf(d, "%s %02hhX", DEVICE_NAME, mac[5]);
	ESP_LOGI(SPP_TAG, "BT Device name: %s", d);
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

	spp_handle = param->data_ind.handle; // save for use in spp_send
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
	ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, DEVICE_NAME) );

	ESP_LOGI(TAG, "wifi_init_sta finished.");
	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
			 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

static void parseJson(cJSON *root)
{
	// channel and duty cycle
	const cJSON *ch = NULL;
	const cJSON *dc = NULL;
	// amount to adjust value by
	const cJSON *adj = NULL;

	// query temperature
	const cJSON *t = NULL;
	// query channel
	const cJSON *q = NULL;
	// channel id and name
	const cJSON *cn = NULL;
	const cJSON *cName = NULL;


	ch = cJSON_GetObjectItemCaseSensitive(root, "ch");
	dc = cJSON_GetObjectItemCaseSensitive(root, "dc");
	adj = cJSON_GetObjectItemCaseSensitive(root, "adj");
	q = cJSON_GetObjectItemCaseSensitive(root, "q");
	// Temperature query
	t = cJSON_GetObjectItemCaseSensitive(root, "t");
	// cn is channel name for renaming device
	cn = cJSON_GetObjectItemCaseSensitive(root, "cn");
	cName = cJSON_GetObjectItemCaseSensitive(root, "cn");

	if (cJSON_IsNumber(t))
	{
		uint8_t temp = readTC74();

		ESP_LOGI(TAG, "Received query for temperature. Sending: %d", temp);
		//char j[40];
		char *j = (char*) malloc(sizeof(char) * 40);
		snprintf(j, 39, "{\"t\": %d}\r\n", temp);
		esp_spp_write(spp_handle, strlen(j), (uint8_t*)j);
		free(j);

		return;
	}

	if (cJSON_IsNumber(q))
	{
		// send value for the requested channel
		char j[40];
		char n[20];
		int d = (int) ledc_get_duty(LEDC_HS_MODE,(ledc_channel_t) q->valueint);
		snprintf(j, 39, "{\"ch\":%d,\"dc\":%d,\"cn\",\"%s\"}\r\n", q->valueint, d, deviceName[q->valueint]);
		ESP_LOGI(TAG, "Received query for channel %d. Sending: %s", q->valueint, j);
		esp_spp_write(spp_handle, strlen(j), (uint8_t*)j);

		return;
	}


	if (cJSON_IsNumber(ch) && cJSON_IsNumber(dc))
	{
		ESP_LOGI(TAG, "Got ch=%d & dc=%d\n", ch->valueint, dc->valueint);
		// ch = -1 means all channels
		// dc = -999 means pulse that channel
		if (dc->valueint == -999)
			pulseLeds(ch->valueint);
		else
			setLeds(ch->valueint, dc->valueint);
	}

	if (cJSON_IsNumber(adj) && cJSON_IsNumber(adj))
	{
		ESP_LOGI(TAG, "Got ch=%d & adj=%d\n", ch->valueint, adj->valueint);
		adjustLeds(ch->valueint, adj->valueint);
	}

	if (cJSON_IsNumber(cn) && cJSON_IsString(cName))
	{
		ESP_LOGI(TAG, "Got cn=%d & name=%s\n", ch->valueint, dc->valuestring);
	}
}

void pollADC( void *parameter)
{
	ESP_LOGI(TAG, "**** pollADC task");

	while (1)
	{
		// display gets corrupted randomly, reset it randomly
		/*
		if (rand() < (RAND_MAX / 20))
		{
			    display.init();

			            display.flipScreenVertically();
				            display.setFont(ArialMT_Plain_24);

					            display.setTextAlignment(TEXT_ALIGN_LEFT);

			ESP_LOGI(TAG, "Reset display");
		}
		*/

		double v = 0;
		for ( int i = 0; i < 32; i++ )
		{
			v += analogRead(36);
		}
		v /= 32;

		double vret = 7.83 * (-0.000000000000016 * pow(v,4) + 0.000000000118171 * pow(v,3)- 0.000000301211691 * pow(v,2)+ 0.001109019271794 * v+ 0.034143524634089);
		// hack for when 12v supply is disconnected
		if (vret < 2.5)
			vret = 0.0;

		char j[16];
		snprintf(j, 12, "Bat: %.1fV", vret);

		// Get board temperature
		Wire.beginTransmission(TC74_ADDRESS);
		Wire.write(0x00);
		Wire.requestFrom(TC74_ADDRESS, 1);
		if (Wire.available())
		{
			int t = Wire.read();
			char ti[32];
			snprintf(ti, 32, "Board temp: %d C", t); 
			ESP_LOGI(TAG, "%s", ti);
		}
		Wire.endTransmission();

		// also get barometric pressure
		
		pressureSensor.ReadProm();
		pressureSensor.Readout();
		temperature = pressureSensor.GetTemp() / 100.0f;
		pressure = pressureSensor.GetPres() / 1000.0f + 0.8; // adding a bit - it seems to consistantly low
		unsigned int calcCRC = pressureSensor.Calc_CRC4();
		unsigned int readCRC = pressureSensor.Read_CRC4();
		if ( calcCRC == readCRC && ( pressure > 0 ) && (temperature > -100) )
		{
			ESP_LOGI( TAG,"------"); //%s Pres: %.1f  Temp: %.1f  %d %d", j, pressure, temperature, calcCRC, readCRC);

			// The display gets corrupted sometimes - hoping this might help
			//vTaskDelay(1000 / portTICK_PERIOD_MS);

			// Determine pressure trend. If first sample set mean to current value
			// Use 3 hours as the period for the approx rolling average
			int n = 3600 * 3 / (ADC_SAMPLE_INTERVAL / 1000);

			char tendency = '-';
			float delta = 0.1;

			// We get spurious readings on occasion - discard it of it's out of range
			if ( (pressure > 90.0) && (pressure < 110))
			{
				if ( meanPressure == 0.0 )
					meanPressure = pressure * (float) n;
				else
				{
					meanPressure -= meanPressure / (float) n;
					meanPressure += pressure;
				}

				if ( (meanPressure / (float) n) - pressure > delta)
				tendency = '\\';

				if ( (meanPressure / (float) n) - pressure < -delta)
					tendency = '/';
			}
			ESP_LOGI(TAG, "Interval: %d  Mean: %f %c", ADC_SAMPLE_INTERVAL, meanPressure / (float) n, tendency);

			display.clear();
			display.setTextAlignment(TEXT_ALIGN_LEFT);
			display.setFont(ArialMT_Plain_16);
			display.drawString(1, 0, j);
			ESP_LOGI(TAG,"%s",j);
			snprintf( j, 12, "%.1f kPa %c", pressure, tendency );
			display.drawString(1, 20, j);
			ESP_LOGI(TAG,"%s",j);
// HACK for fucked up sensor
temperature = temperature * .63;
			snprintf( j, 10, "%.1fC", temperature );
			display.drawString(1, 40, j);
			ESP_LOGI(TAG,"%s",j);

			display.display();
		}
		else
		{
			//ESP_LOGI(TAG,"--");
			//display.drawString(1, 21, "--");
			//display.drawString(1, 42, "--");
		}
		vTaskDelay(ADC_SAMPLE_INTERVAL / portTICK_PERIOD_MS);
	}

	vTaskDelete( NULL );
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

uint8_t readTC74()
{
	Wire.beginTransmission(TC74_ADDRESS);
	Wire.write(0x00);
	Wire.requestFrom(TC74_ADDRESS, 1);
	uint8_t t = 0;

                if (Wire.available())
                {
                        t = Wire.read();
                        //char ti[32];
                        char *ti = (char*) malloc(sizeof(char) * 32);
                        snprintf(ti, 32, "Board temp: %d C", t);
                        ESP_LOGI(TAG, "%s", ti);
			free(ti);
                }
		Wire.endTransmission();
		return t;
}

void getBoardTemperature( void *parameter )
{
		ESP_LOGI(TAG, "Temperature task");
                // Get board temperature
		uint8_t t = readTC74();

		vTaskDelay(ADC_SAMPLE_INTERVAL / portTICK_PERIOD_MS);

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

// use Rec 709 to adjust duty cycle value to something our eyes agree with
int humanizeValue(int val)
{
	
	// assuming value is 0 - 2^resolution (ie: 0-4095 for 12 bit resolution)
	float v = (float) val / 4096.0;

	if ( v < 0.081 )
		v = v / 4.5;
	else
		v = pow((( v + 0.099) / 1.099), (1.0 / 0.45));

	ESP_LOGI(TAG, "Humanized %f (%f)  to %f (%f)\n", (float) val, (float) val / 4096.0, v * 4096, v);

	return (int) (v * 4096.0);
}

void setLeds(int ch, int val)
{
	val = humanizeValue(val);

	if ( ch == -1 )
	{
		for ( int i = 0; i < 8; i++ )
		{
			ledc_set_duty(LEDC_HS_MODE, (ledc_channel_t) i, val);
			ledc_update_duty(LEDC_HS_MODE,(ledc_channel_t) i);

		}
	}
	else
	{
			ledc_set_duty(LEDC_HS_MODE, (ledc_channel_t) ch, val);
			ledc_update_duty(LEDC_HS_MODE,(ledc_channel_t) ch);

			ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}

}

void adjustLeds(int ch, int val)
{
	int x;
	if ( ch == -1 )
	{
		for ( int i = 0; i < 8; i++ )
		{
			x = ledc_get_duty(LEDC_HS_MODE, (ledc_channel_t) i);
			if ((x + val) > 4095)
			{
				x = 4095 - val;
			}
			if ((x + val) < 0)
			{
				x = 0 + val;
			}

			ledc_set_duty(LEDC_HS_MODE, (ledc_channel_t) i, x + val);
			ledc_update_duty(LEDC_HS_MODE,(ledc_channel_t) i);

			ESP_LOGI(TAG, "Adjusted ch %d from %d to %d", i, x, x + val);
		}
	}
	else
	{
			x = ledc_get_duty(LEDC_HS_MODE, (ledc_channel_t) ch);

			if ((x + val) > 4095)
			{
				x = 4095;
				val = 0;
			}
			if ((x + val) < 0)
			{
				x = 0;
				val = 0;
			}

			ledc_set_duty(LEDC_HS_MODE, (ledc_channel_t) ch, x + val);
			ledc_update_duty(LEDC_HS_MODE,(ledc_channel_t) ch);

			ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
			ESP_LOGI(TAG, "Adjusted ch %d from %d to %d", ch, x, x + val);
	}
}

void pulseLeds(int ch)
{
	int fadeTime = 100;
	float maxLevel = 1.5;
	float minLevel = .1;

	ESP_LOGI(TAG, "Pulse channel %d", ch);
	int x;
	if ( ch == -1 )
	{
		for ( int i = 0; i < 8; i++ )
		{
			x = ledc_get_duty(LEDC_HS_MODE, (ledc_channel_t) i);

			if ((maxLevel * x) > 4095)
				maxLevel = 1;

			ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) i, maxLevel * x + 60, fadeTime / 5);
                        ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) i, LEDC_FADE_WAIT_DONE);

			ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) i, minLevel * x, fadeTime / 5);
                        ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) i, LEDC_FADE_WAIT_DONE);

			ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) i, x, fadeTime / 5);
			ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) i, LEDC_FADE_WAIT_DONE);

		}
		ESP_LOGI(TAG, "Done");
	}
	else
	{
		x = ledc_get_duty(LEDC_HS_MODE, (ledc_channel_t) ch);
		
		if ((maxLevel * x) > 4095)
			maxLevel = 1;

		for (int n = 0; n < 3; n++)
		{
			ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) ch, maxLevel * x, fadeTime);
			ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) ch, LEDC_FADE_WAIT_DONE);

			ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) ch, minLevel * x, fadeTime);
			ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) ch, LEDC_FADE_WAIT_DONE);
		}

		ledc_set_fade_with_time( LEDC_HS_MODE, (ledc_channel_t) ch, x, fadeTime);
		ledc_fade_start( LEDC_HS_MODE, (ledc_channel_t) ch, LEDC_FADE_WAIT_DONE);

		ESP_LOGI(TAG, "Done");
	}
}

extern "C" void app_main()
{

	// for fauxmo debugging
	Serial.begin(115200);

	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);



	//Configure ADC
	/*
	if (adc_unit == ADC_UNIT_1) {
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten((adc1_channel_t)adc_channel, adc_atten);
	} else {
	adc2_config_channel_atten((adc2_channel_t)adc_channel, adc_atten);
	}

	//Characterize ADC
	adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(adc_unit, adc_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	//print_char_val_type(val_type);
	*/

	//ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	//wifi_init_sta();

	// init Bluetooth

        //    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	//uint8_t mac[6];
	//esp_efuse_mac_get_default(mac);

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

	const uint8_t *mac = esp_bt_dev_get_address();
        char *n = (char*) malloc(strlen(DEVICE_NAME) + 6);
        sprintf(n, "%s %02hhX", DEVICE_NAME, (int)mac[5]);
        ESP_LOGI(SPP_TAG, "Init Bluetooth - Device name: %s", n);


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
	
	initArduino();
	Wire.setClock(100000);
	Wire.begin(21,22);

	scan1();
// FIXME Temperature test
                // Get board temperature
		ESP_LOGI(TAG, "Getting board temperature....");
                Wire.beginTransmission(TC74_ADDRESS);
                Wire.write(0x00);
                Wire.requestFrom(TC74_ADDRESS, 1);
                if (Wire.available())
                {
                        int t = Wire.read();
                        char ti[32];
                        snprintf(ti, 32, "Board temp: %d C", t);
                        ESP_LOGI(TAG, "%s", ti);
                }
                Wire.endTransmission();

		ESP_LOGI(TAG, "Done");



	// set up the initialization progress bar
	int progress = 0;
	int progressY = 1;


	if (USE_DISPLAY)
	{
	display.init();

	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_24);

	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.drawString(1, 12, "VintLabs\nStarfish");

	display.drawProgressBar(0, progressY, 120, 10, progress);

	display.display();

	// Set up MS5607 Pressure sensor
	pressureSensor.setI2Caddr(MS5607_ADDRESS);
	ESP_LOGI(TAG, "-=-=-=-=-=-=: %d", pressureSensor.connect());
	vTaskDelay(20 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "ZZZZZZ");

	  
	pressureSensor.ReadProm();
	pressureSensor.Readout();
	p_ref =  pressureSensor.GetPres();
	t_ref = pressureSensor.GetTemp() / 100.0f;

	ESP_LOGI(TAG, "==========   %f    %f ", p_ref, t_ref);

	progress += 10;
	display.drawProgressBar(0, progressY, 120, 10, progress);
	display.display();
	} // if USE_DISPLAY
	
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

	if(USE_DISPLAY)
	{
	progress += 10;
	display.drawProgressBar(0, progressY, 120, 10, progress);
	display.display();
	}

	// fauxmo testing
	ESP_LOGI(TAG, "Setting up fauxmoESP");
	fauxmo.createServer(true);
	fauxmo.setPort(80);

	if(USE_DISPLAY)
	{
	progress += 10;
	display.drawProgressBar(0, progressY, 120, 10, progress);
	display.display();
	}

	char d[16];

	for (int i = 0; i < 8; i++)
	{
		ESP_LOGI(TAG, "Adding fauxmo device %c (%d)", i + 49, i + 49);
		snprintf(d, 15, "%s %c", deviceBasename, i + 49);
		// TODO - get these from flash
		strcpy( deviceName[i], d);
		fauxmo.addDevice(d);

		if(USE_DISPLAY)
		{
		progress += 5;
		display.drawProgressBar(0, progressY, 120, 10, progress);
		display.display();
		}
	}

	fauxmo.onSetState([](unsigned char device_id, const char *device_name, bool state, unsigned char val)
	{
		unsigned int v = val << 4;
		ESP_LOGI(TAG, "fauxmo device_id = %d\tstate = %d\tval = %d\tv = %d\n", device_id, state, val, v);

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
		
	if(USE_DISPLAY)
	{
	progress += 10;
	display.drawProgressBar(0, progressY, 120, 10, progress);
	display.display();
	}

	connectWifi();

	if(USE_DISPLAY)
	{
	progress = 100;
	display.drawProgressBar(0, progressY, 120, 10, progress);
	display.display();
	}
	// Create the fauxmo polling task
	xTaskCreate(
                    pollFauxmo,          /* Task function. */
                    "PollFauxmo",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

	xTaskCreate(
                    getBoardTemperature,          /* Task function. */
                    "getBoardTemperature",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    2,                /* Priority of the task. */
                    NULL);            /* Task handle. */

//	xTaskCreate( pollADC,
	xTaskCreatePinnedToCore( pollADC,
		"PollADC",
		8192,
		NULL,
		0,
		NULL,
		1);



	/// MAIN LOOP
	/*
	while (1)
	{
		fauxmo.handle();

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}*/
}
