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

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver/gpio.h"

#include <cJSON.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#include <WiFiUdp.h>
#include "Syslog.h"

#include "SSD1306Wire.h"
#include <math.h>

// For OTA
#include <WebServer.h>
#include <Update.h>
#include "ota.h"
#define OTA_LED 5

#define TC74_ADDRESS 0x4D

#define USE_DISPLAY 0

// Syslog server connection info
//#define SYSLOG_SERVER "172.16.51.159"
#define SYSLOG_PORT 514

#define APP_NAME "Starfish"

/* The examples use WiFi configuration that you can set via 'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID     CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS     CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define deviceBasename  CONFIG_ESP_DEVICE_NAME

#define SPP_TAG "STARFISH_SPP"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "Starfish"

#define LEDC_HS_TIMER             LEDC_TIMER_0
#define LEDC_HS_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO           (18)
#define LEDC_HS_CH0_CHANNEL     LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO           (19)
#define LEDC_HS_CH1_CHANNEL     LEDC_CHANNEL_1

#define LEDC_LS_TIMER             LEDC_TIMER_1
#define LEDC_LS_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO           (4)
#define LEDC_LS_CH2_CHANNEL     LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO           (5)
#define LEDC_LS_CH3_CHANNEL     LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM           (4)
#define LEDC_TEST_DUTY           (4000)
#define LEDC_TEST_FADE_TIME     (3000)

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define ADC_SAMPLE_INTERVAL 60000

// VintLabs Module PWM GPIO Assignments
#define PWM0 12
#define PWM1 13
#define PWM2 14
#define PWM3 15
#define PWM4 16
#define PWM5 17
#define PWM6 18
#define PWM7 19

void scan1();
static void parseJson(cJSON *root);
void setLeds(int ch, int val);
void adjustLeds(int ch, int val);
void pulseLeds(int ch);

uint8_t readTC74();

