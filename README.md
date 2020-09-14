# Starfish - Control PWM outputs via Bluetooth or WiFi on a Vintlabs ESP32 module (or other ESP32 device)
See https://github.com/pvint/Starfish-Android for the Android App

_Note: I've been using this for a couple years, and it started off as a Proff of Concept really, but I keep adding to it (sometimes late at night etc), and it has gotten to be a mess. I'm beginning refactoring now, so it should be a bit cleaner in the near future._

### Configure the project

```
make menuconfig
```

* Set serial port under Serial Flasher Options.

* Set Device Name, WiFi SSID and WiFi Password and Maximum retry under Example Configuration Options.
* Optionally set a syslog server also under Example Configuration

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Here is the console output when the station connects to AP successfully:
```
I (699) cpu_start: Pro cpu start user code
I (47) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (113) BTDM_INIT: BT controller compile version [d4658dd]
I (113) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (213) phy: phy_version: 4100, 6fa5e27, Jan 25 2019, 17:02:06, 0, 0
I (813) STARFISH_SPP: ESP_SPP_INIT_EVT
I (833) STARFISH_SPP: ESP_SPP_START_EVT
I (833) starfish: Setting up fauxmoESP
I (833) starfish: Adding fauxmo device 1 (49)
I (843) starfish: Adding fauxmo device 2 (50)
I (843) starfish: Adding fauxmo device 3 (51)
I (853) starfish: Adding fauxmo device 4 (52)
I (853) starfish: Adding fauxmo device 5 (53)
I (863) starfish: Adding fauxmo device 6 (54)
I (863) starfish: Adding fauxmo device 7 (55)
I (873) starfish: Adding fauxmo device 8 (56)
I (873) starfish: Connecting to WiFi (VintLabs)...
I (883) wifi: wifi driver task: 3ffdfa34, prio:23, stack:3584, core=0
I (883) wifi: wifi firmware version: 955b7af
I (893) wifi: config NVS flash: enabled
I (893) wifi: config nano formating: disabled
I (903) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (903) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (943) wifi: Init dynamic tx buffer num: 32
I (943) wifi: Init data frame dynamic rx buffer num: 32
I (943) wifi: Init management frame dynamic rx buffer num: 32
I (943) wifi: Init management short buffer num: 32
I (943) wifi: Init static rx buffer size: 1600
I (953) wifi: Init static rx buffer num: 10
I (953) wifi: Init dynamic rx buffer num: 32
I (973) wifi: mode : sta (cc:50:e3:be:97:10)
[D][WiFiGeneric.cpp:342] _eventCallback(): Event: 0 - WIFI_READY
[D][WiFiGeneric.cpp:342] _eventCallback(): Event: 2 - STA_START
I (1113) wifi: new:<11,0>, old:<1,0>, ap:<255,255>, sta:<11,0>, prof:1
I (2093) wifi: state: init -> auth (b0)
I (2103) wifi: state: auth -> assoc (0)
I (2113) wifi: state: assoc -> run (10)
I (3143) wifi: connected with VintLabs, channel 11, bssid = 44:d9:e7:8f:45:41
I (3203) wifi: pm start, type: 1

[D][WiFiGeneric.cpp:342] _eventCallback(): Event: 4 - STA_CONNECTED
I (5903) event: sta ip: 172.16.50.121, mask: 255.255.254.0, gw: 172.16.50.1
[D][WiFiGeneric.cpp:342] _eventCallback(): Event: 7 - STA_GOT_IP
[D][WiFiGeneric.cpp:385] _eventCallback(): STA IP: 172.16.50.121, MASK: 255.255.254.0, GW: 172.16.50.1
I (7103) starfish: Enabling fauxmo
I (7103) starfish: ******************** Task

```

