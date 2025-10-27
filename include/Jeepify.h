#ifndef JEEPIFY_H
#define JEEPIFY_H
// Version 3.02

#include <Arduino.h>

#define SEND_CMD_MSG_TTL           2
#define SEND_CMD_MSG_HOLD          10
#define SEND_CMD_JSON_TTL          "TTL"
#define SEND_CMD_JSON_NODE         "NOD"
#define SEND_CMD_JSON_VERSION      "VER"
#define SEND_CMD_JSON_MODULE_TYPE  "TYP"
#define SEND_CMD_JSON_TS           "TS"
#define SEND_CMD_JSON_CONFIRM      "CON"
#define SEND_CMD_JSON_STATUS       "ST"
#define SEND_CMD_JSON_FROM         "SND"
#define SEND_CMD_JSON_TO           "RCV"
#define SEND_CMD_JSON_ORDER        "O"
#define SEND_CMD_JSON_VALUE        "V"
#define SEND_CMD_JSON_PEER_NAME    "PeerName"
#define SEND_CMD_JSON_PERIPH_NAME  "PeriphName"
#define SEND_CMD_JSON_PERIPH_POS   "PeriphPos"
#define SEND_CMD_JSON_PAIRING      "PR"

#define SEND_CMD_STAY_ALIVE        1
#define SEND_CMD_SLEEPMODE_ON      2
#define SEND_CMD_SLEEPMODE_OFF     3
#define SEND_CMD_SLEEPMODE_TOGGLE  4
#define SEND_CMD_DEBUGMODE_ON      5
#define SEND_CMD_DEBUGMODE_OFF     6
#define SEND_CMD_DEBUGMODE_TOGGLE  7
#define SEND_CMD_DEMOMODE_ON       8
#define SEND_CMD_DEMOMODE_OFF      9
#define SEND_CMD_DEMOMODE_TOGGLE  10
#define SEND_CMD_PAIRMODE_ON      11
#define SEND_CMD_PAIRMODE_OFF     12
#define SEND_CMD_PAIRMODE_TOGGLE  13
#define SEND_CMD_SWITCH_ON        14
#define SEND_CMD_SWITCH_OFF       15
#define SEND_CMD_SWITCH_TOGGLE    16
#define SEND_CMD_CURRENT_CALIB    30
#define SEND_CMD_VOLTAGE_CALIB    31
#define SEND_CMD_UPDATE_NAME      40
#define SEND_CMD_UPDATE_VIN       41
#define SEND_CMD_UPDATE_VPERAMP   42
#define SEND_CMD_UPDATE_NULLWERT  43
#define SEND_CMD_RESET            50
#define SEND_CMD_RESTART          51
#define SEND_CMD_PAIR_ME          60
#define SEND_CMD_YOU_ARE_PAIRED   61
#define SEND_CMD_DELETE_ME        70
#define SEND_CMD_CONFIRM_CURRENT  80
#define SEND_CMD_CONFIRM_VOLT     81
#define SEND_CMD_STATUS           82
#define SEND_CMD_SEND_STATE       83
#define SEND_CMD_RETURN_STATE     84
#define SEND_CMD_CONFIRM          85

#define MAX_PERIPHERALS 9
#define MAX_PEERS       10
#define MAX_STATUS      10
#define SCHWELLE        0.2
#define RECORDED_VALUES 10

#define JEEPIFY_SEND_MAX_TRIES 10

// Module-Types
#define SWITCH_1_WAY        1
#define SWITCH_2_WAY        2
#define SWITCH_4_WAY        4
#define SWITCH_8_WAY        8
#define PDC                 9
#define PDC_SENSOR_MIX      10
#define BATTERY_SENSOR      20
#define MONITOR_ROUND       30
#define MONITOR_BIG         31
#define MONITOR_360         32
#define MODULE_ALL          99
#define RELAY_REVERSED      -1
#define RELAY_NORMAL        1

// Sensor-Types
#define SENS_TYPE_SWITCH  101
#define SENS_TYPE_AMP     102
#define SENS_TYPE_VOLT    103
#define SENS_TYPE_SW_AMP  108   // Switch with amp
#define SENS_TYPE_LT      110   // latching relay
#define SENS_TYPE_LT_AMP  109   // latching relay with amp
#define SENS_TYPE_SENS    104
#define SENS_TYPE_EQUAL   105
#define SENS_TYPE_ALL     106
#define SENS_TYPE_COMBO   107
#define SENS_TYPE_SW_ALL  112
#define NOT_FOUND         199

// Intervals
#define MSGLIGHT_INTERVAL 300
#define PING_INTERVAL     1000
#define MSG_INTERVAL      1000
#define STATUS_INTERVAL   5000
#define LOGO_INTERVAL     3000
#define OFFLINE_INTERVAL  20000
#define PAIR_INTERVAL     30000
#define SLEEP_INTERVAL    5000

// Round-Monitor Touch
const int I2C_SDA  = 4;
const int I2C_SCL  = 5;
const int TP_INT   = 0;
const int TP_RST   = 1;
const int Rotation = 0;

// ESP32-3248S035 Touch
#define TOUCH_SDA  33
#define TOUCH_SCL  32
#define TOUCH_INT  21
#define TOUCH_RST 25
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 320

#define MULTI_SCREENS 4
#define MULTI_SCREEN_ROWS 2
#define MULTI_SCREEN_COLS 2
#define PERIPH_PER_SCREEN 4

extern uint8_t broadcastAddressAll[6];
#endif