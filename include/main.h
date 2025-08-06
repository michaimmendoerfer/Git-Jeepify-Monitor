#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "Jeepify.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "pref_manager.h"
#include "PeerClass.h"
#include "LinkedList.h"
#include <lvgl.h>
#include "Module.h"
#include UI_H_DIR
#include UI_EVENTS_H_DIR
#include <nvs_flash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

#define DEBUG1(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 0)) Serial.printf(__VA_ARGS__)
#define DEBUG2(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 1)) Serial.printf(__VA_ARGS__)
#define DEBUG3(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 2)) Serial.printf(__VA_ARGS__)

#define RECORDED_VALUES 10
struct struct_Graph {
  float    Value[RECORDED_VALUES][4];
  uint32_t TSValue[RECORDED_VALUES];
  int      Index;
};

void   PrintMAC(const uint8_t * mac_addr);

void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
#ifdef MODULE_MONITOR_360 
    void OnDataRecv(const esp_now_recv_info *info, const uint8_t* incomingData, int len);
#endif
#ifdef MODULE_MONITOR_240
    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
#endif

//void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );
//void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data );

void   SendPing(lv_timer_t * timer);
bool   ToggleSwitch(PeerClass *P, int PerNr);
bool   ToggleSwitch(PeriphClass *Periph);
void   SendCommand(PeerClass *P, int Cmd, String Value="");
void   SendPairingConfirm(PeerClass *Peer);

void   ShowMessageBox(const char * Titel, const char *Txt, int delay, int opa=255);

bool   ToggleSleepMode();
bool   ToggleDebugMode();
bool   TogglePairMode();

void   CalibVolt();
void   CalibAmp();
void   PrepareJSON();
void   PrintMAC(const uint8_t * mac_addr);
void   MacCharToByte(uint8_t *mac, char *MAC);
void   MacByteToChar(char *MAC, uint8_t *mac);
void   GarbageMessages(lv_timer_t * timer);

void   InitWebServer();

extern volatile uint32_t TSMsgRcv;
extern volatile uint32_t TSMsgSnd;
extern volatile uint32_t TSPair;

extern PeerClass Module;

extern int ActiveMultiScreen;
extern const char *_Version;
extern void ToggleWebServer();
extern bool WebServerActive;
#endif
