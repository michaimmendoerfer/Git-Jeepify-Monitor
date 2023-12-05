#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
//#include <FS.h>
#include <SPIFFS.h>
#include "TAMC_GT911.h"
#include <Adafruit_ADS1X15.h>
#include "../../renegade_members.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#include "NotoSansMonoSCB20.h"

#define NODE_NAME "Jeep_Monitor_V2"
#define NODE_TYPE SWITCH_4_WAY

#define VERSION   "V 0.01"

#define SWIPE_LEFT  10
#define SWIPE_RIGHT 11
#define SWIPE_DOWN  12
#define SWIPE_UP    13
#define LONG_PRESS   5
#define DBLCLICK     4
#define CLICK        3
#define HOLD         2
#define TOUCHED      1

#define LONG_PRESS_INTERVAL 300

struct struct_Sensor {
    char        Name[20];
    int         Type;      //1=Switch, 2=Amp, 3=Volt
    // Sensor
    bool        isADS;
    int         IOPort;
    float       NullWert;
    float       VperAmp;
    int         Vin;
    float       Value;
    float       OldValue;
    bool        Changed;
};
struct struct_Peer {
    char       Name[20] = {};
    u_int8_t   BroadcastAddress[6];
    uint32_t   TSLastSeen = 0;
    int        Type = 0;  // 
    struct_Sensor S[MAX_SENSOR]; 
} P[MAX_PEERS];

struct struct_Button {
  int x, y, w, h;
  int TxtColor, BGColor;
  char Name[20];
  bool Status;
};
struct Touch_struct {
  int x0, x1, y0, y1;
  uint32_t TSTouched;
  uint32_t TSReleased;
  int Gesture;
};
struct_Button Button[10] = {
  { 25, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Volt", false},
  { 95, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Amp",  false},
  {165, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "JSON", false},
  { 60, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "DBG",  false},
  {130, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "Pair", false},
  { 25,  25, 70, 50, TFT_WHITE, TFT_BLACK, "Restart", false},
  {145,  25, 70, 50, TFT_WHITE, TFT_BLACK, "Reset",   false},
  { 25, 100, 70, 50, TFT_WHITE, TFT_BLACK, "Pair",    false},
  {145, 100, 70, 50, TFT_WHITE, TFT_BLACK, "Eichen",  false}
};

void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void   OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void   InitModule();
void   SavePeers();
void   GetPeers();
void   ReportPeers();
void   RegisterPeers();
void   ClearPeers();
void   ClearInit();

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);

int    modeUp(); 
int    modeDown(); 
int    modeLeft(); 
int    modeRight(); 

void   SetMsgIndicator();
void   ScreenUpdate();
void   PushTFT();

void   DrawButton(int z);
bool   ButtonHit(int x, int y, int z);

int    ToggleSwitch (int zNr, int s);
void   ShowSwitch1(int zNr, int s);
void   ShowSwitch4(int zNr);

void   ShowAllValues();
void   ShowMenu();
void   ShowJSON(void);
void   ShowDevices(void);
void   ShowDevice(int zNr);
void   ShowAllText(int zNr);

int    RingMeter(float value, float vmin, float vmax, const char *units, const char *bez, byte scheme);
void   LinearMeter(int val, int x, int y, int w, int h, int g, int n, byte s);

int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();

bool   isSwitch(int Type);
bool   isSensor(int Type);
int    TouchQuarteer(void);

int    TouchRead();

void   PrintMAC(const uint8_t * mac_addr);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int rainbow(byte value);
uint16_t rainbowColor(uint8_t spectrum);

Touch_struct Touch;

uint32_t TSTouch = 0;

void setup() {
  InitModule();

}
loop() {
  if (millis() - TSTouch > TOUCH_INTERVAL) {
    if (TouchRead() > 1) {
      //S_MENU
      switch (Mode) {
        case S_MENU    : 
          switch (TouchQuarter()) {
            case 1: Mode = S_SENSOR1; break;
            case 2: Mode = S_SWITCH1; break;
            case 3: Mode = S_SWITCH4; break;
            case 4: Mode = S_SETTING; break;
          }
          break;
        case S_SWITCH1 : ToggleSwitch(1); break;
        case S_SWITCH4 : 
          switch (TouchQuarter()) {
            case 1: ToggleSwitch(0); break;
            case 2: ToggleSwitch(1); break;
            case 3: ToggleSwitch(2); break;
            case 4: ToggleSwitch(3); break;
          }
          break;
        case S_SENSOR1 : 
          switch(Touch.Gesture) {
            case SWIPE_LEFT:  break; //Sensor +
            case SWIPE_RIGHT: break; //Sensor -
            case SWIPE_UP:    break; //Menu
            case SWIPE_DOWN:  break; //Menu
            case LONG_PRESS:  break; //Change Peer
          }
          break;
        case S_SENSOR4 : 
          switch(Touch.Gesture) {
            case SWIPE_LEFT:  break; //Sensor oder Peer +
            case SWIPE_RIGHT: break; //Sensor oder Peer -
            case SWIPE_UP:    break; //Menu
            case SWIPE_DOWN:  break; //Menu
            case LONG_PRESS:  break; //Change Peer
          }
          break;
        case S_PEER    : ShowPeer();    break;
        case S_PEERS   : ShowPeers();   break;
        case S_JSON    : ShowJSON();    break;
        case S_SETTING : ShowSetting(); break;
      }
              if      (gesture == 1) modeUp();          //swipe up 
      else if (gesture == 2) modeDown();        //swipe down 
      else if (gesture == 3) modeLeft();        //swipe left
      else if (gesture == 4) modeRight();       //swipe right
      
      else if (mode == S_SW_0) ToggleSwitch(AktPDC, 0);
      else if (mode == S_SW_1) ToggleSwitch(AktPDC, 1);
      else if (mode == S_SW_2) ToggleSwitch(AktPDC, 2);
      else if (mode == S_SW_3) ToggleSwitch(AktPDC, 3);
      
      else if (mode == S_SW_ALL) {
             if (touchX<120 and touchY<120) ToggleSwitch(AktPDC, 0);
        else if (touchX>120 and touchY<120) ToggleSwitch(AktPDC, 1);
        else if (touchX<120 and touchY>120) ToggleSwitch(AktPDC, 2);
        else if (touchX>120 and touchY>120) ToggleSwitch(AktPDC, 3);
      }
      else if (mode == S_MENU) {
             if (touchX<120 and touchY<120) mode = S_BAT_EXT_A; 
        else if (touchX>120 and touchY<120) mode = S_SW_0;
        else if (touchX<120 and touchY>120) mode = S_SW_ALL;
        else if (touchX>120 and touchY>120) mode = S_TXT_ALL;
      }
      else if (mode == S_TXT_ALL) {
             if (ButtonHit(touchX, touchY, 0)) { mode = S_CALIB_V; EichenVolt(); }
        else if (ButtonHit(touchX, touchY, 1)) { ForceEichen(); }
        else if (ButtonHit(touchX, touchY, 2)) { mode = S_DBG_JSON; }
        else if (ButtonHit(touchX, touchY, 3)) { 
          ScreenChanged = true;
          oldMode = S_BAT_BAT_V; // for refresh
          Debug = !Debug; 
          preferences.begin("Jeepify", false);
          preferences.putBool("Debug", Debug);
          preferences.end();
        }
        else if (ButtonHit(touchX, touchY, 4)) { 
          if (gesture == 12) { ClearPeers(); }
          else {
            ScreenChanged = true;
            Pairing = !Pairing;
            if (Pairing) TimestampPair = millis();
            else         TimestampPair = 0;
          }
        }
      }
      else if (mode == S_CALIB_V) {
        AddVolt(CalcField(touchX, touchY));
      }
      else if (mode == S_DEVICE) {
             if (ButtonHit(touchX, touchY, 5)) { mode = S_CALIB_V; EichenVolt(); }//restart
        else if (ButtonHit(touchX, touchY, 6)) {                                  //Reset
          jsondata = ""; doc.clear();
  
          doc["Node"] = NAME_NODE; doc["Order"] = "Reset";

          serializeJson(doc, jsondata); esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 100);  
          Serial.print("Sending Ping:"); 
          Serial.println(jsondata);    
        } 
        else if (ButtonHit(touchX, touchY, 7)) { ForceEichen(); }//pair
        else if (ButtonHit(touchX, touchY, 8)) { ForceEichen(); }//Eichen 
      }
    touched = false;
    }
}
void InitModule() {
  preferences.begin("JeepifyInit", true);
  Debug     = preferences.getBool("Debug", true);
  SleepMode = preferences.getBool("SleepMode", false);
  preferences.end();

  TouchHW.begin();

  Serial.begin(74880);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  TFT.init();
  TFT.setRotation(Rotation);
  TFT.setSwapBytes(true);

  Serial.println("InitModule() fertig...");
}
void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  char Buf[10] = {}; char BufNr[5] = {}; String BufS;

  PeerCount = 0;

  for (int Pi=0; Pi< MAX_PEERS; Pi++) {
    if (P[Pi].Type > 0) {
      PeerCount++;
      //P.Type...
      sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Type);
      if (preferences.getInt(Buf, 0) != P[Pi].Type) preferences.putInt(Buf, P[Pi].Type);
      
      //P.BroadcastAddress
      strcpy(Buf, "MAC-"); strcat (Buf, BufNr);
      preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); PrintMAC(MAC);
      
      //P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = P[Pi].Name;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      if (preferences.getString(Buf, "") != BufS) preferences.putString(Buf, BufS);
    }
  }
  if (preferences.getInt("PeerCount") != PeerCount) preferences.putInt("PeerCount", PeerCount);

  preferences.end();
}
void GetPeers() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[10] = {}; char BufNr[5] = {}; String BufS;
  
  PeerCount = 0;
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    // Peer gefüllt?
    sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
    Serial.print("getInt("); Serial.print(Buf); Serial.print(" = "); Serial.print(preferences.getInt(Buf));
    if (preferences.getInt(Buf) > 0) {
      PeerCount++;
      
      // P.Type
      P[Pi].Type = preferences.getInt(Buf);
      
      // P.BroadcastAdress
      strcpy(Buf, "MAC-"); strcat (Buf, BufNr);
      preferences.getBytes(Buf, P[Pi].BroadcastAddress, 6);
      
      // P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = preferences.getString(Buf);
      strcpy(P[Pi].Name, BufS.c_str());

      P[Pi].TSLastSeen = millis();
      
      if (Debug) {
        Serial.print(Pi); Serial.print(": Type="); Serial.print(P[Pi].Type); 
        Serial.print(", Name="); Serial.print(P[Pi].Name);
        Serial.print(", MAC="); PrintMAC(P[Pi].BroadcastAddress);
      }
    }
  }
  preferences.end();
}
void ReportPeers() {
  TFT.fillScreen(TFT_BLACK);
  TFT.setCursor(0, 0, 2);
  TFT.setTextColor(TFT_WHITE,TFT_BLACK);  TFT.setTextSize(1);
  
  TFT.println("Peer-Report:");
  TFT.println();

  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    if (Debug) {
      Serial.println(P[Pi].Name);
      Serial.println(P[Pi].Type);
      Serial.print("MAC: "); PrintMAC(P[Pi].BroadcastAddress);
      Serial.println();
    }
    
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           P[Pi].BroadcastAddress[0], P[Pi].BroadcastAddress[1], P[Pi].BroadcastAddress[2], P[Pi].BroadcastAddress[3], P[Pi].BroadcastAddress[4], P[Pi].BroadcastAddress[5]);

    TFT.print(P[Pi].Name); TFT.print(" - Type:"); TFT.print(P[Pi].Type);
    TFT.print(" - MAC:"); TFT.println(macStr);
  }
  delay(5000);
}
void RegisterPeers() {
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  memset(&peerInfo, 0, sizeof(peerInfo));

  // Register BROADCAST
  for (int b=0; b<6; b++) peerInfo.peer_addr[b] = (uint8_t) broadcastAddressAll[b];
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      PrintMAC(peerInfo.peer_addr); Serial.println(": Failed to add peer");
    }
    else {
      Serial.print (" ("); PrintMAC(peerInfo.peer_addr);  Serial.println(") added...");
    }

  // Register Peers
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    if (P[Pi].Type > 0) {
      for (int b=0; b<6; b++) peerInfo.peer_addr[b] = (uint8_t) P[Pi].BroadcastAddress[b];
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          PrintMAC(peerInfo.peer_addr); Serial.println(": Failed to add peer");
        }
        else {
          Serial.print("Peer: "); Serial.print(P[Pi].Name); 
          Serial.print (" ("); PrintMAC(peerInfo.peer_addr); Serial.println(") added...");
        }
    }
  }
}
void ClearPeers() {
  preferences.begin("JeepifyPeers", false);
    preferences.clear();
    Serial.println("JeepifyPeers cleared...");
  preferences.end();
}
void ClearInit() {
  preferences.begin("JeepifyInit", false);
    preferences.clear();
    Serial.println("JeepifyInit cleared...");
  preferences.end();
}
void PushTFT() {
  if (ScreenChanged) {
    Serial.print("ScreenUpdate: ");
    Serial.println(UpdateCount);
    TFTBuffer.pushSprite(0, 0);
    ScreenChanged = false;
  }
}
void SetSleepMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    SleepMode = Mode;
    if (preferences.getBool("SleepMode", false) != SleepMode) preferences.putBool("SleepMode", SleepMode);
  preferences.end();
}
void SetDebugMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    Debug = Mode;
    if (preferences.getBool("Debug", false) != Debug) preferences.putBool("Debug", Debug);
  preferences.end();
}
bool isSwitch(int Type) {
  return ((Type == SWITCH_1_WAY) or (Type == SWITCH_2_WAY) or (Type == SWITCH_4_WAY) or (Type == SWITCH_8_WAY));      
}
bool isSensor(int Type) {
  return(Type == BATTERY_SENSOR);
}
int  TouchQuarter(void) {
  if ((Touch.x1<120) and (Touch.y1<120)) return 1;
  if ((Touch.x1>120) and (Touch.y1<120)) return 2;
  if ((Touch.x1<120) and (Touch.y1>120)) return 3;
  if ((Touch.x1>120) and (Touch.y1>120)) return 4;
  return 0;
}
int  TouchRead() {
  int TouchX, TouchY, Gesture;
  int ret = 0;

  bool TouchContact = TouchHW.getTouch(&TouchX, &TouchY, &Gesture);
  TouchX = 240-TouchX; 

  //frisch berührt
  if (TouchContact and !Touch.TSTouched) {       
    Touch.TSTouched = millis();
    Touch.x0 = TouchX;
    Touch.y0 = TouchY;
    Touch.Gesture = 0;
    Touch.TSReleased = 0;
    ret = TOUCHED;
  }
  //Finger bleibt drauf
  else if (TouchContact and Touch.TSTouched) {   
    Touch.x0 = TouchX;
    Touch.y0 = TouchY;
    Touch.Gesture = 0;
    Touch.TSReleased = 0;
    ret = HOLD;
  }
  //Release
  else if (!TouchContact and Touch.TSTouched) {  
    Touch.TSReleased = millis();
    Touch.x1 = TouchX;
    Touch.y1 = TouchY;
          
         if ((Touch.x1-Touch.x0) > 50)  { Touch.Gesture = SWIPE_LEFT;  ret = SWIPE_LEFT; }                      // swipe left
    else if ((Touch.x1-Touch.x0) < -50) { Touch.Gesture = SWIPE_RIGHT; ret = SWIPE_RIGHT; }                     // swipe right
    else if ((Touch.y1-Touch.y0) > 50)  { Touch.Gesture = SWIPE_DOWN;  ret = SWIPE_DOWN; }                      // swipe down
    else if ((Touch.y1-Touch.y0) < -50) { Touch.Gesture = SWIPE_UP;    ret = SWIPE_UP; }                        // swipe up
    else if ((Touch.TSReleased - Touch.TSTouched) > LONG_PRESS_INTERVAL) {                                      // longPress
      Touch.Gesture = LONG_PRESS;
      ret = LONG_PRESS;     
    }  
  }                                                                    
  //nicht berührt
  else (!TouchContact and !Touch.TSTouched) {  
    ret = 0;
    T.TSTouched  = 0;
    T.TSReleased = 0;
  }
}
void  PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}