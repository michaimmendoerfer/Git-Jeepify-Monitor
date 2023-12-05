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
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);

int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();

bool   isSwitch(int Type);
bool   isSensor(int Type);

void   PrintMAC(const uint8_t * mac_addr);
unsigned int rainbow(byte value);
uint16_t rainbowColor(uint8_t spectrum);


void InitModule() {
  /*preferences.begin("JeepifyPeers", false);
    preferences.putBool("Initialized", true);
  preferences.end();

  preferences.begin("JeepifyInit", false);
    preferences.putBool("Debug", true);
    preferences.putBool("SleepMode", false);

    preferences.putInt("Null-0", 3134);
    preferences.putFloat("Sens-0", 0.066);
    preferences.putInt("Null-1", 3134);
    preferences.putFloat("Sens-1", 0.066);
    preferences.putInt("Null-2", 3134);
    preferences.putFloat("Sens-2", 0.066);
    preferences.putInt("Null-3", 3134);
    preferences.putFloat("Sens-3", 0.066);

    preferences.putInt("Vin", 200);
  preferences.end();
  */
  preferences.begin("JeepifyInit", true);
  Debug     = preferences.getBool("Debug", true);
  SleepMode = preferences.getBool("SleepMode", false);
  
  strcpy(S[0].Name, NAME_SENSOR_0);
  S[0].Type     = 1;
  S[0].IOPort   = 0;
  S[0].NullWert = preferences.getInt("Null-0", 3134);
  S[0].VperAmp  = preferences.getFloat("Sens-0", 0.066);

  strcpy(S[1].Name, NAME_SENSOR_1);
  S[1].Type     = 1;
  S[1].IOPort   = 1;
  S[1].NullWert = preferences.getInt("Null-1", 3134);
  S[1].VperAmp  = preferences.getFloat("Sens-1", 0.066);

  strcpy(S[2].Name, NAME_SENSOR_2);
  S[2].Type     = 1;
  S[2].IOPort   = 2;
  S[2].NullWert = preferences.getInt("Null-2", 3150);
  S[2].VperAmp  = preferences.getFloat("Sens-2", 0.066);

  strcpy(S[3].Name, NAME_SENSOR_3);
  S[3].Type     = 1;
  S[3].IOPort   = 3;
  S[3].NullWert = preferences.getInt("Null-3", 3150);
  S[3].VperAmp  = preferences.getFloat("Sens-3", 0.066);

  strcpy(S[4].Name, NAME_SENSOR_4);
  S[4].Type     = 2;
  S[4].IOPort   = PIN_VOLTAGE;
  S[4].Vin      = preferences.getInt("Vin", 200);
  S[4].VperAmp  = 1;

  preferences.end();

  Serial.println("InitModule() fertig...");
}
void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  char Buf[10] = {}; char BufNr[5] = {}; char BufB[5] = {}; String BufS;

  PeerCount = 0;

  for (int Pi=0; Pi< MAX_PEERS; Pi++) {
    if (P[Pi].Type > 0) {
      PeerCount++;
      //P.Type...
      sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Type);
      if (preferences.getInt(Buf, 0) != P[Pi].Type) preferences.putInt(Buf, P[Pi].Type);
      
      //P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = P[Pi].Name;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      if (preferences.getString(Buf, "") != BufS) preferences.putString(Buf, BufS);
      
      //P.BroadcastAddress
      strcpy(Buf, "MAC-"); strcat(Buf, BufB); strcat (Buf, BufNr);
      preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      }
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

