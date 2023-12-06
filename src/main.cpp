#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <SPIFFS.h>
#include "../../renegade_members.h"
#include "pix.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "CST816D.h"
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

#define MAX_PERIPHERALS 5
#define MAX_PEERS      10

// Module-Types
#define SWITCH_1_WAY        1
#define SWITCH_2_WAY        2
#define SWITCH_4_WAY        4
#define SWITCH_8_WAY        8
#define BATTERY_SENSOR      9
#define MONITOR_ROUND       10
#define MONITOR_BIG         11

// Sensor-Types
#define SENS_TYPE_SWITCH  1
#define SENS_TYPE_AMP     2
#define SENS_TYPE_VOLT    3
#define NOT_FOUND        99

// Screens
#define S_MENU            1
#define S_SENSOR1       101
#define S_SENSOR4       104
#define S_SWITCH1       111
#define S_SWITCH4       114
#define S_SWITCH8       118
#define S_JSON          110
#define S_PEER          120
#define S_PEERS         121
#define S_SETTING       130


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
    struct_Sensor S[MAX_PERIPHERALS]; 
};
struct struct_Button {
  int x, y, w, h;
  int TxtColor, BGColor;
  char Name[20];
  bool Status;
};
struct struct_Touch {
  uint16_t x0, x1, y0, y1;
  uint32_t TSTouched;
  uint32_t TSReleased;
  uint16_t Gesture;
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

void   ToggleSwitch (int Si);
void   ShowSwitch1(int);
void   ShowSwitch4();

void   ShowMenu();
void   ShowJSON(void);
void   ShowPeer(int);
void   ShowPeers();

int    RingMeter(float value, float vmin, float vmax, const char *units, const char *bez, byte scheme);
void   LinearMeter(int val, int x, int y, int w, int h, int g, int n, byte s);

int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();

bool   SensorChanged(int Start, int Stop=0);
bool   isPDC(int Type);
bool   isBat(int Type);
bool   isSwitch(int Type);
bool   isSensor(int Type);
int    NextSensor();
int    PrevSensor();
int    NextSwitch();
int    PrevSwitch();
int    NextBat();
int    PrevBat();
int    NextPDC();
int    PrevPDC();

int    TouchQuarter(void);
int    TouchRead();

void   PrintMAC(const uint8_t * mac_addr);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int rainbow(byte value);
uint16_t rainbowColor(uint8_t spectrum);

struct_Touch  Touch;
struct_Peer   P[MAX_PEERS];
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

Preferences preferences;

int ActivePeer   = -1;
int ActiveSens   = -1;
int PeerCount    =  0;
int Mode         =  0;
int OldMode      = 99;
int UpdateCount  =  0;

bool ScreenChanged = true;
bool AvailPDC = false;
bool AvailBat = false;
bool ReadyToPair = false;
bool Debug = true;
bool SleepMode = false;

float TempValue[8] = {0,0,0,0,0,0,0,0};

uint32_t TSTouch         = 0;
uint32_t TSPair          = 0;
uint32_t TSMsgStart      = 0;
uint32_t TSScreenRefresh = 0;

TFT_eSPI TFT               = TFT_eSPI();
TFT_eSprite TFTBuffer      = TFT_eSprite(&TFT);
TFT_eSprite TFTGaugeSwitch = TFT_eSprite(&TFT);

CST816D TouchHW(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

void setup() {
  Serial.begin(74880);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);    

  Serial.println("InitModule...");
  InitModule();
  Debug = true;

  Serial.println("GetPeers...");
  GetPeers();
  Serial.println("ReportPeers...");
  ReportPeers();
  Serial.println("RegisterPeers...");
  RegisterPeers();
  Serial.println("RegisterPeers fertig...");

  if (PeerCount == 0) { ReadyToPair = true; TSPair = millis(); }
  
  TSScreenRefresh = millis();
}
void loop() {
  if (millis() - TSTouch > TOUCH_INTERVAL) {
    if (TouchRead() > 1) {
      //S_MENU
      switch (Mode) {
        case S_MENU    : 
          switch (TouchQuarter()) {
            case 0: Mode = S_SENSOR1; break;
            case 1: Mode = S_SWITCH1; break;
            case 2: Mode = S_SWITCH4; break;
            case 3: Mode = S_SETTING; break;
          } 
          break;
        case S_SENSOR1: 
          switch (Touch.Gesture) {
            case CLICK:       ActiveSens = NextSensor(); break;
            case SWIPE_LEFT:  ActiveSens = NextSensor(); break;
            case SWIPE_RIGHT: ActiveSens = PrevSensor(); break;
            case SWIPE_UP:    Mode = S_MENU; break;
          }
          break;
        case S_SWITCH1: 
          switch (Touch.Gesture) {
            case CLICK:       ToggleSwitch(ActiveSens);  break;
            case SWIPE_LEFT:  ActiveSens = NextSwitch(); break;
            case SWIPE_RIGHT: ActiveSens = PrevSwitch(); break;
            case SWIPE_UP:    Mode = S_MENU; break;
          }
          break;
        case S_SWITCH4 : 
          switch (Touch.Gesture) {
            case CLICK:       ToggleSwitch(TouchQuarter()); break;
            case SWIPE_LEFT:  ActivePeer = NextPDC(); break;
            case SWIPE_RIGHT: ActivePeer = PrevPDC(); break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_SENSOR4 : 
          switch(Touch.Gesture) {
            case CLICK:       Mode = S_SENSOR1; ActiveSens = TouchQuarter(); break;
            case SWIPE_LEFT:  ActivePeer = NextBat(); break; //Sensor oder Peer +
            case SWIPE_RIGHT: ActivePeer = PrevBat(); break; //Sensor oder Peer -
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_PEER    : ShowPeer(ActivePeer);    break;
        case S_PEERS   : ShowPeers();   break;
        case S_JSON    : ShowJSON();    break;
        //case S_SETTING : ShowSetting(); break;
      }
    }
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

  TFTGaugeSwitch.createSprite(100,100);
  TFTGaugeSwitch.setSwapBytes(false);
  TFTGaugeSwitch.pushImage(0, 0, 100, 100, BtnSmall);

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
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); PrintMAC(P[Pi].BroadcastAddress);
      
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
      if (isPDC(P[Pi].Type)) AvailPDC = true;
      if (isBat(P[Pi].Type)) AvailBat = true;

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
void ShowSensor1(int SNr) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(SNr))) {
    ScreenChanged = true;             

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);       
        
    switch(P[ActivePeer].S[SNr].Type) {
      case SENS_TYPE_AMP:  RingMeter(TempValue[0], 0, 30, "Amp", P[ActivePeer].S[SNr].Name, GREEN2RED);  break;
      case SENS_TYPE_VOLT: RingMeter(TempValue[0], 0, 15, "Volt", P[ActivePeer].S[SNr].Name, GREEN2RED); break;
    }
    
    TSScreenRefresh = millis();

    P[ActivePeer].S[SNr].OldValue = P[ActivePeer].S[SNr].Value; 
  } 
}
void ShowSwitch1(int SNr) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(SNr))) {
    ScreenChanged = true;
  
    TFTBuffer.pushImage(0,0, 240, 240, Btn);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(P[ActivePeer].S[SNr].Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(P[ActivePeer].Name, 120,160);
    TFTBuffer.unloadFont();

         if (P[ActivePeer].S[ActiveSens].Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (P[ActivePeer].S[ActiveSens].Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TSScreenRefresh = millis();

    P[ActivePeer].S[SNr].OldValue = P[ActivePeer].S[ActiveSens].Value; 
  } 
}
void ShowSensor4(float Value[4]) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(0,3))) {
    ScreenChanged = true;             

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  

    char Buf[20];
    int Si = 0;
    for (int Row=0; Row<2; Row++) {
      for (int Col=0; Col<2; Col++) {
        if (isSensorAmp (P[ActivePeer].S[Si].Type)) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(P[ActivePeer].S[Si].Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(Value[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        if (isSensorVolt(P[ActivePeer].S[Si].Type)) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(P[ActivePeer].S[Si].Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(P[ActivePeer].S[Si].Value, 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        Si++;
      }
    }
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.drawString(P[ActivePeer].Name, 120,15);
    TFTBuffer.unloadFont(); 
  }
}
void ShowSwitch4() {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(0,3))) {
    ScreenChanged = true;             

  TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
  TFTBuffer.setTextDatum(MC_DATUM);
  TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 
  TFTBuffer.loadFont(AA_FONT_SMALL);  

  int Si = 0;
    for (int Row=0; Row<2; Row++) {
      for (int Col=0; Col<2; Col++) {
        TFTGaugeSwitch.pushToSprite(&TFTBuffer, 22+Col*96, 25+Row*90, 0x4529);
        TFTBuffer.drawString(P[ActivePeer].S[Si].Name, 70+Col*100, 85+Row*90);
      }
      Si++;
    }
  }

  TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
  TFTBuffer.drawString(P[ActivePeer].Name, 120,15);
  TFTBuffer.unloadFont(); 
}
void ScreenUpdate() {
  if (!TSMsgStart) {
    switch (Mode) {
      case S_SENSOR1: 
        if ((Mode != OldMode) or (SensorChanged(ActiveSens))) {
          noInterrupts(); 
            TempValue[0] = P[ActivePeer].S[ActiveSens].Value; 
            P[ActivePeer].S[ActiveSens].Changed = false; 
          interrupts();
          if      (isSensorAmp (P[ActivePeer].S[ActiveSens].Type)) {
            RingMeter(TempValue[0], 0, 30, "Amp", P[ActivePeer].S[ActiveSens].Name, GREEN2RED);
          }
          else if (isSensorVolt(P[ActivePeer].S[ActiveSens].Type)) {
            RingMeter(TempValue[0], 0, 16, "Volt", P[ActivePeer].S[ActiveSens].Name, GREEN2RED);
          }
          OldMode = Mode;
        }
        break;          
      case S_SENSOR4:
        if ((Mode != OldMode) or (SensorChanged(0,3))) {
            
          noInterrupts(); 
            for (int Si=0; Si<4; Si++) TempValue[Si] = P[ActivePeer].S[Si].Value;  
          interrupts();
          
          ShowSensor4(TempValue);
          
          OldMode = Mode;
          for (int Si=0; Si<4; Si++) P[ActivePeer].S[Si].Changed = false;
        }
        break;  
      case S_JSON   :   ShowJSON();    OldMode = Mode; break;  
      //case S_DEVICES:   ShowDevices(); OldMode = Mode; break;
      //case S_CALIB_V:   EichenVolt();  OldMode = Mode; break;  
      case S_MENU:      
        if (Mode != OldMode) {
          ShowMenu(); 
          OldMode = Mode; 
        }
      break;        
    }
    PushTFT();
  }
}
void ShowMenu() {
  if (Mode != OldMode) TSScreenRefresh = millis();
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TSScreenRefresh = millis();
  }
}
void PushTFT() {
  SetMsgIndicator();
  
  if (ScreenChanged) {
    Serial.print("ScreenUpdate: ");
    Serial.println(UpdateCount);
    UpdateCount++;
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
void ToggleSwitch(int Si) {
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"] = NODE_NAME;   
  doc["Order"] = "ToggleSwitch";
  doc["Value"] = P[ActivePeer].S[Si].Name;
  
  serializeJson(doc, jsondata);  
  
  esp_now_send(P[ActivePeer].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";
}
bool isPDC(int Type) {
  return ((Type == SWITCH_1_WAY) or (Type == SWITCH_2_WAY) or (Type == SWITCH_4_WAY) or (Type == SWITCH_8_WAY));      
}
bool isPDC1(int Type) {
  return (Type == SWITCH_1_WAY);      
}
bool isPDC2(int Type) {
  return (Type == SWITCH_2_WAY);      
}
bool isPDC4(int Type) {
  return (Type == SWITCH_4_WAY);      
}
bool isPDC8(int Type) {
  return (Type == SWITCH_8_WAY);      
}
bool isBat(int Type) {
  return(Type == BATTERY_SENSOR);
}
bool isSwitch(int Type) {
  return (Type == SENS_TYPE_SWITCH);      
}
bool isSensor(int Type) {
  return((Type == SENS_TYPE_AMP) or (Type == SENS_TYPE_VOLT));
}
bool isSensorVolt(int Type) {
  return (Type == SENS_TYPE_VOLT);
}
bool isSensorAmp(int Type) {
  return (Type == SENS_TYPE_AMP);
}
bool SensorChanged(int Start, int Stop=0) {
  int ret = false;
  if (Stop == 0) Stop = Start;
  for (int Si=Start; Si++; Si<Stop+1) if (P[ActivePeer].S[Si].Changed) ret = true;
  return ret;
}
int  NextSensor(){
  int SNr = ActiveSens;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr++;
    if (SNr == MAX_PERIPHERALS) SNr = 0;
    if ((P[ActivePeer].S[SNr].Type == SENS_TYPE_AMP) or (P[ActivePeer].S[SNr].Type == SENS_TYPE_VOLT)) return SNr;
  }
  return ActiveSens;
}
int  PrevSensor() {
  int SNr = ActiveSens;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr--;
    if (SNr == -1) SNr = MAX_PERIPHERALS-1;
    if ((P[ActivePeer].S[SNr].Type == SENS_TYPE_AMP) or (P[ActivePeer].S[SNr].Type == SENS_TYPE_VOLT)) return SNr;
  }
  return ActiveSens;
}
int  NextSwitch(){
  int SNr = ActiveSens;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr++;
    if (SNr == MAX_PERIPHERALS) SNr = 0;
    if (P[ActivePeer].S[SNr].Type == SENS_TYPE_SWITCH) return SNr;
  }
  return ActiveSens;
}
int  PrevSwitch() {
  int SNr = ActiveSens;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr--;
    if (SNr == -1) SNr = MAX_PERIPHERALS-1;
    if (P[ActivePeer].S[SNr].Type == SENS_TYPE_SWITCH) return SNr;
  }
  return ActiveSens;
}
int  NextPDC() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr++;
    if (PNr == MAX_PEERS) PNr = 0;
    if (isPDC(P[PNr].Type)) return PNr;
  }
  return ActivePeer;
}
int  PrevPDC() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr--;
    if (PNr == -1) PNr = MAX_PEERS-1;
    if (isPDC(P[PNr].Type)) return PNr;
  }
  return ActivePeer;
}
int  NextBat() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr++;
    if (PNr == MAX_PEERS) PNr = 0;
    if (isBat(P[PNr].Type)) return PNr;
  }
  return ActivePeer;
}
int  PrevBat() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr--;
    if (PNr == -1) PNr = MAX_PEERS-1;
    if (isBat(P[PNr].Type)) return PNr;
  }
  return ActivePeer;
}
int  TouchQuarter(void) {
  if ((Touch.x1<120) and (Touch.y1<120)) return 0;
  if ((Touch.x1>120) and (Touch.y1<120)) return 1;
  if ((Touch.x1<120) and (Touch.y1>120)) return 2;
  if ((Touch.x1>120) and (Touch.y1>120)) return 3;
  return NOT_FOUND;
}
int  TouchRead() {
  uint16_t TouchX, TouchY, Gesture;
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
    else { Touch.Gesture = CLICK; ret = CLICK; }
  }                                                                    
  //nicht berührt
  else (!TouchContact and !Touch.TSTouched) {  
    ret = 0;
    T.TSTouched  = 0;
    T.TSReleased = 0;
  }
  return ret;
}
void PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}