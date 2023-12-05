#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
//#include <FS.h>
#include <SPIFFS.h>
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
#define NOT_FOUND        -1

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
  int x0, x1, y0, y1;
  uint32_t TSTouched;
  uint32_t TSReleased;
  int Gesture;
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

bool   isPDC(int Type);
bool   isBattery(int Type);
bool   isSwitch(int Type);
bool   isSensor(int Type);
int    TouchQuarteer(void);

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

int ActivePeer   = 0;
int ActiveSensor = 0;
int Mode         = 0;

float TempValue[8] = {0,0,0,0,0,0,0,0};

uint32_t TSTouch    = 0;
uint32_t TSMsgStart = 0;

TFT_eSPI TFT              = TFT_eSPI();
TFT_eSprite TFTBuffer     = TFT_eSprite(&TFT);
TFT_eSprite TFTGauge      = TFT_eSprite(&TFT);

CST816D TouchHW(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

void setup() {
  InitModule();

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
        case S_SWITCH1 : ToggleSwitch(1); break;
        case S_SWITCH4 : 
          switch (TouchQuarter()) {
            case 0: ToggleSwitch(0); break;
            case 1: ToggleSwitch(1); break;
            case 2: ToggleSwitch(2); break;
            case 3: ToggleSwitch(3); break;
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
void ShowSensor1() {
  ScreenChanged = true;             

  TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
  TFTBuffer.setTextDatum(MC_DATUM);

  if (isSensorAmp(P[ActivePeer].S[ActiveSensor].Type)) {
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);       
    RingMeter(TempValue[0], 0, 30, "Amp", P[ActivePeer].S[ActiveSensor].Name, GREEN2RED);
  }
  if (isSensorVolt(P[ActivePeer].S[ActiveSensor].Type)) {
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    RingMeter(TempValue[0], 0, 15, "Volt", P[ActivePeer].S[ActiveSensor].Name, GREEN2RED);
  }
  if (isSwitch    (P[ActivePeer].S[ActiveSensor].Type)) {
    TFTBuffer.pushImage(0,0, 240, 240, Btn);                    
  
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(P[ActivePeer].S[ActiveSensor].Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(P[ActivePeer].Name, 120,160);
    TFTBuffer.unloadFont();

         if (P[ActivePeer].S[ActiveSensor].Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (P[ActivePeer].S[ActiveSensor].Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TSScreenRefresh = millis();
  }

  P[ActivePeer].S[ActiveSensor].OldValue = P[ActivePeer].S[ActiveSensor].Value;  
}
void ShowSensor4() {
  ScreenChanged = true;
  TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
  
  TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
  TFTBuffer.setTextDatum(MC_DATUM);
  
  int Si = 0;
  for (int Row=0; Row<2; Row++) {
    for (int Col=0; Col<2; Col++) {
      if (isSensorAmp (P[ActivePeer].S[Si].Type)) {
        TFTBuffer.loadFont(AA_FONT_LARGE); 
        TFTBuffer.drawString(P[ActivePeer].S[Si].Name,  60+Col*120, 30+Row*120);
        //Format Output
        TFTBuffer.drawString(P[ActivePeer].S[Si].Value, 60+Col*120, 90+Row*120);
        TFTBuffer.unloadFont();
      }
      if (isSensorVolt(P[ActivePeer].S[Si].Type)) {
        TFTBuffer.loadFont(AA_FONT_LARGE); 
        TFTBuffer.drawString(P[ActivePeer].S[Si].Name,  60+Col*120, 30+Row*120)
        //Format Output
        TFTBuffer.drawString(P[ActivePeer].S[Si].Value, 60+Col*120, 90+Row*120)
        TFTBuffer.unloadFont();
      }
      if (isSwitch    (P[ActivePeer].S[Si].Type)) {
        TFTBuffer.loadFont(AA_FONT_SMALL); 
        TFTGaugeAmp.pushToSprite(&TFTBuffer, 22+Col*96, 25+Row*90, 0x4529);

        TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
        TFTBuffer.drawString(P[ActivePeer].S[Si].Name, 70+Col*100, 85+Row*90);
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
void ScreenUpdate() {
  if (!TSMsgStart) {
    switch (Mode) {
      case S_SENSOR1:
        if ((mode != oldMode) or (P[ActivePeer].S[ActiveSensor].Changed)) {
          noInterrupts(); 
            TempValue[0] = P[ActivePeer].S[ActiveSensor].Value; 
            P[ActivePeer].S[ActiveSensor].Changed = false; 
          interrupts();
          char Unit[20];
          if      (isSensorAMP (P[ActivePeer].S[ActiveSensor].Type)) {
            strcpy (Unit, "Amp");
            RingMeter(TempValue[0], 0, 30, Unit, P[ActivePeer].S[ActiveSensor].Name, GREEN2RED);
          }
          else if (isSensorVolt(P[ActivePeer].S[ActiveSensor].Type)) {
            strcpy (Unit, "Volt");
            RingMeter(TempValue[0], 0, 16, Unit, P[ActivePeer].S[ActiveSensor].Name, GREEN2RED);
          }
          else if (isSwitch    (P[ActivePeer].S[ActiveSensor].Type)) {
            ShowSwitch1();
          }
          oldMode = mode;
        }
        break;          
      case S_SENSOR4:
        if ((mode != oldMode) or 
          (P[ActivePeer].S[0].Changed) or 
          (P[ActivePeer].S[1].Changed) or 
          (P[ActivePeer].S[2].Changed) or 
          (P[ActivePeer].S[3].Changed)) {
            
          noInterrupts(); 
            for (int Si=0; Si<4; Si++) TempValue[Si] = P[ActivePeer].S[Si].Value; 
            for (int Si=0; Si<4; Si++) P[ActivePeer].S[Si].Changed = false; 
          interrupts();
          
          ShowSensor4();
          
          oldMode = mode;
          for (int Si=0; Si<4; Si++) P[ActivePeer].S[i].Changed = false;
        }
        break;       
      case S_SWITCH8:
        if ((mode != oldMode) or 
          (P[ActivePeer].S[0].Changed) or 
          (P[ActivePeer].S[1].Changed) or 
          (P[ActivePeer].S[2].Changed) or 
          (P[ActivePeer].S[3].Changed) or
          (P[ActivePeer].S[4].Changed) or 
          (P[ActivePeer].S[5].Changed) or 
          (P[ActivePeer].S[6].Changed) or 
          (P[ActivePeer].S[7].Changed)) {
           
          noInterrupts(); 
            for (int Si=0; Si<8; Si++) TempValue[Si] = P[ActivePeer].S[Si].Value; 
            for (int Si=0; Si<8; Si++) P[ActivePeer].S[Si].Changed = false; 
          interrupts();
          
          ShowSensor8();
          
          oldMode = mode;
          for (int Si=0; Si<8; Si++) P[ActivePeer].S[i].Changed = false;
        }
        break;          
      case S_JSON   :  ShowJSON();    oldMode = mode; break;  
      case S_DEVICES:   ShowDevices(); oldMode = mode; break;
      case S_CALIB_V:   EichenVolt();  oldMode = mode; break;  
      case S_MENU:      
        if (mode != oldMode) {
          ShowMenu(); 
          oldMode = mode; 
        }
      break;        
    }

    PushTFT();
  }
}

}

void ShowMenu() {
  if (mode != oldMode) TSScreenRefresh = millis();
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TSScreenRefresh = millis();
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
bool isBattery(int Type) {
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
bool isSensorAMP(int Type) {
  return (Type == SENS_TYPE_AMP);
}
int  NextSensor(){
  int SNr = ActiveSensor;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr++;
    if (SNr == MAX_PERIPHERALS) SNr = 0;
    if ((P[ActivePeer].S[SNr].Type == SENS_TYPE_AMP) or (P[ActivePeer].S[SNr].Type == SENS_TYPE_VOLT)) return SNr;
  }
  return NOT_FOUND;
}
int  PrevSensor() {
  int SNr = ActiveSensor;
  
  for (int i=0; i<MAX_PERIPHERALS; i++) {
    SNr--;
    if (SNr == -1) SNr = MAX_PERIPHERALS-1;
    if ((P[ActivePeer].S[SNr].Type == SENS_TYPE_AMP) or (P[ActivePeer].S[SNr].Type == SENS_TYPE_VOLT)) return SNr;
  }
  return NOT_FOUND;
}
int  NextPDC() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr++;
    if (PNr == MAX_PEERS) PNr = 0;
    if (isPDC(P[PNr].Type)) return PNr;
  }
  return NOT_FOUND;
}
int  PrevPDC() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr--;
    if (PNr == -1) PNr = MAX_PEERS-1;
    if (isPDC(P[PNr].Type)) return PNr;
  }
  return NOT_FOUND;
}
int  NextBat() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr++;
    if (PNr == MAX_PEERS) PNr = 0;
    if (isBattery(P[PNr].Type)) return PNr;
  }
  return NOT_FOUND;
}
int  PrevBat() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr--;
    if (PNr == -1) PNr = MAX_PEERS-1;
    if (isBattery(P[PNr].Type)) return PNr;
  }
  return NOT_FOUND;
}
int  TouchQuarter(void) {
  if ((Touch.x1<120) and (Touch.y1<120)) return 0;
  if ((Touch.x1>120) and (Touch.y1<120)) return 1;
  if ((Touch.x1<120) and (Touch.y1>120)) return 2;
  if ((Touch.x1>120) and (Touch.y1>120)) return 3;
  return NOT_FOUND;
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