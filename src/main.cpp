#include <Arduino.h>
#include "../../jeepify.h"
#include <TFT_eSPI.h>
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
#define NODE_TYPE MONITOR_ROUND

#define VERSION   "V 0.94"

void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void   OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void   InitModule();
void   SavePeers();
void   GetPeers();
void   ReportPeers();
void   RegisterPeers();
void   ClearPeers();
void   ClearInit();

void   SendPing();
void   ToggleSwitch(struct_Peer *Peer, struct_Periph *Periph);
void   SendCommand(struct_Peer *Peer, String Cmd);
void   SendPairingConfirm(struct_Peer *Peer);

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);

void   DrawButton(int z, bool Selected = false);
bool   ButtonHit(int b);
int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();

void   SetMsgIndicator();
void   ScreenUpdate();
void   PushTFT();
void   ShowMessage(char *Msg);
void   ShowSwitch1();
void   ShowSwitch4(int Start=0);
void   ShowSensor1();
void   ShowSensor4(int Start=0);
void   ShowMulti(int Start=0);
void   ShowMenu();
void   ShowJSON();
void   ShowSettings();
void   ShowPeer();
void   ShowPeers();
void   ShowPairing();

int    RingMeter(float vmin, float vmax, const char *units, byte scheme);

bool   PeriphChanged(struct_Peer *Peer, int Start, int Stop=99);
bool   isPDC (struct_Peer *Peer);
bool   isPDC1(struct_Peer *Peer);
bool   isPDC2(struct_Peer *Peer);
bool   isPDC4(struct_Peer *Peer);
bool   isPDC8(struct_Peer *Peer);
bool   isBat(struct_Peer *Peer);
bool   isSwitch(struct_Periph *Periph);
bool   isSensor(struct_Periph *Periph);
bool   isSensorAmp (struct_Periph *Periph);
bool   isSensorVolt(struct_Periph *Periph);
struct_Periph *FindPeriphByNumber(struct_Peer *Peer, int Id);
struct_Periph *FindFirstPeriph(struct_Peer *Peer, int Type, bool OnlyActual=false);
struct_Periph *FindNextPeriph (struct_Peer *Peer, struct_Periph *Periph, int Type=SENS_TYPE_EQUAL, bool OnlyActual=false);
struct_Periph *FindPrevPeriph (struct_Peer *Peer, struct_Periph *Periph, int Type=SENS_TYPE_EQUAL, bool OnlyActual=false);
struct_Peer   *FindPeerByName(String Name);
struct_Peer   *FindPeerByMAC(const uint8_t *MAC);
struct_Peer   *FindEmptyPeer();
struct_Peer   *FindFirstPeer(int Type=MODULE_ALL);
struct_Peer   *FindNextPeer (struct_Peer *Peer, int Type=MODULE_ALL);
struct_Peer   *FindPrevPeer (struct_Peer *Peer, int Type=MODULE_ALL);
struct_Peer   *SelectPeer();

int    TouchQuarter(void);
int    TouchRead();

void   PrintMAC(const uint8_t * mac_addr);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int rainbow(byte value);

struct_Touch  Touch;
struct_Peer   P[MAX_PEERS];
struct_Button Button[13] = {
  { 25, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Volt",      false},   // 0
  { 95, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Amp",       false},   // 1
  {165, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "JSON",      false},   // 2
  { 60, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "DBG",       false},   // 3
  {130, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "Pair",      false},   // 4
  
  { 45,  25, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Restart",   false},   // 5
  {125,  25, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Reset",     false},   // 6
  { 45,  65, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Pair",      false},   // 7
  {125,  65, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Eichen",    false},   // 8
  { 45, 115, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "VoltCal.",  false},   // 9
  {125, 115, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Sleep",     false},   // 10
  {125, 155, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Debug",     false},   // 11

  {125,  75, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "JSON",      false}    // 12
};

struct_Periph *PeriphMulti[8];

Preferences preferences;

struct_Peer   *ActivePeer, *ActivePDC, *ActiveBat, *ActiveSelection;
struct_Periph *ActiveSens, *ActiveSwitch, *ActivePeriph;

int PeerCount    =  0;
int Mode         =  S_MENU;
int OldMode      = 99;
int UpdateCount  =  0;

bool ScreenChanged = true;
bool Debug = true;
bool SleepMode = false;
bool ReadyToPair = false;

int   ButtonRd   = 22;
int   ButtonGapX = 6;
int   ButtonGapY = 6;
float VoltCalib;
int   VoltCount;

float   TempValue[8] = {0,0,0,0,0,0,0,0};

String jsondataBuf;

int FirstDisplayedSwitch = 0;
int FirstDisplayedSensor = 0;

uint32_t TSTouch         = 0;
uint32_t TSPing          = 0;
uint32_t TSMsgStart      = 0;
uint32_t TSScreenRefresh = 0;
volatile uint32_t TSMsgRcv  = 0;
volatile uint32_t TSMsgPDC  = 0;
volatile uint32_t TSMsgBat  = 0;
volatile uint32_t TSMsgVolt = 0;
volatile uint32_t TSMsgEich = 0;
volatile uint32_t TSMsgPair = 0;
volatile uint32_t TSPair    = 0;

bool MsgBatAktiv  = false;
bool MsgPDCAktiv  = false;
bool MsgVoltAktiv = false;
bool MsgEichAktiv = false;
bool MsgPairAktiv = false;

TFT_eSPI TFT               = TFT_eSPI();
TFT_eSprite TFTBuffer      = TFT_eSprite(&TFT);
TFT_eSprite TFTGaugeSwitch = TFT_eSprite(&TFT);

CST816D TouchHW(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char* buff = (char*) incomingData;   
  StaticJsonDocument<500> doc;
  String jsondata;
  
  jsondata = "";  jsondata = String(buff); 
  jsondataBuf = String(buff); 

  String BufS;
  char Buf[10] = {};
  
  bool PairingSuccess = false;

  Serial.print("Recieved from:"); PrintMAC(mac); Serial.println(); Serial.println(jsondata);
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    struct_Peer *Peer = FindPeerByMAC(mac); //FindPeerByName(doc["Node"]);
    TSMsgRcv = millis();

    if (Peer) {
      Peer->TSLastSeen = millis();
      Serial.print("bekannter Node: "); Serial.print(Peer->Name); Serial.print(" - "); Serial.println(Peer->TSLastSeen);
      
      if (isBat(Peer)) TSMsgBat = TSMsgRcv;
      if (isPDC(Peer)) TSMsgPDC = TSMsgRcv;
      
      if (doc["Pairing"] == "add me") { SendPairingConfirm(Peer); }
      else {
        for (int i=0; i<MAX_PERIPHERALS; i++) {
          if (doc.containsKey(Peer->S[i].Name)) {
            float TempSensor = (float)doc[Peer->S[i].Name];
        
            if (TempSensor != Peer->S[i].Value) {
              Peer->S[i].Value = TempSensor;
              Peer->S[i].Changed = true;
            }
          }
          if (doc.containsKey("Sleep")) Peer->Sleep = (bool) doc["Sleep"];
          if (doc.containsKey("Debug")) Peer->Debug = (bool) doc["Debug"];
        } 
      }
    } 
    else {
      if ((doc["Pairing"] == "add me") and (ReadyToPair)) { // neuen Peer registrieren
        Peer = FindEmptyPeer();

        if (Peer) {
          Serial.print("gefundener Slot Id="); Serial.println(Peer->Id);
          for (int b = 0; b < 6; b++ ) Peer->BroadcastAddress[b] = mac[b];
          PrintMAC(Peer->BroadcastAddress);
          Serial.println();
          strcpy(Peer->Name, doc["Node"]);
          Peer->Type = doc["Type"];
          Peer->TSLastSeen = millis();
          
          for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
            sprintf(Buf, "T%d", Si);        // Type0
            if (doc.containsKey(Buf)) {
              Peer->S[Si].Type = doc[Buf];
              sprintf(Buf, "N%d", Si);      // Name0
              strcpy(Peer->S[Si].Name, doc[Buf]);
              Peer->S[Si].Id = Si;
              Peer->S[Si].PeerId = Peer->Id;
            }
          }   
          SavePeers();
          RegisterPeers();
          
          SendPairingConfirm(Peer);

          ReadyToPair = false; TSPair = 0;
        }
      }
    }
  }
  else {                                            // Error
    Serial.print(F("deserializeJson() failed: "));  //Just in case of an ERROR of ArduinoJSon
    Serial.println(error.f_str());
    return;
  }
}
void SendPairingConfirm(struct_Peer *Peer) {
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  doc.clear();
              
  doc["Node"]     = NODE_NAME;   
  doc["Peer"]     = Peer->Name;
  doc["Pairing"]  = "you are paired";
  doc["Type"]     = MONITOR_ROUND;
  doc["B0"]       = Peer->BroadcastAddress[0];
  doc["B1"]       = Peer->BroadcastAddress[1];
  doc["B2"]       = Peer->BroadcastAddress[2];
  doc["B3"]       = Peer->BroadcastAddress[3];
  doc["B4"]       = Peer->BroadcastAddress[4];
  doc["B5"]       = Peer->BroadcastAddress[5];

  serializeJson(doc, jsondata);  
  Serial.println("prepared... sending you are paired");
  Serial.println(jsondata);
  esp_now_send(Peer->BroadcastAddress, (uint8_t *) jsondata.c_str(), 200); 
  Serial.print("Sent you are paired"); 
  Serial.println(jsondata);         
}
void setup() {
  Serial.begin(74880);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);    

  Serial.println("InitModule...");

    //Workaround
  for (int PNr=0; PNr<MAX_PEERS; PNr++) P[PNr].Id = PNr;

  InitModule();
  GetPeers();
  RegisterPeers();
  
  if (PeerCount == 0) { Serial.println("PeerCount=0, RTP=True"); ReadyToPair = true; TSPair = millis(); Mode = S_PAIRING;}
  
  PeriphMulti[0] = &P[0].S[0];
  PeriphMulti[1] = &P[0].S[1];
  PeriphMulti[2] = &P[0].S[0];
  PeriphMulti[3] = &P[0].S[1];
  
  Mode = 1;
  TSScreenRefresh = millis();
  TSTouch = millis();
}
void loop() {
  /*if ((TSMsgStart) and (millis() - TSMsgStart > LOGO_INTERVAL)) {
    Mode = S_MENU;
    TSMsgStart = 0;
  }*/
  if (millis() - TSPing  > PING_INTERVAL)  {
    TSPing = millis();
    SendPing();
    //Serial.println("Ping fertig");
  }
  if (millis() - TSTouch > TOUCH_INTERVAL) {
    int TouchErg = TouchRead();
    if (TouchErg > 1) {
      switch (Mode) {
        case S_MENU    : 
          switch (Touch.Gesture) {
            case CLICK:       
              switch (TouchQuarter()) {
                case 0: 
                  Serial.println("vor ActiveBat Suche");
                  if (!ActiveSens) ActiveSens = FindFirstPeriph(ActiveBat, SENS_TYPE_SENS, false);
                  Serial.print("ActiveSens="); Serial.println(ActiveSens->Name);
                  
                  if (!ActiveSens) ShowMessage("No Sensor"); 
                  else  Mode = S_SENSOR1;
                  
                  break;
                case 1: 
                  if (!ActiveSwitch) ActiveSwitch = FindFirstPeriph(ActivePDC, SENS_TYPE_SWITCH);
                  if (ActiveSwitch) Mode = S_SWITCH1;
                  break;
                case 2: 
                  //if (!ActiveSelection) ActiveSelection = FindFirstPeer();
                  //if (!ActiveSelection) { ShowMessage("No Peers"); break; }
                  //else Mode = S_PEER_SEL;
                  //break;
                  Mode = S_MULTI;
                  break;
                case 3: Mode = S_SETTING;  break;
              } 
              break;
            case SWIPE_LEFT:  Mode = S_PEERS; break;
            case SWIPE_RIGHT: 
              if (!ActivePeer) ActivePeer = FindFirstPeer();
              if (ActivePeer)  Mode = S_PEER; 
              break;
            case SWIPE_UP:    break;
          }
          break;
        case S_SENSOR1: 
          switch (Touch.Gesture) {
            case CLICK:       
              Serial.print("vorher: "); Serial.println(ActiveSens->Name);
              ActiveSens = FindNextPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
              ScreenChanged = true; 
              Serial.print("nachher: "); Serial.println(ActiveSens->Name);
              break;
            case SWIPE_LEFT:  
              ActiveSens = FindNextPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
              ScreenChanged = true; 
              break;
            case SWIPE_RIGHT: 
              Serial.print("vorher: "); Serial.println(ActiveSens->Name);
              ActiveSens = FindPrevPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
              ScreenChanged = true; 
              Serial.print("nachher: "); Serial.println(ActiveSens->Name);
              break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_SWITCH1: 
          switch (Touch.Gesture) {
            case CLICK:       ToggleSwitch(ActivePeer, ActiveSwitch);  break;
            case SWIPE_LEFT:  ActiveSwitch = FindNextPeriph(ActivePeer, ActiveSwitch, SENS_TYPE_SWITCH); ScreenChanged = true; break;
            case SWIPE_RIGHT: ActiveSwitch = FindPrevPeriph(ActivePeer, ActiveSwitch, SENS_TYPE_SWITCH); ScreenChanged = true; break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_SWITCH4 : 
          switch (Touch.Gesture) {
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_MULTI    : 
          switch (Touch.Gesture) {
            case CLICK:       
              switch (TouchQuarter()) {
                case 0: ToggleSwitch(&P[PeriphMulti[0]->PeerId], PeriphMulti[0]); break;
                case 1: ToggleSwitch(&P[PeriphMulti[1]->PeerId], PeriphMulti[1]); break;
                case 2: ToggleSwitch(&P[PeriphMulti[2]->PeerId], PeriphMulti[2]); break;
                case 3: ToggleSwitch(&P[PeriphMulti[3]->PeerId], PeriphMulti[3]); break;
              }
              break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_SETTING:
          switch (Touch.Gesture) {
            case CLICK:            if (ButtonHit( 5)) { ESP.restart(); }
                              else if (ButtonHit( 6)) { ClearPeers(); ESP.restart(); }
                              else if (ButtonHit( 7)) { ReadyToPair = true; TSPair = millis(); Mode = S_PAIRING; }
                              else if (ButtonHit(11)) { if (Debug) SetDebugMode(false); else SetDebugMode(true); }
                              else if (ButtonHit(12)) { ScreenChanged = true; Mode = S_JSON; }
                              
                              break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          } 
          break;
        case S_PEER    : 
          switch (Touch.Gesture) {
            case CLICK:            if (ButtonHit( 5)) SendCommand(ActivePeer, "Restart");
                              else if (ButtonHit( 6)) SendCommand(ActivePeer, "Reset");
                              else if (ButtonHit( 7)) SendCommand(ActivePeer, "Pair");
                              else if (ButtonHit( 8)) SendCommand(ActivePeer, "Eichen");
                              else if (ButtonHit( 9)) SendCommand(ActivePeer, "VoltCalib");
                              else if (ButtonHit(10)) SendCommand(ActivePeer, "SleepMode Toggle");
                              else if (ButtonHit(11)) SendCommand(ActivePeer, "Debug Toggle");
                              break;
            case SWIPE_LEFT:  ActivePeer = FindNextPeer(ActivePeer); ScreenChanged = true; break; 
            case SWIPE_RIGHT: ActivePeer = FindPrevPeer(ActivePeer); ScreenChanged = true; break; 
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_PEER_SEL   : 
          switch (Touch.Gesture) {
            case CLICK:       
              ActivePeer = ActiveSelection; 
              if (isBat(ActivePeer)) ActiveBat = ActivePeer;
              if (isPDC(ActivePeer)) ActivePDC = ActivePeer;
              ActiveSens = NULL;
              ActiveSwitch = NULL;
              Mode = S_MENU; 
              break;
            case SWIPE_UP:    ActiveSelection = FindPrevPeer(ActiveSelection); Serial.println(ActiveSelection->Name); break;
            case SWIPE_DOWN:  ActiveSelection = FindNextPeer(ActiveSelection); Serial.println(ActiveSelection->Name); break;
          }
          break;
        case S_PEERS: 
          switch (Touch.Gesture) {
            case CLICK:       Mode = S_MENU; break;
            case SWIPE_LEFT:  Mode = S_MENU; break;
            case SWIPE_RIGHT: Mode = S_MENU; break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;    
        case S_JSON    :
          switch (Touch.Gesture) {
            case CLICK:       Mode = S_MENU; break;
            case SWIPE_LEFT:  Mode = S_MENU; break;
            case SWIPE_RIGHT: Mode = S_MENU; break;
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_PAIRING: 
          switch (Touch.Gesture) {
            case CLICK: TSPair = 0; ReadyToPair = false; Mode = S_MENU; break; 
          }
          break;
      }
    }
  TSTouch = millis();
  ScreenUpdate();  
  }
}
void ScreenUpdate() {
  if (!TSMsgStart) {
    switch (Mode) {
      case S_PAIRING:   ShowPairing();  break;
      case S_SENSOR1:   ShowSensor1();  break;          
      case S_SENSOR4:   ShowSensor4();  break;  
      case S_SWITCH1:   ShowSwitch1();  break;
      case S_SWITCH4:   ShowSwitch4();  break;
      case S_MULTI:     ShowMulti(0);   break;
      
      case S_JSON:      ShowJSON();     break;  
      case S_SETTING:   ShowSettings(); break;
      case S_PEER:      ShowPeer();     break;
      case S_PEERS:     ShowPeers();    break;
      case S_PEER_SEL:  SelectPeer(); break;
      case S_CAL_VOL:   EichenVolt();   break;  
      case S_MENU:      ShowMenu();     break;        
    }
  }
  PushTFT();
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

  TFTBuffer.createSprite(240,240);
  TFTBuffer.setSwapBytes(true);
  
  TFTGaugeSwitch.createSprite(100,100);
  TFTGaugeSwitch.setSwapBytes(false);
  TFTGaugeSwitch.pushImage(0, 0, 100, 100, BtnSmall);

  TFT.pushImage(0,0, 240, 240, JeepifyLogo); 
  TFT.loadFont(AA_FONT_SMALL); 
  TFT.setTextColor(TFT_BLACK, TFT_RED, false);
  TFT.setTextDatum(MC_DATUM); 
  TFT.drawString(VERSION, 120,60);
  
  TSMsgStart = 0;
  
  Serial.println("InitModule() fertig...");
}
void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  char Buf[50] = {}; char BufNr[5] = {}; String BufS;

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
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); PrintMAC(P[Pi].BroadcastAddress); Serial.println();
      
      //P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = P[Pi].Name;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      if (preferences.getString(Buf, "EmptyName") != BufS) preferences.putString(Buf, BufS);

      for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
        sprintf(Buf, "P%d-SensName%d", Pi, Si);
        BufS = P[Pi].S[Si].Name;
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Name);
        if (preferences.getString(Buf, "EmptyName") != BufS) preferences.putString(Buf, BufS);
        
        sprintf(Buf, "P%d-SensType%d", Pi, Si);
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Type);
        if (preferences.getInt(Buf, 0) != P[Pi].S[Si].Type) preferences.putInt(Buf, P[Pi].S[Si].Type);

        sprintf(Buf, "P%d-SensId%d", Pi, Si);
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Id);
        if (preferences.getInt(Buf, 0) != P[Pi].S[Si].Id) preferences.putInt(Buf, P[Pi].S[Si].Id);

      }
    }
  }
  if (preferences.getInt("PeerCount") != PeerCount) preferences.putInt("PeerCount", PeerCount);
  
  preferences.end();
}
void GetPeers() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[50] = {}; char BufNr[5] = {}; String BufS;
  
  PeerCount = 0;
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    // Peer gefüllt?
    sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
    Serial.print(Buf); Serial.print(" = "); Serial.println(preferences.getInt(Buf));
    if (preferences.getInt(Buf) > 0) {
      PeerCount++;
      P[Pi].Id = Pi;

      // P.Type
      P[Pi].Type = preferences.getInt(Buf);
      if (ActivePeer == NULL) ActivePeer = &P[Pi];
      if (isPDC(&P[Pi]) and (ActivePDC == NULL)) ActivePDC = &P[Pi];
      if (isBat(&P[Pi]) and (ActiveBat == NULL)) ActiveBat = &P[Pi];

      // P.BroadcastAdress
      strcpy(Buf, "MAC-"); strcat (Buf, BufNr);
      preferences.getBytes(Buf, P[Pi].BroadcastAddress, 6);
      
      // P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = preferences.getString(Buf);
      strcpy(P[Pi].Name, BufS.c_str());

      Serial.print(Pi); Serial.print(": Type="); Serial.print(P[Pi].Type); 
      Serial.print(", Name="); Serial.print(P[Pi].Name);
      Serial.print(", MAC="); PrintMAC(P[Pi].BroadcastAddress);
      Serial.println();

      for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
        sprintf(Buf, "P%d-SensName%d", Pi, Si);
        BufS = preferences.getString(Buf);
        strcpy(P[Pi].S[Si].Name, BufS.c_str());

        sprintf(Buf, "P%d-SensType%d", Pi, Si);
        P[Pi].S[Si].Type = preferences.getInt(Buf, 0);
        
        sprintf(Buf, "P%d-SensId%d", Pi, Si);
        P[Pi].S[Si].Id = preferences.getInt(Buf, 0);
        
        P[Pi].S[Si].PeerId = P[Pi].Id;
        
        sprintf(Buf, "SName%d=%s, SType%d=%d, SId%d=%d", Si, BufS, Si, P[Pi].S[Si].Type, Si, P[Pi].S[Si].Id);
        Serial.println(Buf);

        if (isSensor(&P[Pi].S[Si]) and (ActiveSens == NULL))   ActiveSens   = &P[Pi].S[Si];
        if (isSwitch(&P[Pi].S[Si]) and (ActiveSwitch == NULL)) ActiveSwitch = &P[Pi].S[Si];

      }
      P[Pi].TSLastSeen = millis();
      
      if (Debug) {
        Serial.print(Pi); Serial.print(": Type="); Serial.print(P[Pi].Type); 
        Serial.print(", Name="); Serial.print(P[Pi].Name);
        Serial.print(", MAC="); PrintMAC(P[Pi].BroadcastAddress);
        Serial.println();
        for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
          sprintf(Buf, "P%d-SensName%d = ", Pi, Si);
          Serial.print(Buf), Serial.println(P[Pi].S[Si].Name);
          sprintf(Buf, "P%d-SensType%d = ", Pi, Si);
          Serial.println(P[Pi].S[Si].Type);
        }
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
  //delay(5000);
}
void RegisterPeers() {
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 1;
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
void SendPing() {
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  
  doc.clear();
  
  doc["Node"] = NODE_NAME;   
  doc["Order"] = "stay alive";

  if (ReadyToPair) {
    doc["Pairing"] = "aktiv";
  }

  serializeJson(doc, jsondata);  
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type) esp_now_send(P[PNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
  }

  //Serial.print("Sending Ping:"); 
  //Serial.println(jsondata);   

  //Serial.print("Mode="); Serial.print(Mode);
  //Serial.print(", OldMode="); Serial.println(OldMode);
     
}
void DrawButton(int z, bool Selected) {
  TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
  TFTBuffer.setTextDatum(MC_DATUM);
  TFTBuffer.setTextColor(Button[z].TxtColor);

  if (Selected) TFTBuffer.fillSmoothRoundRect(Button[z].x, Button[z].y, Button[z].w, Button[z].h, 10, TFT_DARKGREY, Button[z].BGColor);
  else          TFTBuffer.drawSmoothRoundRect(Button[z].x, Button[z].y, 10, 8, Button[z].w, Button[z].h, TFT_LIGHTGREY, Button[z].BGColor);
  TFTBuffer.drawString(Button[z].Name, Button[z].x+Button[z].w/2, Button[z].y+Button[z].h/2);  

   TFTBuffer.unloadFont();
}
bool ButtonHit(int b) {
  return ( ((Touch.x1>Button[b].x) and (Touch.x1<Button[b].x+Button[b].w) and 
            (Touch.y1>Button[b].y) and (Touch.y1<Button[b].y+Button[b].h)) );
}
void ShowSensor1() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (ActiveSens->Changed)) {
    ScreenChanged = true;   
    OldMode = Mode;          

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);       
      
    switch(ActiveSens->Type) {
      case SENS_TYPE_AMP:  RingMeter(0, 30, "Amp",  GREEN2RED); break;
      case SENS_TYPE_VOLT: RingMeter(0, 15, "Volt", GREEN2RED); break;
      default:             ShowMessage("No Sensor"); break;
    }
    
    TSScreenRefresh = millis();
  } 
}
void ShowSensor4(int Start) {
  FirstDisplayedSensor = Start;
  
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (PeriphChanged(ActivePeer, Start,Start+3))) {
    ScreenChanged = true;  
    OldMode = Mode;           

    noInterrupts(); 
      for (int Si=0; Si<4; Si++) TempValue[Si] = ActivePeer->S[Si].Value;  
    interrupts();

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  

    char Buf[20];
    int Si = 0;
    for (int Row=0; Row<2; Row++) {
      for (int Col=0; Col<2; Col++) {
        if (isSensorAmp (&ActivePeer->S[Si])) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(ActivePeer->S[Si].Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(TempValue[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        if (isSensorVolt(&ActivePeer->S[Si])) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(ActivePeer->S[Si].Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(TempValue[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        Si++;
      }
    }
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.drawString(ActivePeer->Name, 120,15);
    TFTBuffer.unloadFont(); 
    
    noInterrupts(); 
      for (int Si=0; Si<4; Si++) ActivePeer->S[Si].Changed = false;
    interrupts();
    
    TSScreenRefresh = millis(); 
  }
}
void ShowMessage(char *Msg) {
    ScreenChanged = true;             
    
    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 

    TFTBuffer.drawString(Msg, 120, 120);
    
    PushTFT();
    delay(200);
    OldMode = S_STATUS;
    Mode    = S_MENU;
    ScreenChanged = true;
}
void ShowSwitch1() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (ActiveSwitch->Changed)) {
    ScreenChanged = true;
    OldMode = Mode;

    TFTBuffer.pushImage(0,0, 240, 240, Btn);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(ActiveSwitch->Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(ActivePeer->Name, 120,160);
    TFTBuffer.unloadFont();

         if (ActiveSwitch->Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (ActiveSwitch->Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TSScreenRefresh = millis();

    //P[ActivePeer].S[SNr].OldValue = P[ActivePeer].S[ActiveSens].Value;
    noInterrupts(); 
      ActiveSwitch->Changed = false;  
    interrupts(); 
  }
}
void ShowSwitch4(int Start) { 
  FirstDisplayedSwitch = Start;

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (PeriphChanged(ActivePeer, Start,Start+3))) {
    ScreenChanged = true;     
    OldMode = Mode;        

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 
    TFTBuffer.loadFont(AA_FONT_SMALL);  

    int Si = Start;
    for (int Row=0; Row<2; Row++) {
      for (int Col=0; Col<2; Col++) {
        if (isSwitch(&ActivePeer->S[Si])) {
          TFTGaugeSwitch.pushToSprite(&TFTBuffer, 22+Col*96, 25+Row*90, 0x4529);
          TFTBuffer.drawString(ActivePeer->S[Si].Name, 70+Col*100, 85+Row*90);
        }
      }
      Si++; 
      if (Si == MAX_PERIPHERALS) break;
    }
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.drawString(ActivePeer->Name, 120,15);
    TFTBuffer.unloadFont(); 
  }
  TSScreenRefresh = millis(); 
}
void ShowMulti(int Start) {
  FirstDisplayedSwitch = Start;
  char Buf[20];

  noInterrupts(); 
    for (int Si=0; Si<4; Si++) TempValue[Si] = PeriphMulti[Si]->Value;  
  interrupts();
  
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;     
    OldMode = Mode;        

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 
    TFTBuffer.loadFont(AA_FONT_SMALL);  

    int Si = Start;
    for (int Row=0; Row<2; Row++) {
      for (int Col=0; Col<2; Col++) {
        if (isSwitch(PeriphMulti[Start+Si])) {
          TFTGaugeSwitch.pushToSprite(&TFTBuffer, 22+Col*96, 25+Row*90, 0x4529);
          if (PeriphMulti[Start+Si]->Value == 1) TFTBuffer.pushImage( 59+Col*96, 53+Row*90, 27, 10, BtnOn) ; 
          else                                   TFTBuffer.pushImage( 59+Col*96, 53+Row*90, 27, 10, BtnOff);
          TFTBuffer.drawString(PeriphMulti[Start+Si]->Name, 70+Col*100, 85+Row*90);
        }
        else if (isSensorAmp (PeriphMulti[Start+Si])) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(PeriphMulti[Start+Si]->Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(TempValue[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        else if (isSensorVolt(&ActivePeer->S[Si])) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(PeriphMulti[Start+Si]->Name,  60+Col*120, 30+Row*120);
          //Format Output
          dtostrf(TempValue[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        Si++;
      }
    }
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.drawString("MultiView", 120,15);
    TFTBuffer.unloadFont(); 
  }
  TSScreenRefresh = millis(); 
}
void ShowPairing() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;             
    OldMode = Mode;

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 

    TFTBuffer.drawString("Pairing...", 120, 120);
    TSScreenRefresh = millis(); 
  }
}
void ShowJSON() {
  StaticJsonDocument<500> doc;
  if ((jsondataBuf != "") or (Mode != OldMode)) {
    ScreenChanged = true;
    OldMode = Mode;

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString("Debug JSON", 120, 200); 
    
    DeserializationError error = deserializeJson(doc, jsondataBuf);
    if (doc["Node"] != NODE_NAME) { 
      int sLength = jsondataBuf.length();

      if (sLength) {
        int len = 20;
        int Abstand = 20;
        int sLength = jsondataBuf.length();

        TFTBuffer.setTextDatum(MC_DATUM);
        TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
      
        for (int i=0 ; i<8 ; i++) {
          if ((i+1)*len < sLength)  TFTBuffer.drawString(jsondataBuf.substring(i*len, (i+1)*len), 120,50+i*Abstand); 
          else if (i*len < sLength) TFTBuffer.drawString(jsondataBuf.substring(i*len, sLength), 120,50+i*Abstand); 
        }
        jsondataBuf = "";
      }
    }
    TSScreenRefresh = millis();
  }
}
void ShowSettings() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    OldMode = Mode;
    ScreenChanged = true;

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString("Settings", 120, 200); 
    
    DrawButton(5);
    DrawButton(6);
    DrawButton(7,  ReadyToPair);
    DrawButton(11, Debug);
    DrawButton(12);

    TSScreenRefresh = millis();
  }
}
void ShowPeer() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    OldMode = Mode;
    ScreenChanged = true;

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString(ActivePeer->Name, 120, 200); 

    DrawButton(5);
    DrawButton(6);
    DrawButton(7);
    DrawButton(8);
    DrawButton(9);
    DrawButton(10, ActivePeer->Sleep);

    TSScreenRefresh = millis();
  }
}
void ShowPeers() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    OldMode = Mode;
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString("Peers", 120, 200); 

    int Abstand = 20;
    String ZName;

    TFTBuffer.setTextDatum(TL_DATUM);
    
    for (int PNr=0 ; PNr<MAX_PEERS ; PNr++) {
      if (P[PNr].Type) {
        TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
        TFTBuffer.drawString(P[PNr].Name, 10, 80+PNr*Abstand);
        switch (P[PNr].Type) {
          case SWITCH_1_WAY:   ZName = "1-way PDC";   break;
          case SWITCH_2_WAY:   ZName = "2-way PDC";   break;
          case SWITCH_4_WAY:   ZName = "4-way PDC";   break;
          case SWITCH_8_WAY:   ZName = "8-way PDC";   break;
          case PDC_SENSOR_MIX: ZName = "PDC-MIX";     break;
          case BATTERY_SENSOR: ZName = "Batt.-Sens."; break;
        }
        TFTBuffer.drawString(ZName, 105, 80+PNr*Abstand);
        
        if (millis()- P[PNr].TSLastSeen > OFFLINE_INTERVAL) { 
          TFTBuffer.setTextColor(TFT_DARKGREY,  TFT_BLACK); 
          TFTBuffer.drawString("off", 200, 80+PNr*Abstand);
        }
        else { 
          TFTBuffer.setTextColor(TFT_DARKGREEN, TFT_BLACK); 
          TFTBuffer.drawString("on",  200, 80+PNr*Abstand); 
        }
      }
    }
    TSScreenRefresh = millis();
  }
}
void ShowMenu() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TSScreenRefresh = millis();
  }
}
void PushTFT() {
  //Serial.println("Beginn push");
  SetMsgIndicator();
  //Serial.println("nach Msgindicator");
  if (ScreenChanged) {
    /*Serial.print("ScreenUpdate: ");
    Serial.print(UpdateCount);
    Serial.print(" - Gesture: ");
    Serial.println(Touch.Gesture);
    */UpdateCount++;
    TFTBuffer.pushSprite(0, 0);
    ScreenChanged = false;
  }
}
void SetMsgIndicator() {
  if (Debug) {
    if (TSMsgVolt) {
        if (millis() - TSMsgVolt > MSGLIGHT_INTERVAL) {
          TSMsgVolt = 0;
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 335, 355, TFT_BLACK, TFT_BLACK);
          MsgVoltAktiv = false;
          ScreenChanged = true;
        }
        else if (MsgVoltAktiv == false) {
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 335, 355, TFT_BLUE, TFT_BLACK);
          MsgVoltAktiv = true;
          ScreenChanged = true;
        }
    }
    if (TSMsgEich) {
        if (millis() - TSMsgEich > MSGLIGHT_INTERVAL) {
          TSMsgEich = 0; 
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 5, 25, TFT_BLACK, TFT_BLACK);
          MsgEichAktiv = false;
          ScreenChanged = true;
        }
        else if (MsgEichAktiv == false) {
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 5, 25, TFT_GREEN, TFT_BLACK);
          MsgEichAktiv = true;
          ScreenChanged = true;
        }
    }
    if (TSMsgPDC) {
        if (millis() - TSMsgPDC > MSGLIGHT_INTERVAL) {
          TSMsgPDC = 0;
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 0, 5, TFT_BLACK, TFT_BLACK);
          MsgPDCAktiv = false;
          ScreenChanged = true;
        }
        else if (MsgPDCAktiv == false) {
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 0, 5, TFT_DARKGREY, TFT_BLACK);
          MsgPDCAktiv = true;
          ScreenChanged = true;
        }
    }
    if (TSMsgBat) {
        if (millis() - TSMsgBat > MSGLIGHT_INTERVAL) {
          TSMsgBat = 0;
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 355, 360, TFT_BLACK, TFT_BLACK);
          MsgBatAktiv = false;
          ScreenChanged = true;
        }
        else if (MsgBatAktiv == false) {
          TFTBuffer.drawSmoothArc(120, 120, 120, 118, 355, 360, TFT_DARKGREEN, TFT_BLACK);
          MsgBatAktiv = true;
          ScreenChanged = true;
        }
    }
    if (!ReadyToPair) {
      TFTBuffer.drawSmoothArc(120, 120, 120, 118,   5,  10, TFT_BLACK, TFT_BLACK);
      MsgPairAktiv = false;
      ScreenChanged = true;
    }
    else {
      TFTBuffer.drawSmoothArc(120, 120, 120, 118,   5,  10, TFT_RED, TFT_BLACK);
      MsgPairAktiv = true;
      ScreenChanged = true;
    }
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
void ToggleSwitch(struct_Peer *Peer, struct_Periph *Periph) {
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"]  = NODE_NAME;   
  doc["Order"] = "ToggleSwitch";
  doc["Value"] = Periph->Name;
  
  serializeJson(doc, jsondata);  
  
  esp_now_send(Peer->BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";
}
void SendCommand(struct_Peer *Peer, String Cmd) {
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"] = NODE_NAME;   
  doc["Order"] = Cmd;
  
  serializeJson(doc, jsondata);  
  
  esp_now_send(Peer->BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";
}
bool isPDC(struct_Peer *Peer) {
  if (Peer) return ((Peer->Type == SWITCH_1_WAY) or (Peer->Type == SWITCH_2_WAY) or 
                    (Peer->Type == SWITCH_4_WAY) or (Peer->Type == SWITCH_8_WAY) or
                    (Peer->Type == PDC_SENSOR_MIX));   
  return false;   
}
bool isPDC1(struct_Peer *Peer) {
  if (Peer) return (Peer->Type == SWITCH_1_WAY);
  return false;   
}
bool isPDC2(struct_Peer *Peer) {
  if (Peer) return (Peer->Type == SWITCH_2_WAY);
  return false;   
}
bool isPDC4(struct_Peer *Peer) {
  if (Peer) return (Peer->Type == SWITCH_4_WAY);
  return false;   
}
bool isPDC8(struct_Peer *Peer) {
  if (Peer) return (Peer->Type == SWITCH_8_WAY);
  return false;   
}
bool isBat(struct_Peer *Peer) {
  if (Peer) return ((Peer->Type == BATTERY_SENSOR) or (Peer->Type == PDC_SENSOR_MIX));
  return false;
}
bool isSwitch(struct_Periph *Periph) {
  if (Periph) return (Periph->Type == SENS_TYPE_SWITCH); 
  return false;     
}
bool isSensor(struct_Periph *Periph) {
  if (Periph) return((Periph->Type == SENS_TYPE_AMP) or (Periph->Type == SENS_TYPE_VOLT));
  return false;
}
bool isSensorVolt(struct_Periph *Periph) {
  if (Periph) return (Periph->Type == SENS_TYPE_VOLT);
  return false;
}
bool isSensorAmp(struct_Periph *Periph) {
  if (Periph) return (Periph->Type == SENS_TYPE_AMP);
  return false;
}
bool PeriphChanged(struct_Peer *Peer, int Start, int Stop) {
  int ret = false;
  if (Stop == 99) Stop = Start;
  for (int Si=Start; Si++; Si<Stop+1) {
    if (Si == MAX_PERIPHERALS) break;
    if (Peer->S[Si].Changed) ret = true;
  }
  return ret;
}
struct_Periph *FindSensorByNumber(struct_Peer *Peer, int Id) {
  for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
    if (Peer->S[SNr].Id == Id) return &Peer->S[SNr];
  }
  return NULL;
}
struct_Periph *FindFirstPeriph(struct_Peer *Peer, int Type, bool OnlyActual){
  if (!Peer) Peer = FindFirstPeer();
   if (Peer) {
    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
        switch (Type) {
          case SENS_TYPE_AMP:    if (isSensorAmp(&Peer->S[SNr]))  { ActivePeer = Peer; return &Peer->S[SNr]; break; }
          case SENS_TYPE_VOLT:   if (isSensorVolt(&Peer->S[SNr])) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
          case SENS_TYPE_SENS:   if (isSensor(&Peer->S[SNr]))     { ActivePeer = Peer; return &Peer->S[SNr]; break; }
          case SENS_TYPE_SWITCH: if (isSwitch(&Peer->S[SNr]))     { ActivePeer = Peer; return &Peer->S[SNr]; break; }
        }
      }
    }
    if (OnlyActual) return NULL;
    Peer = FindNextPeer(Peer);
  }
  return NULL;
}
struct_Periph *FindNextPeriph (struct_Peer *Peer, struct_Periph *Periph, int Type, bool OnlyActual){
  if ((Periph) and (Peer)) {
    if (Type == SENS_TYPE_EQUAL) Type = Periph->Type;

    int SNr = Periph->Id;
    int Id  = Periph->Id;

    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int i=0; i<MAX_PERIPHERALS; i++) {
        SNr++; if (SNr == MAX_PERIPHERALS) SNr = 0;
        char Buf[100];
        sprintf(Buf, "Vergleiche %s, %s (Id=%d) mit %s, %s (Id(%d)...", Peer->Name, Peer->S[SNr].Name, Peer->S[SNr].Id, ActivePeer->Name, ActivePeer->S[Id].Name, ActivePeer->S[Id].Id);
        switch (Type) {
          case SENS_TYPE_SENS:  
            /*Serial.print("ID orig: "); Serial.print(Id);
            Serial.print(", Peer->SId"); Serial.print(SNr); Serial.print("="); Serial.print(Peer->S[SNr].Id);
            Serial.print(", Peer->SType"); Serial.print(SNr); Serial.print("="); Serial.println(Peer->S[SNr].Type);
            */
            if ((Peer->S[SNr].Type == SENS_TYPE_AMP)  and (Peer->S[SNr].Id > Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            if ((Peer->S[SNr].Type == SENS_TYPE_VOLT) and (Peer->S[SNr].Id > Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
          default:
            if ((Peer->S[SNr].Type == Type) and (Peer->S[SNr].Id > Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
        }
      }
      if (OnlyActual) return NULL;
      Peer = FindNextPeer(Peer);
      Id = -1; SNr = -1;
    }
  }
  return Periph;
}
struct_Periph *FindPrevPeriph (struct_Peer *Peer, struct_Periph *Periph, int Type, bool OnlyActual){
  if ((Periph) and (Peer)) {
    if (Type == SENS_TYPE_EQUAL) Type = Periph->Type;
    
    int SNr = Periph->Id;
    int Id  = Periph->Id;

    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int i=0; i<MAX_PERIPHERALS; i++) {
        SNr--; if (SNr == -1) SNr = MAX_PERIPHERALS-1;
        switch (Type) {
          case SENS_TYPE_SENS:  
            /*Serial.print("ID orig: "); Serial.print(Id);
            Serial.print(", Peer->SId"); Serial.print(SNr); Serial.print("="); Serial.print(Peer->S[SNr].Id);
            Serial.print(", Peer->SType"); Serial.print(SNr); Serial.print("="); Serial.println(Peer->S[SNr].Type);
            */
            if ((Peer->S[SNr].Type == SENS_TYPE_AMP)  and (Peer->S[SNr].Id < Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            if ((Peer->S[SNr].Type == SENS_TYPE_VOLT) and (Peer->S[SNr].Id < Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
          default:
            if ((Peer->S[SNr].Type == Type) and (Peer->S[SNr].Id < Id)) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
        }
      }
      if (OnlyActual) return NULL;
      Peer = FindPrevPeer(Peer);
      Id = MAX_PERIPHERALS; SNr = MAX_PERIPHERALS;
    }
  }
  return Periph;
}
struct_Periph *SelectPeriph() {
  if (!ActivePeer) ActivePeer = FindFirstPeer();
  if (ActivePeer) {
    if (!ActivePeriph) ActivePeriph = FindFirstPeriph(ActivePeer, false);
    if (ActivePeriph) {
      if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
        ScreenChanged = true;
        TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
        TFTBuffer.loadFont(AA_FONT_LARGE);
        TFTBuffer.setTextDatum(MC_DATUM);
        TFTBuffer.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        TFTBuffer.drawString(ActivePeriph->Name, 120, 120);
        TFTBuffer.unloadFont();
        
        TFTBuffer.loadFont(AA_FONT_SMALL);
        TFTBuffer.drawString(P[ActivePeriph->PeerId].Name, 120,  80);
        
        switch (ActivePeriph->Type) {
          case SENS_TYPE_AMP:    TFTBuffer.drawString("Amp-Sensor",  120, 160); break;
          case SENS_TYPE_VOLT:   TFTBuffer.drawString("Volt-Sensor", 120, 160); break;
          case SENS_TYPE_SWITCH: TFTBuffer.drawString("Switch",      120, 160); break;
        }
        TSScreenRefresh = millis();      
        return (ActivePeriph);
      }
    }
  }
  return NULL;
}
struct_Peer   *SelectPeer() {
  if (!ActivePeer) ActivePeer = FindFirstPeer();
  if (ActivePeer) {
    if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
      ScreenChanged = true;
      TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
      TFTBuffer.fillSmoothRoundRect(50, 50, 140, 140, 10, TFT_LIGHTGREY, TFT_BLACK);
      TFTBuffer.loadFont(AA_FONT_SMALL);

      TFTBuffer.setTextColor(TFT_RUBICON, TFT_LIGHTGREY);
      TFTBuffer.setTextDatum(MC_DATUM);
      TFTBuffer.drawString("-", 120,  80);
      TFTBuffer.drawString("+", 120, 160);
      TFTBuffer.drawString(ActiveSelection->Name, 120, 120);
      
      TSScreenRefresh = millis();
      return (ActivePeer);
    }
  }
  return NULL;
}
struct_Peer *FindPeerByName(String Name) {
  //Serial.print("gesuchter Name: "); Serial.println(Name.c_str());
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    //Serial.print(P[PNr].Name); Serial.print(" =? "); Serial.println(Name.c_str());
    if ((String)P[PNr].Name == Name) return &P[PNr];
  }
  //Serial.println("durchgelaufen");
  return NULL;
}
struct_Peer *FindPeerByMAC(const uint8_t *MAC) {
  Serial.print("gesuchte MAC: "); PrintMAC(MAC);

  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if(memcmp(P[PNr].BroadcastAddress, MAC, 6) == 0) {
      Serial.print(" ist: "); Serial.println(P[PNr].Name);
      return &P[PNr];
    }
  }
  Serial.println(" nicht gefunden");
  return NULL;
}
struct_Peer *FindEmptyPeer() {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type == 0) return &P[PNr];
  }
  return NULL;
}
struct_Peer *FindFirstPeer(int Type) {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type == Type) return &P[PNr];
    if ((P[PNr].Type) and (Type == MODULE_ALL)) return &P[PNr];
  }
  return NULL;
}
struct_Peer *FindNextPeer(struct_Peer *Peer, int Type) {
  if (Peer) {
    int PNr = Peer->Id;
    int Id  = Peer->Id;
 
    for (int i=0; i<MAX_PEERS; i++) {
      PNr++; if (PNr == MAX_PEERS) { PNr = 0; Id = -1; }
      
      Serial.print("ID orig: "); Serial.print(Peer->Id);
      Serial.print("untersucht: "); Serial.println((P[PNr].Name));
      Serial.print(", Peer->Id"); Serial.print(PNr); Serial.print("="); Serial.print(P[PNr].Id);
      Serial.print(", Peer->Type"); Serial.print(PNr); Serial.print("="); Serial.println(P[PNr].Type);
      
      if ((P[PNr].Type == Type)                   and (P[PNr].Id > Id)) return &P[PNr];
      if ((P[PNr].Type) and (Type == MODULE_ALL)  and (P[PNr].Id > Id)) return &P[PNr];
    }
  }
  return Peer;
}
struct_Peer *FindPrevPeer(struct_Peer *Peer, int Type) {
  if (Peer) {
    int PNr = Peer->Id;
    int Id  = Peer->Id;
    
    for (int i=0; i<MAX_PEERS; i++) {
      PNr--; if (PNr == -1) { PNr = MAX_PEERS-1; Id = MAX_PEERS; }
        
      if ((P[PNr].Type == Type)                  and (P[PNr].Id < Id)) return &P[PNr];
      if ((P[PNr].Type) and (Type == MODULE_ALL) and (P[PNr].Id < Id)) return &P[PNr];
    }
  }
  return Peer;
}
int  TouchQuarter(void) {
  if ((Touch.x1<120) and (Touch.y1<120)) return 0;
  if ((Touch.x1>120) and (Touch.y1<120)) return 1;
  if ((Touch.x1<120) and (Touch.y1>120)) return 2;
  if ((Touch.x1>120) and (Touch.y1>120)) return 3;
  return NOT_FOUND;
}
int  TouchRead() {
  uint16_t TouchX, TouchY = 0;
  uint8_t  Gesture = 0;

  int ret = 0;

  Touch.Touched = TouchHW.getTouch(&TouchX, &TouchY, &Gesture);
  TouchX = 240-TouchX; 

  if(Touch.Touched && !Touch.TouchedOld) {
    Touch.x0 = TouchX;    // erste Berührung
    Touch.y0 = TouchY;
    Touch.TSFirstTouch = millis();
    Touch.TSReleaseTouch = 0;
    ret = TOUCHED;
  } 
  else if (Touch.Touched && Touch.TouchedOld) { 
    Touch.x1 = TouchX;     // gehalten
    Touch.y1 = TouchY;
    ret = HOLD;
  }
  else if (!Touch.Touched && Touch.TouchedOld) {
    Touch.x1 = TouchX;     // losgelassen
    Touch.y1 = TouchY;
    Touch.TSReleaseTouch = millis();
         if ((Touch.x1-Touch.x0) > 50)  { Touch.Gesture = SWIPE_RIGHT;  ret = SWIPE_RIGHT; }                      // swipe left
    else if ((Touch.x1-Touch.x0) < -50) { Touch.Gesture = SWIPE_LEFT; ret = SWIPE_LEFT; }                     // swipe right
    else if ((Touch.y1-Touch.y0) > 50)  { Touch.Gesture = SWIPE_DOWN;  ret = SWIPE_DOWN; }                      // swipe down
    else if ((Touch.y1-Touch.y0) < -50) { Touch.Gesture = SWIPE_UP;    ret = SWIPE_UP; }                        // swipe up
    else if ((Touch.TSReleaseTouch - Touch.TSFirstTouch) > LONG_PRESS_INTERVAL)                                 // longPress
                                        { Touch.Gesture = LONG_PRESS;  ret = LONG_PRESS; }
    else                                { Touch.Gesture = CLICK; ret = CLICK; }
  }
  else if (!Touch.Touched && !Touch.TouchedOld) {
    Touch.x0 = 0;    // nix
    Touch.y0 = 0;
    Touch.x1 = 0;
    Touch.y1 = 0;
    Touch.Gesture = 0;
    Touch.TSFirstTouch = 0;
    Touch.TSReleaseTouch = 0;
    ret = 0;
  }
  Touch.TouchedOld = Touch.Touched;  

  return ret;
}
int  RingMeter(float vmin, float vmax, const char *units, byte scheme) {
  int x = 0;
  int y = 0;
  int r = 120;
  int nk = 0;
  
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    OldMode = Mode;
      
    noInterrupts(); 
      float value = ActiveSens->Value; 
      if (value < SCHWELLE) value = 0;
    interrupts();

    char buf[20];
    float fbuf;
  
    byte len = 5;
  
    x += r; y += r;   // Calculate coords of centre of ring
    int w = r / 4;    // Width of outer ring is 1/4 of radius
    float angle = 150;  // Half the sweep angle of meter (300 degrees)
    float v = mapf(value, vmin, vmax, -angle, angle); // Map the value to an angle v
    
    byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
    byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
  
    // Variable to save "value" text colour from scheme and set default
    int colour = TFT_BLUE;
    
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    
    TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    if (Debug) TFTBuffer.drawString(ActivePeer->Name, 120,225);

    TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    TFTBuffer.drawString(units, 120,150); // Units display
    TFTBuffer.setTextDatum(BC_DATUM);
    TFTBuffer.drawString(ActiveSens->Name, 120,90); // Units display

    TFTBuffer.setTextColor(TFT_GREEN, TFT_BLACK);
    
    //draw Values
    dtostrf(vmin, len, 0, buf);
    
    TFTBuffer.setTextDatum(BL_DATUM);
    TFTBuffer.drawString(buf, 65, 200);
    
    dtostrf(vmin+((vmax-vmin)/4*1), len, 0, buf);
    TFTBuffer.setTextDatum(ML_DATUM);
    TFTBuffer.drawString(buf, w, 120);
    
    dtostrf(vmin+((vmax-vmin)/4*2), len, 0, buf);
    TFTBuffer.setTextDatum(TC_DATUM);
    TFTBuffer.drawString(buf, 115, w+10);
    
    dtostrf(vmin+((vmax-vmin)/4*3), len, 0, buf);
    TFTBuffer.setTextDatum(MR_DATUM);
    TFTBuffer.drawString(buf, 240-w-10, 120);
    
    dtostrf(vmax, len, 0, buf);
    TFTBuffer.setTextDatum(BR_DATUM);
    TFTBuffer.drawString(buf, 162, 200);
    
    // Set the text colour to default
    
    TFTBuffer.unloadFont();
    
    TFTBuffer.setTextDatum(MC_DATUM);
    
    if      (value<10)  nk = 2;
    else if (value<100) nk = 1;
    else                nk = 0;

    dtostrf(value, 0, nk, buf);

    TFTBuffer.loadFont(AA_FONT_LARGE); 
    TFTBuffer.setTextColor(0xF1C7,TFT_BLACK);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.drawString(buf, 120,120); // Value in middle
    
    TFTBuffer.unloadFont(); // Remove the font to recover memory used
  
    // Draw colour blocks every inc degrees
    for (int i = -angle+inc/2; i < angle-inc/2; i += inc) {
      // Calculate pair of coordinates for segment start
      float sx = cos((i - 90) * 0.0174532925);
      float sy = sin((i - 90) * 0.0174532925);
      uint16_t x0 = sx * (r - w) + x;
      uint16_t y0 = sy * (r - w) + y;
      uint16_t x1 = sx * r + x;
      uint16_t y1 = sy * r + y;

      // Calculate pair of coordinates for segment end
      float sx2 = cos((i + seg - 90) * 0.0174532925);
      float sy2 = sin((i + seg - 90) * 0.0174532925);
      int x2 = sx2 * (r - w) + x;
      int y2 = sy2 * (r - w) + y;
      int x3 = sx2 * r + x;
      int y3 = sy2 * r + y;

      if (i < v) { // Fill in coloured segments with 2 triangles
        switch (scheme) {
          case 0: colour = TFT_RED; break; // Fixed colour
          case 1: colour = TFT_GREEN; break; // Fixed colour
          case 2: colour = TFT_BLUE; break; // Fixed colour
          case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
          case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
          case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
          default: colour = TFT_BLUE; break; // Fixed colour
        }
        TFTBuffer.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        TFTBuffer.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
        //text_colour = colour; // Save the last colour drawn
      }
      else // Fill in blank segments
      {
        TFTBuffer.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
        TFTBuffer.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
      }
    }
    noInterrupts();
      ActiveSens->Changed = false; 
    interrupts();

    TSScreenRefresh = millis();
  }
  
  return x + r;
}
int  CalcField(int x, int y) {
  int Field = 1;
  int z    = 0;
  int StartX   = (240 - (6*ButtonRd) - 2*ButtonGapX)/2 + ButtonRd;
  int StartY   = (240 - (6*ButtonRd) - 2*ButtonGapY)/2 + ButtonRd - 5;

  for (int r=0; r<4; r++) {
    int y1 = StartY + r*(ButtonRd*2 + ButtonGapY) - ButtonRd;
    int y2 = y1+ButtonRd*2;
    for (int c=0; c<3; c++) {
      int x1 = StartX + c*(ButtonRd*2 + ButtonGapX) - ButtonRd;
      int x2 = x1+ButtonRd*2;
      if ((x>x1 and x<x2) and (y>y1 and y<y2)) {
        Serial.println(Field);
        return Field;
      }
      Field++;
    }
  }
  return -1;
}
void AddVolt(int i) {
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  //clearing String after data is being sent
  doc.clear();

  if ((i>=0) and (i<=9)) {
    switch (VoltCount) {
      case 0: VoltCalib  = (i)*10;          VoltCount++; break;
      case 1: VoltCalib += (i);             VoltCount++; break;
      case 2: VoltCalib += (float) (i)/10;  VoltCount++; break;
      case 3: VoltCalib += (float) (i)/100; VoltCount++; break;
    }
  } 
  else if (i==11) VoltCount++;
  
  if (VoltCount == 4) {
    char buf[10];
    dtostrf(VoltCalib, 5, 2, buf);

    doc["Node"] = NODE_NAME;   
    doc["Order"] = "VoltCalib";
    doc["Value"] = buf;
    
    serializeJson(doc, jsondata);  
  
    //esp_now_send(broadcastAddressBattery,     (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    //esp_now_send(broadcastAddressBattery8266, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    Serial.println(jsondata);

    TSMsgVolt = millis();
    Mode = S_MENU;
  }
}
void EichenVolt() {
  if (Mode != OldMode) {
    TSScreenRefresh = millis();
    VoltCount = 0;
    VoltCalib = 0;
  }

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    OldMode = Mode;

    TFTBuffer.pushImage(0,0, 240, 240, Keypad);  
    TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.setTextColor(TFT_RUBICON, 0x632C, false);

    char buf[10];
    dtostrf(VoltCalib, 5, 2, buf);

    TFTBuffer.drawString(buf, 120,25);
    TFTBuffer.unloadFont();

    TSScreenRefresh = millis();
  }
}
void ForceEichen() {
  StaticJsonDocument<500> doc;
  String jsondata;
  doc.clear();
  jsondata = "";

  doc["Node"] = NODE_NAME;   
  doc["Order"] = "Eichen";
  
  serializeJson(doc, jsondata);  
  
  //esp_now_send(broadcastAddressBattery,     (uint8_t *) jsondata.c_str(), 50);  //Sending "jsondata"  
  //esp_now_send(broadcastAddressBattery8266, (uint8_t *) jsondata.c_str(), 50);  //Sending "jsondata"  

  Serial.println(jsondata);
  jsondata = "";

  TSMsgEich = millis();
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
unsigned int rainbow(byte value) {
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}
void PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}