#define NODE_NAME "Jeep_Monitor_V2"
#define NODE_TYPE MONITOR_ROUND

#define VERSION   "V 1.11"

#pragma region Includes
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
#pragma endregion Includes
#pragma region Function_Definitions
void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void   OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void   SavePeers();
void   GetPeers();
void   SavePeriphMulti();
void   GetPeriphMulti();
void   ReportPeers();
void   RegisterPeers();
void   ClearPeers();
void   ClearInit();

void   SendPing();
bool   ToggleSwitch(struct_Periph *Periph);
void   SendCommand(struct_Peer *Peer, String Cmd);
void   SendPairingConfirm(struct_Peer *Peer);

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);

void   DrawButton(int z, bool Selected = false);
bool   ButtonHit(int b);
int    CalcField();
void   AddVolt(int i);

void   SetMsgIndicator();
void   ScreenUpdate();
void   PushTFT();
void   ShowMessage(String Msg);
void   ShowSingle(struct_Periph *Periph);
void   ShowMulti(struct_MultiScreen *Screen);
void   ShowEichenVolt();
void   ShowMenu();
void   ShowJSON();
void   ShowSettings();
void   ShowPeer();
void   ShowPeers();
void   ShowPairing();

int    RingMeter(float vmin, float vmax, const char *units, byte scheme);
void   LittleGauge(float Value, int x, int y, int Min, int Max, int StartYellow, int StartRed);

bool   PeriphChanged(struct_Peer *Peer, int Start, int Stop=0);
bool   isPDC (struct_Peer *Peer);
bool   isBat(struct_Peer *Peer);
bool   isSwitch(struct_Periph *Periph);
bool   isSensor(struct_Periph *Periph);
bool   isSensorAmp (struct_Periph *Periph);
bool   isSensorVolt(struct_Periph *Periph);
struct_Periph *FindPeriphById (struct_Peer *Peer, uint16_t Id);
struct_Periph *FindFirstPeriph(struct_Peer *Peer, int Type, bool OnlyActual=false);
struct_Periph *FindNextPeriph (struct_Periph *Periph, int Type=SENS_TYPE_EQUAL, bool OnlyActual=false);
struct_Periph *FindPrevPeriph (struct_Periph *Periph, int Type=SENS_TYPE_EQUAL, bool OnlyActual=false);
struct_Peer   *FindPeerByName (String Name);
struct_Peer   *FindPeerById   (uint16_t Id);
struct_Peer   *FindPeerByMAC  (const uint8_t *MAC);
struct_Peer   *FindEmptyPeer  ();
struct_Peer   *FindFirstPeer  (int Type=MODULE_ALL);
struct_Peer   *FindNextPeer   (struct_Peer *Peer, int Type=MODULE_ALL);
struct_Peer   *FindPrevPeer   (struct_Peer *Peer, int Type=MODULE_ALL);
int            FindHighestPeerId();
struct_Peer   *SelectPeer();
struct_Periph *SelectPeriph();

int    TouchQuarter(void);
int    TouchedField(void);
int    TouchRead();

void   PrintMAC(const uint8_t * mac_addr);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int rainbow(byte value);
#pragma endregion Function_Definitions
#pragma region Globals
#define MULTI_SCREENS 20 // 0-9 Peers, 10-10 Multi
#define MULTI_SCREEN_ROWS 2
#define MULTI_SCREEN_COLS 2
#define PERIPH_PER_SCREEN MULTI_SCREEN_COLS*MULTI_SCREEN_ROWS

struct struct_MultiScreen {
  int            Id;
  int            PeerId; // nicht für Multi, nur für 0-9
  struct_Peer   *Peer;
  int            PeriphId[PERIPH_PER_SCREEN];
  struct_Periph *S[PERIPH_PER_SCREEN];
  bool           isModuleScreen = false;
  char           Name[20];
};
struct_MultiScreen Screen[MULTI_SCREENS];
int ActiveMultiScreen = 0; 
int PeriphToFill;

struct_Touch  Touch;
struct_Peer   P[MAX_PEERS];
struct_Button Button[15] = {
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

  {125,  65, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "JSON",      false},   // 12
  { 45, 155, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Save",      false},   // 13
  { 45, 155, 70, 30, TFT_LIGHTGREY, TFT_BLACK, "Demo",      false}    // 14
};

Preferences preferences;

struct_Peer   *ActivePeer, *ActivePDC, *ActiveBat, *ActiveSelection;
struct_Periph *ActiveSens, *ActiveSwitch, *ActivePeriph;

int PeerCount    =  0;
int Mode         =  S_MENU;
int OldMode      = 99;
int UpdateCount  =  0;

bool ScreenChanged = true;
bool DebugMode = true;
bool SleepMode = false;
bool ReadyToPair = false;
bool ChangesSaved = true;

int   ButtonRd   = 22;
int   ButtonGapX = 6;
int   ButtonGapY = 6;
float VoltCalib;
int   VoltCount;

String jsondataBuf;

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
#pragma endregion Globals
#pragma region Main
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char* buff = (char*) incomingData;   
  StaticJsonDocument<500> doc; 
  String jsondata = String(buff); 
  
  String BufS; char Buf[10] = {};
  
  Serial.print("Recieved from:"); PrintMAC(mac); Serial.println(); Serial.println(jsondata);
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    struct_Peer *Peer = FindPeerByMAC(mac);
    TSMsgRcv = millis();

    if (Peer) { // Peer bekannt 
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
          if (doc.containsKey("Status")) {
            int Status = doc["Status"];
            Peer->DebugMode   = (bool) bitRead(Status, 0);
            Peer->SleepMode   = (bool) bitRead(Status, 1);
            Peer->DemoMode    = (bool) bitRead(Status, 2);
            Peer->ReadyToPair = (bool) bitRead(Status, 3);
          } 
        } 
      }
    } 
    else {      // Peer unbekannt 
      if ((doc["Pairing"] == "add me") and (ReadyToPair)) { // neuen Peer registrieren
        Peer = FindEmptyPeer();
        
        if (Peer) {
          strcpy(Peer->Name, doc["Node"]);
          Peer->Type = doc["Type"];

          Peer->Id = FindHighestPeerId()+1; // PeerId starts with 1;
          
          Serial.print("vergebene Id=");    Serial.println(Peer->Id);
          //Serial.print("vergebener Slot="); Serial.println(Peer->Slot);

          for (int b = 0; b < 6; b++ ) Peer->BroadcastAddress[b] = mac[b];
          Serial.println();

          Peer->TSLastSeen = millis();
          
          for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
            sprintf(Buf, "T%d", Si);        // Type0
            if (doc.containsKey(Buf)) {
              Peer->S[Si].Type = doc[Buf];
              sprintf(Buf, "N%d", Si);      // Name0
              strcpy(Peer->S[Si].Name, doc[Buf]);
              Peer->S[Si].Id = Si+1; //PeriphId starts with 1
              Peer->S[Si].PeerId = Peer->Id;
            }
          }   
          SavePeers();
          RegisterPeers();
          
          SendPairingConfirm(Peer);
          
          for (int s=0; s<MAX_PEERS; s++) {
            if (!Screen[s].PeerId) {
              strcpy(Screen[s].Name, doc["Node"]); 
              Screen[s].isModuleScreen = true;
              Screen[s].PeerId = Peer->Id;
              Screen[s].Peer = Peer;
              for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
                 Screen[s].PeriphId[SNr] = Peer->S[SNr].Id;
                 Screen[s].S[SNr] = &Peer->S[SNr];
              }
            }
          }

          ReadyToPair = false; TSPair = 0; Mode = S_MENU;
        }
      }
    }
  }
  else {        // Error
    Serial.print(F("deserializeJson() failed: ")); 
    Serial.println(error.f_str());
    return;
  }
}
void setup() {
  Serial.begin(74880);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  TFT.init();
  TFT.setRotation(Rotation);
  TFT.setSwapBytes(true);

  TFT.pushImage(0,0, 240, 240, JeepifyLogo); 
  TFT.loadFont(AA_FONT_SMALL); 
  TFT.setTextColor(TFT_BLACK, TFT_RED, false);
  TFT.setTextDatum(MC_DATUM); 
  TFT.drawString(VERSION, 120,60);
  TFT.unloadFont();

  TSMsgStart = millis();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);    

  preferences.begin("JeepifyInit", true);
  DebugMode = preferences.getBool("DebugMode", true);
  SleepMode = preferences.getBool("SleepMode", false);
  preferences.end();

  TouchHW.begin();

  TFTBuffer.createSprite(240,240);
  TFTBuffer.setSwapBytes(true);
  
  TFTGaugeSwitch.createSprite(100,100);
  TFTGaugeSwitch.setSwapBytes(false);
  TFTGaugeSwitch.pushImage(0, 0, 100, 100, BtnSmall);

  GetPeers();
  RegisterPeers();
  
  if (PeerCount == 0) { Serial.println("PeerCount=0, RTP=True"); ReadyToPair = true; TSPair = millis(); Mode = S_PAIRING;}
  
  for (int s=0; s<MULTI_SCREENS; s++) Screen[s].Id = s;

  Mode = S_MENU;

  TSScreenRefresh = millis();
  TSTouch         = millis();
}
void loop() {
  if (!TSMsgStart) {
    if (millis() - TSPing     > PING_INTERVAL)  { TSPing = millis(); SendPing(); }
    if (millis() - TSTouch    > TOUCH_INTERVAL) {
      int TouchErg = TouchRead();
      if (TouchErg > 1) {
        switch (Mode) {
          case S_MENU    : 
            switch (Touch.Gesture) {
              case CLICK:       
                switch (TouchQuarter()) {
                  case 0: 
                    if (!ActiveSens) ActiveSens = FindFirstPeriph(ActivePeer, SENS_TYPE_SENS, false);
                    if (!ActiveSens) ShowMessage("No Sensor"); 
                    else  Mode = S_SENSOR1;
                    break;
                  case 1: 
                    if (!ActiveSwitch) ActiveSwitch = FindFirstPeriph(ActivePeer, SENS_TYPE_SWITCH);
                    if (!ActiveSwitch) ShowMessage("No Switch");
                    else Mode = S_SWITCH1;
                    break;
                  case 2: 
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
                ActiveSens = FindNextPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
                ScreenChanged = true; 
                break;
              case SWIPE_LEFT:  
                ActiveSens = FindNextPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
                ScreenChanged = true; 
                break;
              case SWIPE_RIGHT: 
                ActiveSens = FindPrevPeriph(ActivePeer, ActiveSens, SENS_TYPE_SENS); 
                ScreenChanged = true; 
                break;
              case SWIPE_UP:    Mode = S_MENU; break;
              case SWIPE_DOWN:  Mode = S_MENU; break;
            }
            break;
          case S_SWITCH1: 
            switch (Touch.Gesture) {
              case CLICK:       ToggleSwitch(ActiveSwitch);  break;
              case SWIPE_LEFT:  ActiveSwitch = FindNextPeriph(ActiveSwitch, SENS_TYPE_SWITCH); ScreenChanged = true; break;
              case SWIPE_RIGHT: ActiveSwitch = FindPrevPeriph(ActiveSwitch, SENS_TYPE_SWITCH); ScreenChanged = true; break;
              case SWIPE_UP:    Mode = S_MENU; break;
              case SWIPE_DOWN:  Mode = S_MENU; break;
            }
            break;
          case S_MULTI    : 
            switch (Touch.Gesture) {
               case CLICK:       
                struct_Periph *Periph;
                Periph = Screen[ActiveMultiScreen].S[TouchQuarter()]; break;
                  
                if (isSwitch(Periph)) { 
                  ToggleSwitch(Periph);  
                }
                else if (isSensor(Periph)) {
                  ActivePeer   = Periph->Peer;
                  ActivePeriph = Periph;
                  Mode = S_SENSOR1;
                }
                break;
              case LONG_PRESS:
                PeriphToFill = TouchQuarter();
                Mode = S_PERI_SEL;
                break;
              case SWIPE_LEFT:  if  (ActiveMultiScreen < MULTI_SCREENS-1) ActiveMultiScreen++;
                                else ActiveMultiScreen = 0; 
                                break;
              case SWIPE_RIGHT: if  (ActiveMultiScreen == 0) ActiveMultiScreen = MULTI_SCREENS-1;
                                else ActiveMultiScreen--; 
                                break;
              case SWIPE_UP:    Mode = S_MENU; break;
              case SWIPE_DOWN:  Mode = S_MENU; break;
            }
            break;
          case S_PERI_SEL: 
            switch (Touch.Gesture) {
              case CLICK:       
                Screen[ActiveMultiScreen].PeriphId[PeriphToFill] = ActivePeriph->Id;
                Screen[ActiveMultiScreen].S[PeriphToFill] = ActivePeriph;
                ScreenChanged = true;
                ChangesSaved = false;
                Mode = S_MULTI;
              case SWIPE_LEFT:  
                ActivePeriph = FindNextPeriph(ActivePeer, ActivePeriph, SENS_TYPE_ALL); 
                ScreenChanged = true; 
                break;
              case SWIPE_RIGHT: 
                ActivePeriph = FindPrevPeriph(ActivePeer, ActivePeriph, SENS_TYPE_ALL); 
                ScreenChanged = true; 
                break;
              case SWIPE_UP:    Mode = S_MULTI; break;
              case SWIPE_DOWN:  Mode = S_MULTI; break;
            }
            break;
          case S_SETTING:
            switch (Touch.Gesture) {
              case CLICK:            if (ButtonHit( 5)) { ESP.restart(); }
                                else if (ButtonHit( 7)) { ReadyToPair = true; TSPair = millis(); Mode = S_PAIRING; }
                                else if (ButtonHit(11)) { if (DebugMode) SetDebugMode(false); else SetDebugMode(true); }
                                else if (ButtonHit(12)) { ScreenChanged = true; Mode = S_JSON; }
                                else if (ButtonHit(13)) { SavePeriphMulti(); ChangesSaved = true; ScreenChanged = true; }
                                break;
              case LONG_PRESS:  if (ButtonHit( 6)) { ClearPeers(); ESP.restart(); }
              case SWIPE_UP:    Mode = S_MENU; break;
              case SWIPE_DOWN:  Mode = S_MENU; break;
            } 
            break;
          case S_PEER    : 
            switch (Touch.Gesture) {
              case CLICK:            if (ButtonHit( 5))   SendCommand(ActivePeer, "Restart");
                                else if (ButtonHit( 7))   SendCommand(ActivePeer, "Pair");
                                else if (ButtonHit( 8)) { TSMsgEich = millis(); SendCommand(ActivePeer, "Eichen"); }
                                else if (ButtonHit( 9)) { Mode = S_CAL_VOL; ShowEichenVolt(); }
                                else if (ButtonHit(10))   SendCommand(ActivePeer, "SleepMode Toggle");
                                else if (ButtonHit(11))   SendCommand(ActivePeer, "DebugMode Toggle");
                                else if (ButtonHit(14))   SendCommand(ActivePeer, "DemoMode Toggle");
                                break;
              case LONG_PRESS:  if (ButtonHit( 6))        SendCommand(ActivePeer, "Reset");
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
              case CLICK:       for (int PNr=0 ; PNr<MAX_PEERS ; PNr++) {
                                  if (P[PNr].Type) {
                                    int Abstand = 20;
                                    if ((Touch.y1>80+PNr*Abstand) and (Touch.y1<80+(PNr+1)*Abstand)) {
                                      Serial.print("gefundene PNr: "); Serial.println(PNr);
                                      struct_Peer *TempPeer = &P[PNr];
                                      if (TempPeer) {
                                        ActivePeer = TempPeer;
                                        Mode = S_PEER;
                                        ScreenChanged = true;
                                        break;
                                      }
                                    }
                                  }
                                }
                                break;
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
          case S_CAL_VOL: 
            switch (Touch.Gesture) {
              case CLICK: AddVolt(CalcField()); break; 
              case SWIPE_LEFT:  Mode = S_MENU;  break;
              case SWIPE_RIGHT: Mode = S_MENU;  break;
              case SWIPE_UP:    Mode = S_MENU;  break;
              case SWIPE_DOWN:  Mode = S_MENU;  break;
            }
            break;
        }
      }
      TSTouch = millis();
      ScreenUpdate();  
    }
  }
  else if (millis() - TSMsgStart > LOGO_INTERVAL) { Mode = S_MENU; TSMsgStart = 0; }
}
void ScreenUpdate() {
  if (!TSMsgStart) {
    switch (Mode) {
      case S_PAIRING:   ShowPairing();  break;
      case S_SENSOR1:   ShowSingle(ActivePeer, ActiveSens);  break;          
      case S_SENSOR4:   ShowMulti(ActiveMultiScreen);  break;  
      case S_SWITCH1:   ShowSingle(ActivePeer, ActiveSwitch);  break;   
      case S_SWITCH4:   ShowMulti(ActiveMultiScreen);  break; 
      case S_MULTI:     ShowMulti(9);   break;
      case S_JSON:      ShowJSON();     break;  
      case S_SETTING:   ShowSettings(); break;
      case S_PEER:      ShowPeer();     break;
      case S_PEERS:     ShowPeers();    break;
      case S_PEER_SEL:  SelectPeer();   break;
      case S_PERI_SEL:  SelectPeriph(); break;
      case S_CAL_VOL:   ShowEichenVolt();   break;  
      case S_MENU:      ShowMenu();     break;        
    }
  }
  PushTFT();
}
#pragma endregion Main
#pragma region Peer-Things
void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  
  char Buf[50] = {}; String BufS;

  PeerCount = 0;

  for (int Pi=0; Pi< MAX_PEERS; Pi++) {
    if (P[Pi].Type > 0) {
      PeerCount++;
      //P.Type...
      sprintf(Buf, "Type-%d", Pi);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Type);
      if (preferences.getInt(Buf, 0) != P[Pi].Type) preferences.putInt(Buf, P[Pi].Type);
      
      //P.Id
      sprintf(Buf, "Id-%d", Pi);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Id);
      if (preferences.getInt(Buf, 0) != P[Pi].Id) preferences.putInt(Buf, P[Pi].Id);
      
      //P.BroadcastAddress
      sprintf(Buf, "MAC-%d", Pi);
      preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); PrintMAC(P[Pi].BroadcastAddress); Serial.println();
      
      //P.Name
      sprintf(Buf, "Name-%d", Pi);
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
  
  SavePeriphMulti();

  preferences.end();
}
void GetPeers() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[50] = {}; String BufS;
  
  PeerCount = 0;
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    // Peer gefüllt?
    sprintf(Buf, "Type-%d", Pi);
    Serial.print(Buf); Serial.print(" = "); Serial.println(preferences.getInt(Buf));
    if (preferences.getInt(Buf) > 0) {
      PeerCount++;

      // P.Type
      P[Pi].Type = preferences.getInt(Buf);
      if (ActivePeer == NULL) ActivePeer = &P[Pi];
      if (isPDC(&P[Pi]) and (ActivePDC == NULL)) ActivePDC = &P[Pi];
      if (isBat(&P[Pi]) and (ActiveBat == NULL)) ActiveBat = &P[Pi];
      
      // P.Id
      sprintf(Buf, "Id-%d", Pi);
      Serial.print(Buf); Serial.print(" = "); Serial.println(preferences.getInt(Buf));
      P[Pi].Id = preferences.getInt(Buf);
      
      P[Pi].PNumber = Pi;

      // P.BroadcastAdress
      sprintf(Buf, "MAC-%d", Pi);
      preferences.getBytes(Buf, P[Pi].BroadcastAddress, 6);
      
      // P.Name
      sprintf(Buf, "Name-%d", Pi);
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
      
      if (DebugMode) {
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
  GetPeriphMulti();

  preferences.end();
}
void SavePeriphMulti() {
  char Buf[30];
  
  Serial.println("Save PeriphMulti...");
  preferences.begin("JeepifyPeers", false);
  
  for (int s=0; s<MULTI_SCREENS; s++) {
    if (Screen[s].PeerId) {
      sprintf(Buf, "S%d-Name", s);
      if (preferences.getString(Buf,"") != Screen[s].Name)   preferences.putString(Buf, Screen[s].Name);
      sprintf(Buf, "S%d-Id", s);
      if (preferences.getInt(Buf,0) != Screen[s].Id)     preferences.putInt(Buf, Screen[s].Id);
      sprintf(Buf, "S%d-PeerId", s);
      if (preferences.getInt(Buf,0) != Screen[s].PeerId) preferences.putInt(Buf, Screen[s].PeerId);
      
      for (int p=0; p<PERIPH_PER_SCREEN; p++) {
        sprintf(Buf, "S%d-PeriphId%d", s, p);
        if (preferences.getInt(Buf,0) != Screen[s].PeriphId[p]) preferences.putInt(Buf, Screen[s].PeriphId[p]);
      }
    }  
  }
  preferences.end();
}
void GetPeriphMulti() {
  char Buf[30]; String BufS;

  preferences.begin("JeepifyPeers", true);
  
  for (int s=0; s<MULTI_SCREENS; s++) {
    sprintf(Buf, "S%d-Name", s);
    if (preferences.getString(Buf,"") != "") {
      BufS = preferences.getString(Buf);
      strcpy(Screen[s].Name, BufS.c_str());
      
      sprintf(Buf, "S%d-Id", s);
      Screen[s].Id = preferences.getInt(Buf,0);
      
      sprintf(Buf, "S%d-PeerId", s);
      Screen[s].PeerId = preferences.getInt(Buf,0);
      Screen[s].Peer = FindPeerById(Screen[s].PeerId);
      
      for (int p=0; p<PERIPH_PER_SCREEN; p++) {
        sprintf(Buf, "S%d-PeriphId%d", s, p);
        Screen[s].PeriphId[p] = preferences.getInt(Buf,0);
        Screen[s].S[p] = FindPeriphById(Screen[s].Peer, Screen[s].PeriphId[p]);
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
    if (DebugMode) {
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
void DeletePeer(struct_Peer *Peer) {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Id == Peer->Id) {
      P[PNr].Name[0] = 0;
      P[PNr].Id = 0;
      P[PNr].Type = 0;
      for (int i; i<6; i++) P[PNr].BroadcastAddress[i] = 0;
      P[PNr].TSLastSeen = 0;
      P[PNr].SleepMode = false;
      P[PNr].DebugMode = false;
    }
  }
  SendCommand(Peer, "Reset");
  
  for (int s=0; s<MULTI_SCREENS; s++) {
    for (int p=0; p<PERIPH_PER_SCREEN; p++) {
      if (Screen[s].S[p]->PeerId == Peer->Id) {
        Screen[s].S[p] = NULL;
        Screen[s].PeriphId[p] = 0;
      }
    } 
    if (Screen[s].PeerId == Peer->Id) Screen[s].PeerId = 0;
  }
}
void ClearInit() {
  preferences.begin("JeepifyInit", false);
    preferences.clear();
    Serial.println("JeepifyInit cleared...");
  preferences.end();
}
#pragma endregion Peer-Things
#pragma region Send-Things
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
bool ToggleSwitch(struct_Periph *Periph) {
   if (!Periph)) return false;
  StaticJsonDocument<500> doc;
  String jsondata;
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"]  = NODE_NAME;   
  doc["Order"] = "ToggleSwitch";
  doc["Value"] = Periph->Name;
  
  serializeJson(doc, jsondata);  
  
  esp_now_send(Periph->Peer->BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";
  return true;
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
#pragma endregion Send-Things
#pragma region Sensor-Screens
void ShowSingle(struct_Periph *Periph) {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (Periph->Changed)) {
    ScreenChanged = true;   
    OldMode = Mode;          

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);       
      
    switch(Periph->Type) {
      case SENS_TYPE_AMP:    RingMeter(0, 30, "Amp",  GREEN2RED); break;
      case SENS_TYPE_VOLT:   RingMeter(0, 15, "Volt", GREEN2RED); break;
      case SENS_TYPE_SWITCH: TFTBuffer.pushImage(0,0, 240, 240, Btn);
                             TFTBuffer.loadFont(AA_FONT_LARGE); TFTBuffer.drawString(Periph->Name,       120,130); TFTBuffer.unloadFont(); 
                             TFTBuffer.loadFont(AA_FONT_SMALL); TFTBuffer.drawString(Periph->Peer->Name, 120,160); TFTBuffer.unloadFont();
                             if      (Periph->Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
                             else if (Periph->Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);
                             break;
      default:               ShowMessage("No input"); break;
    }
    
    TSScreenRefresh = millis();
    noInterrupts(); 
      Periph->Changed = false;  
    interrupts();
  } 
}
void ShowMulti(struct_MultiScreen *Screen) {
  char Buf[20];
  float TempValue[PERIPH_PER_SCREEN];

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;     
    OldMode = Mode;        

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, false);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 
    TFTBuffer.loadFont(AA_FONT_SMALL);  

    int Si = 0;
    for (int Row=0; Row<MULTI_SCREEN_ROWS; Row++) {
      for (int Col=0; Col<MULTI_SCREEN_COLS; Col++) {
        if (Screen->S[Si]) {
          noInterrupts(); 
            TempValue[Si] = Screen->S[Si]->Value;  
          interrupts();
          if (isSwitch(Screen->S[Si])) {
            TFTBuffer.loadFont(AA_FONT_SMALL); 
            TFTGaugeSwitch.pushToSprite(&TFTBuffer, 22+Col*96, 25+Row*90, 0x4529);
            if (Screen->S[Si]->Value == 1) TFTBuffer.pushImage( 59+Col*96, 53+Row*90, 27, 10, BtnOn) ; 
            else                           TFTBuffer.pushImage( 59+Col*96, 53+Row*90, 27, 10, BtnOff);
            TFTBuffer.drawString(Screen->S[Si]->Name, 70+Col*96, 85+Row*90);
            TFTBuffer.unloadFont();
          }
          else if (isSensorAmp (Screen->S[Si])) {
            LittleGauge(TempValue[Si], 72+Col*96, 75+Row*90, 0, 35, 20, 30);
            
            TFTBuffer.loadFont(AA_FONT_MONO); 
            dtostrf(TempValue[Si], 0, 1, Buf);
            strcat(Buf, " A");
            TFTBuffer.drawString(Buf, 72+Col*96, 75+Row*90);
            TFTBuffer.unloadFont();
            
            TFTBuffer.loadFont(AA_FONT_SMALL); 
            TFTBuffer.drawString(Screen->S[Si]->Name,  72+Col*96, 105+Row*90);
            TFTBuffer.unloadFont();
          }
          else if (isSensorVolt(Screen->S[Si])) {
            LittleGauge(TempValue[Si], 72+Col*96, 75+Row*90, 0, 15, 13, 14);

            TFTBuffer.loadFont(AA_FONT_MONO); 
            dtostrf(TempValue[Si], 0, 1, Buf);
            strcat(Buf, " V");
            TFTBuffer.drawString(Buf, 72+Col*96, 75+Row*90);
            TFTBuffer.unloadFont();

            TFTBuffer.loadFont(AA_FONT_SMALL); 
            TFTBuffer.drawString(Screen->S[Si]->Name,  72+Col*96, 105+Row*90);
            TFTBuffer.unloadFont();
          }
        }  
        Si++;
      }
    }
    
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.drawString(Screen->Name, 120,15);
    TFTBuffer.unloadFont(); 
  }
  TSScreenRefresh = millis(); 
}
void ShowEichenVolt() {
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
#pragma endregion Sensor-Screens
#pragma region System-Screens
void ShowMessage(String Msg) {
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
void ShowPairing() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;             
    OldMode = Mode;

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 

    TFTBuffer.drawString("Pairing...", 120, 120);
    if (!ReadyToPair) Mode = S_PEERS;
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
    DrawButton(11, DebugMode);
    DrawButton(12);
    DrawButton(13, !ChangesSaved);

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
    DrawButton(10, ActivePeer->SleepMode);
    DrawButton(14, ActivePeer->DemoMode);

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
  SetMsgIndicator();
  if (ScreenChanged) {
    UpdateCount++;
    TFTBuffer.pushSprite(0, 0);
    ScreenChanged = false;
  }
}
void SetMsgIndicator() {
  if (DebugMode) {
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
#pragma endregion System-Screens
#pragma region Peer/Periph-Checks
bool isPDC(struct_Peer *Peer) {
  if (Peer) return ((Peer->Type == SWITCH_1_WAY) or (Peer->Type == SWITCH_2_WAY) or 
                    (Peer->Type == SWITCH_4_WAY) or (Peer->Type == SWITCH_8_WAY) or
                    (Peer->Type == PDC_SENSOR_MIX));   
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
  if (Peer) {
    if (Stop == 0) Stop = Start;
    for (int Si=Start; Si++; Si<Stop+1) {
      if (Si == MAX_PERIPHERALS) break;
      if (Peer->S[Si].Changed) ret = true;
    }
  }
  return ret;
}
struct_Periph *FindPeriphById(struct_Peer *Peer, uint16_t Id) {
  if (Peer) {
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
      if (Peer->S[SNr].Id == Id) return &Peer->S[SNr];
    }
  }
  return NULL;
}
struct_Periph *FindFirstPeriph(struct_Peer *Peer, int Type, bool OnlyActual){
  if (!Peer) Peer = FindFirstPeer();
   if (Peer) {
    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
        switch (Type) {
          case SENS_TYPE_ALL:    if (Peer->S[SNr].Type > 0)       { ActivePeer = Peer; return &Peer->S[SNr]; break; }
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
struct_Periph *FindNextPeriph (struct_Periph *Periph, int Type, bool OnlyActual){
  struct_Peer *Peer = Periph->Peer;     
  if ((Periph) and (Peer)) {
    struct_Peer *Peer = Periph->Peer; 

    if (Type == SENS_TYPE_EQUAL) Type = Periph->Type;

    int SNr = Periph->Id;
    int Id  = Periph->Id;

    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int i=0; i<MAX_PERIPHERALS; i++) {
        SNr++; if (SNr == MAX_PERIPHERALS) SNr = 0;
        char Buf[100];
        sprintf(Buf, "Vergleiche %s, %s (Id=%d) mit %s, %s (Id(%d)...", Peer->Name, Peer->S[SNr].Name, Peer->S[SNr].Id, ActivePeer->Name, ActivePeer->S[Id].Name, ActivePeer->S[Id].Id);
        switch (Type) {
          case SENS_TYPE_ALL:
            if (Peer->S[SNr].Id > Id) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
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
struct_Periph *FindPrevPeriph (struct_Periph *Periph, int Type, bool OnlyActual){
  struct_Peer *Peer = Periph->Peer;     
  if ((Periph) and (Peer)) {
    if (Type == SENS_TYPE_EQUAL) Type = Periph->Type;
    
    int SNr = Periph->Id;
    int Id  = Periph->Id;

    for (int Pi=0; Pi<MAX_PEERS; Pi++) {
      for (int i=0; i<MAX_PERIPHERALS; i++) {
        SNr--; if (SNr == -1) SNr = MAX_PERIPHERALS-1;
        switch (Type) {
          case SENS_TYPE_ALL:
            if (Peer->S[SNr].Id < Id) { ActivePeer = Peer; return &Peer->S[SNr]; break; }
            break;
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
  if (!ActivePeer) {
    ActivePeer = FindFirstPeer();
  }
  //Serial.print("ActivePeer: "); Serial.println(ActivePeer->Name);
  if (ActivePeer) {
    if (!ActivePeriph) {
      ActivePeriph = FindFirstPeriph(ActivePeer, SENS_TYPE_ALL, false);
      //Serial.print("ActivePeriph: "); Serial.println(ActivePeriph->Name);
    }
    if (ActivePeriph) {
      if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
        ScreenChanged = true;
        TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);
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
struct_Peer   *SelectPeer  () {
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
struct_Peer *FindPeerById  (uint16_t Id) {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Id == Id) return &P[PNr];
  }
  return NULL;
}
struct_Peer *FindPeerByMAC (const uint8_t *MAC) {
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
struct_Peer *FindEmptyPeer () {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type == 0) return &P[PNr];
  }
  return NULL;
}
struct_Peer *FindFirstPeer (int Type) {
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type == Type) return &P[PNr];
    if ((P[PNr].Type) and (Type == MODULE_ALL)) return &P[PNr];
  }
  return NULL;
}
struct_Peer *FindNextPeer  (struct_Peer *Peer, int Type) {
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
struct_Peer *FindPrevPeer  (struct_Peer *Peer, int Type) {
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
int          FindHighestPeerId() {
  int HighestId = -1;
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Id > HighestId) HighestId = P[PNr].Id;
  }
  return HighestId;
}
#pragma endregion Peer/Periph-Checks
#pragma region Touch-Things
int  TouchQuarter(void) {
  if ((Touch.x1<120) and (Touch.y1<120)) return 0;
  if ((Touch.x1>120) and (Touch.y1<120)) return 1;
  if ((Touch.x1<120) and (Touch.y1>120)) return 2;
  if ((Touch.x1>120) and (Touch.y1>120)) return 3;
  return NOT_FOUND;
}
int  TouchedField(void) {
  for (int Row=0; Row<MULTI_SCREEN_ROWS; Row++) {
    for (int Col=0; Col<MULTI_SCREEN_COLS; Col++) {
      if ((Touch.x1<(TFT.width/MULTI_SCREEN_ROW)) and (Touch.y1<120)) return 0;    
    }

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
#pragma endregion Touch-Things
#pragma region Gauges
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
      if (abs(value < SCHWELLE)) value = 0;
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
    
    if (DebugMode) TFTBuffer.drawString(ActivePeer->Name, 120,225);

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

    if (value == -99) strcpy(buf, "--"); 
    else dtostrf(value, 0, nk, buf);

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
void LittleGauge(float Value, int x, int y, int Min, int Max, int StartYellow, int StartRed) {
  char Buf[100];

  int R1 = 42;
  int R2 = 35;
  int StartAngle   =  60;
  int EndAngle     = 300;
  int Range = Max-Min;
  
  //LittleGauge(TempValue[Si], 72+Col*120, 75+Row*120, 0, 35, 20, 30);
          
  float ToGo = StartAngle + (EndAngle-StartAngle)/Range*(Value-Min);

  float YellowAngle = StartAngle + (EndAngle-StartAngle)/Range*StartYellow;
  float RedAngle    = StartAngle + (EndAngle-StartAngle)/Range*StartRed;
  
  //sprintf(Buf, "Value:%f, Range:%d, ToGo:%d, StartYellow:%d(%d), StartRed:%d(%d)", Value, Range, (int)ToGo, StartYellow, (int)YellowAngle, StartRed, (int)RedAngle);
  //Serial.println(Buf);

  TFTBuffer.drawSmoothArc(x, y, R1, R2, ToGo, EndAngle, TFT_DARKGREY, 0x18E3, false);
  if  (ToGo < YellowAngle) TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)StartAngle, (int)ToGo,        TFT_GREEN,  0x18E3, false);
  if ((ToGo >= YellowAngle) and (ToGo < RedAngle)) {
                           TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)StartAngle,  (int)YellowAngle, TFT_GREEN,  0x18E3, false);
                           TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)YellowAngle, (int)ToGo,        TFT_YELLOW, 0x18E3, false);
  }
  if (ToGo >= RedAngle)  { TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)StartAngle,  (int)YellowAngle, TFT_GREEN,  0x18E3, false);
                           TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)YellowAngle, (int)RedAngle,    TFT_YELLOW, 0x18E3, false);
                           TFTBuffer.drawSmoothArc(x, y, R1, R2, (int)RedAngle,    (int)ToGo,        TFT_RED,    0x18E3, false);
  }         
}
#pragma endregion Gauges
#pragma region Other
void SetSleepMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    SleepMode = Mode;
    if (preferences.getBool("SleepMode", false) != SleepMode) preferences.putBool("SleepMode", SleepMode);
  preferences.end();
}
void SetDebugMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    DebugMode = Mode;
    if (preferences.getBool("DebugMode", false) != DebugMode) preferences.putBool("DebugMode", DebugMode);
  preferences.end();
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
int  CalcField() {
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
      if ((Touch.x1>x1 and Touch.x1<x2) and (Touch.y1>y1 and Touch.y1<y2)) {
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
  
    esp_now_send(ActivePeer->BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    Serial.println(jsondata);
  
    Serial.println(jsondata);

    TSMsgVolt = millis();
    Mode = S_MENU;
  }
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
#pragma endregion Other
