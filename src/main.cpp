#include <Arduino.h>
#include "../../jeepify.h"
#include <TFT_eSPI.h>
//#include <SPI.h>
//#include <SPIFFS.h>
//#include "../../renegade_members.h"
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

#define VERSION   "V 0.01"

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

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);

void   DrawButton(int z);
bool   ButtonHit(int b);
int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();

void   ToggleSwitch (int Si);
void   SendCommand(String Cmd);

void   SetMsgIndicator();
void   ScreenUpdate();
void   PushTFT();
void   ShowMessage(char *Msg);
void   ShowSwitch1();
void   ShowSwitch4();
void   ShowSensor1();
void   ShowSensor4();
void   ShowMenu();
void   ShowJSON();
void   ShowPeer();
void   ShowPeers();
void   ShowPairingScreen();

int    RingMeter(float vmin, float vmax, const char *units, byte scheme);

bool   SensorChanged(int Start, int Stop=99);
bool   isPDC(int Type);
bool   isBat(int Type);
bool   isSwitch(int Type);
bool   isSensor(int Type);
bool   isSensorAmp (int Type);
bool   isSensorVolt(int Type);
int    NextSensor();
int    PrevSensor();
int    NextSwitch();
int    PrevSwitch();
int    NextBat();
int    PrevBat();
int    NextPDC();
int    PrevPDC();
int    NextPeer();
int    PrevPeer();

int    TouchQuarter(void);
int    TouchRead();

void   PrintMAC(const uint8_t * mac_addr);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
unsigned int rainbow(byte value);

struct_Touch  Touch;
struct_Peer   P[MAX_PEERS];
struct_Button Button[12] = {
  { 25, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Volt",      false},   // 0
  { 95, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "Amp",       false},   // 1
  {165, 150, 50, 30, TFT_RUBICON, TFT_BLACK, "JSON",      false},   // 2
  { 60, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "DBG",       false},   // 3
  {130, 190, 50, 30, TFT_RUBICON, TFT_BLACK, "Pair",      false},   // 4
  { 35,  25, 70, 30, TFT_WHITE,   TFT_BLACK, "Restart",   false},   // 5
  {135,  25, 70, 30, TFT_WHITE,   TFT_BLACK, "Reset",     false},   // 6
  { 35,  75, 70, 30, TFT_WHITE,   TFT_BLACK, "Pair",      false},   // 7
  {135,  75, 70, 30, TFT_WHITE,   TFT_BLACK, "Eichen",    false},   // 8
  { 35, 125, 70, 30, TFT_WHITE,   TFT_BLACK, "VoltCalib", false},   // 9
  {135, 125, 70, 30, TFT_WHITE,   TFT_BLACK, "Sleep",     false},   // 10
  {135, 125, 70, 30, TFT_WHITE,   TFT_BLACK, "Debug",     false}    // 11
};

Preferences preferences;

int ActivePeer   = -1;
int ActiveSens   = -1;
int PeerCount    =  0;
int Mode         =  S_MENU;
int OldMode      = 99;
int UpdateCount  =  0;

bool ScreenChanged = true;
bool AvailPDC = false;
bool AvailBat = false;
bool Debug = true;
bool SleepMode = false;
bool ReadyToPair = false;

int   ButtonRd   = 22;
int   ButtonGapX = 6;
int   ButtonGapY = 6;
float VoltCalib;
int   VoltCount;

float TempValue[8] = {0,0,0,0,0,0,0,0};

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

StaticJsonDocument<300> doc;
String jsondata;

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
  //ReportPeers();
  Serial.println("RegisterPeers...");
  RegisterPeers();
  Serial.println("RegisterPeers fertig...");

  if (PeerCount == 0) { Serial.println("PeerCount=0 -RTP+"); ReadyToPair = true; TSPair = millis(); Mode = S_PAIRING;}
  
  Mode = 1;
  TSScreenRefresh = millis();
  TSTouch = millis();
}
void loop() {
  /*if ((TSMsgStart) and (millis() - TSMsgStart > LOGO_INTERVAL)) {
    Mode = S_MENU;
    TSMsgStart = 0;
  }*/
  if (millis() - TSPing > PING_INTERVAL) {
    TSPing = millis();
    SendPing();
    Serial.println("Ping fertig");
  }
  
  if (millis() - TSTouch > TOUCH_INTERVAL) {
    //Serial.println("Touch-Intervall");
    
    int TouchErg = TouchRead();
    /*Serial.print("TouchRead()="); Serial.print(TouchErg);
    Serial.print(" --- x0="); Serial.print(Touch.x0);
    Serial.print(", y0="); Serial.print(Touch.y0);
    Serial.print(", x1="); Serial.print(Touch.x1);
    Serial.print(", y1="); Serial.print(Touch.y1);
    Serial.print(", Gesture="); Serial.println(Touch.Gesture);
    */
    if (TouchErg > 1) {
      Serial.print("Touch:"); Serial.println(Touch.Gesture);
      //S_MENU
      switch (Mode) {
        case S_MENU    : 
          switch (Touch.Gesture) {
            case CLICK:       
              switch (TouchQuarter()) {
                case 0: Mode = S_SENSOR1; break;
                case 1: Mode = S_SWITCH1; break;
                case 2: Mode = S_SWITCH4; break;
                case 3: Mode = S_JSON;    break;
              } 
              break;
            case SWIPE_LEFT:  Mode = S_PEERS; break;
            case SWIPE_RIGHT: Mode = S_PEER;  break;
            case SWIPE_UP:    break;
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
            case SWIPE_DOWN:  Mode = S_MENU; break;
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
          switch (Touch.Gesture) {
            case CLICK:       Mode = S_SENSOR1; ActiveSens = TouchQuarter(); break;
            case SWIPE_LEFT:  ActivePeer = NextBat(); break; //Sensor oder Peer +
            case SWIPE_RIGHT: ActivePeer = PrevBat(); break; //Sensor oder Peer -
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_PEER    : 
          switch (Touch.Gesture) {
            case CLICK:            if (ButtonHit( 5)) SendCommand("Restart");
                              else if (ButtonHit( 6)) SendCommand("Reset");
                              else if (ButtonHit( 7)) SendCommand("Pair");
                              else if (ButtonHit( 8)) SendCommand("Eichen");
                              else if (ButtonHit( 9)) SendCommand("VoltCalib");
                              else if (ButtonHit(10)) SendCommand("SleepMode On");
                              else if (ButtonHit(11)) SendCommand("Debug On");
                              break;
            case SWIPE_LEFT:  ActivePeer = NextPeer(); break; //Sensor oder Peer +
            case SWIPE_RIGHT: ActivePeer = PrevPeer(); break; //Sensor oder Peer -
            case SWIPE_UP:    Mode = S_MENU; break;
            case SWIPE_DOWN:  Mode = S_MENU; break;
          }
          break;
        case S_PEERS   : 
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
      }
    }
  TSTouch = millis();
  ScreenUpdate();  
  }
}
void ScreenUpdate() {
  if (!TSMsgStart) {
    switch (Mode) {
      case S_PAIRING: ShowPairingScreen(); break;
      case S_SENSOR1: 
          if (isSensorAmp (P[ActivePeer].S[ActiveSens].Type)) {
            RingMeter(0, 30, "Amp", GREEN2RED);
          }
          else if (isSensorVolt(P[ActivePeer].S[ActiveSens].Type)) {
            RingMeter(0, 16, "Volt", GREEN2RED);
          }
          else ShowMessage("No Sensor");
        break;          
      case S_SENSOR4:   ShowSensor4();  break;  
      case S_SWITCH1:   ShowSwitch1();  break;
      case S_SWITCH4:   ShowSwitch4();  break;
      
      case S_JSON:      ShowJSON();    break;  
      case S_PEER:      ShowPeer();    break;
      case S_PEERS:     ShowPeers();   break;
      case S_CAL_VOL:   EichenVolt();  break;  
      case S_MENU:      ShowMenu();    break;        
    }
  }
  PushTFT();
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char* buff = (char*) incomingData;   
  String BufS;

  jsondata = "";  jsondata = String(buff);                 
  Serial.print("Recieved from:"); PrintMAC(mac); Serial.println(jsondata);    
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    TSMsgRcv = millis();
    bool NodeBekannt = false;
    int PNr = 0;

    for (PNr=0; PNr<MAX_PEERS; PNr++) { 
      if (doc["Node"] == P[PNr].Name) {
        NodeBekannt = true; 
        P[PNr].TSLastSeen = millis();
        break; 
      }
    }    
    if (NodeBekannt) {      
      if (isBat(P[PNr].Type)) TSMsgBat = TSMsgRcv;
      if (isPDC(P[PNr].Type)) TSMsgPDC = TSMsgRcv;
      
      for (int i=0; i<MAX_PERIPHERALS; i++) {
        if (doc.containsKey(P[PNr].S[i].Name)) {
          float TempSensor = (float)doc[P[PNr].S[i].Name];
      
          if (TempSensor != P[PNr].S[i].Value) {
            P[PNr].S[i].OldValue = P[PNr].S[i].Value;
            P[PNr].S[i].Value = TempSensor;
            P[PNr].S[i].Changed = true;
          }
        }
      } 
    } 
    else if ((ReadyToPair) and (doc.containsKey("Pairing"))) {
      bool PairingSuccess = false;
      char Buf[10] = {};
      char BufB[5] = {};

      if (doc["Pairing"] == "add me") { 
        //for (int i = 0; i < 6; i++ ) TempBroadcast[i] = (uint8_t) mac[i];
    
        bool exists = esp_now_is_peer_exist(mac);
        if (exists) { 
          PrintMAC(mac); Serial.println(" already exists...");
          PairingSuccess = true;
          
          jsondata = "";  doc.clear();
          
          doc["Node"]    = NODE_NAME;   
          doc["Pairing"] = "you are paired";

          serializeJson(doc, jsondata);  
          esp_now_send(P[PNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
          Serial.print("Sending you are paired"); 
          Serial.println(jsondata);
        }
        else { // neuer Peer
          for (PNr=0; PNr<MAX_PEERS; PNr++) {
            if ((P[PNr].Type == 0) and (!PairingSuccess)) {
              
              for (int b = 0; b < 6; b++ ) P[PNr].BroadcastAddress[b] = mac[b];
              strcpy(P[PNr].Name, doc["Node"]);
              P[PNr].Type = doc["Type"];
              P[PNr].TSLastSeen = millis();
              
              for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
                sprintf(BufB, "%d", Si); 
                strcpy(Buf, "S"); strcat(Buf, BufB);
                if (doc.containsKey(Buf)) strcpy(P[PNr].S[Si].Name, doc[Buf]);
              }   

              esp_now_peer_info_t peerInfo;
              peerInfo.channel = 0;
              peerInfo.encrypt = false;
              memset(&peerInfo, 0, sizeof(peerInfo));

              for (int ii = 0; ii < 6; ++ii ) peerInfo.peer_addr[ii] = (uint8_t) mac[ii];
              if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
                Serial.println("Failed to add peer");
              }
              else { 
                Serial.println(P[PNr].Name); 
                PrintMAC(mac); Serial.println(" peer added...");
                
                PairingSuccess = true; 
                jsondata = "";  doc.clear();
                
                doc["Node"]    = NODE_NAME;   
                doc["Pairing"] = "you are paired";
                doc["Type"]    = MONITOR_ROUND;

                serializeJson(doc, jsondata);  
                esp_now_send(P[PNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
                Serial.print("Sending you are paired"); 
                Serial.println(jsondata);
              }
            }
          }
          if (!PairingSuccess) { PrintMAC(mac); Serial.println(" adding failed..."); } 
          else  {
            Serial.println("Saving Peers...");
            SavePeers();
          }
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
    Serial.print("getInt("); Serial.print(Buf); Serial.print(" = "); Serial.println(preferences.getInt(Buf));
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
        Serial.println();
      }
    }
  }
  if (PeerCount) { ActivePeer = 0; ActiveSens = 0; }
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
void SendPing() {
  jsondata = "";  
  doc.clear();
  
  doc["Node"] = NODE_NAME;   
  doc["Order"] = "stay alive";

  if (ReadyToPair) {
    doc["Pairing"] = "aktiv";
  }

  serializeJson(doc, jsondata);  
  for (int PNr=0; PNr<MAX_PEERS; PNr++) esp_now_send(P[PNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
  Serial.print("Sending Ping:"); 
  Serial.println(jsondata);   

  Serial.print("Mode="); Serial.print(Mode);
  Serial.print(", OldMode="); Serial.println(OldMode);
     
}
void DrawButton(int z) {
  TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
  TFTBuffer.setTextDatum(MC_DATUM);

  if (Button[z].Status) TFTBuffer.fillSmoothRoundRect(Button[z].x, Button[z].y, Button[z].w, Button[z].h, 10, Button[z].TxtColor, Button[z].BGColor);
  TFTBuffer.drawSmoothRoundRect(Button[z].x, Button[z].y, 10, 8, Button[z].w, Button[z].h, Button[z].TxtColor, Button[z].BGColor);
  TFTBuffer.drawString(Button[z].Name, Button[z].x+Button[z].w/2, Button[z].y+Button[z].h/2);  

   TFTBuffer.unloadFont();
}
bool ButtonHit(int b) {
  return ( ((Touch.x1>Button[b].x) and (Touch.x1<Button[b].x+Button[b].w) and 
            (Touch.y1>Button[b].y) and (Touch.y1<Button[b].y+Button[b].h)) );
}
void ShowSensor1() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(ActiveSens))) {
    ScreenChanged = true;   
    OldMode = Mode;          

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);       
        
    switch(P[ActivePeer].S[ActiveSens].Type) {
      case SENS_TYPE_AMP:  RingMeter(0, 30, "Amp",  GREEN2RED); break;
      case SENS_TYPE_VOLT: RingMeter(0, 15, "Volt", GREEN2RED); break;
    }
    
    TSScreenRefresh = millis();

    P[ActivePeer].S[ActiveSens].OldValue = P[ActivePeer].S[ActiveSens].Value; 
  } 
}
void ShowSensor4() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(0,3))) {
    ScreenChanged = true;  
    OldMode = Mode;           

    noInterrupts(); 
      for (int Si=0; Si<4; Si++) TempValue[Si] = P[ActivePeer].S[Si].Value;  
    interrupts();

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
          dtostrf(TempValue[Si], 0, 2, Buf);
          TFTBuffer.drawString(Buf, 60+Col*120, 90+Row*120);
          TFTBuffer.unloadFont();
        }
        if (isSensorVolt(P[ActivePeer].S[Si].Type)) {
          TFTBuffer.loadFont(AA_FONT_LARGE); 
          TFTBuffer.drawString(P[ActivePeer].S[Si].Name,  60+Col*120, 30+Row*120);
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
    TFTBuffer.drawString(P[ActivePeer].Name, 120,15);
    TFTBuffer.unloadFont(); 
    
    noInterrupts(); 
      for (int Si=0; Si<4; Si++) P[ActivePeer].S[Si].Changed = false;
    interrupts();

    TSScreenRefresh = millis(); 
  }
}
void ShowMessage(char *Msg) {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;             
    OldMode = Mode;

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3);
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground); 

    TFTBuffer.drawString(Msg, 120, 120);
    TSScreenRefresh = millis(); 
  }
}
void ShowSwitch1() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(ActiveSens))) {
    ScreenChanged = true;
    OldMode = Mode;
  
    TFTBuffer.pushImage(0,0, 240, 240, Btn);
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(P[ActivePeer].S[ActiveSens].Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(P[ActivePeer].Name, 120,160);
    TFTBuffer.unloadFont();

         if (P[ActivePeer].S[ActiveSens].Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (P[ActivePeer].S[ActiveSens].Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TSScreenRefresh = millis();

    //P[ActivePeer].S[SNr].OldValue = P[ActivePeer].S[ActiveSens].Value;
    noInterrupts(); 
      P[ActivePeer].S[ActiveSens].Changed = false;  
    interrupts(); 
  } 
}
void ShowSwitch4() {
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (SensorChanged(0,3))) {
    ScreenChanged = true;     
    OldMode = Mode;        

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

    TSScreenRefresh = millis(); 
}
void ShowPairingScreen() {
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
  if (((TSScreenRefresh - millis() > SCREEN_INTERVAL) and (jsondata != "") or (Mode != OldMode))) {
    ScreenChanged = true;
    OldMode = Mode;

    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString("Debug JSON", 120, 200); 
  
    int sLength = jsondata.length();

    if (sLength) {
      int len = 20;
      int Abstand = 20;
      int sLength = jsondata.length();

      TFTBuffer.setTextDatum(MC_DATUM);
      TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    
      for (int i=0 ; i<8 ; i++) {
        if ((i+1)*len < sLength)  TFTBuffer.drawString(jsondata.substring(i*len, (i+1)*len), 120,50+i*Abstand); 
        else if (i*len < sLength) TFTBuffer.drawString(jsondata.substring(i*len, sLength), 120,50+i*Abstand); 
      }
      jsondata = "";
    }
    TSScreenRefresh = millis();
  }
}
void ShowPeer() {
  if (ActivePeer != LEER) { 
    if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
      OldMode = Mode;
      ScreenChanged = true;

      TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
      TFTBuffer.loadFont(AA_FONT_SMALL);
      
      TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
      TFTBuffer.setTextDatum(TC_DATUM);
      
      TFTBuffer.drawString(P[ActivePeer].Name, 120, 200); 

      DrawButton(5);
      DrawButton(6);
      DrawButton(7);
      DrawButton(8);
      DrawButton(9);
      DrawButton(10);

      TSScreenRefresh = millis();
    }
  }
  else { ShowMessage("No Peer"); }
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
          case 1: ZName = "1-way PDC";  break;
          case 2: ZName = "2-way PDC";  break;
          case 4: ZName = "4-way PDC";  break;
          case 8: ZName = "8-way PDC";  break;
          case 9: ZName = "Batt.-Sens."; break;
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
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TSScreenRefresh = millis();
  }
}
void PushTFT() {
  SetMsgIndicator();
  
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
void SendCommand(String Cmd) {
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"] = NODE_NAME;   
  doc["Order"] = Cmd;
  
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
bool SensorChanged(int Start, int Stop) {
  int ret = false;
  if (Stop == 99) Stop = Start;
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
int  NextPeer() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr++;
    if (PNr == MAX_PEERS) PNr = 0;
    if (P[PNr].Type != 0) return PNr;
  }
  return ActivePeer;
}
int  PrevPeer() {
  int PNr = ActivePeer;
  
  for (int i=0; i<MAX_PEERS; i++) {
    PNr--;
    if (PNr == -1) PNr = MAX_PEERS-1;
    if (P[PNr].Type != 0) return PNr;
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
  
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or SensorChanged(ActiveSens)) {
    ScreenChanged = true;
    OldMode = Mode;
      
    noInterrupts(); 
      float value = P[ActivePeer].S[ActiveSens].Value; 
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
    
    if (Debug) TFTBuffer.drawString(P[ActivePeer].Name, 120,225);

    TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    TFTBuffer.drawString(units, 120,150); // Units display
    TFTBuffer.setTextDatum(BC_DATUM);
    TFTBuffer.drawString(P[ActivePeer].S[ActiveSens].Name, 120,90); // Units display

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
    P[ActivePeer].S[ActiveSens].Changed = false; 
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
    String jsondata;
    jsondata = "";  //clearing String after data is being sent
    doc.clear();
  
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
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
unsigned int rainbow(byte value)
{
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