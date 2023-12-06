#include <Arduino.h>
#include "../../renegade_members.h"
#include <TFT_eSPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "CST816D.h"
#include <Preferences.h>
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#include "NotoSansMonoSCB20.h"

#include <math.h>
#include "pix.h"

#define NAME_NODE "Jeep_Monitor_1"
#define VERSION   "V 3.01"

struct struct_Peripheral {
    char        Name[20];
    int         Type;      //1=Switch, 2=Amp, 3=Volt
    // Sensor
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
    struct_Peripheral S[MAX_PERIPHERALS];
};
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

void  PrintMAC(const uint8_t * mac_addr);

StaticJsonDocument<300> doc;
String jsondata;

u_int8_t TempBroadcast[6];

int PeerCount = 0;

bool Debug         = true;
bool SleepMode     = true;
bool ReadyToPair   = false;

bool ScreenChanged = true;
int  UpdateCount   = 0;

int   Mode    = S_LOGO;                 //StartMode
int   OldMode = 99;

int ActivePeer = 0;

struct_Peer P[MAX_PEERS];

bool      TouchContact;
bool      Touched;
uint8_t   Gesture;
uint16_t  TouchX, TouchY;

volatile uint32_t TSMsgRcv  = 0;
volatile uint32_t TSMsgPDC  = 0;
volatile uint32_t TSMsgBat  = 0;
volatile uint32_t TSMsgVolt = 0;
volatile uint32_t TSMsgEich = 0;
volatile uint32_t TSPair    = 0;
uint32_t TSMsgStart         = 0;
uint32_t TSPing             = 0;
uint32_t  TSTouch           = 0;
uint32_t TSScreenRefresh    = 0;

bool MsgBatAktiv  = false;
bool MsgPDCAktiv  = false;
bool MsgVoltAktiv = false;
bool MsgEichAktiv = false;
bool MsgPairAktiv = false;

Preferences preferences;

int   ButtonRd   = 22;
int   ButtonGapX = 6;
int   ButtonGapY = 6;
float VoltCalib;
int   VoltCount;

TFT_eSPI TFT              = TFT_eSPI();
TFT_eSprite TFTBuffer     = TFT_eSprite(&TFT);
TFT_eSprite TFTGauge1     = TFT_eSprite(&TFT);

CST816D Touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

void InitModule() {
  preferences.begin("JeepifyInit", true);
  Debug     = preferences.getBool("Debug", true);
  SleepMode = preferences.getBool("SleepMode", false);
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
      
      //P.BroadcastAdress 
      strcpy(Buf, "MAC"); strcat (Buf, BufNr);
      preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      Serial.print("MAC geschrieben: "); preferences.getBytes(Buf, TempBroadcast, 6); PrintMAC(TempBroadcast);
    }
  }
  if (preferences.getInt("PeerCount") != PeerCount) preferences.putInt("PeerCount", PeerCount);

  preferences.end();
}
void GetPeers() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[10] = {}; char BufNr[5] = {}; char BufB[5] = {}; String BufS;
  
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
      
      P[Pi].TSLastSeen = millis();
      
      // P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = preferences.getString(Buf);
      strcpy(P[Pi].Name, BufS.c_str());
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

void  PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

bool isSwitch(int Type) {
  return ((Type == SWITCH_1_WAY) or (Type == SWITCH_2_WAY) or (Type == SWITCH_4_WAY) or (Type == SWITCH_8_WAY));      
}
bool isSensor(int Type) {
  return(Type == BATTERY_SENSOR);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char* buff = (char*) incomingData;   
  String BufS;

  jsondata = "";  jsondata = String(buff);                 
  Serial.print("Recieved from:"); PrintMAC(mac); Serial.println(jsondata);    
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    TSMsgRcv = millis();
    //bool Paired = false;
    bool NodeBekannt = false;
    int zNr = 0;

    for (zNr=0; zNr<MAX_ZUBEHOER; zNr++) { 
      if (doc["Node"] == P[zNr].Name) {
        NodeBekannt = true; 
        P[zNr].TSLastSeen = millis();
        break; 
      }
    }    
    if      ((NodeBekannt)  and (isSensor(P[zNr].Type))) {      
      TSMsgBat = TSMsgRcv;
      for (int i=0; i<ANZ_SENSOR; i++) {
        if (doc.containsKey(P[zNr].S[i].Name)) {
          float TempSensor = (float)doc[P[zNr].S[i].Name];
      
          if (TempSensor != P[zNr].S[i].Value) {
            P[zNr].S[i].OldValue = P[zNr].S[i].Value;
            P[zNr].S[i].Value = TempSensor;
            P[zNr].S[i].Changed = true;
          }
        }
      } 
    }
    else if ((NodeBekannt)  and (isSwitch(P[zNr].Type))) {
      TSMsgPDC = TimestampMsgRcv;
      for (int i=0; i<ANZ_SWITCH; i++) {
        if (doc.containsKey(P[zNr].S[i].Name)) {
          int TempSwitch = (int)doc[P[zNr].S[i].Name];
      
          if (TempSwitch != P[zNr].S[i].Value) {
            P[zNr].S[i].OldValue = P[zNr].S[i].Value;
            P[zNr].S[i].Value = TempSwitch;
            P[zNr].S[i].Changed = true;
          }
        }
      }
    }     
    else if ((ReadyToPair) and (doc.containsKey("Pairing"))) {
      bool PairingSuccess = false;
      char Buf[10] = {};
      char BufB[5] = {};

      if (doc["Pairing"] == "add me") { 
        for (int i = 0; i < 6; i++ ) TempBroadcast[i] = (uint8_t) mac[i];
    
        bool exists = esp_now_is_peer_exist(TempBroadcast);
        if (exists) { 
          printMAC(TempBroadcast); Serial.println(" already exists...");
          PairingSuccess = true;
          
          jsondata = "";  doc.clear();
          
          doc["Node"]    = NAME_NODE;   
          doc["Pairing"] = "you are paired";

          serializeJson(doc, jsondata);  
          esp_now_send(P[zNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
          Serial.print("Sending you are paired"); 
          Serial.println(jsondata);
        }
        else {
          for (zNr=0; zNr<MAX_ZUBEHOER; zNr++) {
            if ((P[zNr].Type == 0) and (!PairingSuccess)) {
              
              for (int b = 0; b < 6; b++ ) P[zNr].BroadcastAddress[b] = TempBroadcast[b];
              strcpy(P[zNr].Name, doc["Node"]);
              P[zNr].Type = doc["Type"];
              P[zNr].TSLastSeen = millis();
              
              if (isSwitch(P[zNr].Type)) {
                for (int b=0; b<ANZ_SWITCH; b++) {
                  sprintf(BufB, "%d", b); 
                  strcpy(Buf, "SW"); strcat(Buf, BufB);
                  if (doc.containsKey(Buf)) strcpy(P[zNr].S[b].Name, doc[Buf]);
                }   
              }
              else if (isSensor(P[zNr].Type)) {
                for (int b=0; b<ANZ_SENSOR; b++) {
                  sprintf(BufB, "%d", b); 
                  strcpy(Buf, "S"); strcat(Buf, BufB);
                  if (doc.containsKey(Buf)) strcpy(P[zNr].S[b].Name, doc[Buf]);
                }   
              }

              esp_now_peer_info_t peerInfo;
              peerInfo.channel = 0;
              peerInfo.encrypt = false;
              memset(&peerInfo, 0, sizeof(peerInfo));

              for (int ii = 0; ii < 6; ++ii ) peerInfo.peer_addr[ii] = (uint8_t) TempBroadcast[ii];
              if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
                Serial.println("Failed to add peer");
              }
              else { 
                printMAC(TempBroadcast); Serial.println(" peer added...");
                PairingSuccess = true; 
                jsondata = "";  doc.clear();
                
                doc["Node"]    = NAME_NODE;   
                doc["Pairing"] = "you are paired";
                doc["Type"]    = MONITOR_ROUND;

                serializeJson(doc, jsondata);  
                esp_now_send(P[zNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
                Serial.print("Sending you are paired"); 
                Serial.println(jsondata);
              }
            }
          }
          if (!PairingSuccess) { printMAC(TempBroadcast); Serial.println(" adding failed..."); } 
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
void setup() {
  Serial.begin(74880);
  Serial.println("Start");

  //CreatePreferences();

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
  TSMsgStart = millis();
  
  TFTBuffer.createSprite(240,240);
  TFTBuffer.setSwapBytes(true);
  
  TFTGauge1.createSprite(100,100);
  TFTGauge1.setSwapBytes(false);
  TFTGauge1.pushImage(0, 0, 100, 100, BtnSmall);
  
  Touch.begin();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);         //Reciever CallBack function
  
  InitModule();
  GetPeers();
  ReportPeers();
  RegisterPeers();

}
void loop() {
  if ((TSMsgStart) and (millis() - TSMsgStart > LOGO_INTERVAL)) {
    Mode = S_BAT_BAT_V;
    TSMsgStart = 0;
  }
  if (millis() - TSPing > PING_INTERVAL) {
    TSPing = millis();
    SendPing();
  }
  if (millis() - TSTouch > TOUCH_INTERVAL) {
    TSTouch = millis();
    
    TouchContact = Touch.getTouch(&TouchX, &TouchY, &Gesture);
    TouchX = 240-TouchX; 

    if (TouchContact) {
      Touched = true;
    }
    else if (Touched) {
      if      (Gesture == 1) ModeUp();          //swipe up 
      else if (Gesture == 2) ModeDown();        //swipe down 
      else if (Gesture == 3) ModeLeft();        //swipe left
      else if (Gesture == 4) ModeRight();       //swipe right
      
      else if (Mode == S_SW_0) ToggleSwitch(0);
      else if (Mode == S_SW_1) ToggleSwitch(1);
      else if (Mode == S_SW_2) ToggleSwitch(2);
      else if (Mode == S_SW_3) ToggleSwitch(3);
      
      else if (Mode == S_SW_ALL) {
             if (TouchX<120 and TouchY<120) ToggleSwitch(0);
        else if (TouchX>120 and TouchY<120) ToggleSwitch(1);
        else if (TouchX<120 and TouchY>120) ToggleSwitch(2);
        else if (TouchX>120 and TouchY>120) ToggleSwitch(3);
      }
      else if (Mode == S_MENU) {
             if (TouchX<120 and TouchY<120) Mode = S_BAT_EXT_A; 
        else if (TouchX>120 and TouchY<120) Mode = S_SW_0;
        else if (TouchX<120 and TouchY>120) Mode = S_SW_ALL;
        else if (TouchX>120 and TouchY>120) Mode = S_TXT_ALL;
      }
      else if (Mode == S_TXT_ALL) {
             if (ButtonHit(TouchX, TouchY, 0)) { Mode = S_CALIB_V; EichenVolt(); }
        else if (ButtonHit(TouchX, TouchY, 1)) { ForceEichen(); }
        else if (ButtonHit(TouchX, TouchY, 2)) { Mode = S_DBG_JSON; }
        else if (ButtonHit(TouchX, TouchY, 3)) { 
          ScreenChanged = true;
          OldMode = S_BAT_BAT_V; // for refresh
          Debug = !Debug; 
          preferences.begin("JeepifyInit", false);
          preferences.putBool("Debug", Debug);
          preferences.end();
        }
        else if (ButtonHit(TouchX, TouchY, 4)) { 
          if (Gesture == 12) { ClearPeers(); }
          else {
            ScreenChanged = true;
            ReadyToPair = !ReadyToPair;
            if (ReadyToPair) TSPair = millis();
            else         TSPair = 0;
          }
        }
      }
      else if (Mode == S_CALIB_V) {
        AddVolt(CalcField(TouchX, TouchY));
      }
      else if (Mode == S_DEVICE) {
             if (ButtonHit(TouchX, TouchY, 5)) { Mode = S_CALIB_V; EichenVolt(); }//restart
        else if (ButtonHit(TouchX, TouchY, 6)) {                                  //Reset
          jsondata = ""; doc.clear();
  
          doc["Node"] = NAME_NODE; doc["Order"] = "Reset";

          serializeJson(doc, jsondata); esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 100);  
          Serial.print("Sending Ping:"); 
          Serial.println(jsondata);    
        } 
        else if (ButtonHit(TouchX, TouchY, 7)) { ForceEichen(); }//pair
        else if (ButtonHit(TouchX, TouchY, 8)) { ForceEichen(); }//Eichen 
      }
    Touched = false;
    }
  }
  ScreenUpdate();
}
void ScreenUpdate() {
  float TempValue = 0;
  if (!TSMsgStart) {
    switch (Mode) {
          case S_BAT_EXT_A:  //Ampere Extern
            if ((Mode != OldMode) or (P[AktBat].S[0].Changed)) {
              noInterrupts(); 
                TempValue = P[AktBat].S[0].Value; 
                P[AktBat].S[0].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "Extern", GREEN2RED);
              OldMode = Mode;
            }
            break;          
          case S_BAT_SOL_A:   //Ampere Solar
            if ((Mode != OldMode) or (P[AktBat].S[1].Changed)) {
              noInterrupts(); 
                TempValue = P[AktBat].S[1].Value; 
                P[AktBat].S[1].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "Solar", GREEN2RED); 
              OldMode = Mode;
            }
            break;          
          case S_BAT_CAR_A:   //Ampere Car
            if ((Mode != OldMode) or (P[AktBat].S[2].Changed)) {
              noInterrupts(); 
                TempValue = P[AktBat].S[2].Value; 
                P[AktBat].S[2].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "In-Car", GREEN2RED); 
              OldMode = Mode;
            }
            break; 
          case S_BAT_LOAD:   //Ladestrom
            if ((Mode != OldMode) or (P[AktBat].S[3].Changed)) {
              noInterrupts(); 
                TempValue = P[AktBat].S[3].Value; 
                P[AktBat].S[3].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 30, "Amp", "Load", GREEN2RED); 
              OldMode = Mode;
            }
            break;          
          case S_BAT_BAT_V:   //Volt Battery
            if ((Mode != OldMode) or (P[AktBat].S[4].Changed)) {
              noInterrupts(); 
                TempValue = P[AktBat].S[4].Value; 
                P[AktBat].S[4].Changed = false; 
              interrupts();
              RingMeter(TempValue, 10, 16, "Volt", "Batterie", RED2GREEN); 
              OldMode = Mode;
            }
            break;  
          case S_BAT_EXT_W:   //Watt Extern
            if ((Mode != OldMode) or 
               (P[AktBat].S[0].Changed) or 
               (P[AktBat].S[4].Changed)) {
                noInterrupts(); 
                  TempValue = P[AktBat].S[0].Value*P[AktBat].S[4].Value; 
                  P[AktBat].S[0].Changed = false; 
                  P[AktBat].S[4].Changed = false; 
                interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "Extern", GREEN2RED); 
              OldMode = Mode;
            }
            break;    
          case S_BAT_SOL_W:   //Watt Solar
            if ((Mode != OldMode) or 
              (P[AktBat].S[1].Changed) or 
              (P[AktBat].S[4].Changed)) {
                noInterrupts(); 
                TempValue = P[AktBat].S[1].Value*P[AktBat].S[4].Value; 
                P[AktBat].S[1].Changed = false; 
                P[AktBat].S[4].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "Solar", GREEN2RED); 
              OldMode = Mode;
            }
            break;    
          case S_BAT_CAR_W:   //Watt InCar
            if ((Mode != OldMode) or 
              (P[AktBat].S[2].Changed) or 
              (P[AktBat].S[4].Changed)) {
                noInterrupts(); 
                  TempValue = P[AktBat].S[2].Value*P[AktBat].S[4].Value; 
                  P[AktBat].S[2].Changed = false; 
                  P[AktBat].S[4].Changed = false; 
                interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "In-Car", GREEN2RED); 
              OldMode = Mode;
            }
            break;    
          case S_BAT_GES_W:   //Watt Gesamt
            if ((Mode != OldMode) or (P[AktBat].S[0].Changed) or 
              (P[AktBat].S[1].Changed) or 
              (P[AktBat].S[2].Changed) or 
              (P[AktBat].S[3].Changed) or 
              (P[AktBat].S[4].Changed)) {
                noInterrupts(); 
                  TempValue = (P[AktBat].S[0].Value-P[AktBat].S[1].Value+P[AktBat].S[2].Value-P[AktBat].S[3].Value)*P[AktBat].S[4].Value;
                  P[AktBat].S[0].Changed = false;
                  P[AktBat].S[1].Changed = false;
                  P[AktBat].S[2].Changed = false;
                  P[AktBat].S[3].Changed = false;
                  P[AktBat].S[4].Changed = false;
                interrupts();
                RingMeter(TempValue, 0, 300, "Watt", "Gesamt", GREEN2RED); 
                OldMode = Mode;
            }
            break; 
          case S_VAL_ALL_1:  //Alle Sensoren (A)
            if ((Mode != OldMode) or 
              (P[AktBat].S[0].Changed) or 
              (P[AktBat].S[1].Changed) or 
              (P[AktBat].S[2].Changed) or 
              (P[AktBat].S[3].Changed)) {
                ShowAllValues();
                OldMode = Mode;
                P[AktBat].S[0].Changed = false;
                P[AktBat].S[1].Changed = false;
                P[AktBat].S[2].Changed = false;
                P[AktBat].S[3].Changed = false;
                P[AktBat].S[4].Changed = false;
            }
            break;
          case S_SW_0:  
            if ((Mode != OldMode) or (P[AktPDC].S[0].Changed)) {
              ShowSwitch(AktPDC, 0);
              OldMode = Mode;
              P[AktPDC].S[0].Changed = false;
            }  
            break;
          case S_SW_1:  
            if ((Mode != OldMode) or (P[AktPDC].S[1].Changed)) {
              ShowSwitch(AktPDC, 1);
              OldMode = Mode;
              P[AktPDC].S[1].Changed = false;
            }  
            break;
          case S_SW_2:  
            if ((Mode != OldMode) or (P[AktPDC].S[2].Changed)) {
              ShowSwitch(AktPDC, 2);
              OldMode = Mode;
              P[AktPDC].S[2].Changed = false;
            }  
            break;
          case S_SW_3:  
            if ((Mode != OldMode) or (P[AktPDC].S[3].Changed)) {
              ShowSwitch(AktPDC, 3);
              OldMode = Mode;
              P[AktPDC].S[3].Changed = false;
            }   
            break;
          case S_SW_ALL: 
            if ((Mode != OldMode) or 
              (P[AktPDC].S[0].Changed) or 
              (P[AktPDC].S[1].Changed) or 
              (P[AktPDC].S[2].Changed) or 
              (P[AktPDC].S[3].Changed)) {
                ShowAllSwitch(AktPDC); 
                OldMode = Mode; 
                P[AktPDC].S[1].Changed = false;
                P[AktPDC].S[2].Changed = false;
                P[AktPDC].S[3].Changed = false;
                P[AktPDC].S[4].Changed = false;
            }
            break;      
          case S_TXT_ALL: 
            if ((Mode != OldMode) or 
              P[AktPDC].S[0].Changed or 
              P[AktPDC].S[1].Changed or 
              P[AktPDC].S[2].Changed or 
              P[AktPDC].S[3].Changed or 
              P[AktPDC].S[4].Changed) {
                ShowAllText(AktBat); 
                OldMode = Mode; 
                P[AktPDC].S[0].Changed = false;
                P[AktPDC].S[1].Changed = false;
                P[AktPDC].S[2].Changed = false;
                P[AktPDC].S[3].Changed = false;
                P[AktPDC].S[4].Changed = false;
            }
            break;
          case S_DBG_JSON:  ShowJSON();    OldMode = Mode; break;  
          case S_DEVICES:   ShowDevices(); OldMode = Mode; break;
          case S_CALIB_V:   EichenVolt();  OldMode = Mode; break;  
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
    if (TSPair) {
        if (millis() - TSPair > TIME_TO_PAIR) {
          TSPair = 0; Pairing = false;
          TFTBuffer.drawSmoothArc(120, 120, 120, 118,   5,  10, TFT_BLACK, TFT_BLACK);
          MsgPairAktiv = false;
          ScreenChanged = true;
        }
        else if (MsgPairAktiv == false) {
          TFTBuffer.drawSmoothArc(120, 120, 120, 118,   5,  10, TFT_RED, TFT_BLACK);
          MsgPairAktiv = true;
          ScreenChanged = true;
        }
    }
  }
}
void ShowSwitch(int zNr, int s) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode) or (P[zNr].S[s].Changed)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, Btn);                    
  
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(P[zNr].S[s].Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(P[zNr].Name, 120,160);
    TFTBuffer.unloadFont();

         if (P[zNr].S[s].Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (P[zNr].S[s].Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TSScreenRefresh = millis();
  }

  P[zNr].S[s].OldValue = P[zNr].S[s].Value;  
}
void SendPing() {
  jsondata = "";  
  doc.clear();
  
  doc["Node"] = NAME_NODE;   
  doc["Order"] = "stay alive";

  if (ReadyToPair) {
    doc["Pairing"] = "aktiv";
  }

  serializeJson(doc, jsondata);  
  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 100);  
  Serial.print("Sending Ping:"); 
  Serial.println(jsondata);      
}
int  ToggleSwitch(int s) {
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"] = NAME_NODE;   
  if (s == 0) doc["Order"] = "ToggleSwitch0";
  if (s == 1) doc["Order"] = "ToggleSwitch1";
  if (s == 2) doc["Order"] = "ToggleSwitch2";
  if (s == 3) doc["Order"] = "ToggleSwitch3";
  
  serializeJson(doc, jsondata);  
  
  for (int ii = 0; ii < 6; ++ii ) TempBroadcast[ii] = (uint8_t) P[zNr].BroadcastAddress[ii];
  esp_now_send(TempBroadcast, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";

  return P[zNr].S[s].Value;
}
void ShowAllSwitch(int zNr) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
  
    TFTGauge1.pushToSprite(&TFTBuffer, 22, 25, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer,118, 25, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer, 22,115, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer,118,115, 0x4529);
    
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    if (Debug) TFTBuffer.drawString(P[zNr].Name, 120,15);

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextPadding(50); 
    TFTBuffer.drawString(P[zNr].S[0].Name, 70, 85); 
    TFTBuffer.drawString(P[zNr].S[1].Name,170, 85);
    TFTBuffer.drawString(P[zNr].S[2].Name, 70,175);
    TFTBuffer.drawString(P[zNr].S[3].Name,170,175);
    
    TFTBuffer.setTextPadding(0); 
    TFTBuffer.unloadFont(); 

    for (int i=0;i<ANZ_SWITCH;i++) P[zNr].S[i].Changed = true;
    } 
    
    if (P[zNr].S[0].Changed) {
      if (P[zNr].S[0].Value == true) TFTBuffer.pushImage( 59, 53, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage( 59, 53, 27, 10,BtnOff);
        P[zNr].S[0].Changed = false;
    }
    if (P[zNr].S[1].Changed) {
      if (P[zNr].S[1].Value == true) TFTBuffer.pushImage(155,53, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage(155,53, 27, 10,BtnOff);
        P[zNr].S[1].Changed = false;
    }
    if (P[zNr].S[2].Changed) {
      if (P[zNr].S[2].Value == true) TFTBuffer.pushImage( 59,143, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage( 59,143, 27, 10,BtnOff);
        P[zNr].S[2].Changed = false;
    }
    
    if (P[zNr].S[3].Changed) {
      if (P[zNr].S[3].Value == true) TFTBuffer.pushImage(155,143, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage(155,143, 27, 10,BtnOff);
        P[zNr].S[3].Changed = false;
    }
    TSScreenRefresh = millis();
}
void ShowAllValues() {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_BLACK, TFT_DARKGREY, false);
    TFTBuffer.setTextDatum(TL_DATUM);
    
    int x = 60;
    int y = 35;
    int GapY = 10;
    int Height = 25;
    float TempValue = 0;

    for (int i=0 ; i<ANZ_SENSOR ; i++) {
      if (P[AktBat].S[i].Changed) {
        noInterrupts(); TempValue = P[AktBat].S[i].Value; interrupts();
        switch (i) {
          case 0: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, GREEN2RED); break;
          case 1: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, GREEN2RED); break;
          case 2: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, RED2GREEN); break;
          case 3: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, RED2GREEN); break;
          case 4: LinearMeter(TempValue, x-5, y+i*(Height+GapY), 7, Height, 0, 15, RED2RED);   break;
        }
      }
      TFTBuffer.drawString(P[AktBat].S[i].Name,x+5, y+5+i*(Height+GapY));
    }
    TFTBuffer.unloadFont();

    TSScreenRefresh = millis();
  }  
}
void ShowJSON() {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if (((TSScreenRefresh - millis() > SCREEN_INTERVAL) and (jsondata != "") or (Mode != OldMode))) {
    ScreenChanged = true;
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
    PushTFT();
    TSScreenRefresh = millis();
  }
}
void ShowDevice(int zNr) {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString(P[zNr].Name, 120, 200); 

    DrawButton(5);
    DrawButton(6);
    DrawButton(7);
    DrawButton(8);

    TSScreenRefresh = millis();
  }
}
void ShowDevices() {
  if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString("Devices", 120, 200); 

    int Abstand = 20;
    String ZName;

    TFTBuffer.setTextDatum(TL_DATUM);
    
    for (int zNr=0 ; zNr<MAX_ZUBEHOER ; zNr++) {
      if (P[zNr].Type) {
        TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
        TFTBuffer.drawString(P[zNr].Name, 10, 80+zNr*Abstand);
        switch (P[zNr].Type) {
          case 1: ZName = "1-way PDC";  break;
          case 2: ZName = "2-way PDC";  break;
          case 4: ZName = "4-way PDC";  break;
          case 8: ZName = "8-way PDC";  break;
          case 9: ZName = "Batt.-Sens."; break;
        }
        TFTBuffer.drawString(ZName, 105, 80+zNr*Abstand);
        if (millis()- P[zNr].TSLastSeen > OFFLINE) { TFTBuffer.setTextColor(TFT_DARKGREY,  TFT_BLACK); TFTBuffer.drawString("off", 200, 80+zNr*Abstand); }
        else                                              { TFTBuffer.setTextColor(TFT_DARKGREEN, TFT_BLACK); TFTBuffer.drawString("on",  200, 80+zNr*Abstand); }
      }
    }
    TSScreenRefresh = millis();
  }
}
void ShowAllText(int zNr)
{ if (Mode != OldMode) TSScreenRefresh = millis();

  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true; 
    char buf[10];

    byte startX1 = 55;
    byte startY  = 25;
    byte startX2 = 115;
    byte startX3 = 155;
      
    byte Abstand = 20;
    
    byte len = 5;
    byte nk = 1;
  
    TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
  
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    
    TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    TFTBuffer.setTextDatum(TL_DATUM);
    
    TFTBuffer.drawString("Extern", startX1,startY); 
    TFTBuffer.drawString("(A):", startX2,startY); 
    dtostrf(P[zNr].S[0].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY); 

    TFTBuffer.drawString("Solar", startX1,startY+Abstand);
    TFTBuffer.drawString("(A):", startX2,startY+Abstand); 
    dtostrf(P[zNr].S[1].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+Abstand); 
    
    TFTBuffer.drawString("Intern", startX1,startY+2*Abstand); 
    TFTBuffer.drawString("(A):", startX2,startY+2*Abstand); 
    dtostrf(P[zNr].S[2].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+2*Abstand); 
  
    TFTBuffer.drawString("Load", startX1,startY+3*Abstand); 
    TFTBuffer.drawString("(A):", startX2,startY+3*Abstand); 
    dtostrf(P[zNr].S[3].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+3*Abstand); 
    
    TFTBuffer.drawString("Battery", startX1,startY+4*Abstand); 
    TFTBuffer.drawString("(V):", startX2,startY+4*Abstand); 
    dtostrf(P[zNr].S[4].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+4*Abstand); 

    TFTBuffer.drawString("Heap:", startX1,startY+5*Abstand); 
    TFTBuffer.drawString("(k):", startX2,startY+5*Abstand); 
    dtostrf(ESP.getFreeHeap(), len, 0, buf);
    TFTBuffer.drawString(buf, startX3,startY+5*Abstand); 

    TFTBuffer.unloadFont();

    DrawButton(0);
    DrawButton(1);
    DrawButton(2);
    DrawButton(3);
    DrawButton(4);
    
    TSScreenRefresh = millis();
  }
}
void DrawButton(int z) {
  TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
  TFTBuffer.setTextDatum(MC_DATUM);

  if (Button[z].Status) TFTBuffer.fillSmoothRoundRect(Button[z].x, Button[z].y, Button[z].w, Button[z].h, 10, Button[z].TxtColor, Button[z].BGColor);
  TFTBuffer.drawSmoothRoundRect(Button[z].x, Button[z].y, 10, 8, Button[z].w, Button[z].h, Button[z].TxtColor, Button[z].BGColor);
  TFTBuffer.drawString(Button[z].Name, Button[z].x+Button[z].w/2, Button[z].y+Button[z].h/2);  

   TFTBuffer.unloadFont();
}
bool ButtonHit(int x, int y, int z) {
  return ( ((x>Button[z].x) and (x<Button[z].x+Button[z].w) and (y>Button[z].y) and (y<Button[z].y+Button[z].h)) );
}
void ShowMenu() {
  if (Mode != OldMode) TSScreenRefresh = millis();
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TSScreenRefresh = millis();
  }
}
int  ModeLeft() {
  int TempzNr = 0;
  switch (Mode) {
          case S_BAT_EXT_A: Mode +=1; break;          
          case S_BAT_SOL_A: Mode +=1; break;   
          case S_BAT_CAR_A: Mode +=1; break; 
          case S_BAT_LOAD : Mode +=1; break; 
          case S_BAT_BAT_V: Mode +=1; break;   
          case S_BAT_EXT_W: Mode +=1; break;   
          case S_BAT_SOL_W: Mode +=1; break;   
          case S_BAT_CAR_W: Mode +=1; break;   
          case S_BAT_GES_W: 
            TempzNr = AktBat;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr < MAX_ZUBEHOER-1) TempzNr++;
              else TempzNr = 0;
              if (P[TempzNr].Type == 9) { AktBat = TempzNr; OldMode = 99; break; }
            }
            break; 
          case S_SW_0:      Mode +=1; break;
          case S_SW_1:      Mode +=1; break;   
          case S_SW_2:      Mode +=1; break;   
          case S_SW_3:      Mode = S_SW_0; break;   
          case S_VAL_ALL_1: Mode = S_SW_ALL; break;   
          case S_VAL_ALL_2: Mode = S_SW_ALL; break;  
          case S_SW_ALL: 
            TempzNr = AktPDC;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr < MAX_ZUBEHOER-1) TempzNr++;
              else TempzNr = 0;
              if ((P[TempzNr].Type == 1) or (P[TempzNr].Type == 2) or (P[TempzNr].Type == 4)) { AktPDC = TempzNr; OldMode = 99; break; }
            }
            break;
          case S_TXT_ALL:   Mode = S_DEVICES; break;   
          case S_DEVICES:   Mode = S_TXT_ALL; break;
          case S_DBG_JSON:  Mode = S_TXT_ALL; break;
        }
  return Mode;
}
int  ModeRight() {
  int TempzNr = 0;
  switch (Mode) {
          case S_BAT_EXT_A: Mode = S_BAT_GES_W; break;          
          case S_BAT_SOL_A: Mode -=1; break;   
          case S_BAT_CAR_A: Mode -=1; break;   
          case S_BAT_LOAD : Mode -=1; break; 
          case S_BAT_BAT_V: Mode -=1; break;   
          case S_BAT_EXT_W: Mode -=1; break;   
          case S_BAT_SOL_W: Mode -=1; break;   
          case S_BAT_CAR_W: Mode -=1; break;   
          case S_BAT_GES_W: 
            TempzNr = AktBat;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr > 0) TempzNr--;
              else TempzNr = MAX_ZUBEHOER-1;
              if (P[TempzNr].Type == 9) { AktBat = TempzNr; OldMode = 99; break; }
            }
            break;    
          case S_SW_0:      Mode = S_SW_3; break;
          case S_SW_1:      Mode -=1; break;   
          case S_SW_2:      Mode -=1; break;   
          case S_SW_3:      Mode -=1; break;   
          case S_VAL_ALL_1: Mode = S_SW_ALL; break;   
          case S_VAL_ALL_2: Mode -=1; break;  
          case S_SW_ALL:    
            TempzNr = AktPDC;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr > 0) TempzNr--;
              else TempzNr = MAX_ZUBEHOER-1;
              if ((P[TempzNr].Type == 1) or (P[TempzNr].Type == 2) or (P[TempzNr].Type == 4)) { AktPDC = TempzNr; OldMode = 99; break; }
            }
            break;   
          case S_TXT_ALL:   Mode = S_DEVICES; break;   
          case S_DEVICES:   Mode = S_TXT_ALL; break;
          case S_DBG_JSON:  Mode = S_TXT_ALL; break;
        }
  return Mode;
}
int  ModeUp() {
  switch (Mode) {
          case S_BAT_EXT_A: Mode = S_VAL_ALL_1; break;          
          case S_BAT_SOL_A: Mode = S_VAL_ALL_1; break;   
          case S_BAT_CAR_A: Mode = S_VAL_ALL_1; break;   
          case S_BAT_LOAD : Mode = S_VAL_ALL_1; break; 
          case S_BAT_BAT_V: Mode = S_VAL_ALL_1; break;   
          case S_BAT_EXT_W: Mode = S_VAL_ALL_2; break;   
          case S_BAT_SOL_W: Mode = S_VAL_ALL_2; break;   
          case S_BAT_CAR_W: Mode = S_VAL_ALL_2; break;   
          case S_BAT_GES_W: Mode = S_BAT_BAT_V; break;   
          case S_SW_0:      Mode = S_SW_ALL; break;
          case S_SW_1:      Mode = S_SW_ALL; break;   
          case S_SW_2:      Mode = S_SW_ALL; break;   
          case S_SW_3:      Mode = S_SW_ALL; break;   
          case S_VAL_ALL_1: Mode = S_BAT_EXT_A; break;   
          case S_VAL_ALL_2: Mode = S_BAT_EXT_W; break;  
          case S_SW_ALL:    Mode = S_SW_0; break; 
          case S_TXT_ALL:   Mode = S_MENU; break;   
          //case S_NODES_ICN: Mode = S_MENU; break; 
          //case S_NODES_TXT: Mode = S_MENU; break;  
        }
  return Mode;
}
int  ModeDown() {
  Mode = S_MENU;
  return Mode;
}
int  RingMeter(float value, float vmin, float vmax, const char *units, const char *bez, byte scheme) {
  int x = 0;
  int y = 0;
  int r = 120;
  int nk = 0;
  
  if (Mode != OldMode) TSScreenRefresh = millis();
  
  if ((TSScreenRefresh - millis() > SCREEN_INTERVAL) or (Mode != OldMode)) {
    ScreenChanged = true;
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
    
    if (Debug) TFTBuffer.drawString(P[AktBat].Name, 120,225);

    TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    TFTBuffer.drawString(units, 120,150); // Units display
    TFTBuffer.setTextDatum(BC_DATUM);
    TFTBuffer.drawString(bez, 120,90); // Units display

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
    PushTFT();
    TSScreenRefresh = millis();
  }
  return x + r;
}
void LinearMeter(int val, int x, int y, int w, int h, int g, int n, byte s) {
  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;
  // Draw n colour blocks
  for (int b = 1; b <= n; b++) {
    if (val > 0 && b <= val) { // Fill in coloured blocks
      switch (s) {
        case 0: colour = TFT_RED; break; // Fixed colour
        case 1: colour = TFT_GREEN; break; // Fixed colour
        case 2: colour = TFT_BLUE; break; // Fixed colour
        case 3: colour = rainbowColor(map(b, 0, n, 127,   0)); break; // Blue to red
        case 4: colour = rainbowColor(map(b, 0, n,  63,   0)); break; // Green to red
        case 5: colour = rainbowColor(map(b, 0, n,   0,  63)); break; // Red to green
        case 6: colour = rainbowColor(map(b, 0, n,   0, 159)); break; // Rainbow (red to violet)
      }
      TFTBuffer.fillSmoothRoundRect(x + b*(w+g), y, w, h, 0, colour, TFT_BLACK);
    }
    else // Fill in blank segments
    {
      TFTBuffer.fillSmoothRoundRect(x + b*(w+g), y, w, h, 0, TFT_DARKGREY, TFT_BLACK);
    }
  }
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

    doc["Node"] = NAME_NODE;   
    doc["Order"] = "VoltCalib";
    doc["Value"] = buf;
    
    serializeJson(doc, jsondata);  
  
    esp_now_send(broadcastAddressBattery,     (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    esp_now_send(broadcastAddressBattery8266, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    Serial.println(jsondata);

    TSMsgVolt = millis();
    Mode = S_TXT_ALL;
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

  doc["Node"] = NAME_NODE;   
  doc["Order"] = "Eichen";
  
  serializeJson(doc, jsondata);  
  
  esp_now_send(broadcastAddressBattery,     (uint8_t *) jsondata.c_str(), 50);  //Sending "jsondata"  
  esp_now_send(broadcastAddressBattery8266, (uint8_t *) jsondata.c_str(), 50);  //Sending "jsondata"  

  Serial.println(jsondata);
  jsondata = "";

  TSMsgEich = millis();
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
uint16_t rainbowColor(uint8_t spectrum) {
  spectrum = spectrum%192;
  
  uint8_t red   = 0; // Red is the top 5 bits of a 16 bit colour spectrum
  uint8_t green = 0; // Green is the middle 6 bits, but only top 5 bits used here
  uint8_t blue  = 0; // Blue is the bottom 5 bits

  uint8_t sector = spectrum >> 5;
  uint8_t amplit = spectrum & 0x1F;

  switch (sector)
  {
    case 0:
      red   = 0x1F;
      green = amplit; // Green ramps up
      blue  = 0;
      break;
    case 1:
      red   = 0x1F - amplit; // Red ramps down
      green = 0x1F;
      blue  = 0;
      break;
    case 2:
      red   = 0;
      green = 0x1F;
      blue  = amplit; // Blue ramps up
      break;
    case 3:
      red   = 0;
      green = 0x1F - amplit; // Green ramps down
      blue  = 0x1F;
      break;
    case 4:
      red   = amplit; // Red ramps up
      green = 0;
      blue  = 0x1F;
      break;
    case 5:
      red   = 0x1F;
      green = 0;
      blue  = 0x1F - amplit; // Blue ramps down
      break;
  }

  return red << 11 | green << 6 | blue;
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}