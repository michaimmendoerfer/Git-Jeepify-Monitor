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
#define VERSION   "V 2.03"

struct struct_Peripheral {
    char        Name[20];
    int         Type;      //1=Switch, 2=Amp, 3=Volt
    // Sensor
    int         IOPort;
    float       NullWert;
    float       VperAmp;
    int         Vin;
    float       Value;
};
struct struct_Peer {
    char       Name[20] = {};
    u_int8_t   BroadcastAddress[6];
    uint32_t   TSLastSeen = 0;
    int        Type = 0;  // 
};

struct struct_Actor {
  struct_Peer       P;
  struct_Peripheral S[MAX_PERIPHERALS];
}

struct struct_zubehoer {
    char          ZubehoerName[20];
    int           Type; // 1=1Way, 2=2Way, 4=4way, 9=Bat
    uint8_t       BroadcastAddress[6];
    uint32_t      TimestampLastSeen;
    struct_sensor Sensor[ANZ_SENSOR];
    struct_sensor Switch[ANZ_SWITCH];
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

int AnzZubehoer = 0;
int AktBat;
int AktPDC;

struct_zubehoer Z[MAX_ZUBEHOER];
uint8_t TempBroadcastAddress[6];

void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void   OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void   printMAC(const uint8_t * mac_addr);
void   SavePeers();
void   GetPeers();
void   ClearPeers();
void   ReportPeers();
void   SendPing();
void   ForceEichen();

int    modeUp(); 
int    modeDown(); 
int    modeLeft(); 
int    modeRight(); 

void   SetMsgIndicator();
void   PushTFT();
void   ScreenUpdate();
int    ToggleSwitch (int zNr, int s);
void   ShowSwitch(int zNr, int s);
void   ShowAllValues();
void   ShowAllSwitch(int zNr);
void   ShowMenu();
void   ShowAllText(int zNr);
void   ShowJSON(void);
void   ShowDevices(void);
void   ShowDevice(int zNr);
int    RingMeter(float value, float vmin, float vmax, const char *units, const char *bez, byte scheme);
void   LinearMeter(int val, int x, int y, int w, int h, int g, int n, byte s);
float  mapf(float x, float in_min, float in_max, float out_min, float out_max);
int    CalcField(int x, int y);
void   AddVolt(int i);
void   EichenVolt();
void   DrawButton(int z);
bool   ButtonHit(int x, int y, int z);
unsigned int rainbow(byte value);
uint16_t rainbowColor(uint8_t spectrum);
bool   isSwitch(int Type);
bool   isSensor(int Type);

StaticJsonDocument<300> doc;
String jsondata;

bool TouchContact;
bool touched;
bool ScreenChanged = true;

uint8_t   gesture;
uint16_t  touchX, touchY;
uint32_t _touchTS = 0;

volatile uint32_t Timestamp        = 0;
volatile uint32_t TimestampMsgRcv  = 0;
volatile uint32_t TimestampMsgPDC  = 0;
volatile uint32_t TimestampMsgBat  = 0;
volatile uint32_t TimestampMsgVolt = 0;
volatile uint32_t TimestampMsgEich = 0;
volatile uint32_t TimestampPair    = 0;
uint32_t TimestampMsgStart         = 0;
uint32_t TimestampPing             = 0;
uint32_t TimestampScreenRefresh    = 0;

bool MsgBatAktiv  = false;
bool MsgPDCAktiv  = false;
bool MsgVoltAktiv = false;
bool MsgEichAktiv = false;
bool MsgPairAktiv = false;

bool Debug        = true;
int  UpdateCount  = 0;

int   mode    = S_LOGO;                 //StartMode
int   oldMode = 99;

Preferences preferences;
bool paired  = false;
bool Pairing = false;
uint8_t TempBroadcast[6];

int   ButtonRd   = 22;
int   ButtonGapX = 6;
int   ButtonGapY = 6;
float VoltCalib;
int   VoltCount;

TFT_eSPI tft              = TFT_eSPI();
TFT_eSprite TFTBuffer     = TFT_eSprite(&tft);
TFT_eSprite TFTGauge1     = TFT_eSprite(&tft);

CST816D touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

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
  Serial.print("Recieved from:"); printMAC(mac); Serial.println(jsondata);    
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    TimestampMsgRcv = millis();
    //bool Paired = false;
    bool NodeBekannt = false;
    int zNr = 0;

    for (zNr=0; zNr<MAX_ZUBEHOER; zNr++) { 
      if (doc["Node"] == Z[zNr].ZubehoerName) {
        NodeBekannt = true; 
        Z[zNr].TimestampLastSeen = millis();
        break; 
      }
    }    
    if      ((NodeBekannt)  and (isSensor(Z[zNr].Type))) {      
      TimestampMsgBat = TimestampMsgRcv;
      for (int i=0; i<ANZ_SENSOR; i++) {
        if (doc.containsKey(Z[zNr].Sensor[i].Name)) {
          float TempSensor = (float)doc[Z[zNr].Sensor[i].Name];
      
          if (TempSensor != Z[zNr].Sensor[i].Value) {
            Z[zNr].Sensor[i].OldValue = Z[zNr].Sensor[i].Value;
            Z[zNr].Sensor[i].Value = TempSensor;
            Z[zNr].Sensor[i].Changed = true;
          }
        }
      } 
    }
    else if ((NodeBekannt)  and (isSwitch(Z[zNr].Type))) {
      TimestampMsgPDC = TimestampMsgRcv;
      for (int i=0; i<ANZ_SWITCH; i++) {
        if (doc.containsKey(Z[zNr].Switch[i].Name)) {
          int TempSwitch = (int)doc[Z[zNr].Switch[i].Name];
      
          if (TempSwitch != Z[zNr].Switch[i].Value) {
            Z[zNr].Switch[i].OldValue = Z[zNr].Switch[i].Value;
            Z[zNr].Switch[i].Value = TempSwitch;
            Z[zNr].Switch[i].Changed = true;
          }
        }
      }
    }     
    else if ((Pairing) and (doc.containsKey("Pairing"))) {
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
          esp_now_send(Z[zNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
          Serial.print("Sending you are paired"); 
          Serial.println(jsondata);
        }
        else {
          for (zNr=0; zNr<MAX_ZUBEHOER; zNr++) {
            if ((Z[zNr].Type == 0) and (!PairingSuccess)) {
              
              for (int b = 0; b < 6; b++ ) Z[zNr].BroadcastAddress[b] = TempBroadcast[b];
              strcpy(Z[zNr].ZubehoerName, doc["Node"]);
              Z[zNr].Type = doc["Type"];
              Z[zNr].TimestampLastSeen = millis();
              
              if (isSwitch(Z[zNr].Type)) {
                for (int b=0; b<ANZ_SWITCH; b++) {
                  sprintf(BufB, "%d", b); 
                  strcpy(Buf, "SW"); strcat(Buf, BufB);
                  if (doc.containsKey(Buf)) strcpy(Z[zNr].Switch[b].Name, doc[Buf]);
                }   
              }
              else if (isSensor(Z[zNr].Type)) {
                for (int b=0; b<ANZ_SENSOR; b++) {
                  sprintf(BufB, "%d", b); 
                  strcpy(Buf, "S"); strcat(Buf, BufB);
                  if (doc.containsKey(Buf)) strcpy(Z[zNr].Sensor[b].Name, doc[Buf]);
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
                esp_now_send(Z[zNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  
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
void SavePeers() {
  preferences.begin("Jeepify", false);
  char Buf[10] = {};
  char BufNr[5] = {};
  char BufB[5] = {};
  String BufS;

  AnzZubehoer = 0;
  for (int zNr=0; zNr< MAX_ZUBEHOER; zNr++) {
    if (Z[zNr].Type > 0) {
      AnzZubehoer++;
      sprintf(BufNr, "%d", zNr); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(Z[zNr].Type);
      if (preferences.getInt(Buf) != Z[zNr].Type) preferences.putInt(Buf, Z[zNr].Type);
      
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = Z[zNr].ZubehoerName;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      if (preferences.getString(Buf) != BufS) preferences.putString(Buf, BufS);
      Serial.print("Check ZubehoerString: "); Serial.println(preferences.getString(Buf));

      for (int b=0; b<6; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "B"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        if (preferences.getUChar(Buf) != Z[zNr].BroadcastAddress[b]) preferences.putUChar(Buf, Z[zNr].BroadcastAddress[b]);
      }
      for (int b=0; b<ANZ_SWITCH; b++) {
        sprintf(BufB, "%d", b); 
        BufS = Z[zNr].Switch[b].Name;
        strcpy(Buf, "SW"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        if (preferences.getString(Buf) != BufS) preferences.putString(Buf, BufS);
      }
      for (int b=0; b<ANZ_SENSOR; b++) {
        sprintf(BufB, "%d", b); 
        BufS = Z[zNr].Sensor[b].Name;
        strcpy(Buf, "S"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        if (preferences.getString(Buf) != BufS) preferences.putString(Buf, BufS);
      }
    }
  }
  if (preferences.getInt("AnzZubehoer") != AnzZubehoer) preferences.putInt("AnzZubehoer", AnzZubehoer);

  preferences.end();
}
void GetPeers() {
  preferences.begin("Jeepify", true);
  char Buf[10] = {};
  char BufNr[5] = {};
  char BufB[5] = {};
  String BufS;
  
  int  TempType;
  char TempZubehoerName[20];
  char TempBroadcast[6];

  AnzZubehoer = preferences.getInt("AnzZubehoer");

  for (int zNr=0; zNr< MAX_ZUBEHOER; zNr++) {
    // "Type-0"
    sprintf(BufNr, "%d", zNr); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
    if (preferences.getInt(Buf) > 0) {
      Z[zNr].Type = preferences.getInt(Buf);
      for (int b=0; b<6; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "B"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        Z[zNr].BroadcastAddress[b] = preferences.getUChar(Buf);
      }
      Z[zNr].TimestampLastSeen = millis();
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = preferences.getString(Buf);
      strcpy(Z[zNr].ZubehoerName, BufS.c_str());
      for (int b=0; b<ANZ_SWITCH; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "SW"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        BufS = preferences.getString(Buf);
        strcpy(Z[zNr].Switch[b].Name, BufS.c_str());
      }
      for (int b=0; b<ANZ_SENSOR; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "S"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        BufS = preferences.getString(Buf);
        strcpy(Z[zNr].Sensor[b].Name, BufS.c_str());
      }
    }
  }
  preferences.end();
}
void ClearPeers() {
  preferences.begin("Jeepify", false);
    preferences.clear();
    Serial.println("Jeepify cleared...");
  preferences.end();
}
void ReportPeers() {
  for (int i=0; i<MAX_ZUBEHOER; i++) {
    Serial.println(Z[i].ZubehoerName);
    Serial.println(Z[i].Switch[0].Name);
    Serial.println(Z[i].Type);
    Serial.print("MAC: "); printMAC(Z[i].BroadcastAddress);
    Serial.println();
    Serial.println();
  }
}
void setup() {
  Serial.begin(74880);
  Serial.println("Start");

  //CreatePreferences();

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  tft.init();
  tft.setRotation(Rotation);
  tft.setSwapBytes(true);
  
  tft.pushImage(0,0, 240, 240, JeepifyLogo); 
  tft.loadFont(AA_FONT_SMALL); 
  tft.setTextColor(TFT_BLACK, TFT_RED, false);
  tft.setTextDatum(MC_DATUM); 
  tft.drawString(VERSION, 120,60);
  TimestampMsgStart = millis();
  
  TFTBuffer.createSprite(240,240);
  TFTBuffer.setSwapBytes(true);
  
  TFTGauge1.createSprite(100,100);
  TFTGauge1.setSwapBytes(false);
  TFTGauge1.pushImage(0, 0, 100, 100, BtnSmall);
  
  touch.begin();
  //ClearPeers();
  GetPeers();
  ReportPeers();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  for (int ii = 0; ii < 6; ++ii ) peerInfo.peer_addr[ii] = (uint8_t) broadcastAddressAll[ii];
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); return; }
  
  for (int zNr=0; zNr<MAX_ZUBEHOER; zNr++) {
    if (Z[zNr].Type > 0) {
      for (int ii = 0; ii < 6; ++ii ) peerInfo.peer_addr[ii] = (uint8_t) Z[zNr].BroadcastAddress[ii];
      Serial.print("Registering: "); Serial.println(Z[zNr].ZubehoerName);
     if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); }
    }
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);         //Reciever CallBack function
  
  preferences.begin("Jeepify", false);
    Debug   = preferences.getBool("Debug", true);
    Pairing = preferences.getBool("Pairing", false);
  preferences.end();
}
void loop() {
  if ((TimestampMsgStart) and (millis() - TimestampMsgStart > LOGO_INTERVAL)) {
    mode = S_BAT_BAT_V;
    TimestampMsgStart = 0;
  }
  if (millis() - TimestampPing > PING_INTERVAL) {
    TimestampPing = millis();
    SendPing();
  }
  if (millis() - _touchTS > TOUCH_INTERVAL) {
    _touchTS = millis();
    
    TouchContact = touch.getTouch(&touchX, &touchY, &gesture);
    touchX = 240-touchX; 

    if (TouchContact) {
      touched = true;
    }
    else if (touched) {
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
  ScreenUpdate();
}
void ScreenUpdate() {
  float TempValue = 0;
  if (!TimestampMsgStart) {
    switch (mode) {
          case S_BAT_EXT_A:  //Ampere Extern
            if ((mode != oldMode) or (Z[AktBat].Sensor[0].Changed)) {
              noInterrupts(); 
                TempValue = Z[AktBat].Sensor[0].Value; 
                Z[AktBat].Sensor[0].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "Extern", GREEN2RED);
              oldMode = mode;
            }
            break;          
          case S_BAT_SOL_A:   //Ampere Solar
            if ((mode != oldMode) or (Z[AktBat].Sensor[1].Changed)) {
              noInterrupts(); 
                TempValue = Z[AktBat].Sensor[1].Value; 
                Z[AktBat].Sensor[1].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "Solar", GREEN2RED); 
              oldMode = mode;
            }
            break;          
          case S_BAT_CAR_A:   //Ampere Car
            if ((mode != oldMode) or (Z[AktBat].Sensor[2].Changed)) {
              noInterrupts(); 
                TempValue = Z[AktBat].Sensor[2].Value; 
                Z[AktBat].Sensor[2].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 12, "Amp", "In-Car", GREEN2RED); 
              oldMode = mode;
            }
            break; 
          case S_BAT_LOAD:   //Ladestrom
            if ((mode != oldMode) or (Z[AktBat].Sensor[3].Changed)) {
              noInterrupts(); 
                TempValue = Z[AktBat].Sensor[3].Value; 
                Z[AktBat].Sensor[3].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 30, "Amp", "Load", GREEN2RED); 
              oldMode = mode;
            }
            break;          
          case S_BAT_BAT_V:   //Volt Battery
            if ((mode != oldMode) or (Z[AktBat].Sensor[4].Changed)) {
              noInterrupts(); 
                TempValue = Z[AktBat].Sensor[4].Value; 
                Z[AktBat].Sensor[4].Changed = false; 
              interrupts();
              RingMeter(TempValue, 10, 16, "Volt", "Batterie", RED2GREEN); 
              oldMode = mode;
            }
            break;  
          case S_BAT_EXT_W:   //Watt Extern
            if ((mode != oldMode) or 
               (Z[AktBat].Sensor[0].Changed) or 
               (Z[AktBat].Sensor[4].Changed)) {
                noInterrupts(); 
                  TempValue = Z[AktBat].Sensor[0].Value*Z[AktBat].Sensor[4].Value; 
                  Z[AktBat].Sensor[0].Changed = false; 
                  Z[AktBat].Sensor[4].Changed = false; 
                interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "Extern", GREEN2RED); 
              oldMode = mode;
            }
            break;    
          case S_BAT_SOL_W:   //Watt Solar
            if ((mode != oldMode) or 
              (Z[AktBat].Sensor[1].Changed) or 
              (Z[AktBat].Sensor[4].Changed)) {
                noInterrupts(); 
                TempValue = Z[AktBat].Sensor[1].Value*Z[AktBat].Sensor[4].Value; 
                Z[AktBat].Sensor[1].Changed = false; 
                Z[AktBat].Sensor[4].Changed = false; 
              interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "Solar", GREEN2RED); 
              oldMode = mode;
            }
            break;    
          case S_BAT_CAR_W:   //Watt InCar
            if ((mode != oldMode) or 
              (Z[AktBat].Sensor[2].Changed) or 
              (Z[AktBat].Sensor[4].Changed)) {
                noInterrupts(); 
                  TempValue = Z[AktBat].Sensor[2].Value*Z[AktBat].Sensor[4].Value; 
                  Z[AktBat].Sensor[2].Changed = false; 
                  Z[AktBat].Sensor[4].Changed = false; 
                interrupts();
              RingMeter(TempValue, 0, 300, "Watt", "In-Car", GREEN2RED); 
              oldMode = mode;
            }
            break;    
          case S_BAT_GES_W:   //Watt Gesamt
            if ((mode != oldMode) or (Z[AktBat].Sensor[0].Changed) or 
              (Z[AktBat].Sensor[1].Changed) or 
              (Z[AktBat].Sensor[2].Changed) or 
              (Z[AktBat].Sensor[3].Changed) or 
              (Z[AktBat].Sensor[4].Changed)) {
                noInterrupts(); 
                  TempValue = (Z[AktBat].Sensor[0].Value-Z[AktBat].Sensor[1].Value+Z[AktBat].Sensor[2].Value-Z[AktBat].Sensor[3].Value)*Z[AktBat].Sensor[4].Value;
                  Z[AktBat].Sensor[0].Changed = false;
                  Z[AktBat].Sensor[1].Changed = false;
                  Z[AktBat].Sensor[2].Changed = false;
                  Z[AktBat].Sensor[3].Changed = false;
                  Z[AktBat].Sensor[4].Changed = false;
                interrupts();
                RingMeter(TempValue, 0, 300, "Watt", "Gesamt", GREEN2RED); 
                oldMode = mode;
            }
            break; 
          case S_VAL_ALL_1:  //Alle Sensoren (A)
            if ((mode != oldMode) or 
              (Z[AktBat].Sensor[0].Changed) or 
              (Z[AktBat].Sensor[1].Changed) or 
              (Z[AktBat].Sensor[2].Changed) or 
              (Z[AktBat].Sensor[3].Changed)) {
                ShowAllValues();
                oldMode = mode;
                Z[AktBat].Sensor[0].Changed = false;
                Z[AktBat].Sensor[1].Changed = false;
                Z[AktBat].Sensor[2].Changed = false;
                Z[AktBat].Sensor[3].Changed = false;
                Z[AktBat].Sensor[4].Changed = false;
            }
            break;
          case S_SW_0:  
            if ((mode != oldMode) or (Z[AktPDC].Switch[0].Changed)) {
              ShowSwitch(AktPDC, 0);
              oldMode = mode;
              Z[AktPDC].Switch[0].Changed = false;
            }  
            break;
          case S_SW_1:  
            if ((mode != oldMode) or (Z[AktPDC].Switch[1].Changed)) {
              ShowSwitch(AktPDC, 1);
              oldMode = mode;
              Z[AktPDC].Switch[1].Changed = false;
            }  
            break;
          case S_SW_2:  
            if ((mode != oldMode) or (Z[AktPDC].Switch[2].Changed)) {
              ShowSwitch(AktPDC, 2);
              oldMode = mode;
              Z[AktPDC].Switch[2].Changed = false;
            }  
            break;
          case S_SW_3:  
            if ((mode != oldMode) or (Z[AktPDC].Switch[3].Changed)) {
              ShowSwitch(AktPDC, 3);
              oldMode = mode;
              Z[AktPDC].Switch[3].Changed = false;
            }   
            break;
          case S_SW_ALL: 
            if ((mode != oldMode) or 
              (Z[AktPDC].Switch[0].Changed) or 
              (Z[AktPDC].Switch[1].Changed) or 
              (Z[AktPDC].Switch[2].Changed) or 
              (Z[AktPDC].Switch[3].Changed)) {
                ShowAllSwitch(AktPDC); 
                oldMode = mode; 
                Z[AktPDC].Switch[1].Changed = false;
                Z[AktPDC].Switch[2].Changed = false;
                Z[AktPDC].Switch[3].Changed = false;
                Z[AktPDC].Switch[4].Changed = false;
            }
            break;      
          case S_TXT_ALL: 
            if ((mode != oldMode) or 
              Z[AktPDC].Sensor[0].Changed or 
              Z[AktPDC].Sensor[1].Changed or 
              Z[AktPDC].Sensor[2].Changed or 
              Z[AktPDC].Sensor[3].Changed or 
              Z[AktPDC].Sensor[4].Changed) {
                ShowAllText(AktBat); 
                oldMode = mode; 
                Z[AktPDC].Sensor[0].Changed = false;
                Z[AktPDC].Sensor[1].Changed = false;
                Z[AktPDC].Sensor[2].Changed = false;
                Z[AktPDC].Sensor[3].Changed = false;
                Z[AktPDC].Sensor[4].Changed = false;
            }
            break;
          case S_DBG_JSON:  ShowJSON();    oldMode = mode; break;  
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
    if (TimestampMsgVolt) {
        if (millis() - TimestampMsgVolt > MSGLIGHT_INTERVAL) {
          TimestampMsgVolt = 0;
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
    if (TimestampMsgEich) {
        if (millis() - TimestampMsgEich > MSGLIGHT_INTERVAL) {
          TimestampMsgEich = 0; 
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
    if (TimestampMsgPDC) {
        if (millis() - TimestampMsgPDC > MSGLIGHT_INTERVAL) {
          TimestampMsgPDC = 0;
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
    if (TimestampMsgBat) {
        if (millis() - TimestampMsgBat > MSGLIGHT_INTERVAL) {
          TimestampMsgBat = 0;
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
    if (TimestampPair) {
        if (millis() - TimestampPair > TIME_TO_PAIR) {
          TimestampPair = 0; Pairing = false;
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
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode) or (Z[zNr].Switch[s].Changed)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, Btn);                    
  
    TFTBuffer.loadFont(AA_FONT_LARGE); 

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    TFTBuffer.drawString(Z[zNr].Switch[s].Name, 120,130);

    TFTBuffer.unloadFont(); 

    TFTBuffer.loadFont(AA_FONT_SMALL);
    TFTBuffer.drawString(Z[zNr].ZubehoerName, 120,160);
    TFTBuffer.unloadFont();

         if (Z[zNr].Switch[s].Value == 1) TFTBuffer.pushImage(107,70,27,10,BtnOn);
    else if (Z[zNr].Switch[s].Value == 0) TFTBuffer.pushImage(107,70,27,10,BtnOff);

    TimestampScreenRefresh = millis();
  }

  Z[zNr].Switch[s].OldValue = Z[zNr].Switch[s].Value;  
}
void SendPing() {
  jsondata = "";  
  doc.clear();
  
  doc["Node"] = NAME_NODE;   
  doc["Order"] = "stay alive";

  if (Pairing) {
    doc["Pairing"] = "aktiv";
  }

  serializeJson(doc, jsondata);  
  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 100);  
  Serial.print("Sending Ping:"); 
  Serial.println(jsondata);      
}
int  ToggleSwitch(int zNr, int s) {
  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["from"] = NAME_NODE;   
  doc["to"] = Z[zNr].ZubehoerName;
  if (s == 0) doc["Order"] = "ToggleSwitch0";
  if (s == 1) doc["Order"] = "ToggleSwitch1";
  if (s == 2) doc["Order"] = "ToggleSwitch2";
  if (s == 3) doc["Order"] = "ToggleSwitch3";
  
  serializeJson(doc, jsondata);  
  
  for (int ii = 0; ii < 6; ++ii ) TempBroadcastAddress[ii] = (uint8_t) Z[zNr].BroadcastAddress[ii];
  esp_now_send(TempBroadcastAddress, (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  Serial.println(jsondata);
  
  jsondata = "";

  return Z[zNr].Switch[s].Value;
}
void ShowAllSwitch(int zNr) {
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
  
    TFTGauge1.pushToSprite(&TFTBuffer, 22, 25, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer,118, 25, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer, 22,115, 0x4529);
    TFTGauge1.pushToSprite(&TFTBuffer,118,115, 0x4529);
    
    TFTBuffer.loadFont(AA_FONT_SMALL); 
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(MC_DATUM);
    
    if (Debug) TFTBuffer.drawString(Z[zNr].ZubehoerName, 120,15);

    TFTBuffer.setTextColor(TFT_WHITE, 0x18E3, true);
    TFTBuffer.setTextPadding(50); 
    TFTBuffer.drawString(Z[zNr].Switch[0].Name, 70, 85); 
    TFTBuffer.drawString(Z[zNr].Switch[1].Name,170, 85);
    TFTBuffer.drawString(Z[zNr].Switch[2].Name, 70,175);
    TFTBuffer.drawString(Z[zNr].Switch[3].Name,170,175);
    
    TFTBuffer.setTextPadding(0); 
    TFTBuffer.unloadFont(); 

    for (int i=0;i<ANZ_SWITCH;i++) Z[zNr].Switch[i].Changed = true;
    } 
    
    if (Z[zNr].Switch[0].Changed) {
      if (Z[zNr].Switch[0].Value == true) TFTBuffer.pushImage( 59, 53, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage( 59, 53, 27, 10,BtnOff);
        Z[zNr].Switch[0].Changed = false;
    }
    if (Z[zNr].Switch[1].Changed) {
      if (Z[zNr].Switch[1].Value == true) TFTBuffer.pushImage(155,53, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage(155,53, 27, 10,BtnOff);
        Z[zNr].Switch[1].Changed = false;
    }
    if (Z[zNr].Switch[2].Changed) {
      if (Z[zNr].Switch[2].Value == true) TFTBuffer.pushImage( 59,143, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage( 59,143, 27, 10,BtnOff);
        Z[zNr].Switch[2].Changed = false;
    }
    
    if (Z[zNr].Switch[3].Changed) {
      if (Z[zNr].Switch[3].Value == true) TFTBuffer.pushImage(155,143, 27, 10,BtnOn) ; 
      else                                TFTBuffer.pushImage(155,143, 27, 10,BtnOff);
        Z[zNr].Switch[3].Changed = false;
    }
    TimestampScreenRefresh = millis();
}
void ShowAllValues() {
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
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
      if (Z[AktBat].Sensor[i].Changed) {
        noInterrupts(); TempValue = Z[AktBat].Sensor[i].Value; interrupts();
        switch (i) {
          case 0: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, GREEN2RED); break;
          case 1: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, GREEN2RED); break;
          case 2: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, RED2GREEN); break;
          case 3: LinearMeter(TempValue, x,   y+i*(Height+GapY), 3, Height, 0, 35, RED2GREEN); break;
          case 4: LinearMeter(TempValue, x-5, y+i*(Height+GapY), 7, Height, 0, 15, RED2RED);   break;
        }
      }
      TFTBuffer.drawString(Z[AktBat].Sensor[i].Name,x+5, y+5+i*(Height+GapY));
    }
    TFTBuffer.unloadFont();

    TimestampScreenRefresh = millis();
  }  
}
void ShowJSON() {
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if (((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) and (jsondata != "") or (mode != oldMode))) {
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
    TimestampScreenRefresh = millis();
  }
}
void ShowDevice(int zNr) {
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyBackground);  
    TFTBuffer.loadFont(AA_FONT_SMALL);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);
    
    TFTBuffer.drawString(Z[zNr].ZubehoerName, 120, 200); 

    DrawButton(5);
    DrawButton(6);
    DrawButton(7);
    DrawButton(8);

    TimestampScreenRefresh = millis();
  }
}
void ShowDevices() {
  if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
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
      if (Z[zNr].Type) {
        TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
        TFTBuffer.drawString(Z[zNr].ZubehoerName, 10, 80+zNr*Abstand);
        switch (Z[zNr].Type) {
          case 1: ZName = "1-way PDC";  break;
          case 2: ZName = "2-way PDC";  break;
          case 4: ZName = "4-way PDC";  break;
          case 8: ZName = "8-way PDC";  break;
          case 9: ZName = "Batt.-Sens."; break;
        }
        TFTBuffer.drawString(ZName, 105, 80+zNr*Abstand);
        if (millis()- Z[zNr].TimestampLastSeen > OFFLINE) { TFTBuffer.setTextColor(TFT_DARKGREY,  TFT_BLACK); TFTBuffer.drawString("off", 200, 80+zNr*Abstand); }
        else                                              { TFTBuffer.setTextColor(TFT_DARKGREEN, TFT_BLACK); TFTBuffer.drawString("on",  200, 80+zNr*Abstand); }
      }
    }
    TimestampScreenRefresh = millis();
  }
}
void ShowAllText(int zNr)
{ if (mode != oldMode) TimestampScreenRefresh = millis();

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
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
    dtostrf(Z[zNr].Sensor[0].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY); 

    TFTBuffer.drawString("Solar", startX1,startY+Abstand);
    TFTBuffer.drawString("(A):", startX2,startY+Abstand); 
    dtostrf(Z[zNr].Sensor[1].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+Abstand); 
    
    TFTBuffer.drawString("Intern", startX1,startY+2*Abstand); 
    TFTBuffer.drawString("(A):", startX2,startY+2*Abstand); 
    dtostrf(Z[zNr].Sensor[2].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+2*Abstand); 
  
    TFTBuffer.drawString("Load", startX1,startY+3*Abstand); 
    TFTBuffer.drawString("(A):", startX2,startY+3*Abstand); 
    dtostrf(Z[zNr].Sensor[3].Value, len, nk, buf);
    TFTBuffer.drawString(buf, startX3,startY+3*Abstand); 
    
    TFTBuffer.drawString("Battery", startX1,startY+4*Abstand); 
    TFTBuffer.drawString("(V):", startX2,startY+4*Abstand); 
    dtostrf(Z[zNr].Sensor[4].Value, len, nk, buf);
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
    
    TimestampScreenRefresh = millis();
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
  if (mode != oldMode) TimestampScreenRefresh = millis();
  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, JeepifyMenu);
    TimestampScreenRefresh = millis();
  }
}
int  modeLeft() {
  int TempzNr = 0;
  switch (mode) {
          case S_BAT_EXT_A: mode +=1; break;          
          case S_BAT_SOL_A: mode +=1; break;   
          case S_BAT_CAR_A: mode +=1; break; 
          case S_BAT_LOAD : mode +=1; break; 
          case S_BAT_BAT_V: mode +=1; break;   
          case S_BAT_EXT_W: mode +=1; break;   
          case S_BAT_SOL_W: mode +=1; break;   
          case S_BAT_CAR_W: mode +=1; break;   
          case S_BAT_GES_W: 
            TempzNr = AktBat;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr < MAX_ZUBEHOER-1) TempzNr++;
              else TempzNr = 0;
              if (Z[TempzNr].Type == 9) { AktBat = TempzNr; oldMode = 99; break; }
            }
            break; 
          case S_SW_0:      mode +=1; break;
          case S_SW_1:      mode +=1; break;   
          case S_SW_2:      mode +=1; break;   
          case S_SW_3:      mode = S_SW_0; break;   
          case S_VAL_ALL_1: mode = S_SW_ALL; break;   
          case S_VAL_ALL_2: mode = S_SW_ALL; break;  
          case S_SW_ALL: 
            TempzNr = AktPDC;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr < MAX_ZUBEHOER-1) TempzNr++;
              else TempzNr = 0;
              if ((Z[TempzNr].Type == 1) or (Z[TempzNr].Type == 2) or (Z[TempzNr].Type == 4)) { AktPDC = TempzNr; oldMode = 99; break; }
            }
            break;
          case S_TXT_ALL:   mode = S_DEVICES; break;   
          case S_DEVICES:   mode = S_TXT_ALL; break;
          case S_DBG_JSON:  mode = S_TXT_ALL; break;
        }
  return mode;
}
int  modeRight() {
  int TempzNr = 0;
  switch (mode) {
          case S_BAT_EXT_A: mode = S_BAT_GES_W; break;          
          case S_BAT_SOL_A: mode -=1; break;   
          case S_BAT_CAR_A: mode -=1; break;   
          case S_BAT_LOAD : mode -=1; break; 
          case S_BAT_BAT_V: mode -=1; break;   
          case S_BAT_EXT_W: mode -=1; break;   
          case S_BAT_SOL_W: mode -=1; break;   
          case S_BAT_CAR_W: mode -=1; break;   
          case S_BAT_GES_W: 
            TempzNr = AktBat;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr > 0) TempzNr--;
              else TempzNr = MAX_ZUBEHOER-1;
              if (Z[TempzNr].Type == 9) { AktBat = TempzNr; oldMode = 99; break; }
            }
            break;    
          case S_SW_0:      mode = S_SW_3; break;
          case S_SW_1:      mode -=1; break;   
          case S_SW_2:      mode -=1; break;   
          case S_SW_3:      mode -=1; break;   
          case S_VAL_ALL_1: mode = S_SW_ALL; break;   
          case S_VAL_ALL_2: mode -=1; break;  
          case S_SW_ALL:    
            TempzNr = AktPDC;  
            for (int i = 0; i<MAX_ZUBEHOER; i++) {
              if (TempzNr > 0) TempzNr--;
              else TempzNr = MAX_ZUBEHOER-1;
              if ((Z[TempzNr].Type == 1) or (Z[TempzNr].Type == 2) or (Z[TempzNr].Type == 4)) { AktPDC = TempzNr; oldMode = 99; break; }
            }
            break;   
          case S_TXT_ALL:   mode = S_DEVICES; break;   
          case S_DEVICES:   mode = S_TXT_ALL; break;
          case S_DBG_JSON:  mode = S_TXT_ALL; break;
        }
  return mode;
}
int  modeUp() {
  switch (mode) {
          case S_BAT_EXT_A: mode = S_VAL_ALL_1; break;          
          case S_BAT_SOL_A: mode = S_VAL_ALL_1; break;   
          case S_BAT_CAR_A: mode = S_VAL_ALL_1; break;   
          case S_BAT_LOAD : mode = S_VAL_ALL_1; break; 
          case S_BAT_BAT_V: mode = S_VAL_ALL_1; break;   
          case S_BAT_EXT_W: mode = S_VAL_ALL_2; break;   
          case S_BAT_SOL_W: mode = S_VAL_ALL_2; break;   
          case S_BAT_CAR_W: mode = S_VAL_ALL_2; break;   
          case S_BAT_GES_W: mode = S_BAT_BAT_V; break;   
          case S_SW_0:      mode = S_SW_ALL; break;
          case S_SW_1:      mode = S_SW_ALL; break;   
          case S_SW_2:      mode = S_SW_ALL; break;   
          case S_SW_3:      mode = S_SW_ALL; break;   
          case S_VAL_ALL_1: mode = S_BAT_EXT_A; break;   
          case S_VAL_ALL_2: mode = S_BAT_EXT_W; break;  
          case S_SW_ALL:    mode = S_SW_0; break; 
          case S_TXT_ALL:   mode = S_MENU; break;   
          //case S_NODES_ICN: mode = S_MENU; break; 
          //case S_NODES_TXT: mode = S_MENU; break;  
        }
  return mode;
}
int  modeDown() {
  mode = S_MENU;
  return mode;
}
int  RingMeter(float value, float vmin, float vmax, const char *units, const char *bez, byte scheme) {
  int x = 0;
  int y = 0;
  int r = 120;
  int nk = 0;
  
  if (mode != oldMode) TimestampScreenRefresh = millis();
  
  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
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
    
    if (Debug) TFTBuffer.drawString(Z[AktBat].ZubehoerName, 120,225);

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
    TimestampScreenRefresh = millis();
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

    TimestampMsgVolt = millis();
    mode = S_TXT_ALL;
  }
}
void EichenVolt() {
  if (mode != oldMode) {
    TimestampScreenRefresh = millis();
    VoltCount = 0;
    VoltCalib = 0;
  }

  if ((TimestampScreenRefresh - millis() > SCREEN_INTERVAL) or (mode != oldMode)) {
    ScreenChanged = true;
    TFTBuffer.pushImage(0,0, 240, 240, Keypad);  
    TFTBuffer.loadFont(AA_FONT_SMALL); // Must load the font first
    TFTBuffer.setTextDatum(MC_DATUM);
    TFTBuffer.setTextColor(TFT_RUBICON, 0x632C, false);

    char buf[10];
    dtostrf(VoltCalib, 5, 2, buf);

    TFTBuffer.drawString(buf, 120,25);
    TFTBuffer.unloadFont();

    TimestampScreenRefresh = millis();
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

  TimestampMsgEich = millis();
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