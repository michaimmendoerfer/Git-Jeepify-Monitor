#define DEBUG_LEVEL 3
//#define KILL_NVS 1

#include "main.h"
#ifdef MODULE_MONITOR_360 
    #include "scr_st77916.h"
#endif
#ifdef MODULE_MONITOR_240
    #include "scr_tft240round.h"
#endif

#pragma region Globals

const char *T[MAX_PERIPHERALS] =           {"T0",   "T1",   "T2",   "T3",   "T4",   "T5",   "T6",   "T7",   "T8"  };
const char *N[MAX_PERIPHERALS] =           {"N0",   "N1",   "N2",   "N3",   "N4",   "N5",   "N6",   "N7",   "N8"  };
const char *B[MAX_PERIPHERALS] =           {"Br0",  "Br1",  "Br2",  "Br3",  "Br4",  "B5r",  "B6r",  "B7r",  "B8r" };
const char *ArrNullwert[MAX_PERIPHERALS] = {"NW0",  "NW1",  "NW2",  "NW3",  "NW4",  "NW5",  "NW6",  "NW7",  "NW8" };
const char *ArrVperAmp[MAX_PERIPHERALS] =  {"VpA0", "VpA1", "VpA2", "VpA3", "VpA4", "VpA5", "VpA6", "VpA7", "VpA8"};
const char *ArrVin[MAX_PERIPHERALS] =      {"Vin0", "Vin1", "Vin2", "Vin3", "Vin4", "Vin5", "Vin6", "Vin7", "Vin8"};
const char *ArrPeriph[MAX_PERIPHERALS]   = {"Per0", "Per1", "Per2", "Per3", "Per4", "Per5", "Per6", "Per7", "Per8"};

int PeerCount;
Preferences preferences;
uint8_t broadcastAddressAll[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct ConfirmStruct {
    uint8_t  Address[6];
    char     Message[250];
    volatile uint32_t TSMessage;
    int      Try;
    bool     Confirmed;
};

MyLinkedList<ConfirmStruct*> ConfirmList = MyLinkedList<ConfirmStruct*>();

PeerClass Module;

String jsondataBuf;

uint32_t TSPing          = 0;

volatile uint32_t TSMsgRcv  = 0;
volatile uint32_t TSMsgSnd  = 0;
volatile uint32_t TSMsgPDC  = 0;
volatile uint32_t TSMsgBat  = 0;
volatile uint32_t TSMsgVolt = 0;
volatile uint32_t TSMsgEich = 0;
volatile uint32_t TSMsgPair = 0;
volatile uint32_t TSPair    = 0;

volatile uint32_t TSConfirm = 0;

lv_timer_t *WDButtonVars;

int ActiveMultiScreen;
bool WebServerActive = true;
#pragma endregion Globals

#pragma region WebServer
AsyncWebServer server(80);
const char* ssid = "Jeepify_Monitor";
const char* password = "";

const char* PARAM_MESSAGE = "message";

PeerClass   *ActiveWebPeer = NULL;
PeriphClass *ActiveWebPeriph = NULL;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Form</title>
<style>
body{ margin: 0;padding: 0;font-family: Arial, Helvetica, sans-serif;background-color: #2c257a;}
.box{ width: 70%%; padding: 10px; position: absolute; top: 50%%; left: 50%%; transform: translate(-50%%,-50%%); background-color: #191919; color: white; text-align: center; border-radius: 24px; box-shadow: 0px 1px 32px 0px rgba(0,227,197,0.59);}
h1{ text-transform: uppercase; font-weight: 500;}
input{ border: 0; background: none; margin: 20px auto; text-align: center; border: 2px solid #4834d4; padding: 14px 10px; width: 40%%; outline: none; border-radius: 24px; color: white; font-size: smaller; transition: 0.3s;}
input:focus{ width: 40%%; border-color:#22a6b3 ;}
input[type='submit']{ border: 0;background: none; margin: 20px auto; text-align: center; border: 2px solid #22a6b3; padding: 14px 10px; width: 120px; outline: none; border-radius: 24px; color: white; transition: 0.3s; cursor: pointer;}
input[type='submit']:hover{ background-color: #22a6b3;}
</style>
</head>
<body>
<p>&nbsp;</p>
<form id="values" class="box" action="/get">
<h1>%PeerName%</h1>
<div class="part"><input name="PeerName" type="text" placeholder="%PeerName%" />
<input name="message" type="submit" value="UpdPeerName" /></div>

<div class="part"><input name="PeriphName" type="%TYPE%" placeholder="%PeriphName%" />
<input name="message" type="submit" value="UpdPeriphName" /></div>
<div class="part"><input name="Nullwert" type="%TYPE%" placeholder="%Nullwert%" />
<input name="message" type="submit" value="UpdNullwert" /></div>
<div class="part"><input name="VperAmp" type="%TYPE%" placeholder="%VperAmp%" />
<input name="message" type="submit" value="UpdVperAmp" /></div>
<div class="part"><input name="Vin" type="%TYPE%" placeholder="%Vin%" />
<input name="message" type="submit" value="UpdVin" /></div>

<div class="part">
<table align="center">
<tbody>
<tr>
<td>
<div class="part"><input name="message" type="submit" value="module" /></div>
</td>
</tr>
<tr>
<td>
<div class="part"><input name="message" type="submit" value="prev" /></div>
</td>
<td>
<div class="part"><input name="message" type="submit" value="next" /></div>
</td>
</tr>
</tbody>
</table>
</div>
</form></body></html>
)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var)
{
    char Buf[10];
    
    if (var == "TYPE")        if (ActiveWebPeriph) return "text";
                              else return "hidden";
    if (var == "PeerName")    return ActiveWebPeer->GetName();
    if (var == "PeriphName")  if (ActiveWebPeriph) return ActiveWebPeriph->GetName();
    if (var == "Nullwert")    if (ActiveWebPeriph) { dtostrf(ActiveWebPeriph->GetNullwert(), 0, 3, Buf); return String(Buf); }
    if (var == "VperAmp")     if (ActiveWebPeriph) { dtostrf(ActiveWebPeriph->GetVperAmp(), 0, 3, Buf); return String(Buf); }
    if (var == "Vin")         if (ActiveWebPeriph) return String(ActiveWebPeriph->GetVin());
    
    return String();
}
bool SendWebPeriphNameChange()
{
    JsonDocument doc; String jsondata; 

    doc["from"]    = Module.GetName();   
    doc["Order"]   = SEND_CMD_UPDATE_NAME;
    doc["NewName"] = ActiveWebPeriph->GetName();
    doc["Pos"]     = ActiveWebPeriph->GetPos();
    
    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(ActiveWebPeer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s\n\r", jsondata.c_str());

    return true;
}
bool SendWebPeerNameChange()
{
    JsonDocument doc; String jsondata; 
    
    doc["from"]    = Module.GetName();
    doc["Order"]   = SEND_CMD_UPDATE_NAME;
    doc["NewName"] = ActiveWebPeer->GetName();
    doc["Pos"]     = 99;
    
    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(ActiveWebPeer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s\n\r", jsondata.c_str());

    return true;
}
bool SendWebVinChange()
{
    JsonDocument doc; String jsondata; 
    
    doc["from"]    = Module.GetName();  
    doc["Order"]   = SEND_CMD_UPDATE_VIN;
    doc["Value"]   = ActiveWebPeriph->GetVin();
    doc["Pos"]     = ActiveWebPeriph->GetPos();
    
    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(ActiveWebPeer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s\n\r", jsondata.c_str());

    return true;
}
bool SendWebVperAmpChange()
{
    JsonDocument doc; String jsondata; 
    
    doc["from"]    = Module.GetName();   
    doc["Order"]   = SEND_CMD_UPDATE_VPERAMP;
    doc["Value"]   = ActiveWebPeriph->GetVperAmp();
    doc["Pos"]     = ActiveWebPeriph->GetPos();
    
    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(ActiveWebPeer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s\n\r", jsondata.c_str());

    return true;
}
bool SendWebNullwertChange()
{
    JsonDocument doc; String jsondata; 
    
    doc["from"]    = Module.GetName();
    doc["Order"]   = SEND_CMD_UPDATE_NULLWERT;
    doc["Value"]   = ActiveWebPeriph->GetNullwert();
    doc["Pos"]     = ActiveWebPeriph->GetPos();
    
    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(ActiveWebPeer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s\n\r", jsondata.c_str());
    return true;
}

void InitWebServer()
{
    Serial.printf("create AP = %d", WiFi.softAP(ssid, password));
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    int txPower = WiFi.getTxPower();
    Serial.print("TX power: ");
    Serial.println(txPower);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    ActiveWebPeer = &Module;
    ActiveWebPeriph = NULL;
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html, processor);
    });
    
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        String WebBuffer;

        bool SaveNeeded  = false;

        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
            if (message == "UpdPeerName") 
            {
                if (request->hasParam("PeerName"))
                {
                  WebBuffer = request->getParam("PeerName")->value();
                  if (WebBuffer != "")
                    { 
                        DEBUG3 ("Received from web: NewPeerName = %s\n\r", WebBuffer.c_str());  
                        if (ActiveWebPeer) 
                        {   
                            SaveNeeded = true;
                            ActiveWebPeer->SetName(WebBuffer.c_str());
                            if (ActiveWebPeer != &Module) SendWebPeerNameChange();
                        }
                    }
                }
            }   
            if (message == "UpdPeriphName") 
            {    
                if (request->hasParam("PeriphName"))
                {
                  WebBuffer = request->getParam("PeriphName")->value();
                  if (WebBuffer != "")
                    { 
                        DEBUG3 ("Received from web: NewPeriphName = %s\n\r", WebBuffer.c_str());  
                        if (ActiveWebPeriph) 
                        {
                            SaveNeeded = true;
                            ActiveWebPeriph->SetName(WebBuffer.c_str());
                            if (ActiveWebPeer != &Module) SendWebPeriphNameChange();
                        }
                    }
                }
            }
            if (message == "UpdNullwert") 
            {
                if (request->hasParam("Nullwert"))
                {
                  WebBuffer = request->getParam("Nullwert")->value();
                  if (WebBuffer != "")
                    { 
                        DEBUG3 ("Received from web: NewNullwert = %s\n\r", WebBuffer.c_str());  
                        if (ActiveWebPeriph) 
                        {
                            SaveNeeded = true;
                            ActiveWebPeriph->SetNullwert(atof(WebBuffer.c_str()));
                            if (ActiveWebPeer != &Module) SendWebNullwertChange();
                        }
                    }
                }
            }
            if (message == "UpdVperAmp") 
            {   
                if (request->hasParam("VperAmp"))
                {
                  WebBuffer = request->getParam("VperAmp")->value();
                  if (WebBuffer != "")
                    { 
                        DEBUG3 ("Received from web: NewVperAmp = %s\n\r", WebBuffer.c_str());  
                        if (ActiveWebPeriph) 
                        {
                            SaveNeeded = true;
                            ActiveWebPeriph->SetVperAmp(atof(WebBuffer.c_str()));
                            if (ActiveWebPeer != &Module) SendWebVperAmpChange();
                        }
                    }
                }
            }
            if (message == "UpdVin") 
            {
                if (request->hasParam("Vin"))
                {
                  WebBuffer = request->getParam("Vin")->value();
                  if (WebBuffer != "")
                    { 
                        DEBUG3 ("Received from web: NewVin = %s\n\r", WebBuffer.c_str());  
                        if (ActiveWebPeriph) 
                        {
                            SaveNeeded = true;
                            ActiveWebPeriph->SetVperAmp(atoi(WebBuffer.c_str()));
                            if (ActiveWebPeer != &Module) SendWebVinChange();
                        }
                    }
                }  
            }
            
            if (message == "module") 
            {
                DEBUG3 ("Module aufgerufen\n\r");
                ActiveWebPeer   = &Module;
                ActiveWebPeriph = NULL;
                DEBUG2 ("aktueller Name = %s\n\r", ActiveWebPeer->GetName());
            }
            if (message == "prev") 
            {
                DEBUG3 ("Prev aufgerufen\n\r");
                if (ActiveWebPeer == &Module) 
                {
                    PeerClass *TempP = FindNextPeer(NULL, MODULE_ALL, CIRCULAR, ONLINE);
                    if (TempP) ActiveWebPeer = TempP;
                    ActiveWebPeriph = FindNextPeriph(ActiveWebPeer, NULL, SENS_TYPE_ALL, ONLINE);  
                }
                else
                {
                    ActiveWebPeriph = FindPrevPeriph(NULL, ActiveWebPeriph, SENS_TYPE_ALL, CIRCULAR, ONLINE);
                    ActiveWebPeer   = FindPeerById(ActiveWebPeriph->GetPeerId());
                }

            }
            if (message == "next") 
            {
                DEBUG3 ("Next aufgerufen\n\r");
                if (ActiveWebPeer == &Module) 
                {
                    PeerClass *TempP = FindNextPeer(NULL, MODULE_ALL, 1);
                    if (TempP) ActiveWebPeer = TempP;
                    ActiveWebPeriph = FindNextPeriph(ActiveWebPeer, NULL, SENS_TYPE_ALL, 1);  
                }
                else
                {
                    ActiveWebPeriph = FindNextPeriph(NULL, ActiveWebPeriph, SENS_TYPE_ALL, true, 1);
                    ActiveWebPeer   = FindPeerById(ActiveWebPeriph->GetPeerId());
                }
            }
        } else {
            message = "No message sent";
        }
        request->send_P(200, "text/html", index_html, processor);
        
        if (SaveNeeded)
        {   
            if (ActiveWebPeer != &Module) SavePeers();
            else 
            {
                preferences.begin("JeepifyInit", false);
                preferences.putString("ModuleName", Module.GetName());
                preferences.end();
                DEBUG2 ("Neuer Module Name:%s gespeichert\n\r", Module.GetName());
            }
            SaveNeeded = false;
        }
    });

  server.onNotFound(notFound);
}
void ToggleWebServer()
{   
    WebServerActive = !WebServerActive;
    if (WebServerActive) 
    {
        ActiveWebPeer = PeerList.get(0);
        ActiveWebPeriph = PeriphList.get(0);
        DEBUG3 ("Server startet\n\r");
        server.begin();
    }
    else 
    {
        DEBUG3 ("Server beendet\n\r");
        server.end();
    }
}
#pragma endregion WebServer

#pragma region Main

#ifdef MODULE_MONITOR_360 
    void OnDataRecv(const esp_now_recv_info *info, const uint8_t* incomingData, int len) 
#endif
#ifdef MODULE_MONITOR_240
    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
#endif
{
    PeerClass *P;
    
    char* buff = (char*) incomingData;   
    JsonDocument doc; 
    String jsondata = String(buff); 
    
    String BufS; char Buf[50] = {};
    bool SaveNeeded = false;
    bool NewPeer    = false;
    char buf[100];
    
    jsondataBuf = jsondata;
    PrepareJSON();
    DEBUG2 ("%s\n\r", jsondata.c_str());

    DeserializationError error = deserializeJson(doc, jsondata);

    if (!error) // erfolgreich JSON
    {
        strcpy(buf, doc["Node"]);

        char       *_PeerName    = strtok(buf, ";");
        uint32_t    _Uptime      = atoi(strtok(NULL, ";"));
        int         _Status      = atoi(strtok(NULL, ";"));
        int         _Type        = (int) (doc["Type"]);
        const char *_PeerVersion = doc["Version"];
        int         _Order       = (int)doc["Order"];   
        int         _TSConfirm   = (int)doc["TSConfirm"];
        
        #ifdef MODULE_MONITOR_360 
            P = FindPeerByMAC(info->src_addr);
        #endif
        #ifdef MODULE_MONITOR_240
            P = FindPeerByMAC(mac);
        #endif

        if (P)
        {
            if ((P) and (Module.GetDebugMode()) and (millis() - P->GetTSLastSeen() > OFFLINE_INTERVAL)) ShowMessageBox("Peer online", P->GetName(), 1000, 200);
            P->SetTSLastSeen(millis());
            #ifdef MODULE_MONITOR_360 
                P->SetdBm(info->rx_ctrl->rssi); 
            #endif
            #ifdef MODULE_MONITOR_240
                P->SetdBm(0); 
            #endif
        }

        TSMsgRcv = millis();

        switch (_Order)
        {
            case SEND_CMD_PAIR_ME:
                // new Peer wants to pair and module too - create it
                if ((!P) and Module.GetPairMode())
                {
                    P = new PeerClass();
                    PeerList.add(P);
                    SaveNeeded = true;
                    NewPeer    = true;
                    Module.SetPairMode(false); TSPair = 0;
                    
                    #ifdef MODULE_MONITOR_360 
                        P->Setup(_PeerName, _Type, _PeerVersion, info->src_addr, (bool) bitRead(_Status, 1), (bool) bitRead(_Status, 0), (bool) bitRead(_Status, 2), (bool) bitRead(_Status, 3));                    
                    #endif
                    #ifdef MODULE_MONITOR_240
                        P->Setup(_PeerName, _Type, _PeerVersion, mac,            (bool) bitRead(_Status, 1), (bool) bitRead(_Status, 0), (bool) bitRead(_Status, 2), (bool) bitRead(_Status, 3));                    
                    #endif

                    if (Module.GetDebugMode()) ShowMessageBox("Peer added...", doc["Node"], 2000, 150);
                    SendPairingConfirm(P); 

                    for (int Si=0; Si<MAX_PERIPHERALS; Si++) 
                    {
                        DEBUG3 ("Check Pairing for: %s\n\r", ArrPeriph[Si]);
                        
                        if (doc[ArrPeriph[Si]].is<JsonVariant>())
                        {
                            strcpy(buf, doc[ArrPeriph[Si]]);
                            int   _PeriphType = atoi(strtok(buf, ";"));
                            char *_PeriphName = strtok(NULL, ";");
                            P->PeriphSetup(Si, _PeriphName, _PeriphType, P->GetId()); 
                            P->SetPeriphChanged(Si, true);
                            PeriphList.add(P->GetPeriphPtr(Si));
                            SaveNeeded = true;
                            DEBUG2 ("%s->Periph[%d].Name is now: %s\n\r", P->GetName(), Si, P->GetPeriphName(Si));
                        }
                    }
                }
                break;
            case SEND_CMD_STATUS:
                if (P)
                {
                    // check for module name change
                    if (strcmp(_PeerName, P->GetName())) P->SetName(_PeerName);
                    for (int Si=0; Si<MAX_PERIPHERALS; Si++) 
                    {
                        //DEBUG3 ("Check values of: %s\n\r", ArrPeriph[Si]);
                        
                        if (doc[ArrPeriph[Si]].is<JsonVariant>())
                        {
                            strcpy(buf, doc[ArrPeriph[Si]]);
                            int   _PeriphType = atoi(strtok(buf, ";"));
                            char *_PeriphName = strtok(NULL, ";");
                            float _Value0     = atof(strtok(NULL, ";"));
                            float _Value1     = atof(strtok(NULL, ";"));
                            float _Value2     = atof(strtok(NULL, ";"));
                            float _Value3     = atof(strtok(NULL, ";"));
                            
                            // check for periph name change
                            if (strcmp(_PeriphName, P->GetPeriphName(Si))) P->SetPeriphName(Si, _PeriphName);
                            
                            P->SetPeriphOldValue(Si, P->GetPeriphValue(Si, 0), 0);
                            P->SetPeriphValue(Si, _Value0, 0);
                            P->SetPeriphOldValue(Si, P->GetPeriphValue(Si, 1), 1);
                            P->SetPeriphValue(Si, _Value1, 1);
                            P->SetPeriphOldValue(Si, P->GetPeriphValue(Si, 2), 2);
                            P->SetPeriphValue(Si, _Value2, 2);
                            P->SetPeriphOldValue(Si, P->GetPeriphValue(Si, 3), 3);
                            P->SetPeriphValue(Si, _Value3, 3);
                            
                            P->SetPeriphChanged(Si, false); //werte wieder uptodate

                            if (_Status)
                            {
                                P->SetDebugMode ((bool) bitRead(_Status, 0));
                                P->SetSleepMode ((bool) bitRead(_Status, 1));
                                P->SetDemoMode  ((bool) bitRead(_Status, 2));
                                P->SetPairMode  ((bool) bitRead(_Status, 3));
                            } 
                            
                            DEBUG3 ("%s->%s values are: %.2f - %.2f - %.2f - %.2f\n\r", P->GetName(), P->GetPeriphName(Si), 
                                P->GetPeriphValue(Si, 0), P->GetPeriphValue(Si, 1), P->GetPeriphValue(Si, 2), P->GetPeriphValue(Si, 3));
                        }
                    }
                }
                break;
    
            case SEND_CMD_CONFIRM:
                if ((P) and (doc["TSConfirm"].is<JsonVariant>()))
                {                    
                    DEBUG2 ("Confirm (%lu) empfangen von %s\n\r", _TSConfirm, P->GetName());
                    for (int i=0; i<ConfirmList.size(); i++)
                    {
                        ConfirmStruct *TempConfirm;
                        TempConfirm = ConfirmList.get(i);
                        DEBUG3 ("empfangener TS ist: %lu - durchsuchter TS (List[%d]) ist: %lu\n\r", _TSConfirm, i, TempConfirm->TSMessage);
                        if (TempConfirm->TSMessage == _TSConfirm)
                        {
                            TempConfirm->Confirmed = true;
                            DEBUG2 ("Found at list[%d] - DELETED\n\r", i);
                        }
                    }
                }
                break;
        } 
        
        if (SaveNeeded)
        {
            SavePeers();
            SaveNeeded = false;
            if (Module.GetDebugMode()) ShowMessageBox("Saving...", "complete", 1000, 200);
        }
    }
    else // Error bei JSON
    {        
        Serial.print(F("deserializeJson() failed: ")); 
        Serial.println(error.f_str());
        Serial.printf("jsondata was: %s\n\r", jsondata);
        return;
    }
}

void setup() 
{
    delay(2000);
    Serial.begin(115200);
    scr_lvgl_init();

    Module.Setup(NODE_NAME, NODE_TYPE, MODULE_VERSION, broadcastAddressAll, false, true, false, false);
    
    #ifdef KILL_NVS
        nvs_flash_erase(); nvs_flash_init();
        while(1)
        {}
    #endif
    
    WiFi.mode(WIFI_AP_STA);
    InitWebServer();
    
    WebServerActive = !WebServerActive;
    ToggleWebServer();

    if (esp_now_init() != ESP_OK) { DEBUG1 ("Error initializing ESP-NOWn\r"); return; }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);   
    
    //Get saved Peers  
    preferences.begin("JeepifyInit", true);
        Module.SetDebugMode(preferences.getBool("DebugMode", true));
        Module.SetSleepMode(preferences.getBool("SleepMode", false));
        Module.SetName(preferences.getString("ModuleName", NODE_NAME).c_str());
    preferences.end();
    
    GetPeers();
    RegisterPeers();
    ReportAll();
  
    static uint32_t user_data = 10;
    lv_timer_t * TimerPing = lv_timer_create(SendPing, PING_INTERVAL,  &user_data);

    ui_init();
}
void loop() 
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(10);
}
#pragma endregion Main

#pragma region Send-Things
esp_err_t  JeepifySend(PeerClass *P, const uint8_t *data, size_t len, uint32_t TSConfirm, bool ConfirmNeeded = false)
{
    esp_err_t SendStatus = esp_now_send(P->GetBroadcastAddress(), data, len);
    
    DEBUG3 ("SendStatus was %d, ConfirmNeeded = %d\n\r", SendStatus, ConfirmNeeded);
    if (ConfirmNeeded)
    {   
        ConfirmStruct *Confirm = new ConfirmStruct;
        memcpy(Confirm->Address, P->GetBroadcastAddress(), 6);
        strcpy(Confirm->Message, (const char *)data);
        Confirm->Confirmed = false;
        Confirm->TSMessage = TSConfirm;
        Confirm->Try = 1;

        ConfirmList.add(Confirm);

        DEBUG2 ("added Msg: %s to ConfirmList\n\r", Confirm->Message, Confirm->Try);   
    }
    return SendStatus;
}
void SendPing(lv_timer_t * timer) {
    JsonDocument doc; String jsondata;

    PeerClass *P;
    
    doc["Node"] = NODE_NAME;   
    doc["Order"] = SEND_CMD_STAY_ALIVE;

    if (Module.GetPairMode())
    {
        doc["Pairing"] = "aktiv";
    }

    serializeJson(doc, jsondata);  
    
    for (int i=0; i<PeerList.size(); i++)
    {
        P = PeerList.get(i);
        //PrintMAC(P->GetBroadcastAddress());
        //Serial.printf(" - Sende Stay Alive an: %s\n\r", P->GetName());
        
        if (P->GetType() > 0) esp_now_send(P->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  
    }

    if (ConfirmList.size() > 0)
    { 
        for (int i=ConfirmList.size()-1; i>=0; i--)
        {
            ConfirmStruct *Confirm = ConfirmList.get(i);
            Confirm->Try++;
            
            if (Confirm->Confirmed == true)
            {
                char TxtBuf[100];
                if (Module.GetDebugMode()) sprintf(TxtBuf, "SUCCESS - Message to %s successful confirmed after %d tries!", FindPeerByMAC(Confirm->Address)->GetName(), Confirm->Try);
                if (Module.GetDebugMode()) ShowMessageBox("SUCCESS", TxtBuf, 1000, 200);
                ConfirmList.remove(i);
                delete Confirm;
            }
            else if (Confirm->Try == JEEPIFY_SEND_MAX_TRIES+1)
            {
                char TxtBuf[100];
                if (Module.GetDebugMode()) sprintf(TxtBuf, "FAILED - Message to %s deleted after %d tries!", FindPeerByMAC(Confirm->Address)->GetName(), Confirm->Try);
                if (Module.GetDebugMode()) ShowMessageBox("FAILED", TxtBuf, 1000, 200);
                ConfirmList.remove(i);
                delete Confirm;
            }
            else
            {
                DEBUG3 ("%d: reSending Msg: %s from ConfirmList Try: %d\n\r", millis(), Confirm->Message, Confirm->Try);
                esp_err_t SendStatus = esp_now_send(Confirm->Address, (uint8_t*) Confirm->Message, 200); 
            }     
        }
    }
}

void SendPairingConfirm(PeerClass *P) {
  JsonDocument doc; String jsondata; 
  
  uint8_t *Broadcast = P->GetBroadcastAddress();
  
  doc["Node"]     = Module.GetName();   
  doc["Peer"]     = P->GetName();
  doc["Order"]    = SEND_CMD_YOU_ARE_PAIRED;
  doc["Type"]     = Module.GetType();
  doc["B0"]       = (uint8_t)Broadcast[0]; doc["B1"] = (uint8_t)Broadcast[1]; doc["B2"] = (uint8_t)Broadcast[2];
  doc["B3"]       = (uint8_t)Broadcast[3]; doc["B4"] = (uint8_t)Broadcast[4]; doc["B5"] = (uint8_t)Broadcast[5];

  serializeJson(doc, jsondata);  
  
  TSMsgSnd = millis();
  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200); 
  DEBUG2 ("Sent you are paired\n\r%s\n\r", jsondata.c_str());  
     
}
bool ToggleSwitch(PeerClass *P, int PerNr)
{
    return ToggleSwitch(P->GetPeriphPtr(PerNr));
}
bool ToggleSwitch(PeriphClass *Periph)
{
    JsonDocument doc; String jsondata; 
    
    doc["From"]         = Module.GetName();  
    doc["Order"]        = SEND_CMD_SWITCH_TOGGLE;
    doc["PeriphName"]   = Periph->GetName();
    doc["PeriphPos"]    = Periph->GetPos();

    serializeJson(doc, jsondata);  
    
    TSMsgSnd = millis();
    esp_now_send(FindPeerById(Periph->GetPeerId())->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
    DEBUG3 ("%s", jsondata.c_str());
    
    return true;
}
void SendCommand(PeerClass *P, int Cmd, String Value) {
  JsonDocument doc; String jsondata; 
  
  doc["From"]  = Module.GetName();   
  doc["Order"] = Cmd;
  if (Value != "") doc["Value"] = Value;
  
  serializeJson(doc, jsondata);  
  
  TSMsgSnd = millis();
  esp_now_send(P->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 100);  //Sending "jsondata"  
  DEBUG3 ("%s", jsondata.c_str());
}
#pragma endregion Send-Things
#pragma region System-Screens
void PrepareJSON() {
  if (jsondataBuf) {
    JsonDocument doc;
  
    DeserializationError error = deserializeJson(doc, jsondataBuf);
    if (doc["Node"] != NODE_NAME) { 
      lv_textarea_set_placeholder_text(ui_TxtJSON1, jsondataBuf.c_str());
      jsondataBuf = "";
    }
  }
}
#pragma endregion System-Screens
#pragma region Other
void ShowMessageBox(const char * Titel, const char *Txt, int delay, int opa)
{
    static const char * btns[] = {""};

    lv_obj_t *MsgBox = lv_msgbox_create(lv_scr_act(), Titel, Txt, NULL, false);
    lv_obj_set_style_bg_color(MsgBox, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(MsgBox, opa, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(MsgBox, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(MsgBox, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(MsgBox, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(MsgBox, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(MsgBox, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(MsgBox, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_font(lv_msgbox_get_title(MsgBox), &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_center(MsgBox);
    lv_obj_del_delayed(MsgBox, delay);
}  
bool ToggleSleepMode() 
{
    preferences.begin("JeepifyInit", false);
        Module.SetSleepMode(!Module.GetSleepMode());
        if (preferences.getBool("SleepMode", false) != Module.GetSleepMode()) preferences.putBool("SleepMode", Module.GetSleepMode());
    preferences.end();
    return Module.GetSleepMode();
}
bool ToggleDebugMode() {
  preferences.begin("JeepifyInit", false);
      Module.SetDebugMode(!Module.GetDebugMode());
      if (preferences.getBool("DebugMode", false) != Module.GetDebugMode()) preferences.putBool("DebugMode", Module.GetDebugMode());
  preferences.end();
  return Module.GetDebugMode();
}
bool TogglePairMode() {
  if (Module.GetPairMode())
  {
      Module.SetPairMode(false);
      TSPair = 0;
  }
  else 
  {
      Module.SetPairMode(true);
      TSPair = millis();
  };

  DEBUG2 ("PairMode changed to: %d\n\r", Module.GetPairMode());
  
  return Module.GetPairMode();
}
void CalibVolt() {
    JsonDocument doc; String jsondata;
    
    uint32_t TSConfirm = millis();

    doc["Node"]  = Module.GetName();  
    doc["Order"] = SEND_CMD_VOLTAGE_CALIB;
    doc["NewVoltage"] = lv_textarea_get_text(ui_TxtVolt);
    doc["TSConfirm"] = TSConfirm;
    
    serializeJson(doc, jsondata);  

    JeepifySend(ActivePeer, (uint8_t *) jsondata.c_str(), 100, TSConfirm, true);  
    Serial.println("zurück von JeepifySend-warte");
    delay(1000);
    Serial.println("zurück von JeepifySend");
        
    DEBUG3 ("%s", jsondata.c_str());
}
void CalibAmp() 
{
    Serial.println("CalibAmp beginnt");
    JsonDocument doc; String jsondata;

    uint32_t TSConfirm = millis();

    doc["Node"]  = Module.GetName();  
    doc["Order"] = SEND_CMD_CURRENT_CALIB;
    doc["TSConfirm"] = TSConfirm;
    
    serializeJson(doc, jsondata);  
    JeepifySend(ActivePeer, (uint8_t *) jsondata.c_str(), 100, TSConfirm, true);  

    DEBUG3 ("%s", jsondata.c_str());
}
void PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{ 
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        DEBUG3 ("Message send SUCCESS\n\r");
    }
    else 
    {
        DEBUG1 ("Message send FAILED\n\r");
    }
}

#pragma endregion Other
//
