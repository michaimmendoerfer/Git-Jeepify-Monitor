/*
include PeerList and PeriphList
Version 3.30
*/

#ifndef PEERCLASS_H
#define PEERCLASS_H

#include <Arduino.h>
#include "Jeepify.h"
#include "LinkedList.h"

#define IO_SW_ON   0
#define IO_SW_OFF  1
#define IO_SW_VOLT 2
#define IO_SW_AMP  3

#define ONLINE     1
#define OFFLINE    0
#define CIRCULAR   1
#define NO_CIRC    0

#define V_ON       0
#define V_OFF      1
#define V_VOLT     2
#define V_AMP      3

class PeriphClass {
    static int  _ClassId;

    private:
        char            _Name[20];
        int             _Id;
        int             _Type;       //1=Switch, 2=Amp, 3=Volt, 4=Switch/Amp, 5=LatchingRelay, 6=LatchingRelay/Amp
        int             _Pos;        //Periph 1..4.. from one peer
        bool            _i2c[4];     //
        int             _IOPort[4];  //on/off/volt/amp
        int             _I2CPort[4]; //corresponding i2c-channel
        float           _Nullwert;
        float           _VperAmp;
        float           _Vin;
        volatile float  _Value[4];   //switch/---/volt/amp
        float           _OldValue[4];
        bool            _Changed;
        int             _PeerId;

    protected:
        float           _SavedValue[RECORDED_VALUES][4];
        uint32_t        _SavedValueTS[RECORDED_VALUES];
        int             _SavedValueIndex;
    
    public:
        PeriphClass();
        void  Setup(const char* Name, int Type, bool isADS, 
                    int I2CPort0, int I2CPort1, int I2CPort2, int I2CPort3, 
                    int IOPort0,  int IOPort1,  int IOPort2,  int IOPort3, 
                    float Nullwert, float VperAmp, float Vin, int PeerId);
        void  Setup(const char* Name, int Type, int PeerId);
        
        bool  SetName(const char* Name) { strcpy(_Name, Name); return true; }
        char *GetName(){ return (_Name); }
        int   GetId() { return _Id; }
        void  SetId(int Id) { _Id = Id; }
        int   GetType() { return _Type; }
        void  SetType(int Type) { _Type = Type; }
        bool  IsType(int Type);
        int   GetPos() { return _Pos; }
        void  SetPos(int Pos) {_Pos = Pos; }
        int   GetIOPort(int i) { return _IOPort[i]; }
        void  SetIOPort(int i, int IOPort) { _IOPort[i] = IOPort; }
        int   GetI2CPort(int i) { return _I2CPort[i]; }
        void  SetI2CPort(int i, int _I2C) { _I2CPort[i] = _I2C; }
        float GetNullwert() { return _Nullwert; }
        void  SetNullwert(float Nullwert) { _Nullwert = Nullwert; }
        float GetVperAmp() { return _VperAmp; }
        void  SetVperAmp(float VperAmp) { _VperAmp = VperAmp; }
        float GetVin() { return _Vin; }
        void  SetVin(float Vin) { _Vin = Vin; }
        float GetValue(int i=0) { return _Value[i]; }
        void  SetValue(float Value, int i=0) { _Value[i] = Value; }
        float GetOldValue(int i=0) { return _OldValue[i]; }
        void  SetOldValue(float OldValue, int i=0) { _OldValue[i] = OldValue; }
        bool  hasChanged() { return _Changed; }
        bool  GetChanged() { return _Changed; }
        void  SetChanged(bool Changed) { _Changed = Changed; }
        int   GetPeerId() { return _PeerId; }
        void  SetPeerId(int PeerId) { _PeerId = PeerId; }
        bool  IsSensor() { return ((_Type == SENS_TYPE_VOLT) or (_Type == SENS_TYPE_AMP)); }
        bool  IsSwitch() { return ((_Type == SENS_TYPE_SWITCH) or (_Type == SENS_TYPE_SW_AMP) or (_Type == SENS_TYPE_LT) or (_Type == SENS_TYPE_LT_AMP)); }
        bool  IsCombo()  { return ((_Type == SENS_TYPE_SW_AMP) or (_Type == SENS_TYPE_LT_AMP)); }
        bool  isEmpty() { return (_Type == 0); }
        
        float GetSavedValue(int Index, int i) { return _SavedValue[Index][i]; }
        void  AddSavedValue(float V0, float V1, float V2, float V3);
        int   GetSavedValueIndex() { return _SavedValueIndex; }

        PeriphClass *GetPtrToSelf() { return this; }
};

class PeerClass 
{
    static int _ClassId;

    private:
        char                _Name[20];
        int                 _Id;
        int                 _Type;  
        char                _Version[10];
        u_int8_t            _BroadcastAddress[6];
        bool                _SleepMode;
        bool                _DebugMode;
        bool                _DemoMode;
        bool                _PairMode;
        bool                _Changed;
        PeriphClass         Periph[MAX_PERIPHERALS]; 
        uint32_t            _TSLastSeen;
        int                 _VoltageMon;
        int                 _RelayType;
        float               _VoltageDevider;
        int                 _Brightness;
        int                 _dBm;
        
    public:
        PeerClass();
        void  Setup(const char* Name, int Type, const char *Version, const uint8_t *BroadcastAddress, 
                    bool SleepMode, bool DebugMode, bool DemoMode, bool PairMode);
        char* Export();
        void  Import(char *Buf);

        void  SetName(const char *Name) { strcpy(_Name, Name); }
        char *GetName() { return (_Name); }
        void  SetVersion(const char *Version) { strcpy(_Version, Version); }
        char *GetVersion() { return (_Version); }
        int   GetId() { return _Id; }
        void  SetId(int Id) { _Id = Id; }
        int   GetType() { return _Type; }
        void  SetType(int Type) { _Type = Type; }
        uint8_t *GetBroadcastAddress() { return _BroadcastAddress; }
        void     SetBroadcastAddress(const uint8_t *BroadcastAddress) { memcpy(_BroadcastAddress, BroadcastAddress, 6); }
        uint32_t GetTSLastSeen() { return _TSLastSeen; }
        void     SetTSLastSeen(uint32_t TSLastSeen) { _TSLastSeen = TSLastSeen; }
        bool  GetSleepMode() { return _SleepMode; }
        void  SetSleepMode(bool SleepMode) { _SleepMode = SleepMode; }
        bool  GetDebugMode() { return _DebugMode; }
        void  SetDebugMode(bool DebugMode) { _DebugMode = DebugMode; }
        bool  GetDemoMode() { return _DemoMode; }
        void  SetDemoMode(bool DemoMode) { _DemoMode = DemoMode; }
        bool  GetPairMode() { return _PairMode; }
        void  SetPairMode(bool PairMode) { _PairMode = PairMode; }
        bool  GetChanged() { return _Changed; }
        void  SetChanged(bool Changed) { _Changed = Changed; }
        int   GetRelayType() { return _RelayType; }
        void  SetRelayType(int RelayType) { _RelayType = RelayType; }

        bool  TogglePairMode() { _PairMode = !_PairMode; return _PairMode; }
    
        int   GetBrightness() { return _Brightness; }
        void  SetBrightness(int Brightness) {_Brightness = Brightness; }
        
        void  PeriphSetup(int Pos, const char* Name, int Type, 
                          int I2CPort0, int I2CPort1, int I2CPort2, int I2CPort3, 
                          int IOPort0, int IOPort1, int IOPort2, int IOPort3,  
                          float Nullwert, float VperAmp, float Vin, int PeerId);
        void  PeriphSetup(int Pos, const char* Name, int Type, int PeerId);
        
        char *GetPeriphName(int P) { return Periph[P].GetName(); }
        bool  SetPeriphName(int P, const char *Name) { Periph[P].SetName(Name); return true; }
        
        int   GetPeriphId(int PosPeriph) { return Periph[PosPeriph].GetId(); }
        
        void  SetPeriphPeerId(int P, int PeerId) { Periph[P].SetPeerId(PeerId); }
        int   GetPeriphPeerId(int P) { return Periph[P].GetPeerId(); }

        int   GetPeriphPos(int P) { return Periph[P].GetPos(); }

        float GetPeriphValue(int P, int i=0) { return Periph[P].GetValue(i); }
        void  SetPeriphValue(int P, float Value, int i=0) { Periph[P].SetValue(Value, i); }
        
        float GetPeriphOldValue(int P, int i=0) { return Periph[P].GetOldValue(i); }
        void  SetPeriphOldValue(int P, float Value, int i=0) { Periph[P].SetOldValue(Value, i); }
        
        void  SetPeriphChanged(int P, bool Changed) { Periph[P].SetChanged(Changed); }
        bool  GetPeriphChanged(int P) { return Periph[P].GetChanged(); }
        bool  PeriphChanged(int P) { return Periph[P].GetChanged(); }
        
        int   GetPeriphType(int P) { return Periph[P].GetType(); }
        
        float GetPeriphVin(int P) { return Periph[P].GetVin(); }
        void  SetPeriphVin(int P, float Vin) { Periph[P].SetVin(Vin); }
        
        float GetPeriphVperAmp(int P){ return Periph[P].GetVperAmp(); }
        void  SetPeriphVperAmp(int P, float VperAmp) { return Periph[P].SetVperAmp(VperAmp); }
        
        int   GetPeriphIOPort(int P, int i=0) { return Periph[P].GetIOPort(i); }
        void  SetPeriphIOPort(int P, int _IOPort, int _Value) { Periph[P].SetIOPort(_IOPort, _Value); }

        int   GetPeriphI2CPort(int P, int i) { return Periph[P].GetI2CPort(i); }
        void  SetPeriphI2CPort(int P, int i, int _I2C) { Periph[P].SetI2CPort(i, _I2C); }
        
        float GetPeriphNullwert(int P) { return Periph[P].GetNullwert(); }
        void  SetPeriphNullwert(int P, float Nullwert) { Periph[P].SetNullwert(Nullwert); }
        
        void  SetdBm(int dBm) { _dBm = dBm ; }
        int   GetdBm()        { return _dBm; }

        float GetPeriphSavedValue(int P, int Index, int i) { return Periph[P].GetSavedValue(Index, i); }
        void  AddPeriphSavedValue(int P, float V0, float V1, float V2, float V3) { Periph[P].AddSavedValue(V0, V1, V2, V3); }

        PeriphClass *GetPeriphPtr(int P) { return &Periph[P]; }
          
        bool isEmpty() { return (_Type == 0); }
        bool isPeriphEmpty(int SNr) { return Periph[SNr].isEmpty(); }
        bool isPeriphSensor(int SNr) { return Periph[SNr].IsSensor(); }
        bool isPeriphSwitch(int SNr) { return Periph[SNr].IsSwitch(); }
};

PeerClass *FindPeerByMAC(const uint8_t *BroadcastAddress);
PeerClass *FindPeerById(int Id);
PeerClass *FindPeerByName(char *Name);
PeerClass *PeerOf(PeriphClass *P);

//PeerClass *FindFirstPeer(int Type, int Status=2);
PeerClass *FindNextPeer(PeerClass *P, int Type, bool circular, int Status=2);
PeerClass *FindPrevPeer(PeerClass *P, int Type, bool circular, int Status=2);
PeriphClass *FindPeriphById(int Id);
//PeriphClass *FindFirstPeriph(PeerClass *P, int Type, int Status=2);
//PeriphClass *FindLastPeriph (PeerClass *P, int Type, int Status=2);
PeriphClass *FindPrevPeriph(PeerClass *P, PeriphClass *Periph, int Type, bool circular, int Status=2);
PeriphClass *FindNextPeriph(PeerClass *P, PeriphClass *Periph, int Type, bool circular, int Status=2);

extern MyLinkedList<PeerClass*>   PeerList;
extern MyLinkedList<PeriphClass*> PeriphList;

char *TypeInText(int Type);
extern PeerClass *ActivePeer;
extern PeriphClass *ActivePeriph;

extern char ExportImportBuffer[300];

#endif