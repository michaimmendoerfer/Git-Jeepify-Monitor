#ifndef COMPBUTTON_H
#define COMPBUTTON_H

#include <Arduino.h>
#include "lvgl.h"
#include "Ui/ui.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include "LinkedList.h"
#include "Jeepify.h"
#include "main.h"
#include "CompButton.h"
class CompThing {
    static int  _ClassId;

    protected:
        int _ClassType;
        
        int _x;
        int _y;
	    int _Width;
	    int _Height;
        int _Id;
        int _Pos;
        int _Size;

        lv_obj_t *_Button;
        lv_obj_t *_LblPeer;
        lv_obj_t *_LblPeriph;
        lv_obj_t *_LblPeriphId;
        lv_obj_t *_LblPos;
        lv_obj_t *_LblValue;

        PeriphClass *_Periph;

        bool _PeerVisible;
        bool _PeriphVisible;
        bool _ValueVisible;
        bool _SystemVisible;
        bool _dBmVisible;
        bool _PeriphValueCombo;

        int _X_Peer;
        int _Y_Peer;
        int _X_Periph;
        int _Y_Periph;
        int _X_Value;
        int _Y_Value;

        lv_event_cb_t _event_cb;

    public:
        CompThing();
        virtual ~CompThing();
        virtual void Update();
        virtual void Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb);
        
        PeriphClass *GetPeriph() { return _Periph; }
        lv_obj_t*    GetButton() { return _Button; }
        bool GetButtonState()   	{ if (lv_obj_has_state(_Button, LV_STATE_CHECKED)) return true; else return false; }
        void SetButtonState(bool State) { if (State) lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_CHECKED_RELEASED); 
                                                else lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_RELEASED); }
        
        void SetPeer(char *Buf) 	    { lv_label_set_text(_LblPeer, Buf); }
        void SetPeerPos(int x, int y) 	{ _X_Peer = x; _Y_Peer = y; lv_obj_set_pos(_LblPeer, x, y); }
	    void ShowPeer() 		        { _PeerVisible = true;  lv_obj_clear_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN); }			
        void HidePeer() 		        { _PeerVisible = false; lv_obj_add_flag  (_LblPeer, LV_OBJ_FLAG_HIDDEN); }	

        void SetPeriph(char *Buf) 	    { lv_label_set_text(_LblPeriph, Buf); }
        void SetPeriphPos(int x, int y) { _X_Periph = x; _Y_Periph = y; lv_obj_set_pos(_LblPeriph, x, y); };
	    void ShowPeriph() 		        { _PeriphVisible = true;  lv_obj_clear_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN); };			
        void HidePeriph() 		        { _PeriphVisible = false; lv_obj_add_flag  (_LblPeriph, LV_OBJ_FLAG_HIDDEN); };	

        void SetValue(char *Buf) 	    { lv_label_set_text(_LblValue, Buf); }
        void SetValuePos(int x, int y) 	{ _X_Value = x; _Y_Value = y; lv_obj_set_pos(_LblValue, x, y);}
        void ShowValue() 		        { _ValueVisible = true;  lv_obj_clear_flag(_LblValue, LV_OBJ_FLAG_HIDDEN); };			
        void HideValue() 		        { _ValueVisible = false; lv_obj_add_flag  (_LblValue, LV_OBJ_FLAG_HIDDEN); };	

        void Hide() 			{ lv_obj_add_flag  (_LblPeer,   LV_OBJ_FLAG_HIDDEN);
			                  lv_obj_add_flag  (_LblPeriph, LV_OBJ_FLAG_HIDDEN); 
			                  lv_obj_add_flag  (_LblValue,  LV_OBJ_FLAG_HIDDEN); 
			                  lv_obj_add_flag  (_Button,    LV_OBJ_FLAG_HIDDEN); }
                      
        void Show() 			{ lv_obj_clear_flag(_Button, LV_OBJ_FLAG_HIDDEN); }

        int  GetPos() 			{ return _Pos; }
        int  GetClassType() 		{ return _ClassType; }

	void SetStyle(lv_obj_t *obj);
};

class CompButton : public CompThing {

    private:
	bool _SpinnerVisible;

        lv_obj_t *_Spinner;

    public:
        CompButton();
        ~CompButton();
        void Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb);
        void Update();
        void ShowSpinner() 		{ _SpinnerVisible = true;  lv_obj_clear_flag(_Spinner, LV_OBJ_FLAG_HIDDEN); }
        void HideSpinner()		{ _SpinnerVisible = false; lv_obj_add_flag  (_Spinner, LV_OBJ_FLAG_HIDDEN); }
};

class CompSensor : public CompThing {

    private:
        lv_obj_t *_Arc;

    public:
        CompSensor();
        ~CompSensor();

        void Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb);
        void Update();
        void Hide() 			{ lv_obj_add_flag  (_Button, LV_OBJ_FLAG_HIDDEN); }
        void Show() 			{ lv_obj_clear_flag(_Button, LV_OBJ_FLAG_HIDDEN); }

        lv_obj_t *GetArc() 		{ return _Arc; }
};

class CompMeter : public CompThing {
	private:
		lv_meter_indicator_t * _Indic;
		lv_meter_indicator_t * _IndicNeedle;
		lv_meter_scale_t *     _Scale;

	public:
		CompMeter();
		~CompMeter();

		void Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb);
        void Update();	
};

#endif
