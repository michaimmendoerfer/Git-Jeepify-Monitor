#include <Arduino.h>
#include "Ui/ui.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include "LinkedList.h"
#include "Jeepify.h"
#include "main.h"
#include "CompButton.h"

//#define DEBUG 1

CompThing::CompThing()
{
    _PeerVisible      = true;
    _PeriphVisible    = true;
    _ValueVisible     = false;
    _SystemVisible    = false;
    _PeriphValueCombo = true;
    _dBmVisible       = false;
}
CompThing::~CompThing()
{
	if (_Button)  { lv_obj_del(_Button); _Button = NULL; }
    	
}
void CompThing::Update()
{ 
	
}
void CompThing::Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb)
{

}
void CompThing::SetStyle(lv_obj_t *obj)
{
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_BtnTxt);
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_BtnTxt);
    lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_BG_COLOR, _ui_theme_color_BtnBg);
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_BG_OPA, _ui_theme_alpha_BtnBg);
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_BORDER_COLOR, _ui_theme_color_BtnBorder);
    ui_object_set_themeable_style_property(obj, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_BORDER_OPA, _ui_theme_alpha_BtnBorder);
    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_align(obj, LV_ALIGN_CENTER);	
}

CompButton::CompButton()
{
    _ClassType = 1;
	_SpinnerVisible = false;
}
CompButton::~CompButton()
{
    lv_obj_remove_event_cb(_Button, _event_cb);
    
	lv_obj_invalidate(_LblPeer);
    lv_obj_invalidate(_LblPeriph);
    lv_obj_invalidate(_LblValue);
	
    if (_Spinner) { lv_obj_del(_Spinner); _Spinner = NULL; }
}
void CompButton::Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb)
{
    _Periph = Periph;
    _event_cb = event_cb;
    _Pos = Pos;
    _Size = size;
   
    _x = x;
    _y = y;			
	
    if (size == 1)
    {
        _Width  =  70;
        _Height = 120;
        _PeerVisible = false;
        _PeriphVisible = true;
        _ValueVisible = true;
        _PeriphValueCombo = true;
    }
    else
    {
        _Width  = 120;
        _Height = 205;
        _PeerVisible = true;
        _PeriphVisible = true;
        _ValueVisible = true;
        _PeriphValueCombo = false;
    }
	
    _Spinner = lv_spinner_create(comp_parent, 1000, 90);
    
    lv_obj_set_align(_Spinner, LV_ALIGN_CENTER);
    lv_obj_set_width(_Spinner,  _Width+30);
    lv_obj_set_height(_Spinner, _Width+30);
	
    lv_obj_set_pos(_Spinner, _x, _y);
    lv_obj_add_flag(_Spinner, LV_OBJ_FLAG_HIDDEN);     
    lv_obj_clear_flag(_Spinner, LV_OBJ_FLAG_CLICKABLE);     
    lv_obj_set_style_arc_color(_Spinner, lv_color_hex(0x83061F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(_Spinner, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(_Spinner, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(_Spinner, 5, LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_set_style_arc_color(_Spinner, lv_color_hex(0x31020B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(_Spinner, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(_Spinner, 5, LV_PART_INDICATOR | LV_STATE_CHECKED);
	
    _Button = lv_imgbtn_create(comp_parent);
    if (size == 1)
    {
        lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1640860301, NULL);
        lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_743505413, NULL);
        lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_CHECKED_PRESSED, NULL, &ui_img_743505413, NULL);
        lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_888658411, NULL);
    }
    else
    {
	    lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1134846501, NULL);
		lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_PRESSED, NULL, &ui_img_1528892059, NULL);
		lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_CHECKED_PRESSED, NULL, &ui_img_1528892059, NULL);
		lv_imgbtn_set_src(_Button, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, &ui_img_715952573, NULL);
    }

    //set Switch-state
    if (Periph->GetValue() == 0) lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_RELEASED);
    else                         lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_CHECKED_RELEASED);
    
    lv_obj_set_align(_Button, LV_ALIGN_CENTER);
    lv_obj_set_height(_Button, _Height);
    lv_obj_set_width (_Button, _Width);   /// 1
    lv_obj_set_pos(_Button, _x, _y);
        
    lv_obj_add_flag(_Button, LV_OBJ_FLAG_CHECKABLE);     
    lv_obj_clear_flag(_Button, LV_OBJ_FLAG_SCROLLABLE);        
    lv_obj_add_flag(_Button, LV_OBJ_FLAG_OVERFLOW_VISIBLE);  

	_LblPeer = lv_label_create(_Button);
	SetStyle(_LblPeer);
    lv_obj_add_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	lv_obj_set_width (_LblPeer, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeer, LV_SIZE_CONTENT);    /// 1
	
	if (size == 1)
	{
        	SetPeerPos(-50, -5);
	    	lv_obj_set_style_text_font(_LblPeer, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
	else
	{
        	SetPeerPos(0, lv_pct(58));
	    	lv_obj_set_style_text_font(_LblPeer, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
    
	_LblPeriph = lv_label_create(_Button);
	SetStyle(_LblPeriph);
    lv_obj_add_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(_LblPeriph, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeriph, LV_SIZE_CONTENT);    /// 1
    
    if (size == 1)
	{
		SetPeriphPos(0, -55);
        lv_obj_set_style_text_font(_LblPeriph, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
	else
	{
		SetPeriphPos(0, lv_pct(-70));
        lv_obj_set_style_text_font(_LblPeriph, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
    
    _LblValue = lv_label_create(_Button);
    SetStyle(_LblPeriph);
    lv_obj_add_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(_LblValue, LV_SIZE_CONTENT);
    lv_obj_set_height(_LblValue, LV_SIZE_CONTENT);  
	
	if (size == 1)
	{
		SetValuePos(50, 70);
    	lv_obj_set_style_text_font(_LblValue, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
	else 
	{
		SetValuePos(70, lv_pct(-40));
		lv_obj_set_style_text_font(_LblValue, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	}
    
	lv_label_set_text(_LblValue, "--.- A");

    _LblPeriphId = lv_label_create(_Button);
    if (!_SystemVisible) lv_obj_add_flag(_LblPeriphId, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(_LblPeriphId, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeriphId, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_pos(_LblPeriphId, 50, 0);
    lv_obj_set_align(_LblPeriphId, LV_ALIGN_CENTER);
    lv_label_set_text_fmt(_LblPeriphId, "%d", _Periph->GetId());
    lv_obj_set_style_text_color(_LblPeriphId, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    _LblPos = lv_label_create(_Button);
    if (!_SystemVisible) lv_obj_add_flag(_LblPos, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(_LblPos, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPos, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblPos, LV_ALIGN_CENTER);
    lv_obj_set_pos(_LblPos, 50, -15);
    lv_label_set_text_fmt(_LblPos, "%d", _Pos);
    lv_obj_set_style_text_color(_LblPos, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_add_event_cb(_Button, _event_cb, LV_EVENT_ALL, NULL);  	
}
void CompButton::Update()
{
    lv_obj_set_pos(_Spinner, _x, _y);
    lv_obj_set_pos(_Button,  _x, _y);

    //set Switch-state
    if (_Periph->GetValue(0) == 0) 
        lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_RELEASED);
    else 			  
        lv_imgbtn_set_state(_Button, LV_IMGBTN_STATE_CHECKED_RELEASED);

    // show Spinner if Value is in changed state
    if (_Periph->GetChanged()) ShowSpinner();
    else HideSpinner();
    //(Periph->GetChanged() ? ShowSpinner() : HideSpinner();
	
	// - show Peer if named and visible
	if ((PeerOf(_Periph)->GetName() == NULL) or (!_PeerVisible))
	{
	    lv_obj_add_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
	    lv_label_set_text_fmt(_LblPeer, "%.6s", PeerOf(_Periph)->GetName());
     	lv_obj_clear_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}

	char Buf_Periph[20] = {};
	char Buf_Value[10] = {};

	// fill Buf_Value if switch is on
	sprintf(Buf_Periph, "%.6s", _Periph->GetName());
	if ((_ValueVisible) and (_Periph->IsCombo()) and (_Periph->GetValue(0) == 1))
	{
		if (_PeriphValueCombo) 
        {
            sprintf(Buf_Value, " %0.2fA", _Periph->GetValue(3)); // amp-wert
        }
        else    
        {
            sprintf(Buf_Value, " %0.2fA", _Periph->GetValue(3)); // amp-wert
            lv_obj_clear_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
        }
	}

	// add Value to periph if desired, hide value then
	if (_PeriphValueCombo)
	{	
		strcat(Buf_Periph, Buf_Value);
		lv_obj_add_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
	}

	// Show Periph (and evtl value) if visible
    if ((_Periph->GetName() == NULL) or (!_PeriphVisible)) 
    {
        lv_obj_add_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    }
    else 
    {
        if (_Size == 1)
        {
            if (_Periph->GetValue(0) == 1) SetPeriphPos(0,  lv_pct(33));
            else                           SetPeriphPos(0,  lv_pct(-33));
        }

        lv_label_set_text(_LblPeriph, Buf_Periph);
        lv_obj_clear_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    
    }

    // show value only if visible, not in periph, switch is on and brother is existent
    if ((_ValueVisible) and (_Periph->IsCombo()) and (!_PeriphValueCombo) and (_Periph->GetValue(0) == 1))
    {
        lv_label_set_text(_LblValue, Buf_Value);
        lv_obj_clear_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
    }  
}

CompSensor::CompSensor()
{
    _ClassType = 2;
}
CompSensor::~CompSensor()
{
    lv_obj_remove_event_cb(_Button, _event_cb);
}
void CompSensor::Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb)
{
    _Periph = Periph;
    _event_cb = event_cb;
    _Pos = Pos;
   
    _x = x;
    _y = y;		

    _Width  = 100;
    _Height = 100;
    _PeerVisible = true;
    _PeriphVisible = true;
    _ValueVisible = true;
    _PeriphValueCombo = false;
    _SystemVisible = true;
    
    //Serial.printf("Pos: %d, x:%d, y:%d\n\r", Pos, x, y);
    
    _Button = lv_btn_create(comp_parent);
    lv_obj_set_width(_Button, _Width);
    lv_obj_set_height(_Button, _Height);
    lv_obj_set_pos(_Button, _x, _y);
    lv_obj_set_align(_Button, LV_ALIGN_CENTER);
    lv_obj_add_flag(_Button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(_Button, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(_Button, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(_Button, lv_color_hex(0x165420), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(_Button, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(_Button, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(_Button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(_Button, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(_Button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(_Button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(_Button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(_Button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    
    _LblPeer = lv_label_create(_Button);
    lv_obj_set_width(_LblPeer, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeer, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblPeer, LV_ALIGN_CENTER);
    SetPeerPos(0, 30);
    lv_obj_set_style_text_font(_LblPeer, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

	
    _LblPeriph = lv_label_create(_Button);
    lv_obj_set_width(_LblPeriph, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeriph, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblPeriph, LV_ALIGN_CENTER);
    SetPeriphPos(0, 5);
    lv_obj_set_style_text_font(_LblPeriph, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    _LblValue = lv_label_create(_Button);
    lv_obj_set_width(_LblValue, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblValue, LV_ALIGN_CENTER);
    SetValuePos(0, -20);

    _LblPeriphId = lv_label_create(_Button);
    lv_obj_set_width(_LblPeriphId, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeriphId, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_pos(_LblPeriphId, 80, 0);
    lv_obj_set_align(_LblPeriphId, LV_ALIGN_CENTER);
    lv_label_set_text_fmt(_LblPeriphId, "%d", _Periph->GetId());
    lv_obj_set_style_text_color(_LblPeriphId, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    if (!_SystemVisible) lv_obj_add_flag(_LblPeriphId, LV_OBJ_FLAG_HIDDEN);

    _LblPos = lv_label_create(_Button);
    lv_obj_set_width(_LblPos, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPos, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_pos(_LblPos, 80, 18);
    lv_obj_set_align(_LblPos, LV_ALIGN_CENTER);
    lv_label_set_text_fmt(_LblPos, "%d", _Pos);
    lv_obj_set_style_text_color(_LblPos, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(_LblPos, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (!_SystemVisible) lv_obj_add_flag(_LblPos, LV_OBJ_FLAG_HIDDEN);

    _Arc = lv_arc_create(_Button);
    lv_obj_set_width(_Arc, _Width);
    lv_obj_set_height(_Arc, _Height);
    lv_obj_set_align(_Arc, LV_ALIGN_CENTER);
    lv_obj_clear_flag(_Arc, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_arc_set_value(_Arc, 0);
    lv_obj_set_style_arc_color(_Arc, lv_color_hex(0x4A4A4A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(_Arc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(_Arc, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(_Arc, lv_color_hex(0x398F30), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(_Arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(_Arc, 6, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(_Arc, lv_color_hex(0xDBDBDB), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(_Arc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(_Arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(_Arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(_Arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(_Arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(_Button, _event_cb, LV_EVENT_ALL, NULL);  
}

void CompSensor::Update()
{
    lv_obj_set_pos(_Button, _x, _y);
    
	if ((PeerOf(_Periph)->GetName() == NULL) or (!_PeerVisible))
	{
	    lv_obj_add_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
	    lv_label_set_text_fmt(_LblPeer, "%.6s", PeerOf(_Periph)->GetName());
     	lv_obj_clear_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}

    if ((_Periph->GetName() == NULL) or (!_PeriphVisible)) 
    {
	lv_obj_add_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    }
    else 
    {
	lv_label_set_text_fmt(_LblPeriph, "%.6s", _Periph->GetName());
	lv_obj_clear_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    
    }

    if (_ValueVisible) 
    {
        char buf[10];
        int nk = 0;
        lv_color_t bg;
        float value;

        if (_Periph->GetType() == SENS_TYPE_AMP)
        {
            value = _Periph->GetValue(3);
        
            if      (value<10)  nk = 2;
            else if (value<100) nk = 1;
            else                nk = 0;

            if (value == -99) strcpy(buf, "--"); 
            else dtostrf(value, 0, nk, buf);
            strcat(buf, " A");
            
            if      (value < 20) bg = lv_color_hex(0x135A25);
            else if (value < 25) bg = lv_color_hex(0x7C7E26);
            else 		     bg = lv_color_hex(0x88182C);
            lv_obj_set_style_bg_color(_Button, bg, LV_PART_MAIN | LV_STATE_DEFAULT);
            
            lv_arc_set_range(_Arc, 0, 400);
            lv_arc_set_value(_Arc, value*10);
        }
        else if (_Periph->GetType() == SENS_TYPE_VOLT)
        {
            float value = _Periph->GetValue(2);
        
            if      (value<10)  nk = 2;
            else if (value<100) nk = 1;
            else                nk = 0;

            if (value == -99) strcpy(buf, "--"); 
            else dtostrf(value, 0, nk, buf);
            strcat(buf, " V");
                        
            if 	(value < 13)   	bg = lv_color_hex(0x135A25);
            else if (value < 14.4) 	bg = lv_color_hex(0x7C7E26);
            else 	 		bg = lv_color_hex(0x88182C);
            lv_obj_set_style_bg_color(_Button, bg, LV_PART_MAIN | LV_STATE_DEFAULT);
            
            lv_arc_set_range(_Arc, 90, 150);
            lv_arc_set_value(_Arc, value*10);
        }
            
        lv_label_set_text(_LblValue, buf);
        lv_obj_clear_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
    }  
}

CompMeter::CompMeter() 
{
    _PeerVisible = true;
    _PeriphVisible = true;
    _ValueVisible = true;
    _SystemVisible = false;
    _PeriphValueCombo = false;
    _dBmVisible = true;
}
CompMeter::~CompMeter() 
{
	lv_obj_remove_event_cb(_Button, _event_cb);
}
static void Meter_cb(lv_event_t * e) {

	lv_obj_draw_part_dsc_t	*dsc  = (lv_obj_draw_part_dsc_t *)lv_event_get_param(e);
	double					value;

	if( dsc->text != NULL ) {		// Filter major ticks...
		value = dsc->value / 10;
		snprintf(dsc->text, sizeof(dsc->text), "%5.1f", value);
	}

}
void CompMeter::Setup(lv_obj_t * comp_parent, int x, int y, int Pos, int size, PeriphClass *Periph, lv_event_cb_t event_cb) 
{
	_Periph = Periph;
    _event_cb = event_cb;
    _Pos = Pos;
    _x = x;
    _y = y;		
    _Width  = size;
    _Height = size;
    _Size = size;	

	_Button = lv_meter_create(comp_parent);
	lv_obj_center(_Button);
	lv_obj_set_style_bg_color(_Button, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(_Button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_size(_Button, _Width, _Height);
	
	_Scale = lv_meter_add_scale(_Button);
	lv_obj_move_background(_Button);
	lv_obj_set_style_text_color(_Button, lv_color_hex(0xdbdbdb), LV_PART_TICKS);
	
	_IndicNeedle = lv_meter_add_needle_line(_Button, _Scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);
	
	if (_Periph->GetType() == SENS_TYPE_AMP)
	{
		lv_meter_set_scale_ticks(_Button, _Scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    	lv_meter_set_scale_major_ticks(_Button, _Scale, 5, 4, 15, lv_color_black(), 15);
    	lv_meter_set_scale_range(_Button, _Scale, 0, 400, 240, 150);
	
		//Add a green arc to the start
		_Indic = lv_meter_add_scale_lines(_Button, _Scale, lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_GREEN), false, 0);
    	lv_meter_set_indicator_start_value(_Button, _Indic, 0);
    	lv_meter_set_indicator_end_value(_Button, _Indic, 250);

		_Indic = lv_meter_add_arc(_Button, _Scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    	lv_meter_set_indicator_start_value(_Button, _Indic, 300);
    	lv_meter_set_indicator_end_value(_Button, _Indic, 400);

		//Make the tick lines red at the end of the scale
		_Indic = lv_meter_add_scale_lines(_Button, _Scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
		lv_meter_set_indicator_start_value(_Button, _Indic, 300);
		lv_meter_set_indicator_end_value(_Button, _Indic, 400);

		lv_obj_add_event_cb(_Button, Meter_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
	}
	else if (_Periph->GetType() == SENS_TYPE_VOLT)
	{	
		lv_meter_set_scale_ticks(_Button, _Scale, 31, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    	lv_meter_set_scale_major_ticks(_Button, _Scale, 5, 4, 15, lv_color_black(), 15);
    	lv_meter_set_scale_range(_Button, _Scale, 90, 150, 240, 150);
	
		_Indic = lv_meter_add_scale_lines(_Button, _Scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    	lv_meter_set_indicator_start_value(_Button, _Indic, 90);
    	lv_meter_set_indicator_end_value(_Button, _Indic, 112);
		
		//Add a green arc to the start
		_Indic = lv_meter_add_scale_lines(_Button, _Scale, lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_GREEN), false, 0);
    	lv_meter_set_indicator_start_value(_Button, _Indic, 112);
    	lv_meter_set_indicator_end_value(_Button, _Indic, 144);

		_Indic = lv_meter_add_arc(_Button, _Scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    	lv_meter_set_indicator_start_value(_Button, _Indic, 144);
    	lv_meter_set_indicator_end_value(_Button, _Indic, 150);

		//Make the tick lines red at the end of the scale
		_Indic = lv_meter_add_scale_lines(_Button, _Scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
		lv_meter_set_indicator_start_value(_Button, _Indic, 144);
		lv_meter_set_indicator_end_value(_Button, _Indic, 150);

		//Add draw callback to override default values
		lv_obj_add_event_cb(_Button, Meter_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    }

    _LblPeer = lv_label_create(_Button);
    lv_obj_set_width(_LblPeer, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeer, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblPeer, LV_ALIGN_CENTER);
    SetPeerPos(0, lv_pct(35));
    lv_obj_set_style_text_font(_LblPeer, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_object_set_themeable_style_property(_LblPeer, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_BtnTxt);
    ui_object_set_themeable_style_property(_LblPeer, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_BtnTxt);
    

    _LblPeriph = lv_label_create(_Button);
    lv_obj_set_width(_LblPeriph, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblPeriph, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblPeriph, LV_ALIGN_CENTER);
    SetPeriphPos(0, lv_pct(-20));
    lv_obj_set_style_text_font(_LblPeriph, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_object_set_themeable_style_property(_LblPeriph, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_BtnTxt);
    ui_object_set_themeable_style_property(_LblPeriph, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_BtnTxt);
    

    _LblValue = lv_label_create(_Button);
    lv_obj_set_width(_LblValue, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(_LblValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(_LblValue, LV_ALIGN_CENTER);
    SetValuePos(0, lv_pct(15));
    lv_obj_set_style_text_font(_LblValue, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_object_set_themeable_style_property(_LblValue, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR, _ui_theme_color_BtnTxt);
    ui_object_set_themeable_style_property(_LblValue, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA, _ui_theme_alpha_BtnTxt);
}
void CompMeter::Update() 
{    
    lv_obj_set_pos(_Button,    _x,        _y);
    lv_obj_set_pos(_LblPeer,   _X_Peer,   _Y_Peer);
    lv_obj_set_pos(_LblPeriph, _X_Periph, _Y_Periph);
    lv_obj_set_pos(_LblValue,  _X_Value,  _Y_Value);
    //nur so
    if ((PeerOf(_Periph)->GetName() == NULL) or (!_PeerVisible))
	{
	    lv_obj_add_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}
	else
	{
	    	lv_label_set_text_fmt(_LblPeer, "%.6s (%d dBm)", PeerOf(_Periph)->GetName(), PeerOf(_Periph)->GetdBm());
     		lv_obj_clear_flag(_LblPeer, LV_OBJ_FLAG_HIDDEN);
	}
    
    if ((_Periph->GetName() == NULL) or (!_PeriphVisible)) 
    {
		lv_obj_add_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    }
    else 
    {
		lv_label_set_text_fmt(_LblPeriph, "%.6s", _Periph->GetName());
		lv_obj_clear_flag(_LblPeriph, LV_OBJ_FLAG_HIDDEN);
    
    }

    char buf[10];
	int nk = 0;
	float value;
    if (_Periph->GetType() == SENS_TYPE_AMP)  value = _Periph->GetValue(3);
	if (_Periph->GetType() == SENS_TYPE_VOLT) value = _Periph->GetValue(2);
	    
    if (abs(value) < SCHWELLE) value = 0;

    if      (value<10)  nk = 2;
    else if (value<100) nk = 1;
    else                nk = 0;

    if (value == -99) strcpy(buf, "--"); 
    else dtostrf(value, 0, nk, buf);

    if (_Periph->GetType() == SENS_TYPE_AMP)  strcat(buf, " A");
    if (_Periph->GetType() == SENS_TYPE_VOLT) strcat(buf, " V");
		
	lv_meter_set_indicator_value(_Button, _IndicNeedle, value*10);

	if (_ValueVisible) 
	{
		lv_label_set_text(_LblValue, buf);
		lv_obj_clear_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
	}
	else 
	{
		lv_obj_add_flag(_LblValue, LV_OBJ_FLAG_HIDDEN);
	}
}
