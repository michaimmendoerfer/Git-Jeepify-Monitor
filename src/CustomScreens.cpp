#include "Ui/ui.h"
#include "CustomScreens.h"
#include <Arduino.h>
#include "lv_meter.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include "LinkedList.h"
#include "Jeepify.h"
#include "main.h"
#include "CompButton.h"

lv_timer_t *SingleTimer;
lv_obj_t *ui_ScrSingle;

void ui_ScrSingle_screen_init(void)
{
    ui_ScrSingle = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrSingle, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_ScrSingle, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScrSingle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_ScrSingle, &ui_img_jeepifybackground_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_opa(ui_ScrSingle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_ScrSingle, ui_event_ui_ScrSingle, LV_EVENT_ALL, NULL);

}
void Ui_Single_Next(lv_event_t * e)
{	
	if (ActivePeriphShown->IsType(SENS_TYPE_SENS))   
	{
		ActivePeriphSensor = FindNextPeriph(NULL, ActivePeriphShown, SENS_TYPE_SENS, CIRCULAR);
		ActivePeriphShown  = ActivePeriphSensor;
	}
	
	if (ActivePeriphShown->IsType(SENS_TYPE_SW_ALL)) 
	{
		ActivePeriphSwitch = FindNextPeriph(NULL, ActivePeriphShown, SENS_TYPE_SW_ALL, CIRCULAR);
		ActivePeriphShown  = ActivePeriphSwitch;
	}

	if (ActivePeriphShown)
	{
		Ui_Single_Leave(e);
		Ui_Single_Prepare(e);
	}
}
void Ui_Single_Prev(lv_event_t * e)
{
	if (ActivePeriphShown->IsType(SENS_TYPE_SENS))   
	{
		ActivePeriphSensor = FindPrevPeriph(NULL, ActivePeriphShown, SENS_TYPE_SENS, CIRCULAR);
		ActivePeriphShown  = ActivePeriphSensor;
	}
	
	if (ActivePeriphShown->IsType(SENS_TYPE_SW_ALL)) 
	{
		ActivePeriphSwitch = FindPrevPeriph(NULL, ActivePeriphShown, SENS_TYPE_SW_ALL, CIRCULAR);
		ActivePeriphShown  = ActivePeriphSwitch;
	}
	
	if (ActivePeriphShown)
	{
		Ui_Single_Leave(e);
		Ui_Single_Prepare(e);
	}
}
void Ui_Single_Prepare(lv_event_t * e)
{
	int Pos = 0;
	if (ActivePeriphShown)
	{
		if (CompThingArray[Pos]) 
			{
				delete CompThingArray[Pos];
				CompThingArray[Pos] = NULL;
			}

		if      (ActivePeriphShown->IsSensor()) CompThingArray[Pos] = new CompMeter;
		else if (ActivePeriphShown->IsSwitch()) CompThingArray[Pos] = new CompButton;
		
		CompThingArray[Pos]->Setup(ui_ScrSingle, 0, 0, 0, 360, ActivePeriphShown, Ui_Single_Clicked);
		CompThingArray[Pos]->Update();
		
		static uint32_t user_data = 10;
		
		if (SingleTimer) 
		{
			lv_timer_resume(SingleTimer);
		}
		else 
		{
			SingleTimer = lv_timer_create(SingleUpdateTimer, 500,  &user_data);
		}
	}
}
void SingleUpdateTimer(lv_timer_t * timer)
{
	if (CompThingArray[0]) CompThingArray[0]->Update();
}
void Ui_Single_Clicked(lv_event_t * e)
{
	lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
	PeriphClass *Periph;

    if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_Single_Next(e);
    }
    else if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_Single_Prev(e);
	}
	else if (event_code == LV_EVENT_CLICKED) {
		if (ActivePeriphShown->IsSwitch())
		{
			Periph = FindPeriphById(atoi(lv_label_get_text(lv_obj_get_child(target, 3))));

			Periph->SetChanged(true);
			
			if (lv_obj_get_state(target) == LV_IMGBTN_STATE_DISABLED) 		  Periph->SetValue(0.0);
			if (lv_obj_get_state(target) == LV_IMGBTN_STATE_CHECKED_RELEASED) Periph->SetValue(1.0);
			
			ToggleSwitch(Periph);
		}
		else 
		{
			if (ActivePeriphShown->IsSensor()) Ui_Single_Next(e);
		}
	}
	else if (event_code == LV_EVENT_LONG_PRESSED) {
    }
}
void Ui_Single_Leave(lv_event_t * e)
{
	lv_timer_del(SingleTimer);
	SingleTimer = NULL;

	int Pos = 0;

	if (CompThingArray[Pos]) 
	{
		delete CompThingArray[Pos];
		CompThingArray[Pos] = NULL;
	}
}
void ui_event_ui_ScrSingle(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_Single_Next(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_Single_Prev(e);
    }
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        Ui_Single_Prepare(e);
    }
    if(event_code == LV_EVENT_SCREEN_UNLOAD_START) {
        Ui_Single_Leave(e);
    }
}

