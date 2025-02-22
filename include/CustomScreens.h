#ifndef CUSTOMSCREENS_H
#define CUSTOMSCREENS_H

#include "PeerClass.h"
#include "Ui/ui.h"
#include "CompButton.h"

// SCREEN: ui_ScrSingle
void ui_ScrSingle_screen_init(void);
void ui_event_ui_ScrSingle(lv_event_t * e);
void Ui_Single_Next(lv_event_t * e);
void Ui_Single_Prev(lv_event_t * e);
void Ui_Single_Prepare(lv_event_t * e);
void Ui_Single_Leave(lv_event_t * e);
void SingleUpdateTimer(lv_timer_t * timer);
void Ui_Single_Clicked(lv_event_t * e);
void ui_event_ui_ScrSingle(lv_event_t * e);

extern lv_obj_t *ui_ScrSingle;

extern PeriphClass *ActivePeriphSensor;
extern PeriphClass *ActivePeriphSwitch;
extern PeriphClass *ActivePeriphShown;
extern CompThing   *CompThingArray[4];

#endif