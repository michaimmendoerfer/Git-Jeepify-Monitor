// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SQ-Jeepify-Monitor240

#ifndef _SQ_JEEPIFY_MONITOR240_UI_H
#define _SQ_JEEPIFY_MONITOR240_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"
#include "ui_theme_manager.h"
#include "ui_themes.h"


// SCREEN: ui_ScrMenu
void ui_ScrMenu_screen_init(void);
void ui_event_ScrMenu(lv_event_t * e);
extern lv_obj_t * ui_ScrMenu;
extern lv_obj_t * ui_ImgRubicon;
extern lv_obj_t * ui_LblMenuVersion;
extern lv_obj_t * ui_Container4;
void ui_event_BtnMenu1(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu1;
void ui_event_BtnMenu2(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu2;
void ui_event_BtnMenu3(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu3;
void ui_event_BtnMenu4(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu4;
// CUSTOM VARIABLES

// SCREEN: ui_ScrSettings
void ui_ScrSettings_screen_init(void);
void ui_event_ScrSettings(lv_event_t * e);
extern lv_obj_t * ui_ScrSettings;
extern lv_obj_t * ui_Container1;
void ui_event_BtnSet1(lv_event_t * e);
extern lv_obj_t * ui_BtnSet1;
extern lv_obj_t * ui_BtnSet1Lbl1;
void ui_event_BtnSet2(lv_event_t * e);
extern lv_obj_t * ui_BtnSet2;
extern lv_obj_t * ui_BtnSet2Lbl;
void ui_event_BtnSet3(lv_event_t * e);
extern lv_obj_t * ui_BtnSet3;
extern lv_obj_t * ui_BtnSet3Lbl3;
void ui_event_BtnSet4(lv_event_t * e);
extern lv_obj_t * ui_BtnSet4;
extern lv_obj_t * ui_BtnSet3Lbl1;
void ui_event_BtnSet5(lv_event_t * e);
extern lv_obj_t * ui_BtnSet5;
extern lv_obj_t * ui_BtnSet5Lbl;
void ui_event_BtnSet6(lv_event_t * e);
extern lv_obj_t * ui_BtnSet6;
extern lv_obj_t * ui_BtnSet3Lbl2;
void ui_event_BtnSet7(lv_event_t * e);
extern lv_obj_t * ui_BtnSet7;
extern lv_obj_t * ui_BtnSet7Lbl;
void ui_event_BtnSet8(lv_event_t * e);
extern lv_obj_t * ui_BtnSet8;
extern lv_obj_t * ui_BtnSet7Lbl1;
// CUSTOM VARIABLES

// SCREEN: ui_ScrPeers
void ui_ScrPeers_screen_init(void);
void ui_event_ScrPeers(lv_event_t * e);
extern lv_obj_t * ui_ScrPeers;
extern lv_obj_t * ui_RollerPeers1;
extern lv_obj_t * ui_LabelPeersName;
void ui_event_BtnPeer9(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer9;
extern lv_obj_t * ui_BtnPeer4Lbl4;
// CUSTOM VARIABLES

// SCREEN: ui_ScrJSON
void ui_ScrJSON_screen_init(void);
void ui_event_ScrJSON(lv_event_t * e);
extern lv_obj_t * ui_ScrJSON;
extern lv_obj_t * ui_TxtJSON1;
extern lv_obj_t * ui_LblJSON2;
// CUSTOM VARIABLES

// SCREEN: ui_ScrPeer
void ui_ScrPeer_screen_init(void);
void ui_event_ScrPeer(lv_event_t * e);
extern lv_obj_t * ui_ScrPeer;
extern lv_obj_t * ui_ImgPeerType;
extern lv_obj_t * ui_LblPeerName;
extern lv_obj_t * ui_LblPeerTypeLbl;
extern lv_obj_t * ui_Image2;
extern lv_obj_t * ui_Image3;
extern lv_obj_t * ui_Image4;
extern lv_obj_t * ui_Image5;
extern lv_obj_t * ui_Container3;
void ui_event_BtnPeer1(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer1;
void ui_event_BtnPeer1Lbl(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer1Lbl;
void ui_event_BtnPeer2(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer2;
extern lv_obj_t * ui_BtnPeer2Lbl1;
void ui_event_BtnPeer3(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer3;
extern lv_obj_t * ui_BtnPeer3Lbl1;
void ui_event_BtnPeer5(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer5;
extern lv_obj_t * ui_BtnPeer5Lbl1;
void ui_event_BtnPeer6(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer6;
extern lv_obj_t * ui_BtnPeer6Lbl1;
void ui_event_BtnPeer4(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer4;
extern lv_obj_t * ui_BtnPeer4Lbl1;
void ui_event_BtnPeer7(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer7;
void ui_event_BtnPeer7Lbl1(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer7Lbl1;
extern lv_obj_t * ui_BtnPeer10;
void ui_event_BtnPeer7Lbl3(lv_event_t * e);
extern lv_obj_t * ui_BtnPeer7Lbl3;
// CUSTOM VARIABLES

// SCREEN: ui_ScrEichen
void ui_ScrEichen_screen_init(void);
void ui_event_ScrEichen(lv_event_t * e);
extern lv_obj_t * ui_ScrEichen;
extern lv_obj_t * ui_Label1;
void ui_event_BtnEichenStart(lv_event_t * e);
extern lv_obj_t * ui_BtnEichenStart;
extern lv_obj_t * ui_BtnPeer4Lbl3;
extern lv_obj_t * ui_LblEichenPeer;
// CUSTOM VARIABLES

// SCREEN: ui_ScrVolt
void ui_ScrVolt_screen_init(void);
void ui_event_ScrVolt(lv_event_t * e);
extern lv_obj_t * ui_ScrVolt;
void ui_event_Keyboard(lv_event_t * e);
extern lv_obj_t * ui_Keyboard;
extern lv_obj_t * ui_TxtVolt;
void ui_event_LblVoltPeer(lv_event_t * e);
extern lv_obj_t * ui_LblVoltPeer;
// CUSTOM VARIABLES

// SCREEN: ui_ScrPeriph
void ui_ScrPeriph_screen_init(void);
void ui_event_ScrPeriph(lv_event_t * e);
extern lv_obj_t * ui_ScrPeriph;
extern lv_obj_t * ui_ImgPeerChoice;
extern lv_obj_t * ui_LblPeriphChoicePeer;
extern lv_obj_t * ui_LblPeriphChoiceType;
extern lv_obj_t * ui_LblPeriphChoiceOnline;
extern lv_obj_t * ui_LblPeriphChoicePeriph;
// CUSTOM VARIABLES

// SCREEN: ui_ScrMulti
void ui_ScrMulti_screen_init(void);
void ui_event_ScrMulti(lv_event_t * e);
extern lv_obj_t * ui_ScrMulti;
extern lv_obj_t * ui_LblMultiScreenName;
extern lv_obj_t * ui_Container2;
void ui_event_ButtonMulti1(lv_event_t * e);
extern lv_obj_t * ui_ButtonMulti1;
extern lv_obj_t * ui_Label6;
void ui_event_ButtonMulti2(lv_event_t * e);
extern lv_obj_t * ui_ButtonMulti2;
extern lv_obj_t * ui_Label2;
void ui_event_ButtonMulti3(lv_event_t * e);
extern lv_obj_t * ui_ButtonMulti3;
extern lv_obj_t * ui_Label3;
void ui_event_ButtonMulti4(lv_event_t * e);
extern lv_obj_t * ui_ButtonMulti4;
extern lv_obj_t * ui_Label4;
// CUSTOM VARIABLES

// EVENTS

void ui_event____initial_actions0(lv_event_t * e);
extern lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_jeepifybackground_png);    // assets/JeepifyBackground.png
LV_IMG_DECLARE(ui_img_rubicon_png);    // assets/Rubicon.png
LV_IMG_DECLARE(ui_img_menubtn1_png);    // assets/MenuBtn1.png
LV_IMG_DECLARE(ui_img_menubtn2_png);    // assets/MenuBtn2.png
LV_IMG_DECLARE(ui_img_menubtn3_png);    // assets/MenuBtn3.png
LV_IMG_DECLARE(ui_img_menubtn4_png);    // assets/MenuBtn4.png
LV_IMG_DECLARE(ui_img_ansgarmodule_4_png);    // assets/AnsgarModule_4.png
LV_IMG_DECLARE(ui_img_friedermodule_disp_png);    // assets/FriederModule_Disp.png
LV_IMG_DECLARE(ui_img_horstrelais2_png);    // assets/HorstRelais2.png
LV_IMG_DECLARE(ui_img_1769637049);    // assets/PeterRelais-1.png
LV_IMG_DECLARE(ui_img_rolfmodule_round_png);    // assets/RolfModule_round.png
LV_IMG_DECLARE(ui_img_1528892059);    // assets/kipp-1-neutral-120.png
LV_IMG_DECLARE(ui_img_1471590615);    // assets/kipp-1-neutral-45.png
LV_IMG_DECLARE(ui_img_743505413);    // assets/kipp-1-neutral-70.png
LV_IMG_DECLARE(ui_img_1134846501);    // assets/kipp-1-off-120.png
LV_IMG_DECLARE(ui_img_237434643);    // assets/kipp-1-off-45.png
LV_IMG_DECLARE(ui_img_1640860301);    // assets/kipp-1-off-70.png
LV_IMG_DECLARE(ui_img_715952573);    // assets/kipp-1-on-120.png
LV_IMG_DECLARE(ui_img_434995191);    // assets/kipp-1-on-45.png
LV_IMG_DECLARE(ui_img_888658411);    // assets/kipp-1-on-70.png

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
