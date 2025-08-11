#ifndef MODULE_H
#define MODULE_H

#include <Jeepify.h>

//#define MODULE_MONITOR_360
//#define MODULE_MONITOR_240
#define MODULE_MONITOR_240_C3
//#define MODULE_MONITOR_240_S3

#define MODULE_VERSION          "4.10"  
#define PROTOKOLL_VERSION       "3.01"

#ifdef MODULE_MONITOR_360
    #define NODE_NAME "Monitor 360"
    #define NODE_TYPE MONITOR_ROUND
    #define SCREEN_RES_HOR 360
    #define SCREEN_RES_VER 360
    #define UI_H_DIR        "Ui_360/ui.h"
    #define UI_EVENTS_H_DIR "Ui_360/ui_events.h" 
#endif

#ifdef MODULE_MONITOR_240
    #define NODE_NAME "Monitor 240"
    #define NODE_TYPE MONITOR_ROUND
    #define SCREEN_RES_HOR 240
    #define SCREEN_RES_VER 240
    #define UI_H_DIR        "Ui_240/ui.h"
    #define UI_EVENTS_H_DIR "Ui_240/ui_events.h" 
#endif

#ifdef MODULE_MONITOR_240_S3
    #define NODE_NAME       "M240_S3"
    #define NODE_TYPE       MONITOR_ROUND
    #define BATTERY_PORT    1
    #define BATTERY_DEVIDER 2
    #define SCREEN_RES_HOR  240
    #define SCREEN_RES_VER  240
    #define UI_H_DIR        "Ui_240/ui.h"
    #define UI_EVENTS_H_DIR "Ui_240/ui_events.h" 
#endif

#ifdef MODULE_MONITOR_240_C3
    #define NODE_NAME       "M240_C3"
    #define NODE_TYPE       MONITOR_ROUND
    #define SCREEN_RES_HOR  240
    #define SCREEN_RES_VER  240
    #define UI_H_DIR        "Ui_240/ui.h"
    #define UI_EVENTS_H_DIR "Ui_240/ui_events.h" 
#endif


#endif
