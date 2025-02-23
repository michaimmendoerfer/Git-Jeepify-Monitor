#ifndef MODULE_H
#define MODULE_H

#include <Jeepify.h>

//#define MODULE_MONITOR_360
#define MODULE_MONITOR_240

#define MODULE_VERSION          "3.70"  
#define PROTOKOLL_VERSION       "2.00"

#ifdef MODULE_MONITOR_360
    #define NODE_NAME "Monitor 360"
    #define NODE_TYPE MONITOR_ROUND
    #define SCREEN_RES_HOR 360
    #define SCREEN_RES_VER 360
    #include "scr_st77916.h"
#endif

#ifdef MODULE_MONITOR_240
    #define NODE_NAME "Monitor 240"
    #define NODE_TYPE MONITOR_ROUND
    #define SCREEN_RES_HOR 240
    #define SCREEN_RES_VER 240
    #include "scr_tft240round.h"
#endif


#endif
