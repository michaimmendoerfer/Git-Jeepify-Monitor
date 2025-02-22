#ifndef MODULE_H
#define MODULE_H

#include <Jeepify.h>

#define MODULE_MONITOR_360
//#define MODULE_MONITOR_240


#define MODULE_VERSION          "3.70"  
#define PROTOKOLL_VERSION       "2.00"

#ifdef MODULE_MONITOR_360
    #define NODE_NAME "Monitor 360"
    #define NODE_TYPE MONITOR_ROUND
#endif

#ifdef MODULE_MONITOR_240
    #define NODE_NAME "Monitor 24"
    #define NODE_TYPE MONITOR_ROUND
#endif


#endif
