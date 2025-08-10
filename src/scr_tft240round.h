#ifndef SCR_TFT240ROUND_H
#define SCR_TFT240ROUND_H

#include <Arduino.h>
#include "CST816D.h"
#include <lvgl.h>

#define SCREEN_RES_HOR 240
#define SCREEN_RES_VER 240

extern CST816D Touch;

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data );
void scr_lvgl_init();
 
#endif