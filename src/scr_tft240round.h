#ifndef SCR_TFT240ROUND_H
#define SCR_TFT240ROUND_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "CST816D.h"
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

#define SCREEN_RES_HOR 240
#define SCREEN_RES_VER 240
#define DRAW_BUF_SIZE (SCREEN_RES_HOR * SCREEN_RES_VER / 20 * (LV_COLOR_DEPTH / 8))

extern TFT_eSPI tft;
extern CST816D Touch;

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data );
void scr_lvgl_init();
 
#endif