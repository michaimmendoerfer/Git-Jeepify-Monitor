#include "scr_tft480.h"

#include <Arduino.h>
#include "Jeepify.h"
#include <lvgl.h>
#include <Module.h>

void scr_lvgl_init()
{
    smartdisplay_init();

    smartdisplay_lcd_set_backlight(0.5);
    
    __attribute__((unused)) auto disp = lv_disp_get_default();
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);

    //lv_init();
}
#pragma endregion Other
//


