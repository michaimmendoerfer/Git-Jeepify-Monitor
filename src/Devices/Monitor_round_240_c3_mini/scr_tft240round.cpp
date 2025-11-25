#include "scr_tft240round.h"

#include <Arduino.h>
#include "Jeepify.h"
#include <TFT_eSPI.h>
#include "CST816D.h"
#include <lvgl.h>
#include <Module.h>

TFT_eSPI tft = TFT_eSPI(SCREEN_RES_HOR, SCREEN_RES_VER); /* TFT instance */
#ifdef MODULE_MONITOR_240   
    CST816D Touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);
#endif
#ifdef MODULE_MONITOR_240_S3  
    CST816D Touch(11, 7, 6, 12);
#endif

#pragma region Globals
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[ SCREEN_RES_HOR * SCREEN_RES_VER / 20 ];

void scr_lvgl_init()
{
    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(false);
    tft.begin();
    Touch.begin();

    lv_init();
    
    lv_disp_draw_buf_init( &draw_buf, buf1, NULL, SCREEN_RES_HOR * SCREEN_RES_VER / 20 );
 
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = SCREEN_RES_HOR;
    disp_drv.ver_res = SCREEN_RES_VER;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv ); 
}

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data ) 
{
    uint16_t touchX, touchY;
    uint8_t  Gesture;

    bool touched = Touch.getTouch( &touchX, &touchY, &Gesture);

    if( !touched ) {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    else {
        data->state = LV_INDEV_STATE_PRESSED;

        data->point.x = SCREEN_RES_HOR - touchX;
        data->point.y = touchY;
    }
}
#pragma endregion Other
//


