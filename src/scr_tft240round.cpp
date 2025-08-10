#include "scr_tft240round.h"

#include <Arduino.h>
#include "Jeepify.h"
#include <Arduino_GFX_Library.h>
#include "CST816D.h"
#include <lvgl.h>
#include <Module.h>

CST816D Touch(4, 5, 1, 0);

#pragma region Globals
#define GFX_BL 3 // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_GFX *gfx = new Arduino_GC9A01(bus, DF_GFX_RST, 0 /* rotation */, true /* IPS */);

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf;
lv_disp_drv_t disp_drv;

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    #ifndef DIRECT_MODE
        uint32_t w = (area->x2 - area->x1 + 1);
        uint32_t h = (area->y2 - area->y1 + 1);

        #if (LV_COLOR_16_SWAP != 0)
            gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
        #else
            gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
        #endif
    #endif 

    lv_disp_flush_ready(disp_drv);
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

        data->point.x = GC9A01_TFTWIDTH - touchX;
        data->point.y = touchY;
    }
}

void scr_lvgl_init()
{
    if (!gfx->begin())
    {
        Serial.println("gfx->begin() failed!");
    }
    gfx->fillScreen(BLACK);

    #ifdef GFX_BL
        pinMode(GFX_BL, OUTPUT);
        digitalWrite(GFX_BL, HIGH);
    #endif
    
    Touch.begin();

    lv_init();

    #ifdef BATTERY_PORT
        pinMode(BATTERY_PORT, INPUT);
    #endif

    #if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
    #endif

    screenWidth = gfx->width();
    screenHeight = gfx->height();

    #ifdef DIRECT_MODE
        bufSize = screenWidth * screenHeight;
    #else
        bufSize = screenWidth * 40;
    #endif

    #if defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL))
        disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
    #else  // !(defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL)))
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!disp_draw_buf)
        {
            // remove MALLOC_CAP_INTERNAL flag try again
            disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
        }
    #endif // !(defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL)))

    if (!disp_draw_buf)
    {
        Serial.println("LVGL disp_draw_buf allocate failed!");
    }
    else
    {
        lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);
        lv_disp_drv_init(&disp_drv);
        
        disp_drv.hor_res = screenWidth;
        disp_drv.ver_res = screenHeight;
        disp_drv.flush_cb = my_disp_flush;
        disp_drv.draw_buf = &draw_buf;

        #ifdef DIRECT_MODE
            disp_drv.direct_mode = true;
        #endif

        lv_disp_drv_register(&disp_drv);

        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init( &indev_drv );
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = my_touchpad_read;
        lv_indev_drv_register( &indev_drv ); 
    }
}
#pragma endregion Other
//

