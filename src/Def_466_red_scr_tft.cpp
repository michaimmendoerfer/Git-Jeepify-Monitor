#include <Arduino.h>
#include "Jeepify.h"
#include <Module.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

void scr_lvgl_init()
{
    Board *board = new Board();
    board->init();
    #if LVGL_PORT_AVOID_TEARING_MODE
        auto lcd = board->getLCD();
        // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
        lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
        auto lcd_bus = lcd->getBus();
        /**
         * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
         * "bounce buffer" functionality to enhance the RGB data bandwidth.
         * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
         */
        if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
        }
    #endif
    #endif
        //delay(10000);
        assert(board->begin());

        Serial.println("Initializing LVGL");
        lvgl_port_init(board->getLCD(), board->getTouch());

        /* Lock the mutex due to the LVGL APIs are not thread-safe */
        //lvgl_port_lock(-1);

        //lvgl_port_unlock();
}
