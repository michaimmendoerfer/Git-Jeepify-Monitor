// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SQ-Jeepify-Monitor360

#include "../ui.h"

void ui_ScrEichen_screen_init(void)
{
    ui_ScrEichen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrEichen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_ScrEichen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScrEichen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_ScrEichen, &ui_img_jeepifybackground_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ScrEichen, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_ScrEichen);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 0);
    lv_obj_set_y(ui_Label1, -63);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1,
                      "Make sure, no devices \nare on the PDC's inputs\nStart, to calibrate input\ncurrent to 0 amps!");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnEichenStart = lv_btn_create(ui_ScrEichen);
    lv_obj_set_width(ui_BtnEichenStart, lv_pct(25));
    lv_obj_set_height(ui_BtnEichenStart, lv_pct(10));
    lv_obj_set_x(ui_BtnEichenStart, 0);
    lv_obj_set_y(ui_BtnEichenStart, 45);
    lv_obj_set_align(ui_BtnEichenStart, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnEichenStart, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnEichenStart, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnEichenStart, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnEichenStart, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnEichenStart, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnEichenStart, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnEichenStart, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnPeer4Lbl3 = lv_label_create(ui_BtnEichenStart);
    lv_obj_set_width(ui_BtnPeer4Lbl3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_BtnPeer4Lbl3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_BtnPeer4Lbl3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_BtnPeer4Lbl3, "Eichen");
    lv_obj_set_style_text_color(ui_BtnPeer4Lbl3, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_BtnPeer4Lbl3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblEichenPeer = lv_label_create(ui_ScrEichen);
    lv_obj_set_width(ui_LblEichenPeer, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblEichenPeer, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblEichenPeer, 1);
    lv_obj_set_y(ui_LblEichenPeer, 108);
    lv_obj_set_align(ui_LblEichenPeer, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblEichenPeer, "Peer");
    lv_obj_set_style_text_color(ui_LblEichenPeer, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblEichenPeer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_BtnEichenStart, ui_event_BtnEichenStart, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScrEichen, ui_event_ScrEichen, LV_EVENT_ALL, NULL);

}
