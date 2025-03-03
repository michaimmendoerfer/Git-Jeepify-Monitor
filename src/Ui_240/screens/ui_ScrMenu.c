// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SQ-Jeepify-Monitor240

#include "../ui.h"

void ui_ScrMenu_screen_init(void)
{
    ui_ScrMenu = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrMenu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_ScrMenu, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScrMenu, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_ScrMenu, &ui_img_jeepifybackground_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgRubicon = lv_img_create(ui_ScrMenu);
    lv_img_set_src(ui_ImgRubicon, &ui_img_rubicon_png);
    lv_obj_set_width(ui_ImgRubicon, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ImgRubicon, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ImgRubicon, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ImgRubicon, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImgRubicon, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_ImgRubicon, 250);

    ui_LblMenuVersion = lv_label_create(ui_ScrMenu);
    lv_obj_set_width(ui_LblMenuVersion, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblMenuVersion, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblMenuVersion, 96);
    lv_obj_set_y(ui_LblMenuVersion, 17);
    lv_obj_set_align(ui_LblMenuVersion, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblMenuVersion, "V 3.41");
    lv_obj_set_style_text_color(ui_LblMenuVersion, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblMenuVersion, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LblMenuVersion, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container4 = lv_obj_create(ui_ScrMenu);
    lv_obj_remove_style_all(ui_Container4);
    lv_obj_set_width(ui_Container4, lv_pct(75));
    lv_obj_set_height(ui_Container4, lv_pct(75));
    lv_obj_set_align(ui_Container4, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Container4, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(ui_Container4, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container4, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_pad_row(ui_Container4, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_Container4, 40, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_border_color(ui_Container4, lv_color_hex(0xFFFFFF), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Container4, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Container4, 2, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_BtnMenu1 = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_BtnMenu1, lv_pct(30));
    lv_obj_set_height(ui_BtnMenu1, lv_pct(30));
    lv_obj_set_x(ui_BtnMenu1, lv_pct(91));
    lv_obj_set_y(ui_BtnMenu1, lv_pct(32));
    lv_obj_set_align(ui_BtnMenu1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu1, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu1, &ui_img_menubtn1_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_BtnMenu1, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu1, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(ui_BtnMenu1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_width(ui_BtnMenu1, 2, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_BtnMenu2 = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_BtnMenu2, lv_pct(30));
    lv_obj_set_height(ui_BtnMenu2, lv_pct(30));
    lv_obj_set_x(ui_BtnMenu2, lv_pct(20));
    lv_obj_set_y(ui_BtnMenu2, lv_pct(-20));
    lv_obj_set_align(ui_BtnMenu2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu2, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu2, &ui_img_menubtn2_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu2, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnMenu2, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(ui_BtnMenu2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_opa(ui_BtnMenu2, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_BtnMenu3 = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_BtnMenu3, lv_pct(30));
    lv_obj_set_height(ui_BtnMenu3, lv_pct(30));
    lv_obj_set_x(ui_BtnMenu3, lv_pct(-20));
    lv_obj_set_y(ui_BtnMenu3, lv_pct(20));
    lv_obj_set_align(ui_BtnMenu3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu3, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu3, &ui_img_menubtn3_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu3, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(ui_BtnMenu3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_opa(ui_BtnMenu3, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_BtnMenu4 = lv_btn_create(ui_Container4);
    lv_obj_set_width(ui_BtnMenu4, lv_pct(30));
    lv_obj_set_height(ui_BtnMenu4, lv_pct(30));
    lv_obj_set_x(ui_BtnMenu4, lv_pct(20));
    lv_obj_set_y(ui_BtnMenu4, lv_pct(20));
    lv_obj_set_align(ui_BtnMenu4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu4, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu4, &ui_img_menubtn4_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu4, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_color(ui_BtnMenu4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_opa(ui_BtnMenu4, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    lv_obj_add_event_cb(ui_BtnMenu1, ui_event_BtnMenu1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu2, ui_event_BtnMenu2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu3, ui_event_BtnMenu3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu4, ui_event_BtnMenu4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScrMenu, ui_event_ScrMenu, LV_EVENT_ALL, NULL);

}
