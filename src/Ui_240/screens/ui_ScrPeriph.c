// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.6
// Project name: SQ-Jeepify-Monitor240

#include "../ui.h"

void ui_ScrPeriph_screen_init(void)
{
    ui_ScrPeriph = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrPeriph, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_ScrPeriph, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScrPeriph, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_ScrPeriph, &ui_img_jeepifybackground_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ScrPeriph, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgPeerChoice = lv_img_create(ui_ScrPeriph);
    lv_img_set_src(ui_ImgPeerChoice, &ui_img_menubtn2_png);
    lv_obj_set_width(ui_ImgPeerChoice, LV_SIZE_CONTENT);   /// 60
    lv_obj_set_height(ui_ImgPeerChoice, LV_SIZE_CONTENT);    /// 79
    lv_obj_set_x(ui_ImgPeerChoice, -2);
    lv_obj_set_y(ui_ImgPeerChoice, -65);
    lv_obj_set_align(ui_ImgPeerChoice, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ImgPeerChoice, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImgPeerChoice, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LblPeriphChoicePeer = lv_label_create(ui_ScrPeriph);
    lv_obj_set_width(ui_LblPeriphChoicePeer, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblPeriphChoicePeer, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblPeriphChoicePeer, 0);
    lv_obj_set_y(ui_LblPeriphChoicePeer, 85);
    lv_obj_set_align(ui_LblPeriphChoicePeer, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblPeriphChoicePeer, "Peer");
    lv_obj_set_style_text_color(ui_LblPeriphChoicePeer, lv_color_hex(0xAD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblPeriphChoicePeer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblPeriphChoiceType = lv_label_create(ui_ScrPeriph);
    lv_obj_set_width(ui_LblPeriphChoiceType, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblPeriphChoiceType, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblPeriphChoiceType, 0);
    lv_obj_set_y(ui_LblPeriphChoiceType, 25);
    lv_obj_set_align(ui_LblPeriphChoiceType, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblPeriphChoiceType, "Type: ");
    lv_obj_set_style_text_color(ui_LblPeriphChoiceType, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblPeriphChoiceType, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblPeriphChoiceOnline = lv_label_create(ui_ScrPeriph);
    lv_obj_set_width(ui_LblPeriphChoiceOnline, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblPeriphChoiceOnline, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblPeriphChoiceOnline, -1);
    lv_obj_set_y(ui_LblPeriphChoiceOnline, 50);
    lv_obj_set_align(ui_LblPeriphChoiceOnline, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblPeriphChoiceOnline, "Online");
    lv_obj_set_style_text_color(ui_LblPeriphChoiceOnline, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblPeriphChoiceOnline, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblPeriphChoicePeriph = lv_label_create(ui_ScrPeriph);
    lv_obj_set_width(ui_LblPeriphChoicePeriph, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblPeriphChoicePeriph, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LblPeriphChoicePeriph, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LblPeriphChoicePeriph, "Name");
    lv_obj_set_style_text_color(ui_LblPeriphChoicePeriph, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblPeriphChoicePeriph, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_ScrPeriph, ui_event_ScrPeriph, LV_EVENT_ALL, NULL);

}
