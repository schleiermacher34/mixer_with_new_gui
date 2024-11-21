// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen9_screen_init(void)
{
    ui_Screen9 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen9, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Keyboard2 = lv_keyboard_create(ui_Screen9);
    lv_keyboard_set_mode(ui_Keyboard2, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_width(ui_Keyboard2, 782);
    lv_obj_set_height(ui_Keyboard2, 318);
    lv_obj_set_x(ui_Keyboard2, -3);
    lv_obj_set_y(ui_Keyboard2, 81);
    lv_obj_set_align(ui_Keyboard2, LV_ALIGN_CENTER);
    lv_obj_set_style_radius(ui_Keyboard2, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Keyboard2, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Keyboard2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Keyboard2, lv_color_hex(0x313841), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Keyboard2, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Keyboard2, lv_color_hex(0xE63029), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Keyboard2, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Keyboard2, &lv_font_montserrat_32, LV_PART_ITEMS | LV_STATE_DEFAULT);

    ui_Button6 = lv_btn_create(ui_Keyboard2);
    lv_obj_set_width(ui_Button6, 291);
    lv_obj_set_height(ui_Button6, 148);
    lv_obj_set_x(ui_Button6, 234);
    lv_obj_set_y(ui_Button6, -76);
    lv_obj_set_align(ui_Button6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button6, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button6, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button6, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image8 = lv_img_create(ui_Button6);
    lv_img_set_src(ui_Image8, &ui_img_enter_line_png);
    lv_obj_set_width(ui_Image8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Image8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image8, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_timevaluemin = lv_textarea_create(ui_Screen9);
    lv_obj_set_width(ui_timevaluemin, 234);
    lv_obj_set_height(ui_timevaluemin, LV_SIZE_CONTENT);    /// 94
    lv_obj_set_x(ui_timevaluemin, -187);
    lv_obj_set_y(ui_timevaluemin, -136);
    lv_obj_set_align(ui_timevaluemin, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_timevaluemin, "Enter time value in minutes");
    lv_textarea_set_one_line(ui_timevaluemin, true);



    ui_minuteslabel = lv_label_create(ui_Screen9);
    lv_obj_set_width(ui_minuteslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_minuteslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_minuteslabel, -192);
    lv_obj_set_y(ui_minuteslabel, -182);
    lv_obj_set_align(ui_minuteslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minuteslabel, "Minutes");
    lv_obj_set_style_text_color(ui_minuteslabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_minuteslabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_minuteslabel, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_twodotslabel = lv_label_create(ui_Screen9);
    lv_obj_set_width(ui_twodotslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_twodotslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_twodotslabel, -38);
    lv_obj_set_y(ui_twodotslabel, -140);
    lv_obj_set_align(ui_twodotslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_twodotslabel, ":");
    lv_obj_set_style_text_color(ui_twodotslabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_twodotslabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_twodotslabel, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timevaluemin2 = lv_textarea_create(ui_Screen9);
    lv_obj_set_width(ui_timevaluemin2, 234);
    lv_obj_set_height(ui_timevaluemin2, LV_SIZE_CONTENT);    /// 94
    lv_obj_set_x(ui_timevaluemin2, 119);
    lv_obj_set_y(ui_timevaluemin2, -139);
    lv_obj_set_align(ui_timevaluemin2, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_timevaluemin2, "Enter time value in seconds");
    lv_textarea_set_one_line(ui_timevaluemin2, true);



    ui_secondslabel = lv_label_create(ui_Screen9);
    lv_obj_set_width(ui_secondslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_secondslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_secondslabel, 118);
    lv_obj_set_y(ui_secondslabel, -186);
    lv_obj_set_align(ui_secondslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_secondslabel, "Seconds");
    lv_obj_set_style_text_color(ui_secondslabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_secondslabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_secondslabel, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_keyboard_set_textarea(ui_Keyboard2, ui_timevaluemin);
    lv_obj_add_event_cb(ui_Button6, ui_event_Button6, LV_EVENT_ALL, NULL);

}
