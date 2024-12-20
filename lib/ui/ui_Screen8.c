// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen8_screen_init(void)
{
    ui_Screen8 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen8, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_backcustom = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_backcustom, 194);
    lv_obj_set_height(ui_backcustom, 132);
    lv_obj_set_x(ui_backcustom, -288);
    lv_obj_set_y(ui_backcustom, -162);
    lv_obj_set_align(ui_backcustom, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_backcustom, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_backcustom, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_backcustom, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_backcustom, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_backcustom, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_backcustom, lv_color_hex(0xD1302A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_backcustom, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_backcustom, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_backcustomimg = lv_img_create(ui_backcustom);
    lv_img_set_src(ui_backcustomimg, &ui_img_back_png);
    lv_obj_set_width(ui_backcustomimg, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_backcustomimg, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_backcustomimg, -5);
    lv_obj_set_y(ui_backcustomimg, -1);
    lv_obj_set_align(ui_backcustomimg, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_backcustomimg, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_backcustomimg, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_time1button = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_time1button, 180);
    lv_obj_set_height(ui_time1button, 132);
    lv_obj_set_x(ui_time1button, -92);
    lv_obj_set_y(ui_time1button, -161);
    lv_obj_set_align(ui_time1button, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_time1button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_time1button, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_time1button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_time1button, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_time1button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_time1button, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_time1button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_time1button, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_minsec1 = lv_label_create(ui_time1button);
    lv_obj_set_width(ui_minsec1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_minsec1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_minsec1, 0);
    lv_obj_set_y(ui_minsec1, -31);
    lv_obj_set_align(ui_minsec1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minsec1, "min:sec");
    lv_obj_set_style_text_font(ui_minsec1, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time1 = lv_label_create(ui_time1button);
    lv_obj_set_width(ui_time1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_time1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_time1, -3);
    lv_obj_set_y(ui_time1, 15);
    lv_obj_set_align(ui_time1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_time1, "02:01");
    lv_obj_set_style_text_font(ui_time1, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time1button2 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_time1button2, 180);
    lv_obj_set_height(ui_time1button2, 132);
    lv_obj_set_x(ui_time1button2, 98);
    lv_obj_set_y(ui_time1button2, -158);
    lv_obj_set_align(ui_time1button2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_time1button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_time1button2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_time1button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_time1button2, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_time1button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_time1button2, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_time1button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_time1button2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_minsec3 = lv_label_create(ui_time1button2);
    lv_obj_set_width(ui_minsec3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_minsec3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_minsec3, 0);
    lv_obj_set_y(ui_minsec3, -31);
    lv_obj_set_align(ui_minsec3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minsec3, "min:sec");
    lv_obj_set_style_text_font(ui_minsec3, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time3 = lv_label_create(ui_time1button2);
    lv_obj_set_width(ui_time3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_time3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_time3, -3);
    lv_obj_set_y(ui_time3, 15);
    lv_obj_set_align(ui_time3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_time3, "02:01");
    lv_obj_set_style_text_font(ui_time3, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time1button3 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_time1button3, 180);
    lv_obj_set_height(ui_time1button3, 132);
    lv_obj_set_x(ui_time1button3, 287);
    lv_obj_set_y(ui_time1button3, -159);
    lv_obj_set_align(ui_time1button3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_time1button3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_time1button3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_time1button3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_time1button3, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_time1button3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_time1button3, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_time1button3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_time1button3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_minsec4 = lv_label_create(ui_time1button3);
    lv_obj_set_width(ui_minsec4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_minsec4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_minsec4, 0);
    lv_obj_set_y(ui_minsec4, -31);
    lv_obj_set_align(ui_minsec4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minsec4, "min:sec");
    lv_obj_set_style_text_font(ui_minsec4, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time4 = lv_label_create(ui_time1button3);
    lv_obj_set_width(ui_time4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_time4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_time4, -3);
    lv_obj_set_y(ui_time4, 15);
    lv_obj_set_align(ui_time4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_time4, "02:01");
    lv_obj_set_style_text_font(ui_time4, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rotationbutton1 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_rotationbutton1, 180);
    lv_obj_set_height(ui_rotationbutton1, 132);
    lv_obj_set_x(ui_rotationbutton1, -91);
    lv_obj_set_y(ui_rotationbutton1, -17);
    lv_obj_set_align(ui_rotationbutton1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationbutton1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_rotationbutton1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_rotationbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rotationbutton1, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rotationbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rotationbutton1, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rotationbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_rotationbutton1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tipatrykutnik1 = lv_img_create(ui_rotationbutton1);
    lv_img_set_src(ui_tipatrykutnik1, &ui_img_rectangle_582_png);
    lv_obj_set_width(ui_tipatrykutnik1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tipatrykutnik1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_tipatrykutnik1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tipatrykutnik1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_tipatrykutnik1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection1 = lv_img_create(ui_rotationbutton1);
    lv_img_set_src(ui_rotationdirection1, &ui_img_left_line1_png);
    lv_obj_set_width(ui_rotationdirection1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection3 = lv_img_create(ui_rotationbutton1);
    lv_img_set_src(ui_rotationdirection3, &ui_img_right_line1_png);
    lv_obj_set_width(ui_rotationdirection3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection3, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationbutton3 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_rotationbutton3, 180);
    lv_obj_set_height(ui_rotationbutton3, 132);
    lv_obj_set_x(ui_rotationbutton3, 100);
    lv_obj_set_y(ui_rotationbutton3, -17);
    lv_obj_set_align(ui_rotationbutton3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationbutton3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_rotationbutton3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_rotationbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rotationbutton3, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rotationbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rotationbutton3, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rotationbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_rotationbutton3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tipatrykutnik3 = lv_img_create(ui_rotationbutton3);
    lv_img_set_src(ui_tipatrykutnik3, &ui_img_169476923);
    lv_obj_set_width(ui_tipatrykutnik3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tipatrykutnik3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tipatrykutnik3, 0);
    lv_obj_set_y(ui_tipatrykutnik3, 23);
    lv_obj_set_align(ui_tipatrykutnik3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tipatrykutnik3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_tipatrykutnik3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection5 = lv_img_create(ui_rotationbutton3);
    lv_img_set_src(ui_rotationdirection5, &ui_img_left_line1_png);
    lv_obj_set_width(ui_rotationdirection5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection2 = lv_img_create(ui_rotationbutton3);
    lv_img_set_src(ui_rotationdirection2, &ui_img_right_line1_png);
    lv_obj_set_width(ui_rotationdirection2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection2, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationbutton4 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_rotationbutton4, 180);
    lv_obj_set_height(ui_rotationbutton4, 132);
    lv_obj_set_x(ui_rotationbutton4, 290);
    lv_obj_set_y(ui_rotationbutton4, -15);
    lv_obj_set_align(ui_rotationbutton4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationbutton4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_rotationbutton4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_rotationbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rotationbutton4, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rotationbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rotationbutton4, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rotationbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_rotationbutton4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tipatrykutnik4 = lv_img_create(ui_rotationbutton4);
    lv_img_set_src(ui_tipatrykutnik4, &ui_img_169479996);
    lv_obj_set_width(ui_tipatrykutnik4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tipatrykutnik4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tipatrykutnik4, -1);
    lv_obj_set_y(ui_tipatrykutnik4, 2);
    lv_obj_set_align(ui_tipatrykutnik4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tipatrykutnik4, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_tipatrykutnik4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection7 = lv_img_create(ui_rotationbutton4);
    lv_img_set_src(ui_rotationdirection7, &ui_img_left_line1_png);
    lv_obj_set_width(ui_rotationdirection7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection7, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_rotationdirection4 = lv_img_create(ui_rotationbutton4);
    lv_img_set_src(ui_rotationdirection4, &ui_img_right_line1_png);
    lv_obj_set_width(ui_rotationdirection4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rotationdirection4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_rotationdirection4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationdirection4, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_rotationdirection4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_speedbutton1 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_speedbutton1, 180);
    lv_obj_set_height(ui_speedbutton1, 132);
    lv_obj_set_x(ui_speedbutton1, -282);
    lv_obj_set_y(ui_speedbutton1, 154);
    lv_obj_set_align(ui_speedbutton1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedbutton1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedbutton1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedbutton1, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedbutton1, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedbutton1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedbutton1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmlabel = lv_label_create(ui_speedbutton1);
    lv_obj_set_width(ui_rpmlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmlabel, -3);
    lv_obj_set_y(ui_rpmlabel, -29);
    lv_obj_set_align(ui_rpmlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmlabel, "RPM");
    lv_obj_set_style_text_font(ui_rpmlabel, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmvalue = lv_label_create(ui_speedbutton1);
    lv_obj_set_width(ui_rpmvalue, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmvalue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmvalue, 0);
    lv_obj_set_y(ui_rpmvalue, 16);
    lv_obj_set_align(ui_rpmvalue, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmvalue, "10");
    lv_obj_set_style_text_font(ui_rpmvalue, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_speedbutton3 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_speedbutton3, 180);
    lv_obj_set_height(ui_speedbutton3, 132);
    lv_obj_set_x(ui_speedbutton3, -92);
    lv_obj_set_y(ui_speedbutton3, 153);
    lv_obj_set_align(ui_speedbutton3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedbutton3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedbutton3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedbutton3, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedbutton3, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedbutton3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedbutton3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmlabel2 = lv_label_create(ui_speedbutton3);
    lv_obj_set_width(ui_rpmlabel2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmlabel2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmlabel2, -3);
    lv_obj_set_y(ui_rpmlabel2, -29);
    lv_obj_set_align(ui_rpmlabel2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmlabel2, "RPM");
    lv_obj_set_style_text_font(ui_rpmlabel2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmvalue2 = lv_label_create(ui_speedbutton3);
    lv_obj_set_width(ui_rpmvalue2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmvalue2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmvalue2, 0);
    lv_obj_set_y(ui_rpmvalue2, 16);
    lv_obj_set_align(ui_rpmvalue2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmvalue2, "10");
    lv_obj_set_style_text_font(ui_rpmvalue2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_speedbutton4 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_speedbutton4, 180);
    lv_obj_set_height(ui_speedbutton4, 132);
    lv_obj_set_x(ui_speedbutton4, 102);
    lv_obj_set_y(ui_speedbutton4, 151);
    lv_obj_set_align(ui_speedbutton4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedbutton4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedbutton4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedbutton4, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedbutton4, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedbutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedbutton4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmlabel3 = lv_label_create(ui_speedbutton4);
    lv_obj_set_width(ui_rpmlabel3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmlabel3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmlabel3, -3);
    lv_obj_set_y(ui_rpmlabel3, -29);
    lv_obj_set_align(ui_rpmlabel3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmlabel3, "RPM");
    lv_obj_set_style_text_font(ui_rpmlabel3, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmvalue3 = lv_label_create(ui_speedbutton4);
    lv_obj_set_width(ui_rpmvalue3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmvalue3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmvalue3, 0);
    lv_obj_set_y(ui_rpmvalue3, 16);
    lv_obj_set_align(ui_rpmvalue3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmvalue3, "10");
    lv_obj_set_style_text_font(ui_rpmvalue3, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_speedbutton5 = lv_btn_create(ui_Screen8);
    lv_obj_set_width(ui_speedbutton5, 180);
    lv_obj_set_height(ui_speedbutton5, 132);
    lv_obj_set_x(ui_speedbutton5, 292);
    lv_obj_set_y(ui_speedbutton5, 148);
    lv_obj_set_align(ui_speedbutton5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedbutton5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedbutton5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedbutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedbutton5, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedbutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedbutton5, lv_color_hex(0xBF312C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedbutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedbutton5, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmlabel4 = lv_label_create(ui_speedbutton5);
    lv_obj_set_width(ui_rpmlabel4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmlabel4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmlabel4, -3);
    lv_obj_set_y(ui_rpmlabel4, -29);
    lv_obj_set_align(ui_rpmlabel4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmlabel4, "RPM");
    lv_obj_set_style_text_font(ui_rpmlabel4, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rpmvalue4 = lv_label_create(ui_speedbutton5);
    lv_obj_set_width(ui_rpmvalue4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_rpmvalue4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_rpmvalue4, 0);
    lv_obj_set_y(ui_rpmvalue4, 16);
    lv_obj_set_align(ui_rpmvalue4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_rpmvalue4, "10");
    lv_obj_set_style_text_font(ui_rpmvalue4, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_backcustom, ui_event_backcustom, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_rotationbutton1, ui_event_rotationbutton1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_rotationbutton3, ui_event_rotationbutton3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_rotationbutton4, ui_event_rotationbutton4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedbutton1, ui_event_speedbutton1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedbutton3, ui_event_speedbutton3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedbutton4, ui_event_speedbutton4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedbutton5, ui_event_speedbutton5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_time1button, ui_event_timeset, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_time1button2, ui_event_timeset, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_time1button3, ui_event_timeset, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedbutton1, ui_event_speedset, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_speedbutton3, ui_event_speedset, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_speedbutton4, ui_event_speedset, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_speedbutton5, ui_event_speedset, LV_EVENT_CLICKED, NULL);


}
