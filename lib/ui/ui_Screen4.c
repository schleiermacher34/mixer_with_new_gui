// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen4, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button7 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Button7, 132);
    lv_obj_set_height(ui_Button7, 132);
    lv_obj_set_x(ui_Button7, -315);
    lv_obj_set_y(ui_Button7, -161);
    lv_obj_set_align(ui_Button7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button7, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button7, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button7, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button7, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel15 = lv_obj_create(ui_Button7);
    lv_obj_set_width(ui_Panel15, 115);
    lv_obj_set_height(ui_Panel15, 116);
    lv_obj_set_x(ui_Panel15, -1);
    lv_obj_set_y(ui_Panel15, -1);
    lv_obj_set_align(ui_Panel15, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel15, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel15, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel15, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel15, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel15, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image12 = lv_img_create(ui_Panel15);
    lv_img_set_src(ui_Image12, &ui_img_settinggroup_png);
    lv_obj_set_width(ui_Image12, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image12, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image12, -1);
    lv_obj_set_y(ui_Image12, 0);
    lv_obj_set_align(ui_Image12, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image12, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button4 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Button4, 354);
    lv_obj_set_height(ui_Button4, 132);
    lv_obj_set_x(ui_Button4, -206);
    lv_obj_set_y(ui_Button4, 149);
    lv_obj_set_align(ui_Button4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button4, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button4, lv_color_hex(0xDF312A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel16 = lv_obj_create(ui_Button4);
    lv_obj_set_width(ui_Panel16, 143);
    lv_obj_set_height(ui_Panel16, 116);
    lv_obj_set_x(ui_Panel16, -95);
    lv_obj_set_y(ui_Panel16, -1);
    lv_obj_set_align(ui_Panel16, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel16, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel16, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel16, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel16, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel16, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel16, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel16, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel16, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image15 = lv_img_create(ui_Panel16);
    lv_img_set_src(ui_Image15, &ui_img_back_png);
    lv_obj_set_width(ui_Image15, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image15, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image15, -9);
    lv_obj_set_y(ui_Image15, 1);
    lv_obj_set_align(ui_Image15, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image15, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image15, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label28 = lv_label_create(ui_Button4);
    lv_obj_set_width(ui_Label28, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label28, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label28, 82);
    lv_obj_set_y(ui_Label28, -1);
    lv_obj_set_align(ui_Label28, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label28, "BACK");
    lv_obj_set_style_text_font(ui_Label28, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_speedchange3 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_speedchange3, 354);
    lv_obj_set_height(ui_speedchange3, 132);
    lv_obj_set_x(ui_speedchange3, -205);
    lv_obj_set_y(ui_speedchange3, 7);
    lv_obj_set_align(ui_speedchange3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedchange3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedchange3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedchange3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedchange3, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedchange3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedchange3, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedchange3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedchange3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel17 = lv_obj_create(ui_speedchange3);
    lv_obj_set_width(ui_Panel17, 175);
    lv_obj_set_height(ui_Panel17, 116);
    lv_obj_set_x(ui_Panel17, -80);
    lv_obj_set_y(ui_Panel17, 0);
    lv_obj_set_align(ui_Panel17, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel17, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel17, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel17, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel17, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel17, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel17, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel17, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel17, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image16 = lv_img_create(ui_Panel17);
    lv_img_set_src(ui_Image16, &ui_img_speed_png);
    lv_obj_set_width(ui_Image16, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image16, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image16, -42);
    lv_obj_set_y(ui_Image16, -2);
    lv_obj_set_align(ui_Image16, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image16, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image16, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label30 = lv_label_create(ui_Panel17);
    lv_obj_set_width(ui_Label30, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label30, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label30, 33);
    lv_obj_set_y(ui_Label30, 1);
    lv_obj_set_align(ui_Label30, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label30, "10");
    lv_obj_set_style_text_color(ui_Label30, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label30, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label30, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label27 = lv_label_create(ui_speedchange3);
    lv_obj_set_width(ui_Label27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label27, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label27, 85);
    lv_obj_set_y(ui_Label27, -1);
    lv_obj_set_align(ui_Label27, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label27, "RPM ");
    lv_obj_set_style_text_font(ui_Label27, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rotationbutton2 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_rotationbutton2, 352);
    lv_obj_set_height(ui_rotationbutton2, 132);
    lv_obj_set_x(ui_rotationbutton2, 197);
    lv_obj_set_y(ui_rotationbutton2, 5);
    lv_obj_set_align(ui_rotationbutton2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationbutton2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_rotationbutton2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_rotationbutton2, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rotationbutton2, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rotationbutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rotationbutton2, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rotationbutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_rotationbutton2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_rotationbutton2, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_rotationbutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_rotationbutton2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_rotationbutton2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel19 = lv_obj_create(ui_rotationbutton2);
    lv_obj_set_width(ui_Panel19, 116);
    lv_obj_set_height(ui_Panel19, 116);
    lv_obj_set_x(ui_Panel19, -109);
    lv_obj_set_y(ui_Panel19, 0);
    lv_obj_set_align(ui_Panel19, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel19, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel19, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel19, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel19, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel19, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel19, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel19, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel19, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image18 = lv_img_create(ui_Panel19);
    lv_img_set_src(ui_Image18, &ui_img_left_line1_png);
    lv_obj_set_width(ui_Image18, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image18, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image18, -3);
    lv_obj_set_y(ui_Image18, 0);
    lv_obj_set_align(ui_Image18, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image18, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image18, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label32 = lv_label_create(ui_rotationbutton2);
    lv_obj_set_width(ui_Label32, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label32, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label32, 57);
    lv_obj_set_y(ui_Label32, -2);
    lv_obj_set_align(ui_Label32, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label32, "Rotation\nchange");
    lv_obj_set_style_text_font(ui_Label32, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label29 = lv_label_create(ui_rotationbutton2);
    lv_obj_set_width(ui_Label29, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label29, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label29, -88);
    lv_obj_set_y(ui_Label29, 0);
    lv_obj_set_align(ui_Label29, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label29, "Rotation\nchange");
    lv_obj_add_flag(ui_Label29, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_text_font(ui_Label29, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel9 = lv_obj_create(ui_rotationbutton2);
    lv_obj_set_width(ui_Panel9, 116);
    lv_obj_set_height(ui_Panel9, 116);
    lv_obj_set_x(ui_Panel9, 121);
    lv_obj_set_y(ui_Panel9, 0);
    lv_obj_set_align(ui_Panel9, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Panel9, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_Panel9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel9, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel9, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel9, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel9, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image14 = lv_img_create(ui_Panel9);
    lv_img_set_src(ui_Image14, &ui_img_right_line1_png);
    lv_obj_set_width(ui_Image14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image14, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image14, -3);
    lv_obj_set_y(ui_Image14, 0);
    lv_obj_set_align(ui_Image14, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image14, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image14, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_modebutton8 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_modebutton8, 354);
    lv_obj_set_height(ui_modebutton8, 132);
    lv_obj_set_x(ui_modebutton8, 194);
    lv_obj_set_y(ui_modebutton8, 148);
    lv_obj_set_align(ui_modebutton8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton8, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton8, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton8, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton8, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton8, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton8, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label34 = lv_label_create(ui_modebutton8);
    lv_obj_set_width(ui_Label34, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label34, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label34, 59);
    lv_obj_set_y(ui_Label34, -31);
    lv_obj_set_align(ui_Label34, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label34, "min: sec");
    lv_obj_set_style_text_font(ui_Label34, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel20 = lv_obj_create(ui_modebutton8);
    lv_obj_set_width(ui_Panel20, 129);
    lv_obj_set_height(ui_Panel20, 116);
    lv_obj_set_x(ui_Panel20, -105);
    lv_obj_set_y(ui_Panel20, 0);
    lv_obj_set_align(ui_Panel20, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel20, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel20, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel20, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel20, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel20, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel20, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel20, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel20, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image19 = lv_img_create(ui_Panel20);
    lv_img_set_src(ui_Image19, &ui_img_clock_png);
    lv_obj_set_width(ui_Image19, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image19, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image19, -2);
    lv_obj_set_y(ui_Image19, 0);
    lv_obj_set_align(ui_Image19, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image19, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image19, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label31 = lv_label_create(ui_modebutton8);
    lv_obj_set_width(ui_Label31, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label31, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label31, 59);
    lv_obj_set_y(ui_Label31, 13);
    lv_obj_set_align(ui_Label31, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label31, "02:15");
    lv_obj_set_style_text_color(ui_Label31, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label31, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label31, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button8 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Button8, 111);
    lv_obj_set_height(ui_Button8, 105);
    lv_obj_set_x(ui_Button8, -50);
    lv_obj_set_y(ui_Button8, -164);
    lv_obj_set_align(ui_Button8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button8, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button8, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button8, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button8, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image20 = lv_img_create(ui_Button8);
    lv_img_set_src(ui_Image20, &ui_img_left_line2_png);
    lv_obj_set_width(ui_Image20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image20, -1);
    lv_obj_set_y(ui_Image20, 2);
    lv_obj_set_align(ui_Image20, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image20, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image20, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button5 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Button5, 111);
    lv_obj_set_height(ui_Button5, 105);
    lv_obj_set_x(ui_Button5, 318);
    lv_obj_set_y(ui_Button5, -167);
    lv_obj_set_align(ui_Button5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button5, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button5, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button5, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button5, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image21 = lv_img_create(ui_Button5);
    lv_img_set_src(ui_Image21, &ui_img_right_line2_png);
    lv_obj_set_width(ui_Image21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image21, 13);
    lv_obj_set_y(ui_Image21, 4);
    lv_obj_set_align(ui_Image21, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image21, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image21, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label33 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label33, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label33, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label33, 138);
    lv_obj_set_y(ui_Label33, -98);
    lv_obj_set_align(ui_Label33, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label33, "PROGRAM");
    lv_obj_set_style_text_color(ui_Label33, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label33, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label33, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label35 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label35, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label35, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label35, 135);
    lv_obj_set_y(ui_Label35, -165);
    lv_obj_set_align(ui_Label35, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label35, "1");
    lv_obj_set_style_text_color(ui_Label35, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label35, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_decor(ui_Label35, LV_TEXT_DECOR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label35, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button7, ui_event_Button7, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button4, ui_event_Button4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedchange3, ui_event_speedchange3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_rotationbutton2, ui_event_rotationbutton2, LV_EVENT_ALL, NULL);

}
