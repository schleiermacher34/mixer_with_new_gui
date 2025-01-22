#include "ui.h"

void ui_Screen11_screen_init(void)
{
    ui_Screen11 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen11, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen11, lv_color_hex(0x26272C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    
    ui_servicespeed = lv_label_create(ui_Screen11);
    lv_obj_set_width(ui_servicespeed, 67);
    lv_obj_set_height(ui_servicespeed, 57);
    lv_obj_set_x(ui_servicespeed, 4);
    lv_obj_set_y(ui_servicespeed, -50);
    lv_obj_set_align(ui_servicespeed, LV_ALIGN_CENTER);
    lv_label_set_text(ui_servicespeed, "10");
    lv_obj_set_style_text_color(ui_servicespeed, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_servicespeed, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_servicespeed, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_servicespeedtext = lv_label_create(ui_Screen11);
    lv_obj_set_width(ui_servicespeedtext, 300);
    lv_obj_set_height(ui_servicespeedtext, 57);
    lv_obj_set_x(ui_servicespeedtext, 4);
    lv_obj_set_y(ui_servicespeedtext, -130);
    lv_obj_set_align(ui_servicespeedtext, LV_ALIGN_CENTER);
    lv_label_set_text(ui_servicespeedtext, "min speed value");
    lv_obj_set_style_text_color(ui_servicespeedtext, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_servicespeedtext, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_servicespeedtext, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_servicespeedtext2 = lv_label_create(ui_Screen11);
    lv_obj_set_width(ui_servicespeedtext2, 300);
    lv_obj_set_height(ui_servicespeedtext2, 57);
    lv_obj_set_x(ui_servicespeedtext2, 4);
    lv_obj_set_y(ui_servicespeedtext2, 20);
    lv_obj_set_align(ui_servicespeedtext2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_servicespeedtext2, "max speed value");
    lv_obj_set_style_text_color(ui_servicespeedtext2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_servicespeedtext2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_servicespeedtext2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_servicespeedmax = lv_label_create(ui_Screen11);
    lv_obj_set_width(ui_servicespeedmax, 67);
    lv_obj_set_height(ui_servicespeedmax, 70);
    lv_obj_set_x(ui_servicespeedmax, 4);
    lv_obj_set_y(ui_servicespeedmax, 90);
    lv_obj_set_align(ui_servicespeedmax, LV_ALIGN_CENTER);
    lv_label_set_text(ui_servicespeedmax, "60");
    lv_obj_set_style_text_color(ui_servicespeedmax, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_servicespeedmax, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_servicespeedmax, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_Button85 = lv_btn_create(ui_Screen11);
    lv_obj_set_width(ui_Button85, 111);
    lv_obj_set_height(ui_Button85, 105);
    lv_obj_set_x(ui_Button85, -150);
    lv_obj_set_y(ui_Button85, -70);
    lv_obj_set_align(ui_Button85, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button85, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button85, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button85, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button85, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button85, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button85, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button85, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button85, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button85, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image155 = lv_img_create(ui_Button85);
    lv_img_set_src(ui_Image155, &ui_img_left_line2_png);
    lv_obj_set_width(ui_Image155, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image155, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image155, -1);
    lv_obj_set_y(ui_Image155, 2);
    lv_obj_set_align(ui_Image155, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image155, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image155, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button55 = lv_btn_create(ui_Screen11);
    lv_obj_set_width(ui_Button55, 111);
    lv_obj_set_height(ui_Button55, 105);
    lv_obj_set_x(ui_Button55, 150);
    lv_obj_set_y(ui_Button55, -70);
    lv_obj_set_align(ui_Button55, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button55, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button55, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button55, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button55, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button55, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button55, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button55, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button55, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button55, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image154 = lv_img_create(ui_Button55);
    lv_img_set_src(ui_Image154, &ui_img_right_line2_png);
    lv_obj_set_width(ui_Image154, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image154, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image154, 13);
    lv_obj_set_y(ui_Image154, 4);
    lv_obj_set_align(ui_Image154, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image154, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image154, LV_OBJ_FLAG_SCROLLABLE);      /// Flags



    ui_Button86 = lv_btn_create(ui_Screen11);
    lv_obj_set_width(ui_Button86, 111);
    lv_obj_set_height(ui_Button86, 105);
    lv_obj_set_x(ui_Button86, -150);
    lv_obj_set_y(ui_Button86, 80);
    lv_obj_set_align(ui_Button86, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button86, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button86, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button86, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button86, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button86, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button86, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button86, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button86, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button86, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image156 = lv_img_create(ui_Button86);
    lv_img_set_src(ui_Image156, &ui_img_left_line2_png);
    lv_obj_set_width(ui_Image156, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image156, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image156, -1);
    lv_obj_set_y(ui_Image156, 2);
    lv_obj_set_align(ui_Image156, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image156, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image156, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button56 = lv_btn_create(ui_Screen11);
    lv_obj_set_width(ui_Button56, 111);
    lv_obj_set_height(ui_Button56, 105);
    lv_obj_set_x(ui_Button56, 150);
    lv_obj_set_y(ui_Button56, 80);
    lv_obj_set_align(ui_Button56, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button56, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button56, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button56, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button56, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button56, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button56, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button56, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button56, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button56, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image159 = lv_img_create(ui_Button56);
    lv_img_set_src(ui_Image159, &ui_img_right_line2_png);
    lv_obj_set_width(ui_Image159, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image159, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image159, 13);
    lv_obj_set_y(ui_Image159, 4);
    lv_obj_set_align(ui_Image159, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image159, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image159, LV_OBJ_FLAG_SCROLLABLE);      /// Flags


    ui_SaveAndSwitchButton = lv_btn_create(ui_Screen11);
    lv_obj_set_width(ui_SaveAndSwitchButton, 354);
    lv_obj_set_height(ui_SaveAndSwitchButton, 132);
    lv_obj_set_x(ui_SaveAndSwitchButton, 0);
    lv_obj_set_y(ui_SaveAndSwitchButton, 350);
    lv_obj_set_align(ui_SaveAndSwitchButton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_SaveAndSwitchButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_SaveAndSwitchButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SaveAndSwitchButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_SaveAndSwitchButton, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SaveAndSwitchButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_SaveAndSwitchButton, lv_color_hex(0xDF312A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_SaveAndSwitchButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_SaveAndSwitchButton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

}