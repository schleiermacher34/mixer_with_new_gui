// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x26272C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel1 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel1, 294);
    lv_obj_set_height(ui_Panel1, 294);
    lv_obj_set_x(ui_Panel1, -226);
    lv_obj_set_y(ui_Panel1, 76);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel1, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel1, lv_color_hex(0x26272C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_shadow_color(ui_Panel1, lv_color_hex(0x363A45), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel1, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel1, 12, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel1, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_Panel1);
    lv_obj_set_width(ui_Panel2, 266);
    lv_obj_set_height(ui_Panel2, 266);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel2, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_starstopbutton = lv_btn_create(ui_Panel2);
    lv_obj_set_width(ui_starstopbutton, 215);
    lv_obj_set_height(ui_starstopbutton, 215);
    lv_obj_set_align(ui_starstopbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_starstopbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_starstopbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_starstopbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_starstopbutton, lv_color_hex(0x434752), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_starstopbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_starstopbutton);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 2);
    lv_obj_set_y(ui_Label1, 75);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "START");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image1 = lv_img_create(ui_starstopbutton);
    lv_img_set_src(ui_Image1, &ui_img_vector_png);
    lv_obj_set_width(ui_Image1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image1, 3);
    lv_obj_set_y(ui_Image1, -1);
    lv_obj_set_align(ui_Image1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label2 = lv_label_create(ui_starstopbutton);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, 0);
    lv_obj_set_y(ui_Label2, 76);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "STOP");
    lv_obj_add_flag(ui_Label2, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image2 = lv_img_create(ui_starstopbutton);
    lv_img_set_src(ui_Image2, &ui_img_vector2_png);
    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Image2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Spinner1 = lv_spinner_create(ui_Panel2, 1000, 90);
    lv_obj_set_width(ui_Spinner1, 266);
    lv_obj_set_height(ui_Spinner1, 266);
    lv_obj_set_align(ui_Spinner1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Spinner1, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_Spinner1, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_obj_set_style_arc_width(ui_Spinner1,32, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(ui_Spinner1, lv_color_hex(0xE63029), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Spinner1, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_modebutton = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_modebutton, 386);
    lv_obj_set_height(ui_modebutton, 132);
    lv_obj_set_x(ui_modebutton, 183);
    lv_obj_set_y(ui_modebutton, -151);
    lv_obj_set_align(ui_modebutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel3 = lv_obj_create(ui_modebutton);
    lv_obj_set_width(ui_Panel3, 116);
    lv_obj_set_height(ui_Panel3, 116);
    lv_obj_set_x(ui_Panel3, -124);
    lv_obj_set_y(ui_Panel3, 0);
    lv_obj_set_align(ui_Panel3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel3, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel3, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel3, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel3, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image3 = lv_img_create(ui_Panel3);
    lv_img_set_src(ui_Image3, &ui_img_group_8_png);
    lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image3, -3);
    lv_obj_set_y(ui_Image3, 0);
    lv_obj_set_align(ui_Image3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label3 = lv_label_create(ui_modebutton);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, 71);
    lv_obj_set_y(ui_Label3, -1);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Manual");
    lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rotationbutton = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_rotationbutton, 386);
    lv_obj_set_height(ui_rotationbutton, 132);
    lv_obj_set_x(ui_rotationbutton, 183);
    lv_obj_set_y(ui_rotationbutton, 1);
    lv_obj_set_align(ui_rotationbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_rotationbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_rotationbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_rotationbutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rotationbutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rotationbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rotationbutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rotationbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_rotationbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_rotationbutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_rotationbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_rotationbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_rotationbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel5 = lv_obj_create(ui_rotationbutton);
    lv_obj_set_width(ui_Panel5, 116);
    lv_obj_set_height(ui_Panel5, 116);
    lv_obj_set_x(ui_Panel5, -124);
    lv_obj_set_y(ui_Panel5, 0);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel5, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel5, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel5, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image5 = lv_img_create(ui_Panel5);
    lv_img_set_src(ui_Image5, &ui_img_left_line1_png);
    lv_obj_set_width(ui_Image5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image5, -3);
    lv_obj_set_y(ui_Image5, 0);
    lv_obj_set_align(ui_Image5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label5 = lv_label_create(ui_rotationbutton);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, 81);
    lv_obj_set_y(ui_Label5, -2);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "Rotation\nchange");
    lv_obj_set_style_text_font(ui_Label5, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label6 = lv_label_create(ui_rotationbutton);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label6, -88);
    lv_obj_set_y(ui_Label6, 0);
    lv_obj_set_align(ui_Label6, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label6, "Rotation\nchange");
    lv_obj_add_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_text_font(ui_Label6, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel6 = lv_obj_create(ui_rotationbutton);
    lv_obj_set_width(ui_Panel6, 116);
    lv_obj_set_height(ui_Panel6, 116);
    lv_obj_set_x(ui_Panel6, 121);
    lv_obj_set_y(ui_Panel6, 0);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Panel6, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel6, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel6, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel6, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image6 = lv_img_create(ui_Panel6);
    lv_img_set_src(ui_Image6, &ui_img_right_line1_png);
    lv_obj_set_width(ui_Image6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image6, -3);
    lv_obj_set_y(ui_Image6, 0);
    lv_obj_set_align(ui_Image6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image6, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_speedchange = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_speedchange, 386);
    lv_obj_set_height(ui_speedchange, 132);
    lv_obj_set_x(ui_speedchange, 177);
    lv_obj_set_y(ui_speedchange, 159);
    lv_obj_set_align(ui_speedchange, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_speedchange, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_speedchange, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_speedchange, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_speedchange, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speedchange, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_speedchange, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speedchange, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_speedchange, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel7 = lv_obj_create(ui_speedchange);
    lv_obj_set_width(ui_Panel7, 221);
    lv_obj_set_height(ui_Panel7, 116);
    lv_obj_set_x(ui_Panel7, -76);
    lv_obj_set_y(ui_Panel7, -1);
    lv_obj_set_align(ui_Panel7, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel7, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel7, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel7, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image7 = lv_img_create(ui_Panel7);
    lv_img_set_src(ui_Image7, &ui_img_speed_png);
    lv_obj_set_width(ui_Image7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image7, -52);
    lv_obj_set_y(ui_Image7, -2);
    lv_obj_set_align(ui_Image7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image7, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label4 = lv_label_create(ui_Panel7);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, 42);
    lv_obj_set_y(ui_Label4, 2);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "10");
    lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label7 = lv_label_create(ui_speedchange);
    lv_obj_set_width(ui_Label7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label7, 91);
    lv_obj_set_y(ui_Label7, -4);
    lv_obj_set_align(ui_Label7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label7, "RPM");
    lv_obj_set_style_text_font(ui_Label7, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_modebutton4 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_modebutton4, 386);
    lv_obj_set_height(ui_modebutton4, 132);
    lv_obj_set_x(ui_modebutton4, 184);
    lv_obj_set_y(ui_modebutton4, -151);
    lv_obj_set_align(ui_modebutton4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton4, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton4, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton4, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton4, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton4, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label13 = lv_label_create(ui_modebutton4);
    lv_obj_set_width(ui_Label13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label13, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label13, -2);
    lv_obj_set_y(ui_Label13, 2);
    lv_obj_set_align(ui_Label13, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label13, "AUTO");
    lv_obj_set_style_text_font(ui_Label13, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_modebutton5 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_modebutton5, 386);
    lv_obj_set_height(ui_modebutton5, 132);
    lv_obj_set_x(ui_modebutton5, 178);
    lv_obj_set_y(ui_modebutton5, 159);
    lv_obj_set_align(ui_modebutton5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton5, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton5, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton5, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton5, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton5, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton5, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton5, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label14 = lv_label_create(ui_modebutton5);
    lv_obj_set_width(ui_Label14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label14, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label14, 15);
    lv_obj_set_y(ui_Label14, 3);
    lv_obj_set_align(ui_Label14, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label14, "Program");
    lv_obj_set_style_text_font(ui_Label14, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label55 = lv_label_create(ui_modebutton5);
    lv_obj_set_width(ui_Label55, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label55, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label55, 105);
    lv_obj_set_y(ui_Label55, 3);
    lv_obj_set_align(ui_Label55, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label55, "2");
    lv_obj_set_style_text_font(ui_Label55, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel11 = lv_obj_create(ui_modebutton5);
    lv_obj_set_width(ui_Panel11, 129);
    lv_obj_set_height(ui_Panel11, 116);
    lv_obj_set_x(ui_Panel11, -118);
    lv_obj_set_y(ui_Panel11, 0);
    lv_obj_set_align(ui_Panel11, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel11, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel11, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel11, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel11, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel11, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label11 = lv_label_create(ui_Panel11);
    lv_obj_set_width(ui_Label11, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label11, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label11, -3);
    lv_obj_set_y(ui_Label11, 0);
    lv_obj_set_align(ui_Label11, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label11, "0/1");
    lv_obj_set_style_text_color(ui_Label11, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label11, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label11, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);





    ui_modebutton6 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_modebutton6, 422);
    lv_obj_set_height(ui_modebutton6, 132);
    lv_obj_set_x(ui_modebutton6, 183);
    lv_obj_set_y(ui_modebutton6, 1);
    lv_obj_set_align(ui_modebutton6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton6, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton6, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton6, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton6, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton6, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton6, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton6, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label96 = lv_label_create(ui_modebutton6);
    lv_obj_set_width(ui_Label96, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label96, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label96, 5);
    lv_obj_set_y(ui_Label96, -15);
    lv_obj_set_align(ui_Label96, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label96, "min : sec");
    lv_obj_set_style_text_font(ui_Label96, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    ui_Label97 = lv_label_create(ui_modebutton6);
    lv_obj_set_width(ui_Label97, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label97, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label97, 10);
    lv_obj_set_y(ui_Label97, 20);
    lv_obj_set_align(ui_Label97, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_Label97, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_Label97, "02:27");
    lv_obj_set_style_text_font(ui_Label97, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label98 = lv_label_create(ui_modebutton6);
    lv_obj_set_width(ui_Label98, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label98, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label98, 100);
    lv_obj_set_y(ui_Label98, -15);
    lv_obj_set_align(ui_Label98, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label98, "RPM");
    lv_obj_set_style_text_font(ui_Label98, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    ui_Label99 = lv_label_create(ui_modebutton6);
    lv_obj_set_width(ui_Label99, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label99, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label99, 105);
    lv_obj_set_y(ui_Label99, 20);
    lv_obj_set_align(ui_Label99, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_Label99, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_Label99, "45");
    lv_obj_set_style_text_font(ui_Label99, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panelmode = lv_obj_create(ui_modebutton6);
    lv_obj_set_width(ui_Panelmode, 129);
    lv_obj_set_height(ui_Panelmode, 116);
    lv_obj_set_x(ui_Panelmode, -140);
    lv_obj_set_y(ui_Panelmode, -1);
    lv_obj_set_align(ui_Panelmode, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panelmode, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panelmode, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelmode, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panelmode, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panelmode, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panelmode, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Imagemode = lv_img_create(ui_Panelmode);
    lv_img_set_src(ui_Imagemode, &ui_img_settinggroup_png);
    lv_obj_set_width(ui_Imagemode, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Imagemode, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Imagemode, 0);
    lv_obj_set_y(ui_Imagemode, 0);
    lv_obj_set_align(ui_Imagemode, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Imagemode, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Imagemode, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_modebutton7 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_modebutton7, 386);
    lv_obj_set_height(ui_modebutton7, 132);
    lv_obj_set_x(ui_modebutton7, 183);
    lv_obj_set_y(ui_modebutton7, -151);
    lv_obj_set_align(ui_modebutton7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_modebutton7, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_modebutton7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_modebutton7, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_modebutton7, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_modebutton7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_modebutton7, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_modebutton7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_modebutton7, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_modebutton7, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_modebutton7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_modebutton7, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_modebutton7, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label20 = lv_label_create(ui_modebutton7);
    lv_obj_set_width(ui_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label20, 2);
    lv_obj_set_y(ui_Label20, 1);
    lv_obj_set_align(ui_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label20, "CUSTOM");
    lv_obj_set_style_text_font(ui_Label20, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_customsetupbutton = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_customsetupbutton, 386);
    lv_obj_set_height(ui_customsetupbutton, 132);
    lv_obj_set_x(ui_customsetupbutton, 174);
    lv_obj_set_y(ui_customsetupbutton, 160);
    lv_obj_set_align(ui_customsetupbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_customsetupbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS); 
    lv_obj_add_flag(ui_customsetupbutton, LV_OBJ_FLAG_HIDDEN);    /// Flags
    lv_obj_clear_flag(ui_customsetupbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_customsetupbutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_customsetupbutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_customsetupbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_customsetupbutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_customsetupbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_customsetupbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_customsetupbutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_customsetupbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_customsetupbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_customsetupbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label23 = lv_label_create(ui_customsetupbutton);
    lv_obj_set_width(ui_Label23, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label23, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label23, 54);
    lv_obj_set_y(ui_Label23, 3);
    lv_obj_set_align(ui_Label23, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label23, "Program 2");
    lv_obj_set_style_text_font(ui_Label23, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel15 = lv_obj_create(ui_customsetupbutton);
    lv_obj_set_width(ui_Panel15, 129);
    lv_obj_set_height(ui_Panel15, 116);
    lv_obj_set_x(ui_Panel15, -118);
    lv_obj_set_y(ui_Panel15, 0);
    lv_obj_set_align(ui_Panel15, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel15, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel15, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel15, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel15, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel15, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label21 = lv_label_create(ui_Panel15);
    lv_obj_set_width(ui_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label21, -3);
    lv_obj_set_y(ui_Label21, 0);
    lv_obj_set_align(ui_Label21, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label21, "0/1");
    lv_obj_set_style_text_color(ui_Label21, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label21, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label21, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_settingsbutton = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_settingsbutton, 132);
    lv_obj_set_height(ui_settingsbutton, 132);
    lv_obj_set_x(ui_settingsbutton, -296);
    lv_obj_set_y(ui_settingsbutton, -162);
    lv_obj_set_align(ui_settingsbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_settingsbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_settingsbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_settingsbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_settingsbutton, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_settingsbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_settingsbutton, lv_color_hex(0xE63029), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_settingsbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_settingsbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_nasrojochki = lv_img_create(ui_settingsbutton);
    lv_img_set_src(ui_nasrojochki, &ui_img_1502005807);
    lv_obj_set_width(ui_nasrojochki, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_nasrojochki, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_nasrojochki, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_nasrojochki, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_nasrojochki, LV_OBJ_FLAG_SCROLLABLE);      /// Flags


    lv_obj_add_event_cb(ui_modebutton, ui_event_modebutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_rotationbutton, ui_event_rotationbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_speedchange, ui_event_speedchange, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_modebutton4, ui_event_modebutton4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_modebutton5, ui_event_modebutton5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_modebutton6, ui_event_modebutton6, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_modebutton7, ui_event_modebutton7, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_customsetupbutton, ui_event_customsetupbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_settingsbutton, ui_event_settingsbutton, LV_EVENT_ALL, NULL);

}
