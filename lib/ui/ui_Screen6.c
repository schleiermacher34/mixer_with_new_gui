// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen6_screen_init(void)
{
    ui_Screen6 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen6, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Screen6, lv_color_hex(0x202429), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen6, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_wifibutton = lv_btn_create(ui_Screen6);
    lv_obj_set_width(ui_wifibutton, 354);
    lv_obj_set_height(ui_wifibutton, 132);
    lv_obj_set_x(ui_wifibutton, -192);
    lv_obj_set_y(ui_wifibutton, -162);
    lv_obj_set_align(ui_wifibutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_wifibutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_wifibutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_wifibutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_wifibutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_wifibutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_wifibutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_wifibutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_wifibutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_wifibutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_wifibutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_wifibutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_wifibutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel26 = lv_obj_create(ui_wifibutton);
    lv_obj_set_width(ui_Panel26, 116);
    lv_obj_set_height(ui_Panel26, 116);
    lv_obj_set_x(ui_Panel26, -110);
    lv_obj_set_y(ui_Panel26, 0);
    lv_obj_set_align(ui_Panel26, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel26, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel26, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel26, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel26, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel26, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel26, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel26, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel26, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image24 = lv_img_create(ui_Panel26);
    lv_img_set_src(ui_Image24, &ui_img_wifi_png);
    lv_obj_set_width(ui_Image24, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image24, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image24, -3);
    lv_obj_set_y(ui_Image24, 2);
    lv_obj_set_align(ui_Image24, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image24, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image24, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label40 = lv_label_create(ui_wifibutton);
    lv_obj_set_width(ui_Label40, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label40, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label40, 42);
    lv_obj_set_y(ui_Label40, -1);
    lv_obj_set_align(ui_Label40, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label40, "WI-FI");
    lv_obj_set_style_text_font(ui_Label40, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_updatebutton = lv_btn_create(ui_Screen6);
    lv_obj_set_width(ui_updatebutton, 354);
    lv_obj_set_height(ui_updatebutton, 132);
    lv_obj_set_x(ui_updatebutton, -193);
    lv_obj_set_y(ui_updatebutton, -6);
    lv_obj_set_align(ui_updatebutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_updatebutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_updatebutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_updatebutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_updatebutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_updatebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_updatebutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_updatebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_updatebutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_updatebutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_updatebutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_updatebutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_updatebutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label43 = lv_label_create(ui_updatebutton);
    lv_obj_set_width(ui_Label43, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label43, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label43, -9);
    lv_obj_set_y(ui_Label43, 1);
    lv_obj_set_align(ui_Label43, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label43, "   Check for \n     update");
    lv_obj_set_style_text_font(ui_Label43, &lv_font_montserrat_46, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button12 = lv_btn_create(ui_Screen6);
    lv_obj_set_width(ui_Button12, 354);
    lv_obj_set_height(ui_Button12, 132);
    lv_obj_set_x(ui_Button12, -190);
    lv_obj_set_y(ui_Button12, 146);
    lv_obj_set_align(ui_Button12, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button12, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button12, lv_color_hex(0xDF312A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button12, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel27 = lv_obj_create(ui_Button12);
    lv_obj_set_width(ui_Panel27, 143);
    lv_obj_set_height(ui_Panel27, 116);
    lv_obj_set_x(ui_Panel27, -95);
    lv_obj_set_y(ui_Panel27, -1);
    lv_obj_set_align(ui_Panel27, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel27, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel27, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel27, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel27, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel27, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel27, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel27, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel27, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image25 = lv_img_create(ui_Panel27);
    lv_img_set_src(ui_Image25, &ui_img_back_png);
    lv_obj_set_width(ui_Image25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image25, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image25, -9);
    lv_obj_set_y(ui_Image25, 1);
    lv_obj_set_align(ui_Image25, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image25, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image25, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label44 = lv_label_create(ui_Button12);
    lv_obj_set_width(ui_Label44, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label44, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label44, 54);
    lv_obj_set_y(ui_Label44, -2);
    lv_obj_set_align(ui_Label44, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label44, "BACK");
    lv_obj_set_style_text_font(ui_Label44, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_logbutton = lv_btn_create(ui_Screen6);
    lv_obj_set_width(ui_logbutton, 354);
    lv_obj_set_height(ui_logbutton, 132);
    lv_obj_set_x(ui_logbutton, 202);
    lv_obj_set_y(ui_logbutton, -7);
    lv_obj_set_align(ui_logbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_logbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_logbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_logbutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_logbutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_logbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_logbutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_logbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_logbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_logbutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_logbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_logbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_logbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel28 = lv_obj_create(ui_logbutton);
    lv_obj_set_width(ui_Panel28, 116);
    lv_obj_set_height(ui_Panel28, 116);
    lv_obj_set_x(ui_Panel28, -110);
    lv_obj_set_y(ui_Panel28, 0);
    lv_obj_set_align(ui_Panel28, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel28, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel28, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel28, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel28, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel28, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel28, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel28, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel28, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label42 = lv_label_create(ui_Panel28);
    lv_obj_set_width(ui_Label42, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label42, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label42, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label42, "0");
    lv_obj_set_style_text_color(ui_Label42, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label42, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label42, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label45 = lv_label_create(ui_logbutton);
    lv_obj_set_width(ui_Label45, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label45, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label45, 42);
    lv_obj_set_y(ui_Label45, -1);
    lv_obj_set_align(ui_Label45, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label45, "     Log\nmessages");
    lv_obj_set_style_text_font(ui_Label45, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_errorbutton = lv_btn_create(ui_Screen6);
    lv_obj_set_width(ui_errorbutton, 354);
    lv_obj_set_height(ui_errorbutton, 132);
    lv_obj_set_x(ui_errorbutton, 201);
    lv_obj_set_y(ui_errorbutton, 144);
    lv_obj_set_align(ui_errorbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_errorbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_errorbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_errorbutton, 2500, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_errorbutton, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_errorbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_errorbutton, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_errorbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_errorbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_errorbutton, lv_color_hex(0x666D82), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_errorbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_errorbutton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_errorbutton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel29 = lv_obj_create(ui_errorbutton);
    lv_obj_set_width(ui_Panel29, 116);
    lv_obj_set_height(ui_Panel29, 116);
    lv_obj_set_x(ui_Panel29, -110);
    lv_obj_set_y(ui_Panel29, 0);
    lv_obj_set_align(ui_Panel29, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel29, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel29, lv_color_hex(0x8690AB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel29, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel29, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel29, lv_color_hex(0x363A45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label48 = lv_label_create(ui_Panel29);
    lv_obj_set_width(ui_Label48, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label48, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label48, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label48, "0");
    lv_obj_set_style_text_color(ui_Label48, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label48, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label48, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label46 = lv_label_create(ui_errorbutton);
    lv_obj_set_width(ui_Label46, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label46, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label46, 42);
    lv_obj_set_y(ui_Label46, -1);
    lv_obj_set_align(ui_Label46, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label46, "    Error\nmessages");
    lv_obj_set_style_text_font(ui_Label46, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label47 = lv_label_create(ui_Screen6);
    lv_obj_set_width(ui_Label47, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label47, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label47, 210);
    lv_obj_set_y(ui_Label47, -171);
    lv_obj_set_align(ui_Label47, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label47, "100000-13(S/N)");
    lv_obj_set_style_text_color(ui_Label47, lv_color_hex(0xE4322E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label47, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label47, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_wifibutton, ui_event_wifibutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_updatebutton, ui_event_updatebutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button12, ui_event_Button12, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_logbutton, ui_event_logbutton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_errorbutton, ui_event_errorbutton, LV_EVENT_ALL, NULL);

}
