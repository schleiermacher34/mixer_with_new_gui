// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen10_screen_init(void)
{

ui_Screen10 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen10, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen10, lv_color_hex(0x26272C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen10, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_minuteslabel2 = lv_label_create(ui_Screen10);
    lv_obj_set_width(ui_minuteslabel2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_minuteslabel2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_minuteslabel2, -91);
    lv_obj_set_y(ui_minuteslabel2, -158);
    lv_obj_set_align(ui_minuteslabel2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minuteslabel2, "Minutes");
    lv_obj_set_style_text_color(ui_minuteslabel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_minuteslabel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_minuteslabel2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_secondslabel2 = lv_label_create(ui_Screen10);
    lv_obj_set_width(ui_secondslabel2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_secondslabel2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_secondslabel2, 89);
    lv_obj_set_y(ui_secondslabel2, -158);
    lv_obj_set_align(ui_secondslabel2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_secondslabel2, "Seconds");
    lv_obj_set_style_text_color(ui_secondslabel2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_secondslabel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_secondslabel2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_rollerminutes2 = lv_roller_create(ui_Screen10);
    lv_roller_set_options(ui_rollerminutes2,
                          "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59",
                          LV_ROLLER_MODE_NORMAL);
    lv_obj_set_width(ui_rollerminutes2, 132);
    lv_obj_set_height(ui_rollerminutes2, 212);
    lv_obj_set_x(ui_rollerminutes2, -92);
    lv_obj_set_y(ui_rollerminutes2, -23);
    lv_obj_set_align(ui_rollerminutes2, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_rollerminutes2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_rollerminutes2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_rollerminutes2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_rollerminutes2, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rollerminutes2, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rollerminutes2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rollerminutes2, lv_color_hex(0xE63029), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rollerminutes2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_rollerminutes2, lv_color_hex(0xFFFFFF), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_rollerminutes2, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_rollerminutes2, &lv_font_montserrat_32, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rollerminutes2, lv_color_hex(0xE63029), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rollerminutes2, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_rollerseconds2 = lv_roller_create(ui_Screen10);
    lv_roller_set_options(ui_rollerseconds2,
                          "10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59",
                          LV_ROLLER_MODE_NORMAL);
    lv_obj_set_width(ui_rollerseconds2, 132);
    lv_obj_set_height(ui_rollerseconds2, 208);
    lv_obj_set_x(ui_rollerseconds2, 89);
    lv_obj_set_y(ui_rollerseconds2, -21);
    lv_obj_set_align(ui_rollerseconds2, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_rollerseconds2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_rollerseconds2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_rollerseconds2, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_rollerseconds2, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rollerseconds2, lv_color_hex(0x202429), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rollerseconds2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_rollerseconds2, lv_color_hex(0xE63029), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_rollerseconds2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_rollerseconds2, lv_color_hex(0xE63029), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_rollerseconds2, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_setuptimebutton2 = lv_btn_create(ui_Screen10);
    lv_obj_set_width(ui_setuptimebutton2, 354);
    lv_obj_set_height(ui_setuptimebutton2, 132);
    lv_obj_set_x(ui_setuptimebutton2, -8);
    lv_obj_set_y(ui_setuptimebutton2, 159);
    lv_obj_set_align(ui_setuptimebutton2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_setuptimebutton2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_setuptimebutton2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_setuptimebutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_setuptimebutton2, lv_color_hex(0x313841), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_setuptimebutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_setuptimebutton2, lv_color_hex(0xDF312A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_setuptimebutton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_setuptimebutton2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BACKTOSETUP4 = lv_label_create(ui_setuptimebutton2);
    lv_obj_set_width(ui_BACKTOSETUP4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_BACKTOSETUP4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_BACKTOSETUP4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_BACKTOSETUP4, "ENTER");
    lv_obj_set_style_text_font(ui_BACKTOSETUP4, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_setuptimebutton2, ui_event_setuptimebutton2, LV_EVENT_ALL, NULL);

}