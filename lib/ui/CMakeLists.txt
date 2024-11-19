SET(SOURCES ui_Screen1.c
    ui_Screen2.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_vector_png.c
    ui_img_vector2_png.c
    ui_img_group_8_png.c
    ui_img_left_line1_png.c
    ui_img_right_line1_png.c
    ui_img_speed_png.c
    ui_img_right_line2_png.c
    ui_img_left_line2_png.c
    ui_img_enter_line_png.c)

add_library(ui ${SOURCES})
