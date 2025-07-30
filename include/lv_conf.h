#ifndef LV_CONF_H
#define LV_CONF_H

/* Max resolution */
#define LV_HOR_RES_MAX          320
#define LV_VER_RES_MAX          340

/* Color depth */
#define LV_COLOR_DEPTH          16
#define LV_COLOR_16_SWAP        0

#define LV_MEM_SIZE             (48U * 1024U)  /* Increased memory for LVGL to prevent heap issues */
#define LV_MEM_ATTR
#define LV_MEM_ADR              0
#define LV_MEM_CUSTOM           0

/* Input device */
#define LV_INDEV_DEF_READ_PERIOD       30
#define LV_INDEV_DEF_DRAG_LIMIT        10
#define LV_INDEV_DEF_DRAG_THROW        20
#define LV_INDEV_DEF_LONG_PRESS_TIME  400
#define LV_INDEV_DEF_LONG_PRESS_REP_TIME 100

/* GPU */
#define LV_USE_GPU_FILLER  0

/* Themes, widgets, etc. (enable what you need) */
#define LV_USE_THEME_DEFAULT   1
#define LV_USE_ANIMATION       1
#define LV_USE_GROUP           1
#define LV_USE_INPUT_DEVICE    1

/* Added */
#define LV_USE_SNPRINTF          1
#define LV_SPRINTF_INCLUDE_FLOAT 1

/* Default font to use */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* Enable ALL built-in Montserrat font sizes */
/* LVGL expects LV_FONT_MONTSERRAT_XX format (not LV_USE_FONT_MONTSERRAT_XX) */
#define LV_FONT_MONTSERRAT_8  1
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_26 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_30 1
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_MONTSERRAT_34 1
#define LV_FONT_MONTSERRAT_36 1
#define LV_FONT_MONTSERRAT_38 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_MONTSERRAT_42 1
#define LV_FONT_MONTSERRAT_44 1
#define LV_FONT_MONTSERRAT_46 1
#define LV_FONT_MONTSERRAT_48 1

#endif /*LV_CONF_H*/