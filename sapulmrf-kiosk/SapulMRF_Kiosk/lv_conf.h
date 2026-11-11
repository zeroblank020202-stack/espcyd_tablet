/*
  Minimal lv_conf.h for LVGL 8.3 – SapulMRF Kiosk
  Copy this file to your Arduino "libraries" folder, NEXT TO the lvgl folder
  (e.g. Documents/Arduino/libraries/lv_conf.h). Anything not set here uses LVGL's defaults.
*/
#if 1
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0          /* RGB panels want 0 */

#define LV_MEM_CUSTOM      1          /* use malloc -> PSRAM is available */
#define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
#define LV_MEM_CUSTOM_ALLOC   malloc
#define LV_MEM_CUSTOM_FREE    free
#define LV_MEM_CUSTOM_REALLOC realloc

#define LV_TICK_CUSTOM     1          /* LVGL reads millis() itself */
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

#define LV_DISP_DEF_REFR_PERIOD 20
#define LV_INDEV_DEF_READ_PERIOD 20

/* Fonts used by the kiosk (Montserrat matches the canvas design) */
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_USE_BTNMATRIX 1
#define LV_USE_TABLE     1
#define LV_USE_IMG       1
#define LV_USE_LABEL     1
#define LV_USE_BTN       1

#endif
#endif
