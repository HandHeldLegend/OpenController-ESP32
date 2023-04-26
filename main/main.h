#ifndef MAIN_H
#define MAIN_H

#include "sdkconfig.h"
#include "leds.h"
#include "hoja_includes.h"

const button_remap_s joybus_map = {
    .dpad_up    = MAPCODE_DUP,
    .dpad_down  = MAPCODE_DDOWN,
    .dpad_left  = MAPCODE_DLEFT,
    .dpad_right = MAPCODE_DRIGHT,
    
    .button_up  = MAPCODE_B_UP,
    .button_down = MAPCODE_B_DOWN,
    .button_left = MAPCODE_B_LEFT,
    .button_right = MAPCODE_B_RIGHT,

    .trigger_l = MAPCODE_T_ZL,
    .trigger_r = MAPCODE_T_ZR,
    .trigger_zl = MAPCODE_T_L,
    .trigger_zr = MAPCODE_T_R,
    
    .button_start = MAPCODE_B_START,
    .button_select = MAPCODE_B_SELECT,
    .button_stick_left = MAPCODE_B_STICKL,
    .button_stick_right = MAPCODE_B_STICKR,
};

#endif
