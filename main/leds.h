#ifndef LEDS_H
#define LEDS_H

#include "sdkconfig.h"
#include "esp32_neopixel.h"
#include "hoja_includes.h"

extern rgb_s led_colors[CONFIG_NP_RGB_COUNT];
extern rgb_s mode_color;
extern rgb_s charge_color;

typedef enum
{
    // Instantly change to a color
    LEDANIM_SNAPTO,
    // Fade to a color
    LEDANIM_FADETO,
    // Blink from previous color to new color and back
    LEDANIM_BLINK,
    // Battery breathing fader
    LEDANIM_BATTERY_BREATHE,
} led_anim_t;

typedef enum
{
    LA_STATUS_IDLE,
    LA_STATUS_READY,
} led_anim_status_t;

typedef struct
{
    led_anim_t  anim_type;
    uint32_t    rgb_color;
} led_msg_s;

extern led_anim_status_t led_anim_status;

void boot_anim(void);

void led_animator_init(void);

void led_animator_send(led_anim_t anim_type, rgb_s rgb_color);

#endif