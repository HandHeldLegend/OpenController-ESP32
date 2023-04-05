#include "leds.h"

// Variable to hold current color data
rgb_s led_colors[CONFIG_NP_RGB_COUNT] = {0};
led_anim_status_t led_anim_status = LA_STATUS_IDLE;

rgb_s mode_color    = {0};
rgb_s charge_color  = COLOR_GREEN;

TaskHandle_t led_taskHandle;
QueueHandle_t led_xQueue;

led_msg_s led_msg = {0};

// LED boot animation
void boot_anim()
{
    int back_forth = 0;
    bool toggle = false;
    bool colorflip = false;
    uint8_t color_idx = 0;
    uint8_t color_last_idx = 0;
    rgb_s colors[6] = {COLOR_RED, COLOR_ORANGE, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE};
    for(int i = 0; i < 12; i++)
    {
        memset(led_colors, 0x00, sizeof(led_colors));
        led_colors[back_forth] = colors[color_idx];
        
        if (!toggle)
        {
            if (back_forth > 0)
            {
                led_colors[back_forth-1] = colors[color_last_idx];
                color_last_idx = color_idx;
            }
            back_forth += 1;
            if (back_forth == CONFIG_NP_RGB_COUNT)
            {
                toggle = true;
                back_forth = CONFIG_NP_RGB_COUNT-1;
            }
        }
        else
        {
            if (back_forth < CONFIG_NP_RGB_COUNT-1)
            {
                led_colors[back_forth+1] = colors[color_last_idx];
                color_last_idx = color_idx;
            }
            back_forth -= 1;
            if (back_forth == -1)
            {
                toggle = false;
                back_forth = 0;
            }
        }

        if (!colorflip)
        {
            if (color_idx + 1 > 5)
            {
                colorflip = true;
                color_idx = 5;
            }
            else
            {
                color_idx += 1;
            }
        }
        else
        {
            if (color_idx - 1 < 0)
            {
                colorflip = false;
                color_idx = 0;
            }
            else
            {
                color_idx -= 1;
            }
        }

        rgb_show();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    rgb_setall(COLOR_BLACK);
    rgb_show();
}

void led_animator_task(void * params)
{
    led_msg_s received_led_msg = {0};

    rgb_s last_color = {0};
    rgb_s current_color = {0};
    rgb_s next_color = {0};

    rgb_s blinkfrom_color = {0};
    rgb_s blinkto_color = {0};
    uint32_t blink_speed = 10;

    // Used to determine if blink should keep repeating.
    bool blinking_en = false;

    led_xQueue = xQueueCreate(4, sizeof(led_msg_s));
    if (led_xQueue != 0)
    {
        led_anim_status = LA_STATUS_READY;
    }

    for(;;)
    {
        // Check if we are supposed to be blinking color
        if (blinking_en)
        {
            // We already blinked TO the desired color, so we must blink back.
            uint8_t fader = 0;
            uint8_t t = 0;
            next_color.rgb = blinkto_color.rgb;
            while(fader < 30)
            {
                
                if ((8 * fader) > 255)
                {
                    t = 255;
                }
                else
                {
                    t = (8 * fader);
                }
                rgb_blend(&current_color, last_color, next_color, t);
                rgb_setall(current_color);
                rgb_show();
                fader += 1;
                vTaskDelay(blink_speed/portTICK_PERIOD_MS);
            }
            last_color.rgb = next_color.rgb;
            rgb_setall(next_color);
            rgb_show();

            // We already blinked TO the desired color, so we must blink back.
            fader = 0;
            next_color.rgb = blinkfrom_color.rgb;
            while(fader < 30)
            {
                t = 0;
                if ((8 * fader) > 255)
                {
                    t = 255;
                }
                else
                {
                    t = (8 * fader);
                }
                rgb_blend(&current_color, last_color, next_color, t);
                rgb_setall(current_color);
                rgb_show();
                fader += 1;
                vTaskDelay(blink_speed/portTICK_PERIOD_MS);
            }
            last_color.rgb = next_color.rgb;
            rgb_setall(next_color);
            rgb_show();
        }
        // If a message is received
        if (xQueueReceive(led_xQueue, &(received_led_msg), (TickType_t) 0))
        {
            // Disable blinking until we interpret message.

            blinking_en = false;
            rgb_s msg_color = {
                .rgb = received_led_msg.rgb_color,
            };

            switch(received_led_msg.anim_type)
            {
                case LEDANIM_SNAPTO:
                    current_color.rgb   = msg_color.rgb;
                    last_color.rgb      = msg_color.rgb;
                    rgb_setall(msg_color);
                    rgb_show();
                    break;

                case LEDANIM_BLINK:
                    // Here we need to blink TO the desired color
                    // not before we store our original color
                    blinkfrom_color.rgb = current_color.rgb;
                    blinkto_color.rgb = msg_color.rgb;
                    blink_speed = 10;
                    blinking_en = true;
                    break;

                case LEDANIM_FADETO:

                    uint8_t fader = 0;
                    next_color.rgb = msg_color.rgb;
                    blink_speed = 10;
                    while(fader < 30)
                    {
                        uint8_t t = 0;
                        if ((8 * fader) > 255)
                        {
                            t = 255;
                        }
                        else
                        {
                            t = (8 * fader);
                        }
                        rgb_blend(&current_color, last_color, next_color, t);
                        rgb_setall(current_color);
                        rgb_show();
                        fader += 1;
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                    last_color.rgb = next_color.rgb;
                    rgb_setall(next_color);
                    rgb_show();

                    break;

                case LEDANIM_BATTERY_BREATHE:
                    // Here we need to blink TO the desired color
                    // not before we store our original color
                    blink_speed = 30;
                    blinkfrom_color.rgb = COLOR_BLACK.rgb;
                    blinkto_color.rgb = msg_color.rgb;

                    blinking_en = true;
                    break;
            }
        }
        else if (!blinking_en)
        {
            // Offload for 500ms
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    }
}

void led_animator_init(void)
{
    neopixel_init(led_colors, VSPI_HOST);
    rgb_setbrightness(25);

    xTaskCreatePinnedToCore(led_animator_task, "LED Animator", 2048, NULL, 3, &led_taskHandle, HOJA_CORE_CPU);
}

void led_animator_send(led_anim_t anim_type, rgb_s rgb_color)
{
    if (led_anim_status == LA_STATUS_IDLE)
    {
        return;
    }

    led_msg.anim_type = anim_type;
    led_msg.rgb_color = rgb_color.rgb;

    if (led_xQueue != 0)
    {
        xQueueSend(led_xQueue, &led_msg, (TickType_t) 0);
    }
}
