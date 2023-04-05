#include "main.h"

// Main file for Open Controller Classic Edition example code. 

// Clear a value from high GPIO register (GPIO 32 and higher)
// GPIO.out1_w1tc.val = (uint32_t) (GPIO_NUM-32);
// Set a value from high GPIO register (GPIO 32 and higher)
// GPIO.out1_w1ts.val = (uint32_t) (GPIO_NUM-32);

// Clear a value from low GPIO register (GPIO 0-31)
// GPIO.out_w1tc = (uint32_t) (1ULL<< GPIO_NUM);
// Set a value from low GPIO register (GPIO 0-31)
// GPIO.out_w1ts = (uint32_t) (1ULL<< GPIO_NUM);

// Scanning pins for keypad config
#define GPIO_BTN_SCANA      GPIO_NUM_5
#define GPIO_BTN_SCANB      GPIO_NUM_18
#define GPIO_BTN_SCANC      GPIO_NUM_19
#define GPIO_BTN_SCAND      GPIO_NUM_32

// Port pins for keypad config
#define GPIO_BTN_PULLA      GPIO_NUM_33  
#define GPIO_BTN_PULLB      GPIO_NUM_25
#define GPIO_BTN_PULLC      GPIO_NUM_26
#define GPIO_BTN_PULLD      GPIO_NUM_27

// Button pins (mostly uneeded but looks nicer in code)
#define GPIO_BTN_A          GPIO_BTN_SCANA
#define GPIO_BTN_B          GPIO_BTN_SCANA
#define GPIO_BTN_X          GPIO_BTN_SCANA
#define GPIO_BTN_Y          GPIO_BTN_SCANA
#define GPIO_BTN_DU         GPIO_BTN_SCAND 
#define GPIO_BTN_DL         GPIO_BTN_SCAND
#define GPIO_BTN_DD         GPIO_BTN_SCAND
#define GPIO_BTN_DR         GPIO_BTN_SCAND
#define GPIO_BTN_L          GPIO_BTN_SCANC    
#define GPIO_BTN_ZL         GPIO_BTN_SCANC
#define GPIO_BTN_R          GPIO_BTN_SCANC
#define GPIO_BTN_ZR         GPIO_BTN_SCANC
#define GPIO_BTN_START      GPIO_BTN_SCANC
#define GPIO_BTN_SELECT     GPIO_NUM_2
#define GPIO_BTN_HOME       GPIO_BTN_SCANC
#define GPIO_BTN_CAPTURE    GPIO_BTN_SCANC

// Buttons that are outside of the keypad scanning config
#define GPIO_BTN_STICKL     GPIO_NUM_22
#define GPIO_BTN_STICKR     GPIO_NUM_21
#define GPIO_BTN_SYNC       GPIO_NUM_16

// Input pin mask creation for keypad scanning setup
#define GPIO_INPUT_PIN_MASK     ( (1ULL<<GPIO_BTN_SCANA) | (1ULL<<GPIO_BTN_SCANB) | (1ULL<<GPIO_BTN_SCANC) | (1ULL<<GPIO_BTN_SCAND) | (1ULL<<GPIO_BTN_SELECT) )
#define GPIO_INPUT_PORT_MASK    ( (1ULL<< GPIO_BTN_PULLA) | (1ULL<<GPIO_BTN_PULLB) | (1ULL<<GPIO_BTN_PULLC) | (1ULL<<GPIO_BTN_PULLD) )

// Masks to clear all relevant bits when doing keypad scan
#define GPIO_INPUT_CLEAR0_MASK  ( (1ULL<< GPIO_BTN_PULLA) | (1ULL<<GPIO_BTN_PULLB) | (1ULL<<GPIO_BTN_PULLD) )
#define GPIO_INPUT_CLEAR1_MASK  ( (1ULL<<(GPIO_BTN_PULLC-32)) )

// ADC channel for battery voltage reading
#define ADC_BATTERY_LVL     ADC1_CHANNEL_0

// Variables used to store register reads
uint32_t regread_low = 0;
uint32_t regread_high = 0;

// Reboot system properly.
void enter_reboot()
{
    led_animator_send(LEDANIM_FADETO, COLOR_BLACK);
    vTaskDelay(250/portTICK_PERIOD_MS);
    util_battery_set_charge_rate(35);
    esp_restart();
}

// Sleep mode should check the charge level every 30 seconds or so. 
void enter_sleep()
{
    led_animator_send(LEDANIM_FADETO, COLOR_BLACK);
    vTaskDelay(250/portTICK_PERIOD_MS);
    util_battery_enable_ship_mode();
}

TaskHandle_t local_battery_TaskHandle = NULL;
// Bool to store whether we should do LED charging update
bool charge_display = false;

#define VOLTAGE_MAX_READ 2315
#define VOLTAGE_MIN_READ VOLTAGE_MAX_READ-255
void local_get_battery_task(void * params)
{
    const char* TAG = "local_get_battery_task";
    ESP_LOGI(TAG, "Starting local battery get task...");
    for(;;)
    {
        int lvl = adc1_get_raw(ADC_BATTERY_LVL);
        lvl = lvl - VOLTAGE_MIN_READ;
        
        if (lvl > 255)
        {
            lvl = 255;
        }
        else if (lvl < 0)
        {
            lvl = 0;
        }
        uint8_t out_lvl = 255-lvl;

        ESP_LOGI(TAG, "%d", (unsigned int) out_lvl);

        hoja_set_battery_lvl(out_lvl);

        vTaskDelay(1500/portTICK_PERIOD_MS);
    }
}

// Set up function to update inputs
// Used to determine delay period between each scan (microseconds)
#define US_READ  15
void local_button_cb()
{
    // First set port D as low output
    GPIO.out_w1tc = (uint32_t) (1ULL<<GPIO_BTN_PULLD);
    ets_delay_us(US_READ);

    // Read the GPIO registers and mask the data
    regread_low = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_MASK;
    regread_high = REG_READ(GPIO_IN1_REG);

    // Y button
    hoja_button_data.button_left    = !util_getbit(regread_low, GPIO_BTN_Y);
    // Dpad Down
    hoja_button_data.dpad_down      = !util_getbit(regread_high, GPIO_BTN_DD);
    // L trigger
    hoja_button_data.trigger_l      = !util_getbit(regread_low, GPIO_BTN_L);

    // Release port D Set port C
    GPIO.out_w1ts = (uint32_t) (1ULL<<GPIO_BTN_PULLD);
    GPIO.out_w1tc = (uint32_t) (1ULL<<GPIO_BTN_PULLC);
    //GPIO.out1_w1tc.val = (uint32_t) 0x1; //GPIO_BTN_PULLC
    ets_delay_us(US_READ);

    // Read the GPIO registers and mask the data
    regread_low = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_MASK;
    regread_high = REG_READ(GPIO_IN1_REG);

    // X button
    hoja_button_data.button_up      = !util_getbit(regread_low, GPIO_BTN_X);
    // Dpad Down
    hoja_button_data.dpad_left      = !util_getbit(regread_high, GPIO_BTN_DL);

    // Release port C set port B
    GPIO.out_w1ts = (uint32_t) (1ULL<<GPIO_BTN_PULLC);
    GPIO.out_w1tc = (uint32_t) (1ULL<<GPIO_BTN_PULLB);
    ets_delay_us(US_READ);

    // Read the GPIO registers and mask the data
    regread_low = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_MASK;
    regread_high = REG_READ(GPIO_IN1_REG);

    // B button
    hoja_button_data.button_down    = !util_getbit(regread_low, GPIO_BTN_B);
    // Dpad Up
    hoja_button_data.dpad_up        = !util_getbit(regread_high, GPIO_BTN_DU);
    // Start button
    hoja_button_data.button_start   = !util_getbit(regread_low, GPIO_BTN_START);

    // Release port B set port A
    GPIO.out_w1ts = (uint32_t) (1ULL<<GPIO_BTN_PULLB);
    GPIO.out1_w1tc.val = (uint32_t) (1ULL << 1);
    ets_delay_us(US_READ);

    // Read the GPIO registers and mask the data
    regread_low = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_MASK;
    regread_high = REG_READ(GPIO_IN1_REG);
    
    // Release port A
    GPIO.out1_w1ts.val = (uint32_t) (1ULL << 1);

    // A button
    hoja_button_data.button_right   = !util_getbit(regread_low, GPIO_BTN_A);
    // Dpad Right
    hoja_button_data.dpad_right     = !util_getbit(regread_high, GPIO_BTN_DR);
    // R trigger
    hoja_button_data.trigger_r      = !util_getbit(regread_low, GPIO_BTN_R);

    // Read select button (not tied to matrix)
    hoja_button_data.button_select  = !util_getbit(regread_low, GPIO_BTN_SELECT);

    // Reset macros
    hoja_button_data.button_capture = 0;
    hoja_button_data.button_home    = 0;
    hoja_button_data.button_pair    = 0;
    hoja_button_data.button_sleep   = 0;
    hoja_button_data.trigger_zl     = 0;
    hoja_button_data.trigger_zr     = 0;


    // Tie the select button to our sleep button.
    if (hoja_button_data.button_select)
    {
        hoja_button_data.button_sleep = 1;
    }

    // Tie the start button to our sleep button
    if (hoja_button_data.button_start)
    {
        hoja_button_data.button_pair = 1;
    }

    if (hoja_button_data.button_start && hoja_button_data.trigger_l)
    {
        hoja_button_data.trigger_l = 0;
        hoja_button_data.button_start = 0;
        hoja_button_data.button_capture = 1;
    }

    if (hoja_button_data.button_start && hoja_button_data.trigger_r)
    {
        hoja_button_data.trigger_r = 0;
        hoja_button_data.button_start = 0;
        hoja_button_data.button_home = 1;
    }

    if (hoja_button_data.button_select && hoja_button_data.trigger_l)
    {
        hoja_button_data.trigger_l = 0;
        hoja_button_data.button_select = 0;
        hoja_button_data.trigger_zl = 1;
    }

    if (hoja_button_data.button_select && hoja_button_data.trigger_r)
    {
        hoja_button_data.trigger_r = 0;
        hoja_button_data.button_select = 0;
        hoja_button_data.trigger_zr = 1;
    }
}

// Separate task to read sticks.
// This is essential to have as a separate component as ADC scans typically take more time and this is only
// scanned once between each polling interval. This varies from core to core.
void local_analog_cb()
{
    const char* TAG = "stick_task";
    // read stick 1 and 2

    /*
    hoja_analog_data.ls_x = (uint16_t) adc1_get_raw(ADC_STICK_LX);
    hoja_analog_data.rs_x = (uint16_t) adc1_get_raw(ADC_STICK_RX);
    hoja_analog_data.rs_y = (uint16_t) adc1_get_raw(ADC_STICK_RY);
    */

    hoja_analog_data.ls_x = 2048;
    hoja_analog_data.ls_y = 2048;

    hoja_analog_data.rs_x = 2048;
    hoja_analog_data.rs_y = 2048;

    hoja_analog_data.lt_a = 0;
    hoja_analog_data.rt_a = 0;
}

// Handle System events
void local_system_evt(hoja_system_event_t evt, uint8_t param)
{
    const char* TAG = "local_system_evt";
    switch(evt)
    {
        esp_err_t err = ESP_OK;

        // Called after API initialize function
        case HEVT_API_INIT_OK:
        {
            ESP_LOGI(TAG, "HOJA initialized OK callback.");

            local_button_cb();
            local_button_cb();

            if (hoja_button_data.button_pair)
            {
                hoja_set_force_wired(true);
            }
            else
            {
                hoja_set_force_wired(false);
            }

            // Check to see what buttons are being held. Adjust state accordingly.
            if (hoja_button_data.button_left)
            {
                if (loaded_settings.controller_mode != HOJA_CONTROLLER_MODE_RETRO)
                {
                    loaded_settings.controller_mode = HOJA_CONTROLLER_MODE_RETRO;
                    hoja_settings_saveall();
                }
            }
            else if (hoja_button_data.button_right)
            {
                if (loaded_settings.controller_mode != HOJA_CONTROLLER_MODE_NS)
                {
                    loaded_settings.controller_mode = HOJA_CONTROLLER_MODE_NS;
                    hoja_settings_saveall();
                }
            }
            else if (hoja_button_data.button_up)
            {
                if (loaded_settings.controller_mode != HOJA_CONTROLLER_MODE_XINPUT)
                {
                    loaded_settings.controller_mode = HOJA_CONTROLLER_MODE_XINPUT;
                    hoja_settings_saveall();
                }
            }
            else if (hoja_button_data.button_down)
            {
                if (loaded_settings.controller_mode != HOJA_CONTROLLER_MODE_DINPUT)
                {
                    loaded_settings.controller_mode = HOJA_CONTROLLER_MODE_DINPUT;
                    hoja_settings_saveall();
                }
            }
            
            // Get boot mode and it will perform a callback.
            err = util_battery_boot_status();
            if (err != HOJA_OK)
            {
                ESP_LOGE(TAG, "Issue when getting boot battery status.");
            }

        }
            break;
        
        // Called when shutdown triggers from input loop
        case HEVT_API_SHUTDOWN:
        {
            ESP_LOGI(TAG, "HEV_API_SHUTDOWN");
            enter_sleep();
        }
            break;
        
        // Called when reboot is requested
        case HEVT_API_REBOOT:
        {
            enter_reboot();
        }
            break;

        case HEVT_API_PLAYERNUM:
            // TO DO
        {
            if (param == 1)
            {
                led_animator_send(LEDANIM_FADETO, COLOR_RED);
            }
            else if (param == 2)
            {
                led_animator_send(LEDANIM_FADETO, COLOR_BLUE);
            }
            else if (param == 3)
            {
                led_animator_send(LEDANIM_FADETO, COLOR_GREEN);
            }
            else if (param == 4)
            {
                led_animator_send(LEDANIM_FADETO, COLOR_PURPLE);
            }
            else
            {
                led_animator_send(LEDANIM_FADETO, COLOR_ORANGE);
            }

        }
            break;

        case HEVT_API_RUMBLE:
            // TO DO
        {

        }
            break;

        default:
            ESP_LOGI(TAG, "Unknown event type: %d", evt);
            break;

    }
}

void local_bt_evt(hoja_bt_event_t evt)
{
    const char* TAG = "local_bt_evt";

    switch (evt)
    {
        default:
            ESP_LOGI(TAG, "Unknown bt event");
            break;

        case HEVT_BT_STARTED:
            ESP_LOGI(TAG, "BT Started OK.");
            break;

        case HEVT_BT_CONNECTING:
            ESP_LOGI(TAG, "Connecting BT...");
            break;

        case HEVT_BT_PAIRING:
            ESP_LOGI(TAG, "Pairing BT Device.");
            break;

        case HEVT_BT_CONNECTED:
            ESP_LOGI(TAG, "BT Device Connected.");
            led_animator_send(LEDANIM_FADETO, mode_color);
            break;

        case HEVT_BT_DISCONNECTED:
            ESP_LOGI(TAG, "BT Device Disconnected.");
            
            break;
    }
}

void local_usb_evt(hoja_usb_event_t evt)
{
    switch (evt)
    {
        default:
        case HEVT_USB_DISCONNECTED:
            if (util_battery_external_power())
            {
                charge_display = true;
                led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
            }
            else
            {
                enter_sleep();
            }
            break;

        case HEVT_USB_CONNECTED:
            charge_display = false;
            led_animator_send(LEDANIM_FADETO, mode_color);
            break;
    }
}

// Events from the wired utility
void local_wired_evt(hoja_wired_event_t evt)
{
    const char* TAG = "local_wired_evt";
    hoja_err_t err = HOJA_OK;

    switch(evt)
    {
        default:
        case HEVT_WIRED_NO_DETECT:
            err = HOJA_FAIL;
            break;
        
        case HEVT_WIRED_SNES_DETECT:
            hoja_set_core(HOJA_CORE_SNES);
            led_animator_send(LEDANIM_FADETO, COLOR_WHITE);
            err = hoja_start_core();

            break;

        case HEVT_WIRED_JOYBUS_DETECT:
            hoja_set_core(HOJA_CORE_GC);
            led_animator_send(LEDANIM_FADETO, COLOR_PURPLE);
            err = hoja_start_core();

            break;
    }

    if (err != HOJA_OK)
    {
        ESP_LOGE(TAG, "Failed to start retro core.");
        enter_sleep();
    }
    else
    {
        ESP_LOGI(TAG, "Started retro core OK.");
        rgb_show();
    }
}

void local_battery_evt(hoja_battery_event_t evt, uint8_t param)
{
    const char* TAG = "local_battery_evt";
    hoja_err_t err = HOJA_OK;

    switch(evt)
    {
        case HEVT_BATTERY_CHARGING:
            ESP_LOGI(TAG, "Battery is charging.");
            charge_color.rgb = COLOR_GREEN.rgb;
            if(charge_display)
            {
                led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
            }
            break;

        case HEVT_BATTERY_CHARGECOMPLETE:
            ESP_LOGI(TAG, "Battery charging completed.");
            charge_color.rgb = COLOR_BLUE.rgb;
            if(charge_display)
            {
                led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
            }
            break;

        case HEVT_BATTERY_NOCHARGE:
            ESP_LOGI(TAG, "Battery not charging.");
            charge_color.rgb = COLOR_RED.rgb;
            if(charge_display)
            {
                led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
            }
            break;

        default:
        case HEVT_BATTERY_LVLCHANGE:
            // Not implemented
            ESP_LOGE(TAG, "Not implemented.");
            break;
    }
}

void local_charger_evt(hoja_charger_event_t evt)
{
    const char* TAG = "local_charger_evt";
    switch(evt)
    {
        case HEVT_CHARGER_PLUGGED:
            ESP_LOGI(TAG, "Charger plugged in.");
            hoja_set_external_power(true);
            if(charge_display)
            {
                led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
            }
            break;
        case HEVT_CHARGER_UNPLUGGED:
            ESP_LOGI(TAG, "Charger unplugged.");
            enter_sleep();
            break;
    }
}

void local_boot_evt(hoja_boot_event_t evt)
{
    esp_err_t err;
    const char* TAG = "local_boot_evt";

    switch(evt)
    {
        // With no battery connected
        case HEVT_BOOT_NOBATTERY:
            ESP_LOGI(TAG, "No battery detected.");
        case HEVT_BOOT_PLUGGED:
        {
            ESP_LOGI(TAG, "Plugged in on boot.");
            charge_display = true;

            switch(loaded_settings.controller_mode)
            {
                case HOJA_CONTROLLER_MODE_RETRO:
                {
                    charge_display = false;
                    util_battery_set_charge_rate(35);

                    err = util_wired_detect_loop();
                    if (!err)
                    {
                        ESP_LOGI(TAG, "Started wired retro loop OK.");
                        led_animator_send(LEDANIM_BLINK, COLOR_ORANGE);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to start wired retro loop.");
                    }   
                }
                    break;

                default:
                case HOJA_CONTROLLER_MODE_DINPUT:
                {
                    util_battery_set_charge_rate(100);
                    hoja_set_core(HOJA_CORE_USB);
                    core_usb_set_subcore(USB_SUBCORE_DINPUT);

                    mode_color.rgb = COLOR_BLUE.rgb;
                    led_animator_send(LEDANIM_FADETO, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {

                    }
                    else
                    {

                    }
                }
                    break;

                case HOJA_CONTROLLER_MODE_XINPUT:
                {
                    util_battery_set_charge_rate(100);

                    hoja_set_core(HOJA_CORE_USB);
                    core_usb_set_subcore(USB_SUBCORE_XINPUT);

                    mode_color.rgb = COLOR_GREEN.rgb;
                    led_animator_send(LEDANIM_FADETO, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {

                    }
                }
                    break;
                
                case HOJA_CONTROLLER_MODE_NS:
                {
                    util_battery_set_charge_rate(100);

                    hoja_set_core(HOJA_CORE_USB);
                    core_usb_set_subcore(USB_SUBCORE_NS);

                    mode_color.rgb = COLOR_YELLOW.rgb;
                    led_animator_send(LEDANIM_FADETO, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {

                    }
                    else
                    {

                    }

                }
                    break;
            }

            // Start battery monitor utility
            util_battery_start_monitor();

            vTaskDelay(200/portTICK_PERIOD_MS);
            if(charge_display)
            {
                util_battery_status_t stat = util_get_battery_charging_status();
                if ((stat == BATSTATUS_UNDEFINED) || (stat == BATSTATUS_NOTCHARGING))
                {
                    charge_color.rgb = COLOR_RED.rgb;
                    led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
                }
                else if ((stat == BATSTATUS_TRICKLEFAST) || (stat == BATSTATUS_CONSTANT))
                {
                    charge_color.rgb = COLOR_GREEN.rgb;
                    led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
                }
                else if(stat == BATSTATUS_COMPLETED)
                {
                    charge_color.rgb = COLOR_BLUE.rgb;
                    led_animator_send(LEDANIM_BATTERY_BREATHE, charge_color);
                }
            }
            
        }
            break;

        // This case is reached if
        // the controller is unplugged but has a battery
        case HEVT_BOOT_UNPLUGGED:
        {
            ESP_LOGI(TAG, "Unplugged.");
            // Do not show charger display changes.
            charge_display = false;

            if (hoja_get_force_wired())
            {
                // Boot as if we are wired if USB standby is enabled.
                hoja_event_cb(HOJA_EVT_BOOT, HEVT_BOOT_PLUGGED, 0x00);
                return;
            }

            switch(loaded_settings.controller_mode)
            {
                case HOJA_CONTROLLER_MODE_RETRO:
                {
                    util_battery_set_charge_rate(35);

                    err = util_wired_detect_loop();
                    if (!err)
                    {
                        ESP_LOGI(TAG, "Started wired retro loop OK.");
                        led_animator_send(LEDANIM_BLINK, COLOR_ORANGE);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to start wired retro loop.");
                    }   
                    
                }
                    break;

                default:
                case HOJA_CONTROLLER_MODE_DINPUT:
                {
                    util_battery_set_charge_rate(100);
                    err = hoja_set_core(HOJA_CORE_BT_DINPUT);

                    mode_color.rgb = COLOR_BLUE.rgb;
                    led_animator_send(LEDANIM_BLINK, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {
                        ESP_LOGI(TAG, "Started BT Dinput OK.");      
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to start Dinput BT.");
                    }
                }
                    break;

                case HOJA_CONTROLLER_MODE_NS:
                {
                    util_battery_set_charge_rate(100);
                    core_ns_set_subcore(NS_TYPE_SNES);
                    err = hoja_set_core(HOJA_CORE_NS);

                    mode_color.rgb = COLOR_YELLOW.rgb;
                    led_animator_send(LEDANIM_BLINK, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {
                        ESP_LOGI(TAG, "Started BT Switch OK.");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to start Switch BT.");
                    }
                }
                    break;

                case HOJA_CONTROLLER_MODE_XINPUT:
                {
                    util_battery_set_charge_rate(100);
                    err = hoja_set_core(HOJA_CORE_BT_XINPUT);

                    mode_color.rgb = COLOR_GREEN.rgb;
                    led_animator_send(LEDANIM_BLINK, mode_color);

                    err = hoja_start_core();

                    if (err == HOJA_OK)
                    {
                        ESP_LOGI(TAG, "Started BT XInput OK.");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to start XInput BT.");
                    }
                }
                    break;
            }
            
        }
            break;
    }
}

// Callback to handle HOJA events
void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param)
{   
    const char* TAG = "local_event_cb";

    switch(type)
    {
        default:
            ESP_LOGI(TAG, "Unrecognized event occurred: %X", (unsigned int) type);
            break;

        case HOJA_EVT_BOOT:
            local_boot_evt(evt);
            break;

        case HOJA_EVT_SYSTEM:
            local_system_evt(evt, param);
            break;

        case HOJA_EVT_CHARGER:
            local_charger_evt(evt);
            break;

        case HOJA_EVT_BT:
            local_bt_evt(evt);
            break;

        case HOJA_EVT_BATTERY:
            local_battery_evt(evt, param);
            break;

        case HOJA_EVT_USB:
            local_usb_evt(evt);
            break;

        case HOJA_EVT_WIRED:
            local_wired_evt(evt);
            break;
    }
}

void app_main()
{
    const char* TAG = "app_main";

    hoja_err_t err;

    // IO configuration we can reuse
    gpio_config_t io_conf = {};

    // Set up IO pins for scanning button matrix
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PORT_MASK;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    GPIO.out_w1ts = GPIO_INPUT_CLEAR0_MASK;
    GPIO.out1_w1ts.val = (uint32_t) 0x3;

    // Set up ADC
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_BATTERY_LVL, ADC_ATTEN_DB_11));

    util_i2c_initialize();
    util_battery_set_type(BATTYPE_BQ25180);
    util_battery_set_charge_rate(35);

    led_animator_init();

    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    xTaskCreate(local_get_battery_task, "BatTask", 2048, NULL, 3, &local_battery_TaskHandle);

    err = hoja_init();
}
