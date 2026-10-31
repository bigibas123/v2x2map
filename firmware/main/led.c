#include "sdkconfig.h"

#include <stdint.h>
#include <stdbool.h>

#include "esp_event.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "config.h"
#include "events.h"

#include "led.h"

#define LED_SYSTEM 0

static led_strip_handle_t led_strip; // LED strip handle
#define LED_IRGB(i, r, g, b) SET_IRGB(i, r, g, b)

bool sniffer_running;
bool mqtt_connected;

/* LED 0 (the only WS2812 on the Waveshare devboard) doubles as a status +
 * activity indicator. Idle background reflects sniffer + BLE state; pulses
 * (sniffer-rx, ble-tx, usb-tx) briefly override it for visual feedback. */
static bool s_ble_connected;
static struct {
    uint8_t  r, g, b;
    bool     active;
} s_pulse;
static esp_timer_handle_t s_pulse_timer;

static uint8_t ble_led_state = 0xFF;
static uint8_t sniffer_led_state = 0x00;
static uint8_t cits_led_state    = 0x00;

static esp_timer_handle_t system_led_timer_handle;
static esp_timer_handle_t cits_led_timer_handle;

static uint8_t led_brightness;

static void set_led_with_brightness(led_indicator_handle_t handle, uint32_t irgb)
{
    uint8_t i = GET_INDEX(irgb);
    uint8_t r = GET_RED(irgb);
    uint8_t g = GET_GREEN(irgb);
    uint8_t b = GET_BLUE(irgb);

    r = ((uint32_t)r) * ((uint32_t)led_brightness) / 255u;
    g = ((uint32_t)g) * ((uint32_t)led_brightness) / 255u;
    b = ((uint32_t)b) * ((uint32_t)led_brightness) / 255u;

    led_strip_set_pixel(handle, i, r, g, b);
    led_strip_refresh(handle);
}

static void set_led_color_based_on_state() {
    set_led_with_brightness(led_strip, LED_IRGB(LED_SYSTEM, sniffer_led_state, cits_led_state, ble_led_state));
}


static void set_cits_led_idle(void)
{
    cits_led_state = 0x00;
    set_led_color_based_on_state();
}

static void set_cits_led_active(void)
{
    cits_led_state = 0x78;
    set_led_color_based_on_state();
}

static void set_sniffer_led_stopped(void)
{
    sniffer_led_state = 0x00;
    set_led_color_based_on_state();
}

static void set_sniffer_led_running(void)
{
    sniffer_led_state = 0x78;
    set_led_color_based_on_state();
}

static void sniffer_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case SNIFFER_RECEIVED_PACKET:
        set_cits_led_active();
        if (esp_timer_restart(cits_led_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
        {
            esp_timer_start_once(cits_led_timer_handle, 50000);
        }
        led_pulse_sniffer_rx();
        break;
    case SNIFFER_STARTED:
        set_sniffer_led_running();
        set_cits_led_idle();
        led_status_sniffer_running(true);
        break;
    case SNIFFER_STOPPED:
        set_sniffer_led_stopped();
        set_cits_led_idle();
        led_status_sniffer_running(false);
        break;
    }
}


static void apply_activity_led(void)
{
    if (s_pulse.active) {
        set_led_with_brightness(led_strip,
            LED_IRGB(LED_SYSTEM, s_pulse.r, s_pulse.g, s_pulse.b));
        return;
    }

    ble_led_state = 0xFF;
    sniffer_led_state = 0x00;

    if(sniffer_running){
        sniffer_led_state = 0x78;
    }
    if(s_ble_connected){
        ble_led_state = 0x00;
    }
    cits_led_state    = 0x00;

    set_led_color_based_on_state();
}

static void pulse_end_cb(void *arg)
{
    (void)arg;
    s_pulse.active = false;
    apply_activity_led();
}

static void led_pulse(uint8_t r, uint8_t g, uint8_t b, int ms)
{
    s_pulse.r = r; s_pulse.g = g; s_pulse.b = b;
    s_pulse.active = true;
    apply_activity_led();
    if (esp_timer_restart(s_pulse_timer, ms * 1000) == ESP_ERR_INVALID_STATE) {
        esp_timer_start_once(s_pulse_timer, ms * 1000);
    }
}

void led_pulse_sniffer_rx(void) { led_pulse(0x00, 0xFF, 0x00, 80); }
void led_pulse_ble_tx(void)     { led_pulse(0x00, 0x00, 0xFF, 60); }
void led_pulse_usb_tx(void)     { led_pulse(0xFF, 0xFF, 0x00, 40); }

void led_status_ble_connected(bool c)
{
    s_ble_connected = c;
    /* Show the transition with a brief signal pulse: cyan = link up,
     * red = link gone, then settle into the new idle background. */
     if(c){
        led_pulse(0x00, 0xFF, 0xFF, 250);
     }else{
        led_pulse(0xFF, 0x00, 0x00, 250);
     }
}

void led_status_sniffer_running(bool r)
{
    sniffer_running = r;
    apply_activity_led();
}

static void system_led_timer_cb(void *)
{
    /* No longer used for blinking — apply_activity_led drives LED 0
     * directly. We keep this around to refresh the idle colour periodically
     * in case some external write touched the strip. */
    if (!s_pulse.active) apply_activity_led();
}

static void cits_led_timer_cb(void *)
{
    set_cits_led_idle();
}

void led_update(void)
{
    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    apply_activity_led();

    /* Refresh idle background once per second as a fail-safe in case some
     * other writer touched the strip. apply_activity_led skips if a pulse
     * is currently active. */
    esp_timer_start_periodic(system_led_timer_handle, 1000000);
}

void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_LEDSTRIP_PIN,                         // Set GPIO pin
        .max_leds = 1,                                                   // Set number of LEDs
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // Set color format
    };

    // RMT backend-specific configuration
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // RMT resolution, 10MHz
        .flags.with_dma = false,           // Disable DMA
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));


    esp_timer_create_args_t create_args = {
        .callback = cits_led_timer_cb,
        .arg = NULL,
        .name = "cits_led"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &cits_led_timer_handle));

    create_args.callback = system_led_timer_cb;
    create_args.name = "system_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &system_led_timer_handle));

    create_args.callback = pulse_end_cb;
    create_args.name = "led_pulse";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &s_pulse_timer));

    ESP_ERROR_CHECK(esp_event_handler_register(SNIFFER_EVENT_BASE, ESP_EVENT_ANY_ID, sniffer_event_handler, NULL));

    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    led_strip_clear(led_strip); // Clear the strip to off initially

    /* Boot self-test on LED 0 so a missing onboard WS2812 / wrong GPIO is
     * obvious at first glance: bright R → G → B → W, ~150 ms each. */
    set_led_with_brightness(led_strip, LED_IRGB(LED_SYSTEM, 0xFF, 0x00, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_strip, LED_IRGB(LED_SYSTEM, 0x00, 0xFF, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_strip, LED_IRGB(LED_SYSTEM, 0x00, 0x00, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_strip, LED_IRGB(LED_SYSTEM, 0xFF, 0xFF, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
}
