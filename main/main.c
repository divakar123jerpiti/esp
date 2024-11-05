#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "LCD_Driver/ST7701S.h"
#include "Touch/GT911.h"
#include "demos/lv_demos.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include <math.h>
#include "esp_timer.h"
#include "esp_wifi.h"  // Include WiFi header
#include "nvs_flash.h" // Include NVS header
#include "esp_event.h" // Include event header
#include "esp_netif.h"
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> // for close()
#include "driver/uart.h"
#include "driver/gpio.h"
#include <freertos/event_groups.h>
static bool wifi_scanning = false;
static TaskHandle_t wifi_scan_task_handle = NULL;
static char wifi_ssid[32] = {0};
static char wifi_password[64] = {0};
lv_obj_t *ssid_label = NULL;
lv_obj_t *ssid_input = NULL;
lv_obj_t *password_label = NULL;
lv_obj_t *password_input = NULL;
lv_obj_t *connect_btn = NULL;
lv_obj_t *keyboard = NULL;
lv_obj_t *wifi_symbol = NULL; // Global for Wi-Fi symbol
lv_obj_t *connect_label = NULL;

static lv_style_t style_text_muted;
void lvgl_task(void *arg);
#define CONNECTED_BIT 0
static EventGroupHandle_t wifi_event_group;
void display_ssids(wifi_ap_record_t *ap_records, uint16_t ap_count);
void wifi_init(const char *ssid, const char *password);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void display_wifi_symbol();
void hide_wifi_symbol();
void clear_wifi_input_ui();
static void rx_task(void);
static void tx_task(void);

void screen_init(void);

void create_wifi_input_ui(void);
lv_obj_t *ssid_text_area = NULL;

static bool is_slider_adjusting = false; // Flag to track if slider is being adjusted
static esp_timer_handle_t debounce_timer;
static bool timer_running = false;
static uint32_t last_change_time = 0;
// static const uint32_t DEBOUNCE_DELAY_MS = 70; // 50ms debounce delay

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO 6 // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (0)                   // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

static const char *TAG = "example";
/********************* I2C *********************/
#define I2C_Touch_SCL_IO 7          /*!< GPIO number used for I2C master clock */
#define I2C_Touch_SDA_IO 15         /*!< GPIO number used for I2C master data  */
#define I2C_Touch_INT_IO 16         /*!< GPIO number used for I2C master data  */
#define I2C_Touch_RST_IO -1         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
/********************* LCD *********************/
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (32 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT -1
#define EXAMPLE_PIN_NUM_HSYNC 38
#define EXAMPLE_PIN_NUM_VSYNC 39
#define EXAMPLE_PIN_NUM_DE 40
#define EXAMPLE_PIN_NUM_PCLK 41
#define EXAMPLE_PIN_NUM_DATA0 5   // B0
#define EXAMPLE_PIN_NUM_DATA1 45  // B1
#define EXAMPLE_PIN_NUM_DATA2 48  // B2
#define EXAMPLE_PIN_NUM_DATA3 47  // B3
#define EXAMPLE_PIN_NUM_DATA4 21  // B4
#define EXAMPLE_PIN_NUM_DATA5 14  // G0
#define EXAMPLE_PIN_NUM_DATA6 13  // G1
#define EXAMPLE_PIN_NUM_DATA7 12  // G2
#define EXAMPLE_PIN_NUM_DATA8 11  // G3
#define EXAMPLE_PIN_NUM_DATA9 10  // G4
#define EXAMPLE_PIN_NUM_DATA10 9  // G5
#define EXAMPLE_PIN_NUM_DATA11 46 // R0
#define EXAMPLE_PIN_NUM_DATA12 3  // R1
#define EXAMPLE_PIN_NUM_DATA13 8  // R2
#define EXAMPLE_PIN_NUM_DATA14 18 // R3
#define EXAMPLE_PIN_NUM_DATA15 17 // R4
#define EXAMPLE_PIN_NUM_DISP_EN -1
// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 480
#define EXAMPLE_LCD_V_RES 480

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB 2
#else
#define EXAMPLE_LCD_NUM_FB 1
#endif

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

#define SPI_SDA 1
#define SPI_SCL 2
#define SPI_CS 42
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)
int flag;
uint8_t prev_x = 0;
uint8_t prev_y = 0;
void create_sliding_navigation(void);
uint8_t p_red = 0;
uint8_t p_blue = 0;
uint8_t p_green = 0;
static bool long_press_triggered = false; // Global flag to track long press
static void update_label_pos(lv_obj_t *slider, lv_obj_t *label);
// void lv_demo_widgets(void);
extern const uint8_t test2_map[];
uint8_t byte1; // High byte
uint8_t byte2;
float outer_radius;
float inner_radius;
void init_lighting_control();
void esp_rom_delay_us(uint32_t us);
// Variables to store last sent RGB values
static uint8_t last_red = 0;
static uint8_t last_green = 0;
static uint8_t last_blue = 0;
static uint8_t last_intensity = 0;
static lv_style_t style_percentage_label;
static lv_obj_t *percentage_label;
static lv_style_t style_percentage_label1;
static lv_obj_t *percentage_label1;
static lv_style_t style_percentage_label2;
static lv_obj_t *percentage_label2;
static lv_style_t style_percentage_label3;
static lv_obj_t *percentage_label3;
static lv_style_t style_percentage_label4;
static lv_obj_t *percentage_label4;
static lv_style_t style_percentage_label5;
static lv_obj_t *percentage_label5;
static lv_style_t style_percentage_label6;
static lv_obj_t *percentage_label6;
static lv_obj_t *ssid_container = NULL;
static lv_obj_t *ssid_list = NULL;
typedef struct
{
    lv_obj_t *main_screen;
    lv_obj_t *rgb_screen;
    lv_obj_t *wifi_screen;
    lv_obj_t *warm_cool_screen;
    lv_obj_t *tuning_screen;
    lv_obj_t *rgb_wheel;
    lv_obj_t *rgb_intensity_slider;
    lv_obj_t *warm_cool_slider;
    lv_obj_t *warm_cool_slider1;
    lv_obj_t *tuning_slider;
    lv_obj_t *tuning_slider1;
    lv_obj_t *tuning_slider2;
    lv_obj_t *All_tuning_slider;
    lv_obj_t *tuning_slider3;
    lv_obj_t *sunlight_btn;
    lv_obj_t *rgb_slider;
    lv_obj_t *cool_btn;
    lv_obj_t *warm_btn;
    lv_obj_t *both_btn;
    lv_obj_t *cool_btn1;
    lv_obj_t *warm_btn1;
    lv_obj_t *both_btn1;
    lv_obj_t *channel1_label;
    lv_obj_t *channel1_container;
    lv_obj_t *channel2_label;
    lv_obj_t *channel2_container;
    lv_obj_t *warm_intensity_slider;
    lv_obj_t *warm_intensity_slider1;
    lv_obj_t *channel3_label;
    lv_obj_t *channel3_container;
    lv_obj_t *rgb_button;
    lv_obj_t *channel4_label;
    lv_obj_t *channel4_container;
    lv_obj_t *All_channel_label;
    lv_obj_t *All_channel_container;
    lv_obj_t *warm_cool_container;
    lv_obj_t *warm_cool_container1;
    lv_obj_t *warm_intensity_container;

    lv_obj_t *warm_intensity_container1;
    lv_obj_t *warm_intensity_container2;
    lv_obj_t *button_container;
    bool is_interacting_with_control;
    lv_obj_t *warm_intensity_label;
    lv_obj_t *warm_intensity_label1;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t warm;
    uint8_t cool;
    uint8_t tuning;
    bool is_interacting_with_rgb_wheel;
    int channel1_value;
    int channel2_value;
    int channel3_value;
    int channel4_value;
    int all_channels_value;
    lv_obj_t *percentage_label; // New member for percentage display

} lighting_control_t;
static void rgb_slider_event_cb(lv_event_t *e);
static void cool_btn_event_cb(lv_event_t *e);
static void warm_btn_event_cb(lv_event_t *e);
static void both_btn_event_cb(lv_event_t *e);
static lighting_control_t *lighting_control;
void create_wifi_screen(void);
// 1900K (Warm)
typedef struct
{
    lv_obj_t *obj;
    bool pressed;
    lv_color_t current_color;
    float knob_x;
    float knob_y;
    float current_angle; // Add this to track knob position
    lv_color_t *wheel_buffer;
    bool wheel_buffer_created;
    bool initialized;
    uint16_t r_v; // Added RGB value variables
    uint16_t g_v;
    uint16_t b_v;
    bool update_rgb_values;
    bool power_state;    // Add this to track power state
    lv_obj_t *power_btn; // Add this for the power button
    struct
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } rgb_values;
    lv_obj_t *curved_slider_container;
    lv_obj_t *curved_slider;
} custom_rgb_wheel_t;
static bool is_slider_active = false; // Flag to indicate if slider is in use

lv_color_t rgb_color;
lv_draw_rect_dsc_t knob_dsc;
unsigned char check;
int all_tuning_value;
/********************* BackLight *********************/
unsigned char tx_data[26] = {0x2D, 0x19, 0x01, 0x3C, 0xC1, 0xF6, 0x06, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x01, 0x10, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
    {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/*Read the touchpad*/
void example_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        ESP_LOGI(TAG, "X=%u Y=%u", data->point.x, data->point.y);
        if ((data->point.x != prev_x) && (data->point.y != prev_y))
        {
            prev_x = data->point.x;
            prev_y = data->point.y;
            int index = (data->point.y * 480 + data->point.x) * 2;
            // Retrieve the two bytes from the image buffer
            byte1 = test2_map[index];     // High byte
            byte2 = test2_map[index + 1]; // Low byte
            // Combine to form the 16-bit RGB565 value
            uint16_t rgb565 = (byte1 << 8) | byte2;
            rgb565 = (byte2 << 8) | byte1;

            // Extract the red, green, and blue components
            uint8_t red = (rgb565 >> 11) & 0x1F;  // 5 bits for red
            uint8_t green = (rgb565 >> 5) & 0x3F; // 6 bits for green
            uint8_t blue = rgb565 & 0x1F;         // 5 bits for blue

            // Optionally scale to 8-bit (0-255)
            red = (red * 255) / 31;
            green = (green * 255) / 63;
            blue = (blue * 255) / 31;

            // One of the values changed, do something
            if (p_red != red || p_green != green || p_blue != blue)
            {
                //  uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));

                // uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
                p_red = red;
                p_green = green;
                p_blue = blue;
            }
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_Touch_SDA_IO,
        .scl_io_num = I2C_Touch_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void wifi_init_ap(void)
{
    ESP_LOGI(TAG, "Starting WiFi in AP mode...");

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_Config_AP",
            .ssid_len = strlen("ESP32_Config_AP"),
            .password = "12345678", // Customize your AP password here
            .max_connection = 4,    // Max number of connections
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((const char *)wifi_config.ap.password) == 0) // Cast to const char*
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN; // Open network if no password
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "AP Mode started with SSID: %s", wifi_config.ap.ssid);
}

// LV_IMG_DECLARE(test2);
void app_main(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    init();

    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    ST7701S_handle st7701s = ST7701S_newObject(SPI_SDA, SPI_SCL, SPI_CS, SPI3_HOST, SPI_METHOD);

    ST7701S_screen_init(st7701s, 1);
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    /********************* Touch *********************/
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = I2C_Touch_RST_IO,
        .int_gpio_num = I2C_Touch_INT_IO,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    /********************* RGB LCD panel driver *********************/
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_PLL240M,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_back_porch = 10,
            .hsync_front_porch = 50,
            .hsync_pulse_width = 8,
            .vsync_back_porch = 18,
            .vsync_front_porch = 8,
            .vsync_pulse_width = 2,
            .flags.pclk_active_neg = false,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};

    /********************* LVGL *********************/
    ESP_LOGI(TAG, "Register display indev to LVGL");
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_touchpad_read;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // lv_obj_t *icon = lv_img_create(lv_scr_act());
    // lv_img_set_src(icon, &test2);
    ESP_LOGI(TAG, "Starting app_main...");

    // Initialize NVS (for WiFi storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS Flash is full or version mismatch. Erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start WiFi in AP mode for configuration

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay to allow IP assignment

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        char ip_str[16];
        esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
        printf("ESP32 IP Address: %s\n", ip_str);
    }
    else
    {
        printf("Failed to get IP address\n");
    }
    wifi_init_ap();
    create_sliding_navigation();
    while (1)
    {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
static void generic_control_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED)
    {
        lighting_control->is_interacting_with_control = true;
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        lighting_control->is_interacting_with_control = false;
    }
    lv_obj_t *target_obj = lv_event_get_target(e);

    // Call the original event handler
    lv_event_cb_t original_cb = (lv_event_cb_t)lv_obj_get_event_user_data(target_obj, generic_control_event_cb);
    if (original_cb)
        original_cb(e);
}

static void send_rgb_data(void *arg)
{
    uint8_t red = (lighting_control->r * 255) / 31;
    uint8_t green = (lighting_control->g * 255) / 63;
    uint8_t blue = (lighting_control->b * 255) / 31;

    /* if (red != last_red || green != last_green || blue != last_blue)
     {
         last_red = red;
         last_green = green;
         last_blue = blue;

         tx_data[22] = red;
         tx_data[23] = green;
         tx_data[24] = blue;

         uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
     }*/

    timer_running = false;
}
static void save_tuning_values()
{
    lighting_control->channel1_value = lv_slider_get_value(lighting_control->tuning_slider);
    lighting_control->channel2_value = lv_slider_get_value(lighting_control->tuning_slider1);
    lighting_control->channel3_value = lv_slider_get_value(lighting_control->tuning_slider2);
    lighting_control->channel4_value = lv_slider_get_value(lighting_control->tuning_slider3);
}
static void restore_tuning_values()
{
    lv_slider_set_value(lighting_control->tuning_slider, lighting_control->channel1_value, LV_ANIM_OFF);
    lv_slider_set_value(lighting_control->tuning_slider1, lighting_control->channel2_value, LV_ANIM_OFF);
    lv_slider_set_value(lighting_control->tuning_slider2, lighting_control->channel3_value, LV_ANIM_OFF);
    lv_slider_set_value(lighting_control->tuning_slider3, lighting_control->channel4_value, LV_ANIM_OFF);
}

static void screen_slide_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());

    if (lighting_control->is_interacting_with_control)
    {
        return;
    }
    if (obj == lighting_control->main_screen)
    {
        if (dir == LV_DIR_LEFT)
        {
            lv_scr_load_anim(lighting_control->warm_cool_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 10, 0, false);
        }

        /*  else if (dir == LV_DIR_BOTTOM)
          {
              lv_obj_add_flag(lighting_control->rgb_wheel, LV_OBJ_FLAG_HIDDEN);
              lv_obj_clear_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_HIDDEN);
          }
          else if (dir == LV_DIR_TOP)
          {
              lv_obj_add_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_HIDDEN);
              lv_obj_clear_flag(lighting_control->rgb_wheel, LV_OBJ_FLAG_HIDDEN);
          }*/
    }
    else if (obj == lighting_control->warm_cool_screen)
    {
        if (dir == LV_DIR_LEFT)
        {
            restore_tuning_values(); // Restore values when entering the tuning screen
            if (lv_slider_get_value(lighting_control->All_tuning_slider) == 0)
            {
            }
            lv_scr_load_anim(lighting_control->tuning_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);

            // knob_dsc.bg_color = rgb_color;
        }
        else if (dir == LV_DIR_RIGHT)
        {
            knob_dsc.bg_color = rgb_color;
            lv_scr_load_anim(lighting_control->main_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 10, 0, false);
        }
        else if (dir == LV_DIR_BOTTOM)
        {
            lv_obj_add_flag(lighting_control->warm_cool_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_cool_slider, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_slider, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->cool_btn, LV_OBJ_FLAG_HIDDEN);

            lv_obj_add_flag(lighting_control->both_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_btn, LV_OBJ_FLAG_HIDDEN);

            lv_obj_clear_flag(lighting_control->warm_cool_container1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_cool_slider1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_container2, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_slider1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_label1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->cool_btn1, LV_OBJ_FLAG_HIDDEN);

            lv_obj_clear_flag(lighting_control->both_btn1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_btn1, LV_OBJ_FLAG_HIDDEN);
        }
        else if (dir == LV_DIR_TOP)
        {
            lv_obj_clear_flag(lighting_control->warm_cool_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_cool_slider, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_container, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_slider, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_intensity_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->cool_btn, LV_OBJ_FLAG_HIDDEN);

            lv_obj_clear_flag(lighting_control->both_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lighting_control->warm_btn, LV_OBJ_FLAG_HIDDEN);

            lv_obj_add_flag(lighting_control->warm_cool_container1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_cool_slider1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_container2, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_slider1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_intensity_label1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->cool_btn1, LV_OBJ_FLAG_HIDDEN);

            lv_obj_add_flag(lighting_control->both_btn1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lighting_control->warm_btn1, LV_OBJ_FLAG_HIDDEN);
        }
    }
    else if (obj == lighting_control->tuning_screen)
    {
        if (dir == LV_DIR_RIGHT)
        {
            save_tuning_values(); // Save before leaving the screen
            lv_scr_load_anim(lighting_control->warm_cool_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        }
        /* else if (dir == LV_DIR_BOTTOM)
         {
             lv_obj_add_flag(lighting_control->tuning_slider, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->tuning_slider1, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->tuning_slider2, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->tuning_slider3, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->All_tuning_slider, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel1_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel2_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel3_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel4_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->All_channel_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel1_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel2_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel3_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->channel4_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->All_channel_label, LV_OBJ_FLAG_HIDDEN);
         }
         else if (dir == LV_DIR_TOP)
         {
             lv_obj_add_flag(lighting_control->All_tuning_slider, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->All_channel_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_add_flag(lighting_control->All_channel_label, LV_OBJ_FLAG_HIDDEN);

             lv_obj_clear_flag(lighting_control->tuning_slider, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->tuning_slider1, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->tuning_slider2, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->tuning_slider3, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel1_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel2_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel3_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel4_container, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel1_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel2_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel3_label, LV_OBJ_FLAG_HIDDEN);
             lv_obj_clear_flag(lighting_control->channel4_label, LV_OBJ_FLAG_HIDDEN);
         }*/
    }
}

/*static void rgb_wheel_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        uint32_t current_time = lv_tick_get();
        if (current_time - last_change_time > DEBOUNCE_DELAY_MS)
        {
            lv_color_t color = lv_colorwheel_get_rgb(obj);
            lighting_control->r = color.ch.red;
            lighting_control->g = color.ch.green;
            lighting_control->b = color.ch.blue;

            if (!timer_running)
            {
                esp_timer_start_once(debounce_timer, DEBOUNCE_DELAY_MS * 1000); // Convert ms to us
                timer_running = true;
            }

            last_change_time = current_time;
        }
    }
    if (code == LV_EVENT_LONG_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT)
    {
        lv_event_stop_processing(e);
    }
    //  lv_event_stop_bubbling(e);
}*/
/*void init_rgb_wheel_debounce()
{
    esp_timer_create_args_t timer_args = {
        .callback = &send_rgb_data,
        .name = "rgb_debounce"};
    esp_timer_create(&timer_args, &debounce_timer);
}*/
static void rgb_intensity_slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    uint8_t intensity = lv_slider_get_value(slider);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED)
    {
        is_slider_active = true; // Mark slider as active when user adjusts it

        // Optionally, you could reset the timer here if you want to extend the visible time when user interacts
    }
    else if (code == LV_EVENT_RELEASED)
    {
        is_slider_active = false; // Mark slider as inactive when user stops adjusting
    }
    // Only send data if the intensity value has changed
    if (intensity != last_intensity)
    {
        last_intensity = intensity;

        tx_data[25] = intensity;
        uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
        esp_rom_delay_us(5 * 1000); // 1000 microseconds = 1 millisecond
    }

    // Update your lighting intensity here
}
static void intensity_slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label5, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label5, LV_OBJ_FLAG_HIDDEN);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label5, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_align_to(percentage_label5, slider, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    }
}
static void intensity_slider_event_cb1(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label6, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label6, LV_OBJ_FLAG_HIDDEN);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label6, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_align_to(percentage_label6, slider, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    }
}
static void warm_cool_slider_event_cb1(lv_event_t *e)
{

    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED)
        return;

    lv_obj_t *slider = lv_event_get_target(e);
    int value = lv_slider_get_value(slider);
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_color_t current_color;

    // Apply a custom easing function to make the transition more gradual

    float factor = (255 - value) / 255.0f; // Directly interpolate from 0 to 100

    // Calculate the current color by blending directly between cool and warm
    current_color = lv_color_mix(cool_color, warm_color, (uint8_t)(factor * 255));

    // Apply the calculated color to the knob
    lv_obj_set_style_bg_color(slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);

    // lv_obj_set_style_bg_color(lighting_control->warm_cool_screen, bg_color, LV_PART_MAIN);
}
static void warm_cool_slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED)
        return;

    lv_obj_t *slider = lv_event_get_target(e);
    int value = lv_slider_get_value(slider);
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_color_t current_color;

    // Apply a custom easing function to make the transition more gradual

    float factor = (255 - value) / 255.0f; // Directly interpolate from 0 to 100

    // Calculate the current color by blending directly between cool and warm
    current_color = lv_color_mix(cool_color, warm_color, (uint8_t)(factor * 255));

    // Apply the calculated color to the knob
    lv_obj_set_style_bg_color(slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);

    // Calculate the interpolation factor
    /* float factor = value / 100.0f;

     // Apply a custom easing function to make the transition more gradual
     factor = 1.0f - powf(1.0f - factor, 3.0f);

     if (value <= 50)
     {
         // Blend from neutral to cool
         uint8_t r = (1 - factor) * neutral_color.ch.red + factor * cool_color.ch.red;
         uint8_t g = (1 - factor) * neutral_color.ch.green + factor * cool_color.ch.green;
         uint8_t b = (1 - factor) * neutral_color.ch.blue + factor * cool_color.ch.blue;
         current_color = lv_color_make(r, g, b);
     }
     else
     {
         // Blend from neutral to warm
         factor = (value - 50) / 50.0f;
         factor = powf(factor, 2.0f); // Make warm colors appear more gradually
         uint8_t r = (1 - factor) * neutral_color.ch.red + factor * warm_color.ch.red;
         uint8_t g = (1 - factor) * neutral_color.ch.green + factor * warm_color.ch.green;
         uint8_t b = (1 - factor) * neutral_color.ch.blue + factor * warm_color.ch.blue;
         current_color = lv_color_make(r, g, b);
     }
     // Apply the interpolated color to the slider
     lv_obj_set_style_bg_color(slider, current_color, LV_PART_KNOB);
     lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
     lv_obj_set_style_border_color(slider, lv_color_darken(current_color, LV_OPA_20), LV_PART_KNOB);
 */
    // Update the main background as per the slider's current color
    //  lv_obj_set_style_bg_color(lighting_control->warm_cool_screen, current_color, LV_PART_MAIN);
}

static void update_label_pos(lv_obj_t *slider, lv_obj_t *label)
{
    int32_t slider_left = lv_obj_get_x(slider);
    int32_t slider_width = lv_obj_get_width(slider);
    int32_t label_width = lv_obj_get_width(label);
    int32_t label_height = lv_obj_get_height(label);

    int32_t value = lv_slider_get_value(slider);
    int32_t min = lv_slider_get_min_value(slider);
    int32_t max = lv_slider_get_max_value(slider);

    // Calculate knob position
    int32_t knob_pos = slider_left + ((value - min) * slider_width) / (max - min);

    // Center label above knob position
    int32_t label_x = knob_pos - (label_width / 2);
    int32_t label_y = lv_obj_get_y(slider) - label_height - 10; // 10px gap

    // Keep label within slider bounds
    if (label_x < slider_left)
    {
        label_x = slider_left;
    }
    else if (label_x + label_width > slider_left + slider_width)
    {
        label_x = slider_left + slider_width - label_width;
    }

    lv_obj_set_pos(label, label_x, label_y);
}

static void tuning_slider_event_cb2(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label, LV_OBJ_FLAG_HIDDEN);
        update_label_pos(slider, percentage_label);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        update_label_pos(slider, percentage_label);
    }

    // Update your tuning values
    lighting_control->channel1_value = value;
    lighting_control->tuning = value;
}
static void tuning_slider_event_cb3(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label1, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label1, LV_OBJ_FLAG_HIDDEN);
        update_label_pos(slider, percentage_label1);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label1, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        update_label_pos(slider, percentage_label1);
    }
    // Update your tuning here
    lighting_control->channel2_value = value;
    lighting_control->tuning = value;
    //  lv_slider_set_value(lighting_control->tuning_slider1, lighting_control->tuning, LV_ANIM_OFF);
}
static void tuning_slider_event_cb4(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label3, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label3, LV_OBJ_FLAG_HIDDEN);
        update_label_pos(slider, percentage_label3);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label3, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        update_label_pos(slider, percentage_label3);
    }
    // Update your tuning here
    // Update your tuning here
    lighting_control->channel3_value = value;
    lighting_control->tuning = value;
    // lv_slider_set_value(lighting_control->tuning_slider2, lighting_control->tuning, LV_ANIM_OFF);
}
static void tuning_slider_event_cb5(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label4, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label4, LV_OBJ_FLAG_HIDDEN);
        update_label_pos(slider, percentage_label4);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label4, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        update_label_pos(slider, percentage_label4);
    }
    // Update your tuning here
    lighting_control->channel4_value = value;
    lighting_control->tuning = value;
    // lv_slider_set_value(lighting_control->tuning_slider3, lighting_control->tuning, LV_ANIM_OFF);
}
static void tuning_slider_event_cb1(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    lv_obj_t *slider = lv_event_get_target(e);

    // Calculate percentage (0-100)
    int value = lv_slider_get_value(slider);
    int percentage = (value * 100) / 255; // Convert slider value (0-255) to percentage

    // Update percentage label
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", percentage);
    lv_label_set_text(percentage_label2, buf);

    // Show label when dragging starts
    // Show label and update position when dragging starts
    if (code == LV_EVENT_PRESSED)
    {
        lv_obj_clear_flag(percentage_label2, LV_OBJ_FLAG_HIDDEN);
        update_label_pos(slider, percentage_label2);
    }
    // Hide label when dragging ends
    else if (code == LV_EVENT_RELEASED)
    {
        lv_obj_add_flag(percentage_label2, LV_OBJ_FLAG_HIDDEN);
    }
    // Update label position while dragging

    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        all_tuning_value = lv_slider_get_value(lighting_control->All_tuning_slider);
        lighting_control->all_channels_value = all_tuning_value;

        // Update individual channel values

        // Update the value for each individual channel slider
        lv_slider_set_value(lighting_control->tuning_slider, all_tuning_value, LV_ANIM_OFF);
        lv_slider_set_value(lighting_control->tuning_slider1, all_tuning_value, LV_ANIM_OFF);
        lv_slider_set_value(lighting_control->tuning_slider2, all_tuning_value, LV_ANIM_OFF);
        lv_slider_set_value(lighting_control->tuning_slider3, all_tuning_value, LV_ANIM_OFF);
        update_label_pos(slider, percentage_label2);
    }
    // Optionally, update the global tuning variable if needed
    lighting_control->tuning = all_tuning_value;
    // Update your tuning here
}
static void cool_btn_event_cb(lv_event_t *e)
{
    lv_slider_set_value(lighting_control->warm_cool_slider, 0, LV_ANIM_ON);
    // lv_slider_set_value(lighting_control->warm_cool_slider1, 0, LV_ANIM_ON);
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    //  lv_obj_t *slider = lv_event_get_target(e);
    lv_obj_t *slider = lighting_control->warm_cool_slider;

    lv_color_t current_color;
    lighting_control->warm = 0;
    lighting_control->cool = 255;

    lv_obj_set_style_bg_color(slider, cool_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, cool_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);
    // Update your lighting to cool setting
}

static void warm_btn_event_cb(lv_event_t *e)
{
    lv_slider_set_value(lighting_control->warm_cool_slider, 255, LV_ANIM_ON);
    // lv_slider_set_value(lighting_control->warm_cool_slider1, 100, LV_ANIM_ON);

    lighting_control->warm = 255;
    lighting_control->cool = 0;
    // Update your lighting to warm setting
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_obj_t *slider = lighting_control->warm_cool_slider;

    lv_obj_set_style_bg_color(slider, warm_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, warm_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);
}
static void both_btn_event_cb(lv_event_t *e)
{
    lv_slider_set_value(lighting_control->warm_cool_slider, 127.5, LV_ANIM_ON);
    // lv_slider_set_value(lighting_control->warm_cool_slider1, 50, LV_ANIM_ON);

    lighting_control->warm = 127.5;
    lighting_control->cool = 127.5;
    // Update your lighting to balanced warm/cool setting
    // Update your lighting to warm setting
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_color_t current_color;

    // Apply a custom easing function to make the transition more gradual

    float factor = (100 - 50) / 100.0f; // Directly interpolate from 0 to 100

    // Calculate the current color by blending directly between cool and warm
    current_color = lv_color_mix(cool_color, warm_color, (uint8_t)(factor * 255));

    // Apply the calculated color to the knob
    lv_obj_set_style_bg_color(lighting_control->warm_cool_slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(lighting_control->warm_cool_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(lighting_control->warm_cool_slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(lighting_control->warm_cool_slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(lighting_control->warm_cool_slider, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_cool_slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);

    // Calculate the interpolation factor
}

static void cool_btn_event_cb1(lv_event_t *e)
{
    // lv_slider_set_value(lighting_control->warm_cool_slider, 0, LV_ANIM_ON);
    lv_slider_set_value(lighting_control->warm_cool_slider1, 0, LV_ANIM_ON);

    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    //  lv_obj_t *slider = lv_event_get_target(e);
    lv_obj_t *slider = lighting_control->warm_cool_slider1;

    lv_color_t current_color;
    lighting_control->warm = 0;
    lighting_control->cool = 255;

    lv_obj_set_style_bg_color(slider, cool_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, cool_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);
}

static void warm_btn_event_cb1(lv_event_t *e)
{
    // lv_slider_set_value(lighting_control->warm_cool_slider, 100, LV_ANIM_ON);
    lv_slider_set_value(lighting_control->warm_cool_slider1, 255, LV_ANIM_ON);

    lighting_control->warm = 255;
    lighting_control->cool = 0;
    // Update your lighting to warm setting
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_obj_t *slider = lighting_control->warm_cool_slider1;

    lv_obj_set_style_bg_color(slider, warm_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(slider, warm_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_NONE, LV_PART_KNOB);
}

static void both_btn_event_cb1(lv_event_t *e)
{
    // lv_slider_set_value(lighting_control->warm_cool_slider, 50, LV_ANIM_ON);
    lv_slider_set_value(lighting_control->warm_cool_slider1, 127.5, LV_ANIM_ON);

    lighting_control->warm = 127.5;
    lighting_control->cool = 127.5;
    // Update your lighting to balanced warm/cool setting
    lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t warm_color = lv_color_hex(0xFFB46B);    // 1900K (Warm)
    lv_color_t white_color = lv_color_hex(0x000000);   // 1900K (Warm)

    lv_color_t current_color;

    // Apply a custom easing function to make the transition more gradual

    float factor = (100 - 50) / 100.0f; // Directly interpolate from 0 to 100

    // Calculate the current color by blending directly between cool and warm
    current_color = lv_color_mix(cool_color, warm_color, (uint8_t)(factor * 255));

    // Apply the calculated color to the knob
    lv_obj_set_style_bg_color(lighting_control->warm_cool_slider1, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(lighting_control->warm_cool_slider1, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(lighting_control->warm_cool_slider1, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(lighting_control->warm_cool_slider1, 2, LV_PART_KNOB);

    // Remove any gradient from the knob
    lv_obj_set_style_bg_grad_color(lighting_control->warm_cool_slider1, current_color, LV_PART_KNOB);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_cool_slider1, LV_GRAD_DIR_NONE, LV_PART_KNOB);
}
static void init_warm_cool_slider(lv_obj_t *slider)
{
    lv_color_t COOL_COLOR = lv_color_hex(0x80C9FF);    // 8000K (Cool)
    lv_color_t NEUTRAL_COLOR = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
    lv_color_t WARM_COLOR = lv_color_hex(0xFFB46B);
    lv_color_t white_color = lv_color_hex(0x000000); // 1900K (Warm)

    // Set the gradient colors for the slider background
    lv_obj_set_style_bg_color(slider, COOL_COLOR, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(slider, WARM_COLOR, LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(slider, COOL_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(slider, WARM_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(slider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    // Style the knob (slider button)
    lv_obj_set_style_bg_color(slider, COOL_COLOR, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, lv_color_darken(white_color, LV_OPA_20), LV_PART_KNOB);
    lv_obj_set_style_border_width(slider, 2, LV_PART_KNOB);

    // Set initial value to middle (neutral)
    //  lv_slider_set_value(slider, 50, LV_ANIM_OFF);
}

static void draw_rgb_wheel(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
    custom_rgb_wheel_t *wheel = (custom_rgb_wheel_t *)lv_obj_get_user_data(obj);

    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    lv_coord_t width = lv_obj_get_width(obj);
    lv_coord_t height = lv_obj_get_height(obj);
    lv_coord_t radius = (width < height ? width : height) / 2;

    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;

    float outer_radius = radius * 0.95f;
    float inner_radius = outer_radius * 0.65f;
    float center_radius = inner_radius * 0.25f; // Made center smaller for power button

    // Draw the color wheel
    const float two_pi = 2 * M_PI;
    const float angle_step = two_pi / 180;

    for (float angle = 0; angle < two_pi; angle += angle_step)
    {
        float cos_angle = cosf(angle);
        float sin_angle = sinf(angle);
        for (float r = inner_radius; r <= outer_radius; r += 1.0f)
        {
            lv_coord_t x = cx + (lv_coord_t)(r * cos_angle);
            lv_coord_t y = cy + (lv_coord_t)(r * sin_angle);

            uint16_t hue = (angle * 360.0f) / two_pi;
            lv_color_t color = lv_color_hsv_to_rgb(hue, 100, 100);

            lv_area_t px_area;
            px_area.x1 = x - 3;
            px_area.y1 = y - 3;
            px_area.x2 = x + 3;
            px_area.y2 = y + 3;

            lv_draw_rect_dsc_t rect_dsc;
            lv_draw_rect_dsc_init(&rect_dsc);
            rect_dsc.bg_color = color;
            rect_dsc.bg_opa = LV_OPA_COVER;
            lv_draw_rect(draw_ctx, &rect_dsc, &px_area);
        }
    }

    // Draw the selection knob
    float knob_radius = (inner_radius + outer_radius) / 2;
    wheel->knob_x = cx + knob_radius * cosf(wheel->current_angle);
    wheel->knob_y = cy + knob_radius * sinf(wheel->current_angle);

    if (wheel->update_rgb_values)
    {
        uint16_t hue = (wheel->current_angle * 360.0f) / two_pi;
        rgb_color = lv_color_hsv_to_rgb(hue, 100, 100);
        wheel->update_rgb_values = false;
    }
    if (wheel->power_state)
    {
        // Power ON - update button style
        lv_obj_set_style_bg_color(wheel->power_btn, rgb_color, 0);
    }
    lv_coord_t knob_size = (outer_radius - inner_radius) * 0.4f;
    lv_draw_rect_dsc_t knob_dsc;
    lv_draw_rect_dsc_init(&knob_dsc);
    knob_dsc.bg_color = rgb_color;
    knob_dsc.border_color = lv_color_white();
    knob_dsc.border_width = 4;
    knob_dsc.radius = LV_RADIUS_CIRCLE;

    lv_area_t knob_area;
    knob_area.x1 = wheel->knob_x - knob_size;
    knob_area.y1 = wheel->knob_y - knob_size;
    knob_area.x2 = wheel->knob_x + knob_size;
    knob_area.y2 = wheel->knob_y + knob_size;
    lv_draw_rect(draw_ctx, &knob_dsc, &knob_area);

    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = lv_color_white();
    arc_dsc.width = 6;
    lv_point_t center_point = {.x = cx, .y = cy};
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, outer_radius + arc_dsc.width, 0, 360);
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, inner_radius, 0, 360);
}
static bool is_point_in_wheel(lv_obj_t *obj, lv_point_t *point)
{
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;

    float dx = point->x - cx;
    float dy = point->y - cy;
    float distance = sqrtf(dx * dx + dy * dy);

    lv_coord_t width = lv_obj_get_width(obj);
    lv_coord_t height = lv_obj_get_height(obj);
    lv_coord_t radius = (width < height ? width : height) / 2;

    float outer_radius = radius * 0.95f;
    float inner_radius = outer_radius * 0.65f;

    return (distance >= inner_radius && distance <= outer_radius);
}
static void hide_u_shape_slider_timer_cb(lv_timer_t *timer)
{
    if (!is_slider_active)
    {
        lv_obj_add_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_HIDDEN);
        lv_timer_del(timer); // Delete the timer after it runs
    }
}

static void rgb_wheel_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    custom_rgb_wheel_t *wheel = (custom_rgb_wheel_t *)lv_obj_get_user_data(obj);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED)
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t point;
        lv_indev_get_point(indev, &point);

        // Check if the point is within the wheel's ring
        if (is_point_in_wheel(obj, &point))
        {
            wheel->pressed = true;
            // Rest of your existing PRESSED event code...
        }
    }
    else if (code == LV_EVENT_LONG_PRESSED && wheel->pressed)
    {
        lv_obj_clear_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_HIDDEN);
        // Ensure the slider does not block other controls like the power button
        //  lv_obj_add_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_GESTURE_BUBBLE);
        //  lv_obj_add_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_CLICK_FOCUSABLE);

        // Move slider to background to avoid blocking power button

        lv_timer_t *hide_timer = lv_timer_create(hide_u_shape_slider_timer_cb, 2000, NULL); // 10,000 milliseconds = 10 seconds
    }
    else if (code == LV_EVENT_PRESSING && wheel->pressed)
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t point;
        lv_indev_get_point(indev, &point);

        lv_area_t obj_coords;
        lv_obj_get_coords(obj, &obj_coords);
        lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
        lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;

        float dx = point.x - cx;
        float dy = point.y - cy;
        float angle = atan2f(dy, dx);
        if (angle < 0)
            angle += 2 * M_PI;

        wheel->current_angle = angle;
        wheel->current_color = lv_color_hsv_to_rgb((angle * 360.0f) / (2 * M_PI), 100, 100);
        wheel->update_rgb_values = true;

        // Add UART sending code here for continuous updates while pressing
        if (wheel->power_state)
        {
            uint8_t red_5bit = wheel->current_color.ch.red;
            uint8_t green_6bit = wheel->current_color.ch.green;
            uint8_t blue_5bit = wheel->current_color.ch.blue;

            // Scale to 8-bit range (0-255)
            uint8_t red = (red_5bit * 255) / 31;
            uint8_t green = (green_6bit * 255) / 63;
            uint8_t blue = (blue_5bit * 255) / 31;

            if (red != last_red || green != last_green || blue != last_blue)
            {
                last_red = red;
                last_green = green;
                last_blue = blue;

                tx_data[22] = red;
                tx_data[23] = green;
                tx_data[24] = blue;

                uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
            }
        }

        lv_obj_invalidate(obj);
    }
    else if (code == LV_EVENT_RELEASED)
    {
        wheel->pressed = false;
    }
}
static void power_button_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    custom_rgb_wheel_t *wheel = (custom_rgb_wheel_t *)lv_obj_get_user_data(lv_obj_get_parent(btn));

    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        wheel->power_state = !wheel->power_state;

        if (wheel->power_state)
        {
            // Power ON - update button style
            lv_obj_set_style_bg_color(btn, rgb_color, 0);

            // Send initial color when powered on
            uint8_t red_5bit = rgb_color.ch.red;
            uint8_t green_6bit = rgb_color.ch.green;
            uint8_t blue_5bit = rgb_color.ch.blue;

            uint8_t red = (red_5bit * 255) / 31;
            uint8_t green = (green_6bit * 255) / 63;
            uint8_t blue = (blue_5bit * 255) / 31;

            tx_data[22] = red;
            tx_data[23] = green;
            tx_data[24] = blue;
            uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
        }
        else
        {
            // Power OFF
            lv_obj_set_style_bg_color(btn, lv_color_make(64, 64, 64), 0);

            // Send all zeros to turn off the lights
            tx_data[22] = 0;
            tx_data[23] = 0;
            tx_data[24] = 0;
            uart_write_bytes(UART_NUM_1, (const char *)tx_data, sizeof(tx_data));
        }
    }
}

lv_obj_t *custom_rgb_wheel_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_create(parent);

    custom_rgb_wheel_t *wheel = (custom_rgb_wheel_t *)lv_mem_alloc(sizeof(custom_rgb_wheel_t));
    if (wheel == NULL)
        return NULL;

    // Initialize wheel structure
    wheel->pressed = false;
    wheel->current_color = lv_color_white();
    wheel->current_angle = 0;
    wheel->power_state = false; // Start powered off

    wheel->obj = obj;

    lv_coord_t wheel_size = 380;
    lv_obj_set_size(obj, wheel_size, wheel_size);
    float center_radius = wheel_size * 0.10f;
    wheel->power_btn = lv_btn_create(obj);
    lv_obj_set_size(wheel->power_btn, center_radius * 2, center_radius * 2);
    lv_obj_center(wheel->power_btn);

    // Style the power button
    lv_obj_set_style_radius(wheel->power_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wheel->power_btn, lv_color_make(64, 64, 64), 0);
    // lv_obj_set_style_border_color(wheel->power_btn, lv_color_white(), 0);
    // lv_obj_set_style_border_width(wheel->power_btn, 2, 0);

    // Add power symbol
    lv_obj_t *power_symbol = lv_label_create(wheel->power_btn);
    lv_label_set_text(power_symbol, LV_SYMBOL_POWER);
    lv_obj_set_style_text_font(power_symbol, &lv_font_montserrat_22, 0); // Replace 24 with desired size

    lv_obj_set_style_text_color(power_symbol, lv_color_white(), 0);

    lv_obj_center(power_symbol);

    // Add click event
    lv_obj_add_event_cb(wheel->power_btn, power_button_event_cb, LV_EVENT_CLICKED, NULL);

    // Center the wheel
    lv_coord_t parent_width = lv_obj_get_width(parent);
    lv_coord_t parent_height = lv_obj_get_height(parent);
    lv_obj_set_pos(obj, (parent_width - wheel_size) / 2, (parent_height - wheel_size) / 2);

    lv_obj_set_user_data(obj, wheel);

    // Ultra-fast event handling
    lv_obj_add_event_cb(obj, draw_rgb_wheel, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(obj, rgb_wheel_event_cb, LV_EVENT_PRESSED | LV_EVENT_PRESSING | LV_EVENT_LONG_PRESSED, NULL);

    // Maximum performance optimizations
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_0, 0);
    lv_obj_set_style_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);

    return obj;
}

static void draw_slider_arc_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    // Get coordinates
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    // Calculate center and radius
    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;
    lv_coord_t radius = 12; // Smaller central circle

    /*lv_color_t cool_color = lv_color_hex(0x80C9FF);    // 8000K (Cool)
       lv_color_t neutral_color = lv_color_hex(0xFFF4E5); // 4000K (Neutral)
       lv_color_t warm_color = lv_color_hex(0xFFB46B);*/
    // Draw the central circle
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.width = 12;                     // Thickness of the circle
    arc_dsc.color = lv_color_hex(0x80C9FF); // Black color to match image

    lv_point_t center_point = {
        .x = cx,
        .y = cy};

    // Draw complete circle (0 to 360 degrees)
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, radius, 0, 360);

    // Initialize line descriptor for rays
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color_hex(0x000000); // Black color to match image
    line_dsc.width = 3;                      // Thicker lines for rays
}

static void draw_slider_arc_cb1(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    // Get coordinates
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    // Calculate center and radius
    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;
    lv_coord_t radius = 12; // Smaller central circle

    // Draw the central circle
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.width = 12;                     // Thickness of the circle
    arc_dsc.color = lv_color_hex(0xFFB46B); // Black color to match image

    lv_point_t center_point = {
        .x = cx,
        .y = cy};

    // Draw complete circle (0 to 360 degrees)
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, radius, 0, 360);

    // Initialize line descriptor for rays
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color_hex(0x000000); // Black color to match image
    line_dsc.width = 3;                      // Thicker lines for rays
}
static void draw_slider_arc_left_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    // Get coordinates
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    // Calculate center and radius
    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;
    lv_coord_t radius = 8; // Smaller central circle

    // Draw the central circle
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.width = 12;                     // Thickness of the circle
    arc_dsc.color = lv_color_hex(0x000000); // Black color to match image

    lv_point_t center_point = {
        .x = cx,
        .y = cy};

    // Draw complete circle (0 to 360 degrees)
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, radius, 0, 360);

    // Initialize line descriptor for rays
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color_hex(0x000000); // Black color to match image
    line_dsc.width = 3;                      // Thicker lines for rays

    // Draw 8 rays
    for (int i = 0; i < 8; i++)
    {
        // Calculate angle (45 degrees apart, offset by 22.5 degrees to match image)
        /* float angle = (float)(i * 45 + 22.5) * M_PI / 180.0f;

         // Ray start point (on the circle)
         int ray_start_x = cx + (radius + 2) * cosf(angle); // Start from circle edge
         int ray_start_y = cy + (radius + 2) * sinf(angle);

         // Ray end point (longer than the circle)
         int ray_end_x = cx + (radius + 8) * cosf(angle); // Extend further out
         int ray_end_y = cy + (radius + 8) * sinf(angle);

         // Draw the line for the ray
         lv_point_t ray_points[] = {
             {.x = ray_start_x, .y = ray_start_y}, // Start point
             {.x = ray_end_x, .y = ray_end_y}      // End point
         };
         lv_draw_line(draw_ctx, &line_dsc, &ray_points[0], &ray_points[1]);*/
        // Calculate angle (45 degrees apart, offset by 22.5 degrees to match image)
        float angle = (float)(i * 45 + 22.5) * M_PI / 180.0f;

        // Ray start point (on the circle)
        int ray_start_x = cx + (radius + 2) * cosf(angle); // Start from circle edge
        int ray_start_y = cy + (radius + 2) * sinf(angle);

        // Ray end point (longer than the circle)
        int ray_end_x = cx + (radius + 8) * cosf(angle); // Extend further out
        int ray_end_y = cy + (radius + 8) * sinf(angle);

        // Draw the line for the ray
        lv_point_t ray_points[] = {
            {.x = ray_start_x, .y = ray_start_y}, // Start point
            {.x = ray_end_x, .y = ray_end_y}      // End point
        };
        lv_draw_line(draw_ctx, &line_dsc, &ray_points[0], &ray_points[1]);

        // Draw small dot at the end of each ray
    }
}
static void draw_slider_arc_right_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    // Get coordinates
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    // Calculate center and radius
    lv_coord_t cx = (obj_coords.x2 + obj_coords.x1) / 2;
    lv_coord_t cy = (obj_coords.y2 + obj_coords.y1) / 2;
    lv_coord_t radius = 8; // Smaller central circle

    // Draw the central circle
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.width = 12;                     // Thickness of the circle
    arc_dsc.color = lv_color_hex(0xffffff); // Black color to match image

    lv_point_t center_point = {
        .x = cx,
        .y = cy};

    // Draw complete circle (0 to 360 degrees)
    lv_draw_arc(draw_ctx, &arc_dsc, &center_point, radius, 0, 360);

    // Initialize line descriptor for rays
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color_hex(0xffffff); // Black color to match image
    line_dsc.width = 3;                      // Thicker lines for rays

    // Draw 8 rays
    for (int i = 0; i < 8; i++)
    {
        // Calculate angle (45 degrees apart, offset by 22.5 degrees to match image)
        float angle = (float)(i * 45 + 22.5) * M_PI / 180.0f;

        // Ray start point (on the circle)
        int ray_start_x = cx + (radius + 2) * cosf(angle); // Start from circle edge
        int ray_start_y = cy + (radius + 2) * sinf(angle);

        // Ray end point (longer than the circle)
        int ray_end_x = cx + (radius + 8) * cosf(angle); // Extend further out
        int ray_end_y = cy + (radius + 8) * sinf(angle);

        // Draw the line for the ray
        lv_point_t ray_points[] = {
            {.x = ray_start_x, .y = ray_start_y}, // Start point
            {.x = ray_end_x, .y = ray_end_y}      // End point
        };
        lv_draw_line(draw_ctx, &line_dsc, &ray_points[0], &ray_points[1]);
    }
}

static void create_arcs_warm_cool(lv_obj_t *container, lv_obj_t *slider)
{
    // Create and position left sun icon
    lv_obj_t *sun_container_left = lv_obj_create(container);
    lv_obj_set_size(sun_container_left, 30, 30); // Increased size for better visibility
    lv_obj_set_style_bg_opa(sun_container_left, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sun_container_left, 0, 0);

    lv_obj_clear_flag(sun_container_left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(sun_container_left, slider, LV_ALIGN_OUT_LEFT_MID, -40, 0);
    lv_obj_add_event_cb(sun_container_left, draw_slider_arc_cb, LV_EVENT_DRAW_MAIN, NULL);

    // Create and position right sun icon
    lv_obj_t *sun_container_right = lv_obj_create(container);
    lv_obj_set_size(sun_container_right, 30, 30); // Increased size for better visibility
    lv_obj_set_style_bg_opa(sun_container_right, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sun_container_right, 0, 0);

    lv_obj_clear_flag(sun_container_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(sun_container_right, slider, LV_ALIGN_OUT_RIGHT_MID, 40, 0);
    lv_obj_add_event_cb(sun_container_right, draw_slider_arc_cb1, LV_EVENT_DRAW_MAIN, NULL);
}

static void create_arcs_warm_cool_left(lv_obj_t *container, lv_obj_t *slider)
{
    // Create and position left arc

    // Create and position left sun icon
    lv_obj_t *sun_container_left = lv_obj_create(container);
    lv_obj_set_size(sun_container_left, 40, 40); // Increased size for better visibility
    lv_obj_set_style_bg_opa(sun_container_left, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sun_container_left, 0, 0);

    lv_obj_clear_flag(sun_container_left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(sun_container_left, slider, LV_ALIGN_OUT_LEFT_MID, -40, 0);
    lv_obj_add_event_cb(sun_container_left, draw_slider_arc_left_cb, LV_EVENT_DRAW_MAIN, NULL);

    // lv_obj_add_event_cb(sun_container_left, draw_slider_arc_left_cb, LV_EVENT_DRAW_MAIN, NULL);
}
static void create_arcs_warm_cool_right(lv_obj_t *container, lv_obj_t *slider)
{

    lv_obj_t *sun_container_right = lv_obj_create(container);
    lv_obj_set_size(sun_container_right, 40, 40); // Increased size for better visibility
    lv_obj_set_style_bg_opa(sun_container_right, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sun_container_right, 0, 0);

    lv_obj_clear_flag(sun_container_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align_to(sun_container_right, slider, LV_ALIGN_OUT_RIGHT_MID, 40, 0);
    lv_obj_add_event_cb(sun_container_right, draw_slider_arc_right_cb, LV_EVENT_DRAW_MAIN, NULL);
}
#define MAX_AP_NUM 20 // Maximum number of access points to scan

// Function to scan available Wi-Fi networks
// Event handler for handling the selection of an SSID from the list
// Function to handle the selection of an SSID from the list
/*static void ssid_list_event_handler(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, 0);           // Get the first child (the label) of the button
    const char *selected_ssid = lv_label_get_text(label); // Get the label text (SSID)

    strncpy(wifi_ssid, selected_ssid, sizeof(wifi_ssid) - 1); // Copy selected SSID to wifi_ssid
    wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';                  // Ensure null-termination

    // Display the selected SSID in the SSID input field
    lv_textarea_set_text(ssid_input, wifi_ssid);

    ESP_LOGI(TAG, "Selected SSID: %s", wifi_ssid);
}*/
static void ssid_select_event_handler(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    const char *selected_ssid = lv_label_get_text(label);

    // Set the selected SSID to the input field
    strncpy(wifi_ssid, selected_ssid, sizeof(wifi_ssid) - 1);
    wifi_ssid[sizeof(wifi_ssid) - 1] = '\0'; // Ensure null-termination

    // Display the selected SSID in the input field
    lv_textarea_set_text(ssid_input, wifi_ssid);

    // Hide the SSID container
    lv_obj_add_flag(ssid_container, LV_OBJ_FLAG_HIDDEN);
    // Hide the SSID list
    // lv_obj_add_flag(ssid_container, LV_OBJ_FLAG_HIDDEN);
}
// Function to display the scanned SSIDs on the UI
void display_ssids(wifi_ap_record_t *ap_records, uint16_t ap_count)
{
    // Clear previous list if it exists
    if (ssid_list != NULL)
    {
        lv_obj_del(ssid_list);
    }

    // Show the SSID container
    lv_obj_clear_flag(ssid_container, LV_OBJ_FLAG_HIDDEN);

    // Create new list
    ssid_list = lv_list_create(ssid_container);
    lv_obj_set_size(ssid_list, 240, 140); // Slightly smaller than container
    lv_obj_center(ssid_list);

    // Add SSIDs to the list
    for (int i = 0; i < ap_count; i++)
    {
        lv_obj_t *btn = lv_list_add_btn(ssid_list, NULL, (const char *)ap_records[i].ssid);
        lv_obj_add_event_cb(btn, ssid_select_event_handler, LV_EVENT_CLICKED, NULL);

        // Style the button
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}
static void update_ssid_list_cb(void *data)
{
    struct
    {
        wifi_ap_record_t *aps;
        uint16_t count;
    } *scan_data = (void *)data;

    // Clear previous list
    lv_obj_clean(ssid_container);

    // Show the container
    lv_obj_clear_flag(ssid_container, LV_OBJ_FLAG_HIDDEN);

    // Create list for SSIDs
    for (int i = 0; i < scan_data->count; i++)
    {
        // Create button for each SSID
        lv_obj_t *btn = lv_btn_create(ssid_container);
        lv_obj_set_size(btn, 230, 40);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, i * 45);
        lv_obj_add_event_cb(btn, ssid_select_event_handler, LV_EVENT_CLICKED, NULL);

        // Create label on the button
        lv_obj_t *label = lv_label_create(btn);
        char ssid_str[33]; // Maximum SSID length + null terminator
        memcpy(ssid_str, scan_data->aps[i].ssid, sizeof(scan_data->aps[i].ssid));
        ssid_str[32] = '\0'; // Ensure null termination
        lv_label_set_text(label, ssid_str);
        lv_obj_center(label);
    }
}
static void wifi_scan_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting WiFi scan task");

    uint16_t ap_count = MAX_AP_NUM;
    wifi_ap_record_t ap_records[MAX_AP_NUM];

    // Initialize WiFi if not already initialized
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED)
    {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
        wifi_scanning = false;
        vTaskDelete(NULL);
        return;
    }

    // Start scan
    err = esp_wifi_scan_start(NULL, true);
    if (err == ESP_OK)
    {
        err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Total APs scanned: %d", ap_count);

            // Update UI from main task using a callback
            lv_async_call(update_ssid_list_cb, &(struct {
                              wifi_ap_record_t *aps;
                              uint16_t count;
                          }){ap_records, ap_count});
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(err));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
    }

    wifi_scanning = false;
    vTaskDelete(NULL);
}
static void scan_wifi_networks(void)
{
    if (wifi_scanning)
    {
        ESP_LOGI(TAG, "Scan already in progress");
        return;
    }

    wifi_scanning = true;

    // Show scanning indicator in UI
    lv_textarea_set_placeholder_text(ssid_input, "Scanning...");

    // Create task for WiFi scanning
    xTaskCreatePinnedToCore(
        wifi_scan_task,
        "wifi_scan_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &wifi_scan_task_handle,
        0 // Run on core 0, assuming LVGL runs on core 1
    );
}
// WiFi initialization function
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define SSID_MAX_LEN 32
#define PASSWORD_MAX_LEN 64
static QueueHandle_t uart_queue;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi Connected");
        clear_wifi_input_ui();
        // Display Wi-Fi symbol
        display_wifi_symbol();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi Disconnected");
        // Optionally hide Wi-Fi symbol
        hide_wifi_symbol();
    }
}

void wifi_init(const char *ssid, const char *password)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack
    esp_netif_init();
    esp_event_loop_create_default();

    // Register the event handler for Wi-Fi events
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Set Wi-Fi mode to station
    esp_wifi_set_mode(WIFI_MODE_STA);

    // Set SSID and password
    wifi_config_t wifi_config = {};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    // Start Wi-Fi
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}
lv_obj_t *wifi_icon;

void display_wifi_symbol()
{
    wifi_icon = lv_label_create(lv_scr_act());
    lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI);         // Use LVGL's built-in Wi-Fi symbol
    lv_obj_align(wifi_icon, LV_ALIGN_TOP_RIGHT, -10, 10); // Align to the top-right corner
}

void hide_wifi_symbol()
{
    if (wifi_icon)
    {
        lv_obj_del(wifi_icon); // Delete the Wi-Fi icon if it exists
    }
}
void clear_wifi_input_ui()
{
    lv_obj_t *screen = lv_scr_act();

    // Delete all objects on the screen
    lv_obj_clean(screen);
}

static void btn_event_handler(lv_event_t *e)
{
    strncpy(wifi_password, lv_textarea_get_text(password_input), sizeof(wifi_password) - 1);
    wifi_password[sizeof(wifi_password) - 1] = '\0'; // Ensure null-termination

    if (strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0)
    {
        ESP_LOGI(TAG, "Attempting to connect to SSID: %s", wifi_ssid);
        wifi_init(wifi_ssid, wifi_password); // Initialize Wi-Fi with the selected SSID and entered password
    }
    else
    {

        ESP_LOGI(TAG, "SSID or Password is empty!");
    }
}

// Keyboard event handler
lv_obj_t *active_textarea = NULL;

// Keyboard event handler
static void keyboard_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL)
    {
        lv_keyboard_set_textarea(keyboard, NULL);      // Unlink keyboard from any textarea
        lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN); // Hide the keyboard
    }
}
static void textarea_event_handler(lv_event_t *e)
{
    lv_obj_t *textarea = lv_event_get_target(e);

    // Set the current textarea to be edited by the keyboard
    active_textarea = textarea;
    lv_keyboard_set_textarea(keyboard, active_textarea); // Link the keyboard to the active textarea
    lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);     // Show the keyboard when textarea is clicked
}
// Initialize the display and UI
static void ssid_event_handler(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    // Check if the event is "focused"
    if (event == LV_EVENT_CLICKED)
    {
        ESP_LOGI(TAG, "SSID input field focused. Scanning for Wi-Fi networks...");
        scan_wifi_networks(); // Call your function to scan for Wi-Fi networks
    }
}
void create_wifi_input_ui(void)
{
    lv_obj_t *wifi_container = lv_obj_create(lighting_control->wifi_screen);
    lv_obj_set_size(wifi_container, LV_PCT(100), LV_PCT(80));
    lv_obj_align(wifi_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(wifi_container, LV_OBJ_FLAG_SCROLLABLE);

    // Create SSID input section
    ssid_label = lv_label_create(wifi_container);
    lv_label_set_text(ssid_label, "Enter SSID:");
    lv_obj_align(ssid_label, LV_ALIGN_TOP_MID, 0, 10);

    ssid_input = lv_textarea_create(wifi_container);
    lv_textarea_set_text(ssid_input, "");
    lv_textarea_set_placeholder_text(ssid_input, "Click to scan networks");
    lv_obj_set_size(ssid_input, 250, 40);
    lv_obj_align(ssid_input, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_add_event_cb(ssid_input, ssid_event_handler, LV_EVENT_CLICKED, NULL);

    // Create container for SSID list with scrolling
    ssid_container = lv_obj_create(wifi_container);
    lv_obj_set_size(ssid_container, 250, 150); // Adjust size as needed
    lv_obj_align(ssid_container, LV_ALIGN_TOP_MID, 0, 90);
    lv_obj_set_style_bg_color(ssid_container, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ssid_container, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ssid_container, lv_color_hex(0xDDDDDD), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Initially hide the SSID container
    lv_obj_add_flag(ssid_container, LV_OBJ_FLAG_HIDDEN);

    // Password input section
    password_label = lv_label_create(wifi_container);
    lv_label_set_text(password_label, "Enter Password:");
    lv_obj_align(password_label, LV_ALIGN_TOP_MID, 0, 120);

    password_input = lv_textarea_create(wifi_container);
    lv_textarea_set_placeholder_text(password_input, "Password");
    lv_obj_set_size(password_input, 250, 40);
    lv_obj_align(password_input, LV_ALIGN_TOP_MID, 0, 160);
    lv_obj_add_event_cb(password_input, textarea_event_handler, LV_EVENT_CLICKED, NULL);

    // Connect button
    connect_btn = lv_btn_create(wifi_container);
    lv_obj_set_size(connect_btn, 120, 40);
    lv_obj_align(connect_btn, LV_ALIGN_TOP_MID, 0, 220);
    lv_obj_add_event_cb(connect_btn, btn_event_handler, LV_EVENT_CLICKED, NULL);

    connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "Connect");
    lv_obj_center(connect_label);

    // Keyboard
    keyboard = lv_keyboard_create(lighting_control->wifi_screen);
    lv_obj_set_size(keyboard, 320, 200);
    lv_obj_align(keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(keyboard, keyboard_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
}

// Initialize WiFi in Access Point Mode
void init_wifi()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Modified RGB button event handler
static void rgb_button_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        lv_obj_add_flag(lighting_control->rgb_screen, LV_OBJ_FLAG_HIDDEN);
        // Add your button click handling code here
        lv_obj_add_flag(lighting_control->warm_cool_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lighting_control->tuning_screen, LV_OBJ_FLAG_HIDDEN);
        create_wifi_screen();
        // First, show the WiFi screen
        lv_obj_clear_flag(lighting_control->wifi_screen, LV_OBJ_FLAG_HIDDEN);

        // Remove gesture event from main screen while in WiFi mode
        lv_obj_remove_event_cb(lighting_control->main_screen, screen_slide_event_cb);
        create_wifi_input_ui();
        init_wifi();

        // scan_wifi();
        //  Initialize WiFi components

        // xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL, 1); // Pin to CPU 1
    }
}

// Modified back button event handler
void back_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {

        // Show RGB screen and hide WiFi screen
        lv_obj_clear_flag(lighting_control->rgb_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lighting_control->warm_cool_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lighting_control->tuning_screen, LV_OBJ_FLAG_HIDDEN);
        // Then delete WiFi screen (this will clean up all WiFi UI elements)
        if (lighting_control->wifi_screen != NULL)
        {
            lv_obj_del(lighting_control->wifi_screen);
            lighting_control->wifi_screen = NULL;
            ssid_list = NULL; // Reset SSID list pointer
        }

        // Restore gesture events
        lv_obj_add_event_cb(lighting_control->main_screen, screen_slide_event_cb, LV_EVENT_GESTURE, NULL);

        // Clear WiFi UI elements
    }
}

// Modified WiFi screen creation

// Modified WiFi screen creation
void create_wifi_screen(void)
{
    // Create WiFi screen as a full overlay
    lighting_control->wifi_screen = lv_obj_create(lv_scr_act()); // Create on active screen instead of main_screen
    lv_obj_set_size(lighting_control->wifi_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(lighting_control->wifi_screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(lighting_control->wifi_screen, LV_OBJ_FLAG_HIDDEN);

    // Set solid background to prevent RGB screen from showing through
    lv_obj_set_style_bg_color(lighting_control->wifi_screen, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(lighting_control->wifi_screen, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Create a container for the SSID list with proper scrolling

    // Create back button with improved styling
    lv_obj_t *back_btn = lv_btn_create(lighting_control->wifi_screen);
    lv_obj_set_size(back_btn, 120, 40);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_obj_t *btn_label = lv_label_create(back_btn);
    lv_label_set_text(btn_label, "Back");
    lv_obj_center(btn_label);

    // Style the back button
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Add click event handler
    lv_obj_add_event_cb(back_btn, back_btn_event_cb, LV_EVENT_CLICKED, NULL);
}
void create_sliding_navigation(void)
{

    // init_rgb_wheel_debounce();
    lighting_control = (lighting_control_t *)lv_mem_alloc(sizeof(lighting_control_t));
    lv_memset_00(lighting_control, sizeof(lighting_control_t));

    lighting_control->main_screen = lv_obj_create(NULL);
    lv_obj_add_event_cb(lighting_control->main_screen, screen_slide_event_cb, LV_EVENT_GESTURE, NULL);

    lighting_control->rgb_screen = lv_obj_create(lighting_control->main_screen);
    lv_obj_set_size(lighting_control->rgb_screen, LV_PCT(100), LV_PCT(100));
    // lv_obj_add_event_cb(lighting_control->rgb_screen, rgb_screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(lighting_control->rgb_screen, LV_OBJ_FLAG_SCROLLABLE);

    lighting_control->rgb_wheel = custom_rgb_wheel_create(lighting_control->rgb_screen);
    //  lv_obj_set_size(lighting_control->rgb_wheel, 380, 380);
    lv_obj_center(lighting_control->rgb_wheel);
    lv_obj_set_style_bg_color(lighting_control->rgb_screen, lv_color_hex(0xffdbba), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *rgb_label_container = lv_obj_create(lighting_control->rgb_screen);
    lv_obj_set_size(rgb_label_container, 120, 35);               // Adjust size
    lv_obj_set_flex_flow(rgb_label_container, LV_FLEX_FLOW_ROW); // Arrange labels horizontally
    lv_obj_set_flex_align(rgb_label_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(rgb_label_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(rgb_label_container, lv_color_hex(0xF5F5DC), 0); // Light grey
    lv_obj_set_style_bg_opa(rgb_label_container, LV_OPA_COVER, 0);             // Ensure full background color

    // Create Red, Green, and Blue labels
    lv_obj_t *red_label = lv_label_create(rgb_label_container);
    lv_label_set_text(red_label, "R");
    lv_obj_set_style_text_font(red_label, &lv_font_montserrat_22, 0);

    lv_obj_set_style_text_color(red_label, lv_color_hex(0xFF0000), 0); // Red color

    lv_obj_t *green_label = lv_label_create(rgb_label_container);
    lv_label_set_text(green_label, "G");
    lv_obj_set_style_text_font(green_label, &lv_font_montserrat_22, 0);

    lv_obj_set_style_text_color(green_label, lv_color_hex(0x00FF00), 0); // Green color

    lv_obj_t *blue_label = lv_label_create(rgb_label_container);
    lv_label_set_text(blue_label, "B");
    lv_obj_set_style_text_font(blue_label, &lv_font_montserrat_22, 0);

    lv_obj_set_style_text_color(blue_label, lv_color_hex(0x0000FF), 0); // Blue color

    // Position container above RGB wheel
    lv_obj_align(rgb_label_container, LV_ALIGN_TOP_MID, 0, 2);

    lighting_control->rgb_intensity_slider = lv_arc_create(lighting_control->rgb_screen);

    // Set size
    lv_obj_set_size(lighting_control->rgb_intensity_slider, 200, 200); // Make it square for better appearance

    // Configure the arc's appearance
    lv_arc_set_rotation(lighting_control->rgb_intensity_slider, 135);     // Start position
    lv_arc_set_bg_angles(lighting_control->rgb_intensity_slider, 0, 270); // Creates U shape
    lv_arc_set_angles(lighting_control->rgb_intensity_slider, 0, 0);      // Initial value
    lv_arc_set_range(lighting_control->rgb_intensity_slider, 0, 255);     // Same range as before

    // Center alignment
    lv_obj_align(lighting_control->rgb_intensity_slider, LV_ALIGN_CENTER, 0, 0);

    // Add flags
    lv_obj_add_flag(lighting_control->rgb_intensity_slider, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(lighting_control->rgb_intensity_slider, rgb_intensity_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lighting_control->rgb_button = lv_btn_create(lighting_control->rgb_screen);
    lv_obj_set_size(lighting_control->rgb_button, 120, 40);                // Set width and height
    lv_obj_align(lighting_control->rgb_button, LV_ALIGN_BOTTOM_MID, 0, 5); // Position at bottom, 20px margin
    lv_obj_clear_flag(lighting_control->rgb_button, LV_OBJ_FLAG_SCROLLABLE);

    // Add a label to the button

    lv_obj_t *btn_label = lv_label_create(lighting_control->rgb_button);

    lv_label_set_text(btn_label, "Settings"); // Set button text
    lv_obj_center(btn_label);                 // Center the label in the button
    lv_obj_clear_flag(btn_label, LV_OBJ_FLAG_SCROLLABLE);

    // Style the button (optional)
    lv_obj_set_style_bg_color(lighting_control->rgb_button, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT); // Blue color
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);                  // White text

    // Add click event handler
    lv_obj_add_event_cb(lighting_control->rgb_button, rgb_button_event_cb, LV_EVENT_CLICKED, NULL);

    lighting_control->warm_cool_screen = lv_obj_create(NULL);
    lv_obj_add_event_cb(lighting_control->warm_cool_screen, screen_slide_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_set_size(lighting_control->warm_cool_screen, LV_PCT(100), LV_PCT(100)); // Full size container
    lv_obj_set_flex_flow(lighting_control->warm_cool_screen, LV_FLEX_FLOW_COLUMN);

    lighting_control->rgb_button = lv_btn_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->rgb_button, 120, 40);                // Set width and height
    lv_obj_align(lighting_control->rgb_button, LV_ALIGN_BOTTOM_MID, 0, 5); // Position at bottom, 20px margin
    lv_obj_clear_flag(lighting_control->rgb_button, LV_OBJ_FLAG_SCROLLABLE);
    btn_label = lv_label_create(lighting_control->rgb_button);

    lv_label_set_text(btn_label, "Settings"); // Set button text
    lv_obj_center(btn_label);                 // Center the label in the button
    lv_obj_clear_flag(btn_label, LV_OBJ_FLAG_SCROLLABLE);

    // Style the button (optional)
    lv_obj_set_style_bg_color(lighting_control->rgb_button, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT); // Blue color
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);                  // White text

    // Add click event handler
    lv_obj_add_event_cb(lighting_control->rgb_button, rgb_button_event_cb, LV_EVENT_CLICKED, NULL);

    // Set padding and spacing
    lv_obj_set_style_pad_row(lighting_control->warm_cool_screen, 25, 0);
    lv_obj_set_style_pad_top(lighting_control->warm_cool_screen, 7, 0);
    lv_obj_set_style_pad_bottom(lighting_control->warm_cool_screen, 10, 0);
    lv_obj_set_flex_align(lighting_control->warm_cool_screen, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);
    // Warm Intensity container
    lv_obj_set_style_bg_color(lighting_control->warm_cool_screen, lv_color_hex(0xffdbba), LV_PART_MAIN | LV_STATE_DEFAULT);
    // Make sure background is visible
    // lv_obj_set_style_bg_opa(lighting_control->warm_cool_screen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lighting_control->warm_intensity_container1 = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->warm_intensity_container1, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(lighting_control->warm_intensity_container1, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(lighting_control->warm_intensity_container1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    static lv_style_t style_container;
    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->warm_intensity_container1, &style_container, LV_PART_MAIN);

    lighting_control->warm_intensity_label = lv_label_create(lighting_control->warm_intensity_container1);
    lv_label_set_text(lighting_control->warm_intensity_label, "Tunable 1");
    lv_obj_set_style_text_color(lighting_control->warm_intensity_label, lv_color_hex(0xffffff), 0);

    lv_obj_set_style_text_font(lighting_control->warm_intensity_label, &lv_font_montserrat_22, 0);

    lighting_control->warm_intensity_label1 = lv_label_create(lighting_control->warm_intensity_container1);
    lv_label_set_text(lighting_control->warm_intensity_label1, "Tunable 2");
    lv_obj_set_style_text_color(lighting_control->warm_intensity_label1, lv_color_hex(0xffffff), 0);

    lv_obj_set_style_bg_color(lighting_control->warm_intensity_container1, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_container1, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_container1, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_obj_set_style_text_font(lighting_control->warm_intensity_label1, &lv_font_montserrat_22, 0);

    lv_obj_add_flag(lighting_control->warm_intensity_label1, LV_OBJ_FLAG_HIDDEN);

    // Warm/Cool container
    lighting_control->warm_cool_container = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->warm_cool_container, 415, 60);
    // lv_obj_set_flex_flow(lighting_control->warm_cool_container, LV_FLEX_FLOW_COLUMN);
    // lv_obj_set_flex_align(lighting_control->warm_cool_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->warm_cool_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(lighting_control->warm_cool_container, 0, 0);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->warm_cool_container, &style_container, LV_PART_MAIN);

    lv_obj_set_style_bg_color(lighting_control->warm_cool_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_cool_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_cool_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lighting_control->warm_cool_slider = lv_slider_create(lighting_control->warm_cool_container);
    lv_obj_set_size(lighting_control->warm_cool_slider, 250, 15);
    lv_obj_center(lighting_control->warm_cool_slider); // Center in parent

    lv_slider_set_range(lighting_control->warm_cool_slider, 0, 255);

    lv_obj_add_event_cb(lighting_control->warm_cool_slider, warm_cool_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    init_warm_cool_slider(lighting_control->warm_cool_slider);
    // Warm/Cool container
    create_arcs_warm_cool(lighting_control->warm_cool_container, lighting_control->warm_cool_slider);

    lighting_control->warm_cool_container1 = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->warm_cool_container1, 415, 60);
    // lv_obj_set_flex_flow(lighting_control->warm_cool_container1, LV_FLEX_FLOW_COLUMN);
    // lv_obj_set_flex_align(lighting_control->warm_cool_container1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(lighting_control->warm_cool_container1, 0, 0);

    lv_obj_add_flag(lighting_control->warm_cool_container1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lighting_control->warm_cool_container1, LV_OBJ_FLAG_SCROLLABLE);
    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->warm_cool_container1, &style_container, LV_PART_MAIN);

    lv_obj_set_style_bg_color(lighting_control->warm_cool_container1, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_cool_container1, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_cool_container1, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lighting_control->warm_cool_slider1 = lv_slider_create(lighting_control->warm_cool_container1);
    lv_obj_set_size(lighting_control->warm_cool_slider1, 250, 15);
    lv_slider_set_range(lighting_control->warm_cool_slider1, 0, 255);
    lv_obj_center(lighting_control->warm_cool_slider1); // Center in parent

    lv_obj_add_flag(lighting_control->warm_cool_slider1, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(lighting_control->warm_cool_slider1, warm_cool_slider_event_cb1, LV_EVENT_VALUE_CHANGED, NULL);
    init_warm_cool_slider(lighting_control->warm_cool_slider1);
    create_arcs_warm_cool(lighting_control->warm_cool_container1, lighting_control->warm_cool_slider1);

    // Warm Intensity container
    lighting_control->warm_intensity_container = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->warm_intensity_container, 415, 60);
    // lv_obj_set_flex_flow(lighting_control->warm_intensity_container, LV_FLEX_FLOW_COLUMN);
    // lv_obj_set_flex_align(lighting_control->warm_intensity_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->warm_intensity_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->warm_intensity_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->warm_intensity_container, &style_container, LV_PART_MAIN);

    lighting_control->warm_intensity_slider = lv_slider_create(lighting_control->warm_intensity_container);
    lv_obj_set_size(lighting_control->warm_intensity_slider, 250, 15);
    lv_slider_set_range(lighting_control->warm_intensity_slider, 0, 255);
    lv_obj_center(lighting_control->warm_intensity_slider); // Center in parent

    // Set the gradient colors for the slider background
    lv_obj_set_style_bg_color(lighting_control->warm_intensity_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_slider, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->warm_intensity_slider, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->warm_intensity_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_slider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->warm_intensity_slider, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label5);
    lv_style_set_bg_color(&style_percentage_label5, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label5, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label5, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label5, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label5, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label5, &lv_font_montserrat_14);

    percentage_label5 = lv_label_create(lighting_control->warm_intensity_container);
    lv_obj_add_style(percentage_label5, &style_percentage_label5, 0);
    lv_obj_add_flag(percentage_label5, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label5, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label5);
    update_label_pos(lighting_control->warm_intensity_slider, percentage_label5);
    lv_obj_add_event_cb(lighting_control->warm_intensity_slider, intensity_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Button container
    create_arcs_warm_cool_left(lighting_control->warm_intensity_container, lighting_control->warm_intensity_slider);
    create_arcs_warm_cool_right(lighting_control->warm_intensity_container, lighting_control->warm_intensity_slider);

    // Warm Intensity container
    lighting_control->warm_intensity_container2 = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->warm_intensity_container2, 415, 60);
    // lv_obj_set_flex_flow(lighting_control->warm_intensity_container2, LV_FLEX_FLOW_COLUMN);
    // lv_obj_set_flex_align(lighting_control->warm_intensity_container2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(lighting_control->warm_intensity_container2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lighting_control->warm_intensity_container2, LV_OBJ_FLAG_SCROLLABLE);
    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->warm_intensity_container2, &style_container, LV_PART_MAIN);

    lv_obj_set_style_bg_color(lighting_control->warm_intensity_container2, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_container2, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_container2, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lighting_control->warm_intensity_slider1 = lv_slider_create(lighting_control->warm_intensity_container2);
    lv_obj_set_size(lighting_control->warm_intensity_slider1, 250, 15);
    lv_slider_set_range(lighting_control->warm_intensity_slider1, 0, 255);
    lv_obj_center(lighting_control->warm_intensity_slider1); // Center in parent

    lv_obj_add_flag(lighting_control->warm_intensity_slider1, LV_OBJ_FLAG_HIDDEN);
    // Set the gradient colors for the slider background
    lv_obj_set_style_bg_color(lighting_control->warm_intensity_slider1, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_slider1, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_slider1, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->warm_intensity_slider1, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->warm_intensity_slider1, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->warm_intensity_slider1, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->warm_intensity_slider1, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->warm_intensity_slider1, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator
    lv_style_init(&style_percentage_label6);
    lv_style_set_bg_color(&style_percentage_label6, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label6, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label6, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label6, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label6, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label6, &lv_font_montserrat_14);

    percentage_label6 = lv_label_create(lighting_control->warm_intensity_container2);
    lv_obj_add_style(percentage_label6, &style_percentage_label6, 0);
    lv_obj_add_flag(percentage_label6, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label6, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label6);
    update_label_pos(lighting_control->warm_intensity_slider1, percentage_label5);
    lv_obj_add_event_cb(lighting_control->warm_intensity_slider1, intensity_slider_event_cb1, LV_EVENT_VALUE_CHANGED, NULL);
    // Button container
    create_arcs_warm_cool_left(lighting_control->warm_intensity_container2, lighting_control->warm_intensity_slider1);
    create_arcs_warm_cool_right(lighting_control->warm_intensity_container2, lighting_control->warm_intensity_slider1);

    lighting_control->button_container = lv_obj_create(lighting_control->warm_cool_screen);
    lv_obj_set_size(lighting_control->button_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(lighting_control->button_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(lighting_control->button_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(lighting_control->button_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->button_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->button_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    // Cool button
    lighting_control->cool_btn = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->cool_btn, 80, 40);
    lv_obj_set_style_bg_color(lighting_control->cool_btn, lv_color_hex(0xADD8E6), 0);
    lv_obj_add_event_cb(lighting_control->cool_btn, cool_btn_event_cb, LV_EVENT_CLICKED, NULL);
    // Cool button
    lighting_control->cool_btn1 = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->cool_btn1, 80, 40);
    lv_obj_add_flag(lighting_control->cool_btn1, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_bg_color(lighting_control->cool_btn1, lv_color_hex(0xADD8E6), 0);
    lv_obj_add_event_cb(lighting_control->cool_btn1, cool_btn_event_cb1, LV_EVENT_CLICKED, NULL);

    lv_obj_t *cool_label = lv_label_create(lighting_control->cool_btn);
    lv_label_set_text(cool_label, "Cool");
    lv_obj_set_style_text_color(cool_label, lv_color_hex(0x000000), 0);

    lv_obj_center(cool_label);

    lv_obj_t *cool_label1 = lv_label_create(lighting_control->cool_btn1);
    lv_label_set_text(cool_label1, "Cool");
    lv_obj_set_style_text_color(cool_label1, lv_color_hex(0x000000), 0);

    lv_obj_center(cool_label1);

    // Both button
    lighting_control->both_btn = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->both_btn, 100, 40);
    lv_obj_set_style_bg_color(lighting_control->both_btn, lv_color_hex(0xFFF4E5), 0);
    lv_obj_add_event_cb(lighting_control->both_btn, both_btn_event_cb, LV_EVENT_CLICKED, NULL);
    // Both button
    lighting_control->both_btn1 = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->both_btn1, 100, 40);
    lv_obj_add_flag(lighting_control->both_btn1, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_bg_color(lighting_control->both_btn1, lv_color_hex(0xFFF4E5), 0);
    lv_obj_add_event_cb(lighting_control->both_btn1, both_btn_event_cb1, LV_EVENT_CLICKED, NULL);

    lv_obj_t *both_label = lv_label_create(lighting_control->both_btn);
    lv_label_set_text(both_label, "Natural");
    lv_obj_set_style_text_color(both_label, lv_color_hex(0x000000), 0);
    lv_obj_center(both_label);
    lv_obj_t *both_label1 = lv_label_create(lighting_control->both_btn1);
    lv_label_set_text(both_label1, "Natural");
    lv_obj_set_style_text_color(both_label1, lv_color_hex(0x000000), 0);
    lv_obj_center(both_label1);
    // Warm button
    lighting_control->warm_btn = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->warm_btn, 80, 40);
    lv_obj_set_style_bg_color(lighting_control->warm_btn, lv_color_hex(0xFFA500), 0);
    lv_obj_add_event_cb(lighting_control->warm_btn, warm_btn_event_cb, LV_EVENT_CLICKED, NULL);
    // Warm button
    lighting_control->warm_btn1 = lv_btn_create(lighting_control->button_container);
    lv_obj_set_size(lighting_control->warm_btn1, 80, 40);
    lv_obj_add_flag(lighting_control->warm_btn1, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_bg_color(lighting_control->warm_btn1, lv_color_hex(0xFFA500), 0);
    lv_obj_add_event_cb(lighting_control->warm_btn1, warm_btn_event_cb1, LV_EVENT_CLICKED, NULL);

    lv_obj_t *warm_label = lv_label_create(lighting_control->warm_btn);
    lv_label_set_text(warm_label, "Warm");
    lv_obj_set_style_text_color(warm_label, lv_color_hex(0x000000), 0);

    lv_obj_center(warm_label);
    lv_obj_t *warm_label1 = lv_label_create(lighting_control->warm_btn1);
    lv_label_set_text(warm_label1, "Warm");
    lv_obj_set_style_text_color(warm_label1, lv_color_hex(0x000000), 0);

    lv_obj_center(warm_label1);

    lighting_control->tuning_screen = lv_obj_create(NULL);
    lv_obj_add_event_cb(lighting_control->tuning_screen, screen_slide_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_set_size(lighting_control->tuning_screen, LV_PCT(100), LV_PCT(100)); // Full size container
    lv_obj_set_flex_flow(lighting_control->tuning_screen, LV_FLEX_FLOW_COLUMN);
    // Set padding and spacing
    lv_obj_set_style_pad_row(lighting_control->tuning_screen, 5, 0);                                                                // Add space between rows
    lv_obj_set_style_pad_top(lighting_control->tuning_screen, 2, 0);                                                                // Add space at the top
    lv_obj_set_style_pad_bottom(lighting_control->tuning_screen, 2, 0);                                                             // Add                                                     // Optional: Flexbox layout
    lv_obj_set_flex_align(lighting_control->tuning_screen, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // Space sliders evenly
    lv_obj_set_style_bg_color(lighting_control->tuning_screen, lv_color_hex(0xffdbba), LV_PART_MAIN | LV_STATE_DEFAULT);
    lighting_control->rgb_button = lv_btn_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->rgb_button, 120, 40);                // Set width and height
    lv_obj_align(lighting_control->rgb_button, LV_ALIGN_BOTTOM_MID, 0, 5); // Position at bottom, 20px margin
    lv_obj_clear_flag(lighting_control->rgb_button, LV_OBJ_FLAG_SCROLLABLE);
    btn_label = lv_label_create(lighting_control->rgb_button);

    lv_label_set_text(btn_label, "Settings"); // Set button text
    lv_obj_center(btn_label);                 // Center the label in the button
    lv_obj_clear_flag(btn_label, LV_OBJ_FLAG_SCROLLABLE);

    // Style the button (optional)
    lv_obj_set_style_bg_color(lighting_control->rgb_button, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT); // Blue color
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);                  // White text

    // Add click event handler
    lv_obj_add_event_cb(lighting_control->rgb_button, rgb_button_event_cb, LV_EVENT_CLICKED, NULL);

    // Channel 1
    lighting_control->channel1_container = lv_obj_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->channel1_container, 380, 60);
    lv_obj_set_flex_flow(lighting_control->channel1_container, LV_FLEX_FLOW_ROW); // Set flow to row
    lv_obj_set_flex_align(lighting_control->channel1_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->channel1_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->channel1_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->channel1_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->channel1_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->channel1_container, &style_container, LV_PART_MAIN);

    lighting_control->channel1_label = lv_label_create(lighting_control->channel1_container);
    lv_label_set_text(lighting_control->channel1_label, "Ch 1");
    lv_obj_set_style_pad_right(lighting_control->channel1_label, 10, 0); // Add padding from left edge

    create_arcs_warm_cool_left(lighting_control->channel1_container, lighting_control->channel1_label);

    lighting_control->tuning_slider = lv_slider_create(lighting_control->channel1_container);
    lv_obj_set_size(lighting_control->tuning_slider, 200, 10);
    // lv_obj_align(lighting_control->tuning_slider, LV_ALIGN_TOP_MID, 0, 30); // Positioning at top
    lv_slider_set_range(lighting_control->tuning_slider, 0, 255);

    lv_obj_center(lighting_control->tuning_slider);
    // Move slider up
    lv_obj_set_style_bg_color(lighting_control->tuning_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->tuning_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label);
    lv_style_set_bg_color(&style_percentage_label, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label, &lv_font_montserrat_14);

    percentage_label = lv_label_create(lighting_control->channel1_container);
    lv_obj_add_style(percentage_label, &style_percentage_label, 0);
    lv_obj_add_flag(percentage_label, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label);
    update_label_pos(lighting_control->tuning_slider, percentage_label);
    // Position the label above the slider
    lv_obj_align_to(percentage_label, lighting_control->tuning_slider, LV_ALIGN_OUT_TOP_MID, 0, -10);

    lv_obj_add_event_cb(lighting_control->tuning_slider, tuning_slider_event_cb2, LV_EVENT_VALUE_CHANGED, NULL);
    create_arcs_warm_cool_right(lighting_control->channel1_container, lighting_control->tuning_slider);

    lighting_control->channel2_container = lv_obj_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->channel2_container, 380, 60);
    lv_obj_set_flex_flow(lighting_control->channel2_container, LV_FLEX_FLOW_ROW); // Set flow to row
    lv_obj_set_flex_align(lighting_control->channel2_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->channel2_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->channel2_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->channel2_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->channel2_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->channel2_container, &style_container, LV_PART_MAIN);

    lighting_control->channel2_label = lv_label_create(lighting_control->channel2_container);
    lv_label_set_text(lighting_control->channel2_label, "Ch 2");
    lv_obj_set_style_pad_right(lighting_control->channel2_label, 10, 0); // Add padding from left edge
    create_arcs_warm_cool_left(lighting_control->channel2_container, lighting_control->tuning_slider1);

    lighting_control->tuning_slider1 = lv_slider_create(lighting_control->channel2_container);
    lv_obj_set_size(lighting_control->tuning_slider1, 200, 10);
    lv_slider_set_range(lighting_control->tuning_slider1, 0, 255);
    lv_obj_center(lighting_control->tuning_slider1);
    // Move slider up
    lv_obj_set_style_bg_color(lighting_control->tuning_slider1, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider1, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider1, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider1, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->tuning_slider1, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider1, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider1, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider1, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label1);
    lv_style_set_bg_color(&style_percentage_label1, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label1, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label1, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label1, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label1, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label1, &lv_font_montserrat_14);

    percentage_label1 = lv_label_create(lighting_control->channel2_container);
    lv_obj_add_style(percentage_label1, &style_percentage_label1, 0);
    lv_obj_add_flag(percentage_label1, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label1, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label1);
    update_label_pos(lighting_control->tuning_slider1, percentage_label1);
    // Position the label above the slider
    lv_obj_align_to(percentage_label1, lighting_control->tuning_slider1, LV_ALIGN_OUT_TOP_MID, 0, -10);

    // lv_obj_align(lighting_control->tuning_slider1, LV_ALIGN_TOP_MID, 0, 100); // Positioning below the first
    lv_obj_add_event_cb(lighting_control->tuning_slider1, tuning_slider_event_cb3, LV_EVENT_VALUE_CHANGED, NULL);
    create_arcs_warm_cool_right(lighting_control->channel2_container, lighting_control->tuning_slider1);

    lighting_control->All_channel_container = lv_obj_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->All_channel_container, 440, 60);

    lv_obj_set_flex_flow(lighting_control->All_channel_container, LV_FLEX_FLOW_ROW); // Set flow to row
    lv_obj_set_flex_align(lighting_control->All_channel_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->All_channel_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->All_channel_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->All_channel_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->All_channel_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->All_channel_container, &style_container, LV_PART_MAIN);
    // lv_obj_add_flag(lighting_control->All_channel_container, LV_OBJ_FLAG_HIDDEN);

    lighting_control->All_channel_label = lv_label_create(lighting_control->All_channel_container);
    lv_label_set_text(lighting_control->All_channel_label, " All ");
    // lv_obj_add_flag(lighting_control->All_channel_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_pad_right(lighting_control->All_channel_label, 10, 0); // Move text left by 20 pixels
    create_arcs_warm_cool_left(lighting_control->All_channel_container, lighting_control->All_tuning_slider);

    lighting_control->All_tuning_slider = lv_slider_create(lighting_control->All_channel_container);
    lv_obj_set_size(lighting_control->All_tuning_slider, 250, 20);

    lv_slider_set_range(lighting_control->All_tuning_slider, 0, 255);
    lv_obj_center(lighting_control->All_tuning_slider);
    // Move slider up
    lv_obj_set_style_bg_color(lighting_control->All_tuning_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->All_tuning_slider, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->All_tuning_slider, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->All_tuning_slider, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->All_tuning_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->All_tuning_slider, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->All_tuning_slider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->All_tuning_slider, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label2);
    lv_style_set_bg_color(&style_percentage_label2, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label2, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label2, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label2, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label2, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label2, &lv_font_montserrat_14);

    percentage_label2 = lv_label_create(lighting_control->All_channel_container);
    lv_obj_add_style(percentage_label2, &style_percentage_label2, 0);
    lv_obj_add_flag(percentage_label2, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label2, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label2);
    update_label_pos(lighting_control->All_tuning_slider, percentage_label2);
    // Position the label above the slider
    lv_obj_align_to(percentage_label2, lighting_control->All_tuning_slider, LV_ALIGN_OUT_TOP_MID, 0, -10);

    lv_obj_add_event_cb(lighting_control->All_tuning_slider, tuning_slider_event_cb1, LV_EVENT_VALUE_CHANGED, NULL);
    create_arcs_warm_cool_right(lighting_control->All_channel_container, lighting_control->All_tuning_slider);

    // Channel 3
    lighting_control->channel3_container = lv_obj_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->channel3_container, 380, 60);
    lv_obj_set_flex_flow(lighting_control->channel3_container, LV_FLEX_FLOW_ROW); // Set flow to row
    lv_obj_set_flex_align(lighting_control->channel3_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->channel3_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->channel3_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->channel3_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->channel3_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->channel3_container, &style_container, LV_PART_MAIN);

    lighting_control->channel3_label = lv_label_create(lighting_control->channel3_container);
    lv_label_set_text(lighting_control->channel3_label, "Ch 3");
    lv_obj_set_style_pad_right(lighting_control->channel3_label, 10, 0); // Move text left by 20 pixels
    create_arcs_warm_cool_left(lighting_control->channel3_container, lighting_control->tuning_slider2);

    lighting_control->tuning_slider2 = lv_slider_create(lighting_control->channel3_container);
    lv_obj_set_size(lighting_control->tuning_slider2, 200, 10);
    lv_slider_set_range(lighting_control->tuning_slider2, 0, 255);
    lv_obj_center(lighting_control->tuning_slider2);
    // Move slider up
    lv_obj_set_style_bg_color(lighting_control->tuning_slider2, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider2, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider2, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider2, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->tuning_slider2, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider2, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider2, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider2, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label3);
    lv_style_set_bg_color(&style_percentage_label3, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label3, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label3, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label3, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label3, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label3, &lv_font_montserrat_14);

    percentage_label3 = lv_label_create(lighting_control->channel3_container);
    lv_obj_add_style(percentage_label3, &style_percentage_label3, 0);
    lv_obj_add_flag(percentage_label3, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label3, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label3);
    update_label_pos(lighting_control->tuning_slider2, percentage_label3);
    // Position the label above the slider
    lv_obj_align_to(percentage_label3, lighting_control->tuning_slider2, LV_ALIGN_OUT_TOP_MID, 0, -10);

    // lv_obj_align(lighting_control->tuning_slider2, LV_ALIGN_BOTTOM_MID, 0, 160); // Positioning below the first
    lv_obj_add_event_cb(lighting_control->tuning_slider2, tuning_slider_event_cb4, LV_EVENT_VALUE_CHANGED, NULL);
    create_arcs_warm_cool_right(lighting_control->channel3_container, lighting_control->tuning_slider2);

    // Channel 4
    lighting_control->channel4_container = lv_obj_create(lighting_control->tuning_screen);
    lv_obj_set_size(lighting_control->channel4_container, 380, 60);
    lv_obj_set_flex_flow(lighting_control->channel4_container, LV_FLEX_FLOW_ROW); // Set flow to row
    lv_obj_set_flex_align(lighting_control->channel4_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(lighting_control->channel4_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lighting_control->channel3_container, lv_color_hex(0xdc7a2a), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->channel4_container, lv_color_hex(0x7b5637), LV_ANIM_IMG_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->channel4_container, LV_GRAD_DIR_HOR, LV_ANIM_IMG_PART_MAIN);

    lv_style_init(&style_container);
    lv_style_set_radius(&style_container, 30); // Rounded corners (half of container height)
    lv_style_set_bg_color(&style_container, lv_color_hex(0xdc7a2a));
    lv_style_set_bg_grad_color(&style_container, lv_color_hex(0x7b5637));
    lv_style_set_bg_grad_dir(&style_container, LV_GRAD_DIR_HOR);
    lv_style_set_pad_left(&style_container, 10);  // Add padding for label
    lv_style_set_pad_right(&style_container, 10); // Add padding for symmetry
    lv_style_set_pad_top(&style_container, 8);    // Add vertical padding
    lv_style_set_pad_bottom(&style_container, 12);
    lv_obj_add_style(lighting_control->channel4_container, &style_container, LV_PART_MAIN);

    lighting_control->channel4_label = lv_label_create(lighting_control->channel4_container);
    lv_label_set_text(lighting_control->channel4_label, "Ch 4");
    lv_obj_set_style_pad_right(lighting_control->channel4_label, 10, 0); // Move text left by 20 pixels
    create_arcs_warm_cool_left(lighting_control->channel4_container, lighting_control->tuning_slider3);

    lighting_control->tuning_slider3 = lv_slider_create(lighting_control->channel4_container);
    lv_obj_set_size(lighting_control->tuning_slider3, 200, 10);
    lv_slider_set_range(lighting_control->tuning_slider3, 0, 255);
    lv_obj_center(lighting_control->tuning_slider3);
    // Move slider up
    lv_obj_set_style_bg_color(lighting_control->tuning_slider3, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider3, lv_color_hex(0xFFffff), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider3, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider3, LV_OPA_COVER, LV_PART_MAIN); // Ensure solid colors

    // Set the indicator (filled part) to use the same gradient
    lv_obj_set_style_bg_color(lighting_control->tuning_slider3, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(lighting_control->tuning_slider3, lv_color_hex(0x5da2f1), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(lighting_control->tuning_slider3, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(lighting_control->tuning_slider3, LV_OPA_COVER, LV_PART_INDICATOR); // Solid color indicator

    lv_style_init(&style_percentage_label4);
    lv_style_set_bg_color(&style_percentage_label4, lv_color_hex(0x87CEEB)); // Light blue background
    lv_style_set_bg_opa(&style_percentage_label4, LV_OPA_COVER);
    lv_style_set_radius(&style_percentage_label4, 5);                          // Rounded corners
    lv_style_set_pad_all(&style_percentage_label4, 5);                         // Padding around text
    lv_style_set_text_color(&style_percentage_label4, lv_color_hex(0xFFFFFF)); // White text
    lv_style_set_text_font(&style_percentage_label4, &lv_font_montserrat_14);

    percentage_label4 = lv_label_create(lighting_control->channel4_container);
    lv_obj_add_style(percentage_label4, &style_percentage_label4, 0);
    lv_obj_add_flag(percentage_label4, LV_OBJ_FLAG_HIDDEN); // Hidden by default
    lv_label_set_text(percentage_label4, "0%");

    // Make sure the label is drawn above other elements
    lv_obj_move_foreground(percentage_label4);
    update_label_pos(lighting_control->tuning_slider3, percentage_label4);
    // Position the label above the slider
    lv_obj_align_to(percentage_label4, lighting_control->tuning_slider3, LV_ALIGN_OUT_TOP_MID, 0, -10);

    // lv_obj_align(lighting_control->tuning_slider3, LV_ALIGN_BOTTOM_MID, 0, 220); // Positioning below the second
    lv_obj_add_event_cb(lighting_control->tuning_slider3, tuning_slider_event_cb5, LV_EVENT_VALUE_CHANGED, NULL);
    create_arcs_warm_cool_right(lighting_control->channel4_container, lighting_control->tuning_slider3);

    lv_obj_add_event_cb(lighting_control->rgb_wheel, generic_control_event_cb, LV_EVENT_ALL, rgb_wheel_event_cb);
    lv_obj_add_event_cb(lighting_control->warm_cool_slider, generic_control_event_cb, LV_EVENT_ALL, warm_cool_slider_event_cb);
    lv_obj_add_event_cb(lighting_control->warm_cool_slider1, generic_control_event_cb, LV_EVENT_ALL, warm_cool_slider_event_cb1);

    lv_obj_add_event_cb(lighting_control->rgb_intensity_slider, generic_control_event_cb, LV_EVENT_ALL, rgb_intensity_slider_event_cb);
    lv_obj_add_event_cb(lighting_control->tuning_slider, generic_control_event_cb, LV_EVENT_ALL, tuning_slider_event_cb2);
    lv_obj_add_event_cb(lighting_control->tuning_slider1, generic_control_event_cb, LV_EVENT_ALL, tuning_slider_event_cb3);
    lv_obj_add_event_cb(lighting_control->tuning_slider2, generic_control_event_cb, LV_EVENT_ALL, tuning_slider_event_cb4);
    lv_obj_add_event_cb(lighting_control->tuning_slider3, generic_control_event_cb, LV_EVENT_ALL, tuning_slider_event_cb5);
    lv_obj_add_event_cb(lighting_control->All_tuning_slider, generic_control_event_cb, LV_EVENT_ALL, tuning_slider_event_cb1);
    lv_obj_add_event_cb(lighting_control->warm_intensity_slider, generic_control_event_cb, LV_EVENT_ALL, intensity_slider_event_cb);
    lv_obj_add_event_cb(lighting_control->warm_intensity_slider1, generic_control_event_cb, LV_EVENT_ALL, intensity_slider_event_cb1);

    lv_scr_load(lighting_control->main_screen);
}