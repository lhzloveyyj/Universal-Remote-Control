#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include "esp_lcd_gc9a01.h"   // 屏幕驱动
#include "lv_demos.h"

// Using SPI2 
#define LCD_HOST SPI2_HOST

// 屏幕尺寸
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 240

// 引脚接线
// | Name | Description            				| GPIO   |
// | BLK  | 背光，悬空使能接地关闭，默认上拉至3.3V 	  | 6     |
// | CS   | 片选，低电平使能               	       | 15      |
// | DC   | 数据/命令选择，低电平命令，高电平数据     | 7     |
// | RES  | 复位，低电平使能               	  	   | 18     |
// | SDA  | SPI数据输入端口              		  | 17     |
// | SCL  | SPI时钟信号输入端口            		  | 16     |
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT      32
#define EXAMPLE_PIN_NUM_LCD_CS 15
#define EXAMPLE_PIN_NUM_LCD_DC 7
#define EXAMPLE_PIN_NUM_LCD_RST 33
#define EXAMPLE_PIN_NUM_DATA0 17
#define EXAMPLE_PIN_NUM_SCLK 16

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2 // LVGL tick period in milliseconds
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS  100

#define lvgl_task_stack_size (5 * 1024) // LVGL task stack size
#define lvgl_task_priority 2 // LVGL task priority

#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS  500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS  1

static const char *TAG = "gc9a01_test";

static SemaphoreHandle_t lvgl_mutex = NULL; // LVGL mutex for thread safety


static const gc9a01_lcd_init_cmd_t lcd_init_cmds[] = {
//  {cmd, { data }, data_size, delay_ms}
    // Enable Inter Register
    {0xfe, (uint8_t []){0x00}, 0, 0},
    {0xef, (uint8_t []){0x00}, 0, 0},
    {0xeb, (uint8_t []){0x14}, 1, 0},
    {0x84, (uint8_t []){0x60}, 1, 0},
    {0x85, (uint8_t []){0xff}, 1, 0},
    {0x86, (uint8_t []){0xff}, 1, 0},
    {0x87, (uint8_t []){0xff}, 1, 0},
    {0x8e, (uint8_t []){0xff}, 1, 0},
    {0x8f, (uint8_t []){0xff}, 1, 0},
    {0x88, (uint8_t []){0x0a}, 1, 0},
    {0x89, (uint8_t []){0x23}, 1, 0},
    {0x8a, (uint8_t []){0x00}, 1, 0},
    {0x8b, (uint8_t []){0x80}, 1, 0},
    {0x8c, (uint8_t []){0x01}, 1, 0},
    {0x8d, (uint8_t []){0x03}, 1, 0},
    {0x90, (uint8_t []){0x08, 0x08, 0x08, 0x08}, 4, 0},
    {0xff, (uint8_t []){0x60, 0x01, 0x04}, 3, 0},
    {0xC3, (uint8_t []){0x13}, 1, 0},
    {0xC4, (uint8_t []){0x13}, 1, 0},
    {0xC9, (uint8_t []){0x30}, 1, 0},
    {0xbe, (uint8_t []){0x11}, 1, 0},
    {0xe1, (uint8_t []){0x10, 0x0e}, 2, 0},
    {0xdf, (uint8_t []){0x21, 0x0c, 0x02}, 3, 0},
    // Set gamma
    {0xF0, (uint8_t []){0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6, 0},
    {0xF1, (uint8_t []){0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6, 0},
    {0xF2, (uint8_t []){0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6, 0},
    {0xF3, (uint8_t []){0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6, 0},
    {0xed, (uint8_t []){0x1b, 0x0b}, 2, 0},
    {0xae, (uint8_t []){0x77}, 1, 0},
    {0xcd, (uint8_t []){0x63}, 1, 0},
    {0x70, (uint8_t []){0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09, 0x07, 0x08, 0x03}, 9, 0},
    {0xE8, (uint8_t []){0x34}, 1, 0}, // 4 dot inversion
    {0x60, (uint8_t []){0x38, 0x0b, 0x6D, 0x6D, 0x39, 0xf0, 0x6D, 0x6D}, 8, 0},
    {0x61, (uint8_t []){0x38, 0xf4, 0x6D, 0x6D, 0x38, 0xf7, 0x6D, 0x6D}, 8, 0},
    {0x62, (uint8_t []){0x38, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x38, 0x0F, 0x71, 0xEF, 0x70, 0x70}, 12, 0},
    {0x63, (uint8_t []){0x38, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x38, 0x13, 0x71, 0xF3, 0x70, 0x70}, 12, 0},
    {0x64, (uint8_t []){0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}, 7, 0},
    {0x66, (uint8_t []){0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10, 0},
    {0x67, (uint8_t []){0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10, 0},
    {0x74, (uint8_t []){0x10, 0x45, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7, 0},
    {0x98, (uint8_t []){0x3e, 0x07}, 2, 0},
    {0x99, (uint8_t []){0x3e, 0x07}, 2, 0},
};

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
	lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx; // 获取显示驱动
	// 通知LVGL刷新完成
	lv_disp_flush_ready(disp_drv);

	return false; // 返回false表示不需要继续处理
}

// 回调函数:刷新屏幕
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    /* Copy a buffer's content to a specific area of the display */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

}

static void increase_lvgl_tick(void *arg)
{
	// 增加LVGL的tick计数
	lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mutex && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mutex, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mutex && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mutex);
}

void test_ui(void) {
    ESP_LOGI(TAG, "Create test UI");

    lv_obj_t *scr = lv_disp_get_scr_act(NULL);

    lv_obj_t *rect = lv_obj_create(scr);
    lv_obj_set_size(rect, 120, 60);
    lv_obj_align(rect, LV_ALIGN_CENTER, 0, -40);

    lv_obj_set_style_bg_color(rect, lv_color_hex(0x0000FF), LV_PART_MAIN);      // 蓝色背景
    lv_obj_set_style_border_color(rect, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // 白色边框
    lv_obj_set_style_border_width(rect, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(rect, 10, LV_PART_MAIN);  // 圆角

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Hello, LCD!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 40);
}


static void lvgl_port_task(void *arg)
{
	ESP_LOGI(TAG, "Starting LVGL task");
    ESP_LOGI(TAG, "Display LVGL UI");

	test_ui();
	//lv_demo_widgets();
	//lv_demo_music();

    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
		ESP_LOGI(TAG, "LVGL task");
        /* Lock the mutex due to the LVGL APIs are not thread-safe */
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            /* Release the mutex */
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void)
{
	static lv_disp_drv_t 		disp_drv; // 显示驱动
	static lv_disp_draw_buf_t 	draw_buf; // 显示缓冲区

	/* =============== Initialize SPI bus =============== */
	ESP_LOGI(TAG, "Initialize SPI bus");

	const spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_SCLK, EXAMPLE_PIN_NUM_DATA0, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
	ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

	/* =============== Install panel IO =============== */
	ESP_LOGI(TAG, "Install panel IO");

	esp_lcd_panel_io_handle_t io_handle = NULL;
	const esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_DC, notify_lvgl_flush_ready, &disp_drv);

	/* Attach the LCD to the SPI bus */
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

	/* =============== Install gc9a01 panel driver =============== */
	ESP_LOGI(TAG, "Install gc9a01 panel driver");

	esp_lcd_panel_handle_t panel_handle = NULL;
	gc9a01_vendor_config_t vendor_config = {  
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
	};
	 const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,      // Set to -1 if not use
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,                           // Implemented by LCD command `3Ah` (16/18)
        //.vendor_config = &vendor_config,            // Uncomment this line if use custom initialization commands
    };
    // 创建屏幕实例
	ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
	// 屏幕复位
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	// 初始化屏幕
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	// 反转颜色
	ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true)); 
	// 镜像
	ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false)); 
	// 打开屏幕
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
	// 打开背光
	ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));

	 /* =============== Initialize LVGL =============== */
	ESP_LOGI(TAG, "Initialize LVGL");

	lv_init();
	 /* Alloc draw buffers used by LVGL */
    /* It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
	lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf1 != NULL); // Ensure memory allocation was successful
	lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf2 != NULL); // Ensure memory allocation was successful
	/* Initialize LVGL draw buffers */
	lv_disp_draw_buf_init(&draw_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 50); // Initialize display buffer

	/* =============== Register display driver to LVGL =============== */
	ESP_LOGI(TAG, "Initialize display driver for LVGL");

	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = EXAMPLE_LCD_H_RES; // Horizontal resolution
	disp_drv.ver_res = EXAMPLE_LCD_V_RES; // Vertical resolution
	disp_drv.flush_cb = lvgl_flush_cb; // Flush callback function
	disp_drv.draw_buf = &draw_buf; // Set the display buffer
	disp_drv.user_data = panel_handle; // Set user data
	lv_disp_t *disp = lv_disp_drv_register(&disp_drv); // Register the display driver

	/* =============== Register touch driver to LVGL =============== */
	ESP_LOGI(TAG,"Initialize LVGL tick timer");

	/* Tick interface for LVGL (using esp_timer to generate 2ms periodic event) */
	const esp_timer_create_args_t timer_args = {
		.callback = increase_lvgl_tick, // LVGL tick task callback
		.name = "lvgl_tick", // Timer name
	};
	esp_timer_handle_t lvgl_tick_timer = NULL;
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &lvgl_tick_timer)); // Create the timer
	ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000)); // Start the timer with a period of 10 ms

	/* =============== Create LVGL Task =============== */
	ESP_LOGI(TAG, "Run LVGL Task");

	lvgl_mutex = xSemaphoreCreateMutex();
	assert(lvgl_mutex != NULL); // Ensure mutex creation was successful
	xTaskCreate(lvgl_port_task, "lv_task_handler", lvgl_task_stack_size, NULL, lvgl_task_priority, NULL);


}
