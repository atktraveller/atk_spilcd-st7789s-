#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "atk_spilcd_st7789s.h"
#include <stdio.h>

/* IO declaration */
#define LCD_SCLK_PIN        GPIO_NUM_30 /* LCD SCLK */
#define LCD_MOSI_PIN        GPIO_NUM_29 /* LCD MOSI */
#define LCD_MISO_PIN        GPIO_NUM_27 /* LCD MISO */
#define LCD_RST_PIN         GPIO_NUM_0  /* LCD RTS */
#define LCD_PWR_PIN         GPIO_NUM_1  /* LCD BL */
#define LCD_DC_PIN          GPIO_NUM_31 /* LCD WD(DC) */
#define LCD_CS_PIN          GPIO_NUM_28 /* LCD CS */

#define LCD_H_RES           240         /* width */
#define LCD_V_RES           320         /* height */

#define LCD_HOST            SPI2_HOST   /* SPI PORT */

/* 16 bit color value */
#define WHITE           0xFFFF
#define BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x001F
#define MAGENTA         0XF81F
#define YELLOW          0XFFE0
#define CYAN            0X07FF
#define BROWN           0XBC40
#define BRRED           0XFC07
#define GRAY            0X8430
#define DARKBLUE        0X01CF
#define LIGHTBLUE       0X7D7C
#define GRAYBLUE        0X5458
#define LIGHTGREEN      0X841F
#define LGRAY           0XC618 
#define LGRAYBLUE       0XA651
#define LBBLUE          0X2B12

/* Screen parameter configuration */
typedef struct
{
    uint32_t pwidth;    /* Temporary setting value (width) */
    uint32_t pheight;   /* Temporary setting value (height) */
    uint8_t dir;        /* orientation */
    uint16_t width;     /* width */
    uint16_t height;    /* height */
} _spilcd_dev; 

_spilcd_dev spilcddev;

esp_lcd_panel_handle_t panel_handle = NULL;

/**
 * @brief       Set screen orientation
 * @param       dir: 0 indicates portrait screen and 1 indicates landscape screen
 * @retval      None
 */
void spilcd_display_dir(uint8_t dir)
{
    spilcddev.dir = dir;

    if (spilcddev.dir == 0)
    {
        spilcddev.width = spilcddev.pheight;
        spilcddev.height = spilcddev.pwidth;
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
    }
    else if (spilcddev.dir == 1)
    {
        spilcddev.width = spilcddev.pwidth;
        spilcddev.height = spilcddev.pheight;
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
    }
}

/**
 * @brief       SPILCD initializes
 * @param       无
 * @retval      ESP_OK:Successful initialization
 */
esp_err_t spilcd_init(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK_PIN,
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = LCD_MISO_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_V_RES * 80 * sizeof(uint16_t),
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC_PIN,      /* DC IO */
        .cs_gpio_num = LCD_CS_PIN,      /* CS IO */
        .pclk_hz = (20 * 1000 * 1000),  /* PCLK 20MHz */
        /* Bit number used to represent command and parameter */
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,                  /* mode */
        .trans_queue_depth = 7,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    spilcddev.pheight = LCD_H_RES;
    spilcddev.pwidth = LCD_V_RES;

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST_PIN,                  /* RTS IO */
        .rgb_ele_order = COLOR_RGB_ELEMENT_ORDER_RGB,   /* RGB/BGR */
        .bits_per_pixel = 16,                           /* color depth */
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789s(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    spilcd_display_dir(1);

    return ESP_OK;
}

/**
 * @brief       clear screen
 * @param       color: color value
 * @retval      None
 */
void spilcd_clear(uint16_t color)
{
    uint16_t *buffer = heap_caps_malloc(spilcddev.width * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    color = SPI_SWAP_DATA_TX(color, 16);

    if (NULL == buffer)
    {
        ESP_LOGE("TAG", "Memory for bitmap is not enough");
    }
    else
    {
        for (uint16_t i = 0; i < spilcddev.width; i++)
        {
            buffer[i] = color;
        }
        
        for (uint16_t y = 0; y < spilcddev.height; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, spilcddev.width, y + 1, buffer);
        }
        
        heap_caps_free(buffer);
    }
}

void app_main(void)
{
    uint8_t x = 0;
    gpio_config_t gpio_init_struct = {0};

    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT;
    gpio_init_struct.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_init_struct.pin_bit_mask = 1ull << LCD_PWR_PIN;
    ESP_ERROR_CHECK(gpio_config(&gpio_init_struct));

    spilcd_init();  /* SPILCD initializes */

    gpio_set_level(LCD_PWR_PIN,1);

    while(1)
    {
        switch (x)
        {
            case 0:
            {
                spilcd_clear(WHITE);
                break;
            }
            case 1:
            {
                spilcd_clear(BLACK);
                break;
            }
            case 2:
            {
                spilcd_clear(BLUE);
                break;
            }
            case 3:
            {
                spilcd_clear(RED);
                break;
            }
            case 4:
            {
                spilcd_clear(MAGENTA);
                break;
            }
            case 5:
            {
                spilcd_clear(GREEN);
                break;
            }
            case 6:
            {
                spilcd_clear(CYAN);
                break;
            }
            case 7:
            {
                spilcd_clear(YELLOW);
                break;
            }
            case 8:
            {
                spilcd_clear(BRRED);
                break;
            }
            case 9:
            {
                spilcd_clear(GRAY);
                break;
            }
            case 10:
            {
                spilcd_clear(LGRAY);
                break;
            }
            case 11:
            {
                spilcd_clear(BROWN);
                break;
            }
        }

        x++;

        if (x == 12)
        {
            x = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
