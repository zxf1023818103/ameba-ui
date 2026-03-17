#include <ameba_soc.h>
#include <os_wrapper.h>
#include <os_wrapper_time.h>
#include <spi_api.h>
#include <spi_ex_api.h>
#include <gpio_api.h>

#include <stdlib.h>
#include <lvgl.h>

#define WIDTH 320
#define HEIGHT 172

#define X_OFFSET 0
#define Y_OFFSET 34

#define SPI_SCLK_PIN _PB_18 //SCL
#define SPI_MOSI_PIN _PB_19 //SDA
#define SPI_MISO_PIN _PB_20
#define RST PA_26 //RES
#define SPI_CS_PIN _PB_17

static rtos_sema_t spi_tx_done_sem;
static uint8_t g_buffer_0[WIDTH * HEIGHT * 2];
static uint16_t g_line_buf[WIDTH * 2 * 4 + 1] __attribute__((aligned(CACHE_LINE_SIZE)));
static spi_t spi_lcd;
static gpio_t gpio_reset;

static void write_reg(uint8_t reg, uint8_t *value, size_t len)
{
    g_line_buf[0] = reg;
    for (size_t i = 0; i < len; i++) {
        uint16_t data = value[i];
        data |= 0x100;
        g_line_buf[i + 1] = data;
    }
    spi_master_write_stream_dma(&spi_lcd, g_line_buf, 2 * (len + 1));
    rtos_sema_take(spi_tx_done_sem, RTOS_SEMA_MAX_COUNT);
}

#define WR_REG(reg) write_reg(reg, NULL, 0)

#define WR_DATA(reg, ...) do { uint8_t data[] = { __VA_ARGS__ }; write_reg(reg, data, sizeof data); } while (0)

static void st7789v_config_init(void)
{
    /// HW Reset
    gpio_write(&gpio_reset, 1);
    rtos_time_delay_ms(100);
    gpio_write(&gpio_reset, 0);
    rtos_time_delay_ms(100);
    gpio_write(&gpio_reset, 1);
    rtos_time_delay_ms(120);

    /// SW Reset
    WR_REG(0x11);
    rtos_time_delay_ms(120);

    WR_DATA(0x36, 0xa0);
    WR_DATA(0x3a, 0x05);
    WR_DATA(0xb2, 0x0c, 0x0c, 0x00, 0x33, 0x33);
    WR_DATA(0xb7, 0x35);
    WR_DATA(0xbb, 0x35);
    WR_DATA(0xc0, 0x2c);
    WR_DATA(0xc2, 0x01);
    WR_DATA(0xc3, 0x13);
    WR_DATA(0xc4, 0x20);
    WR_DATA(0xc6, 0x0f);
    WR_DATA(0xd0, 0xa4, 0xa1);
    WR_DATA(0xd6, 0xa1);
    WR_DATA(0xe0, 0xf0, 0x00, 0x04, 0x04, 0x04, 0x05, 0x29, 0x33, 0x3e, 0x38, 0x12, 0x12, 0x28, 0x30);
    WR_DATA(0xe1, 0xf0, 0x07, 0x0a, 0x0d, 0x0b, 0x07, 0x28, 0x33, 0x3e, 0x36, 0x14, 0x14, 0x29, 0x32);

    WR_REG(0x21);
    WR_REG(0x29);
}

static void spi_comp_handler(uint32_t id, SpiIrq event)
{
    (void) id;
    if (event == SpiTxIrq) {
        rtos_sema_give(spi_tx_done_sem);
    }
}

static void st7789v_init(void)
{
    Pinmux_Swdoff();

    rtos_sema_create_binary(&spi_tx_done_sem);

    spi_lcd.spi_idx = MBED_SPI1;
    spi_init(&spi_lcd, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN, SPI_CS_PIN);
    spi_format(&spi_lcd, 9, 3, 0);
    spi_frequency(&spi_lcd, 48000000);
    spi_irq_hook(&spi_lcd, spi_comp_handler, 0);

    gpio_init(&gpio_reset, RST);
    gpio_dir(&gpio_reset, PIN_OUTPUT);
    gpio_mode(&gpio_reset, PullNone);

    /* Init LCD */
    st7789v_config_init();
}

static void st7789v_display_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    (void) disp_drv;
    (void) color_p;

    uint16_t x1 = area->x1 + X_OFFSET;
    uint16_t y1 = area->y1 + Y_OFFSET;
    uint16_t x2 = area->x2 + X_OFFSET;
    uint16_t y2 = area->y2 + Y_OFFSET;

    WR_DATA(0x2a, (uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2);
    WR_DATA(0x2b, (uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2);

    g_line_buf[0] = 0x2c;
    const size_t g_line_buf_count = sizeof g_line_buf / sizeof g_line_buf[0];
    int i = 1;
    for (uint16_t y = area->y1 * 2; y <= area->y2 * 2; y++) {
        for (uint16_t x = area->x1; x <= area->x2; x++) {
            uint16_t data = g_buffer_0[y * WIDTH + x];
            data |= 0x100;
            g_line_buf[i++] = data;
            if (i == g_line_buf_count) {
                spi_master_write_stream_dma(&spi_lcd, g_line_buf, 2 * g_line_buf_count);
                rtos_sema_take(spi_tx_done_sem, RTOS_SEMA_MAX_COUNT);
                i = 0;
            }
        }
    }
    spi_master_write_stream_dma(&spi_lcd, g_line_buf, 2 * i);
    rtos_sema_take(spi_tx_done_sem, RTOS_SEMA_MAX_COUNT);

    lv_disp_flush_ready(disp_drv);
}

void lv_hal_init(void)
{
    lv_init();

    st7789v_init();

    // Initialize a descriptor for the buffer
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, g_buffer_0, NULL, WIDTH * HEIGHT);

    // Initialize and register a display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf   = &disp_buf;
    disp_drv.flush_cb   = st7789v_display_flush;
    disp_drv.hor_res    = WIDTH;
    disp_drv.ver_res    = HEIGHT;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);
}

// Set in lv_conf.h as `LV_TICK_CUSTOM_SYS_TIME_EXPR`
uint32_t custom_tick_get(void)
{
    return rtos_time_get_current_system_time_ms();
}
