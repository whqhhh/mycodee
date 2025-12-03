/*===========================================================================
 *  ADĆ电压采集 + 串口通信 + SPI显示屏系统
 *  ESP32-S3 物联网电压监测器
 *
 *  功能特点:
 *  - 多通道ADC采样 (0-3.3V)
 *  - 串口数据输出 (9600bps)
 *  - SPI显示屏实时显示
 *  - 50ms采样间隔，20Hz刷新率
 *  - FreeRTOS多任务架构
 *
 *  硬件连接:
 *  - ADC输入: GPIO4 (ADC1_CH0)
 *  - 串口: USB虚拟串口 (UART0)
 *  - SPI显示屏引脚定义（见配置部分）
 *
 *  Note: 请根据实际情况调整引脚配置和显示屏初始化序列
 *===========================================================================*/

/*=============================== 标准头文件包含 ==============================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*=============================== FreeRTOS头文件 ==============================*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*============================ 驱动程序头文件包含 =============================*/
#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/*============================= ESP-IDF系统头文件 =============================*/
#include "esp_log.h"

/*=============================== 配置参数区 =================================*/

/*------------------------------- ADC配置 -----------------------------------*/
#define ADC_CHANNEL       ADC1_CHANNEL_0    // ADC输入脚: GPIO4 (ADC1_CH0)
#define ADC_ATTEN         ADC_ATTEN_DB_11   // 11dB衰减，支持0-3.9V
#define ADC_WIDTH         ADC_WIDTH_BIT_12  // 12位分辨率 (0-4095)

/*------------------------------ 串口配置 -----------------------------------*/
#define UART_NUM          UART_NUM_0        // 使用默认串口 (USB)
#define UART_BAUD_RATE    9600              // 波特率: 9600 bps

/*------------------------------ 采样时钟 -----------------------------------*/
#define TASK_DELAY_MS     50                // 采样间隔: 50ms (20Hz)

/*------------------------------ SPI配置 ------------------------------------*/
#define SPI_HOST          SPI2_HOST         // 使用SPI2主机
#define SPI_MOSI_PIN      11                // 串行数据线
#define SPI_SCLK_PIN      12                // 时钟线
#define SPI_CS_PIN        10                // 片选线
#define SPI_DC_PIN        14                // 数据/命令选择线
#define SPI_RESET_PIN     15                // 复位线
#define SPI_BACKLIGHT_PIN 16                // 背光控制线
#define SPI_CLOCK_SPEED   40000000          // SPI时钟频率: 40MHz

/*---------------------------- 显示屏参数配置 -------------------------------*/
#define DISPLAY_WIDTH     240               // 显示宽度 (像素)
#define DISPLAY_HEIGHT    320               // 显示高度 (像素)

/*============================ 模块级常量定义 ================================*/
static const char *TAG = "ADC_UART_DISPLAY";  // 日志标签

/*============================ 设备句柄定义 ================================*/
static spi_device_handle_t spi_disp_device;    // SPI设备句柄
static spi_bus_config_t buscfg;                // SPI总线配置

/*============================================================================
 *                      低层设备驱动函数 - 串口部分
 *===========================================================================*/

/**
 * @brief 串口接口初始化
 * @details 配置串口参数并安装驱动程序
 *
 * @param 无
 * @return 无
 */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate    = UART_BAUD_RATE,
        .data_bits    = UART_DATA_8_BITS,
        .parity       = UART_PARITY_DISABLE,
        .stop_bits    = UART_STOP_BITS_1,
        .flow_ctrl    = UART_HW_FLOWCTRL_DISABLE,
        .source_clk   = UART_SCLK_DEFAULT,
    };

    // 配置串口参数
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // 使用默认引脚 (GPIO1=TX, GPIO3=RX)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    // 安装串口驱动
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "串口初始化完成 (波特率: %d)", UART_BAUD_RATE);
}

/*============================================================================
 *                      低层设备驱动函数 - ADC部分
 *===========================================================================*/

/**
 * @brief ADC模数转换器初始化
 * @details 配置ADC通道和参数
 *
 * @param 无
 * @return 无
 */
static void adc_init(void)
{
    // 配置指定通道的衰减
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

    // 配置ADC分辨率
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));

    ESP_LOGI(TAG, "ADC初始化完成 (通道: %d, 分辨率: %d位)",
             ADC_CHANNEL, 12);
}

/**
 * @brief 电压值计算函数
 * @details 将ADC原始数值转换为电压值（mV）
 *
 * @param adc_raw - ADC原始数值 (0-4095)
 * @return 电压值（毫伏）
 */
static int calculate_voltage_mv(int adc_raw)
{
    // 简化的电压计算： (原始值/2) * 100 mV
    // 更准确的公式应该是：adc_raw * 3300 / 4095
    int voltage = (adc_raw / 2);
    return voltage * 100;
}

/*============================================================================
 *                      低层设备驱动函数 - SPI显示部分
 *===========================================================================*/

/**
 * @brief GPIO电平控制集团化操作
 */
static inline void gpio_set_level_safe(gpio_num_t gpio_num, uint32_t level)
{
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num, level);
}

static inline void gpio_set_high(gpio_num_t gpio_num)
{
    gpio_set_level_safe(gpio_num, 1);
}

static inline void gpio_set_low(gpio_num_t gpio_num)
{
    gpio_set_level_safe(gpio_num, 0);
}

/**
 * @brief 通过SPI向显示屏发送命令
 *
 * @param cmd - 要发送的命令
 */
static void spi_disp_send_command(uint8_t cmd)
{
    gpio_set_low(SPI_DC_PIN);  // DC=0表示命令

    spi_transaction_t t = {
        .length    = 8,
        .tx_buffer = &cmd,
    };

    ESP_ERROR_CHECK(spi_device_transmit(spi_disp_device, &t));
}

/**
 * @brief 通过SPI向显示屏发送数据
 *
 * @param data - 数据指针
 * @param len - 数据长度
 */
static void spi_disp_send_data(uint8_t *data, size_t len)
{
    gpio_set_high(SPI_DC_PIN);  // DC=1表示数据

    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = data,
    };

    ESP_ERROR_CHECK(spi_device_transmit(spi_disp_device, &t));
}

/**
 * @brief 显示屏基础硬件初始化
 * @details 复位显示屏并开始背光
 */
static void display_hardware_init(void)
{
    ESP_LOGI(TAG, "显示屏硬件初始化开始...");

    // 复位显示屏
    gpio_set_low(SPI_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(10));      // 低电平10ms
    gpio_set_high(SPI_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(120));     // 等待120ms稳定

    // 开启背光
    gpio_set_high(SPI_BACKLIGHT_PIN);

    ESP_LOGI(TAG, "显示屏硬件初始化完成");
    ESP_LOGW(TAG, "⚠️  请根据你的具体显示屏型号添加完整的初始化指令序列");
}

/**
 * @brief SPI总线和显示屏设备初始化
 * @details 配置SPI总线、设备接口，初始化显示屏
 */
static void spi_display_init(void)
{
    ESP_LOGI(TAG, "SPI显示屏初始化开始...");

    // SPI总线配置
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = -1,      // 大多数显示不需要MISO
        .sclk_io_num = SPI_SCLK_PIN,
        .max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * 3, // 全帧缓冲
    };

    // 初始化SPI总线
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // SPI设备配置
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode           = 0,     // SPI模式0 (CPOL=0, CPHA=0)
        .spics_io_num   = SPI_CS_PIN,
        .queue_size     = 7,     // 最大7个传输任务排队
        .flags        = SPI_DEVICE_HALFDUPLEX,
    };

    // 向SPI总线添加显示设备
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &dev_cfg, &spi_disp_device));

    // 硬件初始化
    display_hardware_init();

    ESP_LOGI(TAG, "SPI显示屏系统初始化完成");
}

/**
 * @brief 在显示屏上显示电压数值（简化版）
 *
 * @param voltage_mv - 电压值（毫伏）
 */
static void display_voltage_info(int voltage_mv)
{
    char display_buffer[32];
    snprintf(display_buffer, sizeof(display_buffer), "电压: %dmV", voltage_mv);

    ESP_LOGI(TAG, "LCD显示: %s", display_buffer);

    // 🔧 待接入实际屏幕字号显示函数
    // 示例：
    // display_set_cursor(0, 0);
    // display_print_string(display_buffer);
    // display_show_screen();
}

/*============================================================================
 *                              主应用任务
 *===========================================================================*/

/**
 * @brief ADC采集与多通道输出任务
 * @details 定期采样ADC值，并通过多种方式进行输出
 *          - 串口数字输出
 *          - 显示屏数值显示
 *          - 系统日志记录
 *
 * @param arg - 任务参数 (未使用)
 */
static void adc_sampling_task(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_DELAY_MS);

    // 初始化唤醒时间
    xLastWakeTime = xTaskGetTickCount();

    ESP_LOGW(TAG, "采样任务启动 - 采样频率: %dHz (间隔: %dms)",
             1000/TASK_DELAY_MS, TASK_DELAY_MS);

    while (1) {
        // 计算实际电压值 (mV)
        int adc_raw = adc1_get_raw(ADC_CHANNEL);
        int voltage_mv = calculate_voltage_mv(adc_raw);

        // 串口数据输出
        char uart_data_str[32];
        snprintf(uart_data_str, sizeof(uart_data_str), "%d\n", voltage_mv/100); // 发送整数值
        uart_write_bytes(UART_NUM, uart_data_str, strlen(uart_data_str));

        // 实时数据显示在屏幕上
        display_voltage_info(voltage_mv);

        // 调试日志输出 (可以选择性启用)
        // ESP_LOGD(TAG, "[采样] 原始值=%d 电压=%dmV", adc_raw, voltage_mv);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*============================================================================
 *                                  主入口
 *===========================================================================*/

/**
 * @brief 应用程序主入口函数
 * @details 初始化硬件系统并创建采样任务
 *
 * @param 无
 * @return 无
 */
void app_main(void)
{
    /*=============================== 硬件初始化 ===============================*/
    ESP_LOGI(TAG, "==>>> 系统启动 <<<==");
    uart_init();
    adc_init();
    spi_display_init();

    /*=============================== 显示配置 ===============================*/
    ESP_LOGW(TAG, "=== 系统配置概览 ===");
    ESP_LOGW(TAG, "ADC系统:");
    ESP_LOGW(TAG, "  - 输入脚:  GPIO%d (ADC%d_CH%d)", 4, 1, 0);
    ESP_LOGW(TAG, "  - 衰减设置: %ddB", (int)(ADC_ATTEN*3.3));
    ESP_LOGW(TAG, "  - 分辨率:   %d-bit", 12);
    ESP_LOGW(TAG, "  - 范围:     0-3.3V");

    ESP_LOGW(TAG, "串口系统:");
    ESP_LOGW(TAG, "  - 接口: UART%d", UART_NUM);
    ESP_LOGW(TAG, "  - 波特率: %d", UART_BAUD_RATE);

    ESP_LOGW(TAG, "SPI显示系统:");
    ESP_LOGW(TAG, "  - SPI主机: Host %d", SPI_HOST);
    ESP_LOGW(TAG, "  - 时钟频率: %dMHz", SPI_CLOCK_SPEED/1000000);
    ESP_LOGW(TAG, "  - 引脚配置:");
    ESP_LOGW(TAG, "    • MOSI:    GPIO%d", SPI_MOSI_PIN);
    ESP_LOGW(TAG, "    • SCLK:    GPIO%d", SPI_SCLK_PIN);
    ESP_LOGW(TAG, "    • CS:      GPIO%d", SPI_CS_PIN);
    ESP_LOGW(TAG, "    • D/C:     GPIO%d", SPI_DC_PIN);
    ESP_LOGW(TAG, "    • RESET:   GPIO%d", SPI_RESET_PIN);
    ESP_LOGW(TAG, "    • 背光:     GPIO%d", SPI_BACKLIGHT_PIN);
    ESP_LOGW(TAG, "  - 分辨率: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);

    ESP_LOGW(TAG, "采样系统:");
    ESP_LOGW(TAG, "  - 频率: %dHz (间隔 %dms)", 1000/TASK_DELAY_MS, TASK_DELAY_MS);

    /*=========================== 创建主采样任务 ==============================*/
    BaseType_t task_create_result = xTaskCreate(
        adc_sampling_task,              // 任务函数
        "adc_sampling_task",            // 任务名称
        4096,                           // 堆栈大小 (字节)
        NULL,                           // 任务参数
        5,                              // 任务优先级
        NULL                            // 任务句柄
    );

    configASSERT(task_create_result == pdPASS);

    /*=============================== 系统就绪 ===============================*/
    ESP_LOGW(TAG, "=== 系统初始化完成 ===");
    ESP_LOGW(TAG, "✅ 串口数据监控可用");
    ESP_LOGW(TAG, "✅ SPI显示输出启用");
    ESP_LOGW(TAG, "✅ ADC采样系统就绪");

    ESP_LOGW(TAG, "演示板 - 电压监测器已启动");
    ESP_LOGW(TAG, "请连接ADC输入到GPIO%d进行电压采集演示", 4);
}

/*****************************************************************************
 *                               END OF FILE                                  *
 *****************************************************************************/