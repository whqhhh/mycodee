#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_log.h"
// 配置参数（测量范围0-3.3V）
#define ADC_CHANNEL     ADC1_CHANNEL_0  // 使用GPIO4作为ADC输入 (ADC1_CH0)
#define ADC_ATTEN       ADC_ATTEN_DB_11 // 11dB衰减，支持0-3.9V（覆盖3.3V需求）
#define ADC_WIDTH       ADC_WIDTH_BIT_12 // 12位分辨率，取值范围0-4095

#define UART_NUM        UART_NUM_0      // 使用默认UART0（USB串口）
#define UART_BAUD_RATE  9600          // 波特率
#define TASK_DELAY_MS  100            // 采样间隔(毫秒)

static const char *TAG = "ADC_UART_3.3V";

// UART初始化（默认使用USB串口，无需额外接线）
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // 使用默认引脚（UART0-TX:GPIO1, RX:GPIO3，ESP32-S3通常通过USB虚拟串口连接）
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0));
}

// ADC初始化（适配3.3V测量）
static void adc_init(void) {
    // 配置指定通道的衰减（11dB支持0-3.9V，满足3.3V需求）
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));
    // 配置ADC分辨率为12位（0-4095）
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    
    ESP_LOGI(TAG, "ADC初始化完成（测量范围0-3.3V）");
}
// ADC采样并通过串口发送任务
static void adc_uart_task(void *arg) {
    while (1) {
        // 读取原始ADC值（0-4095）
        int adc_raw = adc1_get_raw(ADC_CHANNEL);
        
        // 计算电压值（基于3.3V满量程换算）
        // 公式：电压 = (原始值 / 最大原始值) * 满量程电压
        int voltage = adc_raw / 2 ;

        // 格式化输出字符串
        char data_str[128];

        snprintf(data_str, sizeof(data_str), "%d\n", voltage); 
        // 通过串口发送数据
        uart_write_bytes(UART_NUM, data_str, strlen(data_str));
        
        // ESP_LOGI(TAG, "已发送: %s", data_str);
        
        // 间隔指定时间后再次采样
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

void app_main(void) {
    uart_init();   // 初始化串口
    adc_init();    // 初始化ADC
    // 创建采样任务（栈大小4096，优先级5）
    xTaskCreate(adc_uart_task, "adc_uart_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "ADC采样（3.3V范围）+ 串口发送程序已启动");
}

