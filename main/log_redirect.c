#include "esp_log.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>  // 添加字符串处理函数支持

/* 
 * 解决方案：将日志重定向到UART1/GPIO 19-20（USB调试口）
 * 保持UART0只用于小车控制命令通信
 */

// 定义UART1参数
#define DEBUG_UART UART_NUM_1   // 用于日志输出的USB口
#define DEBUG_UART_TX_PIN 19    // USB口TX GPIO
#define DEBUG_UART_RX_PIN 20    // USB口RX GPIO
#define DEBUG_UART_BAUD_RATE 115200

// 全局标志，表示是否已初始化
static bool debug_uart_initialized = false;

// 初始化UART1用于日志输出
static void init_debug_uart(void) {
    // 配置UART1参数
    uart_config_t uart_config = {
        .baud_rate = DEBUG_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // 安装UART1驱动
    ESP_ERROR_CHECK(uart_driver_install(DEBUG_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DEBUG_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DEBUG_UART, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// 安全地检查字符串是否包含指定子串
static bool safe_contains(const char *str, const char *substr) {
    if (!str || !substr) return false;
    return strstr(str, substr) != NULL;
}

// 简化版日志输出函数，将日志发送到UART1
static int uart1_log_vprintf(const char *fmt, va_list args) {
    // 安全检查
    if (!fmt) {
        return 0; // 格式化字符串为空，安全返回
    }
    
    // 在格式化之前进行简单检查，筛选明显的UART0命令
    if (safe_contains(fmt, "MOVE") || 
        safe_contains(fmt, "LED") || 
        safe_contains(fmt, "RELAY") || 
        safe_contains(fmt, "SERVO") || 
        safe_contains(fmt, "ARM")) {
        // 直接跳过可能的命令日志
        return vprintf(fmt, args); // 仍然输出到标准控制台
    }
    
    // 使用临时缓冲区
    char buf[256]; // 缓冲区尺寸减小以降低内存占用
    int len;
    
    // 格式化字符串
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len <= 0) {
        return 0; // 格式化失败
    }
    
    // 防止缓冲区溢出
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    
    // 使用UART1发送格式化的字符串
    uart_write_bytes(DEBUG_UART, buf, len);
    uart_write_bytes(DEBUG_UART, "\r\n", 2); // 增加换行符确保格式正确
    
    // 同时输出到标准调试控制台
    return vprintf(fmt, args);
}

// 放弃与ESP-IDF日志接口冲突的UART0配置
static bool release_uart0_resources(void) {
    // 尝试释放UART0资源，这可能已被系统内部使用
    // 注意：这里我们通常不需要错误检查，因为可能还没有初始化
    uart_driver_delete(UART_NUM_0);
    
    // 设置所有模块的UART0日志级别为无（完全禁用UART0日志）
    esp_log_level_set("uart", ESP_LOG_NONE);
    esp_log_level_set("bootloader_log", ESP_LOG_NONE);
    
    return true;
}

// 公开的初始化函数，在main.cc中手动调用
void log_redirect_init(void) {
    // 如果已经初始化，不要重复初始化
    if (debug_uart_initialized) {
        return;
    }
    
    // 1. 首先释放UART0的资源，确保串口助手可以访问
    release_uart0_resources();
    
    // 2. 初始化UART1用于日志输出
    init_debug_uart();
    
    // 3. 标记为已初始化
    debug_uart_initialized = true;
    
    // 4. 设置自定义日志系统，重定向到UART1
    esp_log_set_vprintf(uart1_log_vprintf);
    
    // 5. 设置全局日志级别为INFO - 启用系统日志
    esp_log_level_set("*", ESP_LOG_INFO);
    
    // 6. 不发送初始化信息
}
