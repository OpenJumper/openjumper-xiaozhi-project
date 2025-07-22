#include "car_controller.h"
#include "esp_log.h"
#include "esp_system.h"
#include <regex>
#include <sstream>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cctype>
#include <cstring>
#include "driver/gpio.h"

// 机械臂动作定义
enum class ArmAction {
    GRAB,    // 抓取
    RELEASE, // 释放
    UP,      // 上升
    DOWN,    // 下降
    LEFT,    // 左移
    RIGHT    // 右移
};

// LED动作定义
enum class LedAction {
    ON,      // 开启
    OFF,     // 关闭
    BLINK    // 闪烁
};

#define TAG "CarController"

CarController::~CarController() {
    if (initialized_) {
        uart_driver_delete(uart_port_);
    }
}

void CarController::Initialize(uart_port_t uart_port, int tx_pin, int rx_pin, int baud_rate) {
    uart_port_ = uart_port;
    
    // 创建命令处理互斥锁
    if (this->command_mutex == NULL) {
        this->command_mutex = xSemaphoreCreateMutex();
        if (this->command_mutex == NULL) {
            ESP_LOGE(TAG, "创建命令互斥锁失败");
        } else {
            ESP_LOGI(TAG, "命令互斥锁创建成功");
        }
    }
    
    ESP_LOGI(TAG, "Initializing car control UART, port: %d, TX: %d, RX: %d, baud: %d", 
             uart_port, tx_pin, rx_pin, baud_rate);
    
    // 如果已经初始化，删除驱动
    if (initialized_) {
        ESP_LOGI(TAG, "Deleting existing UART driver");
        uart_driver_delete(uart_port_);
        initialized_ = false;
    }
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动
    esp_err_t ret = uart_driver_install(uart_port_, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure UART parameters
    ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return;
    }
    
    // 设置UART引脚
    ret = uart_set_pin(uart_port_, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return;
    }
    
    // 清空UART缓冲区
    uart_flush(uart_port_);
    
    initialized_ = true;
    ESP_LOGI(TAG, "Car control UART initialized successfully");
    
    // 发送测试命令
    SendCommand("MOVE S");
}

// 防抖相关变量
static std::string last_command = "";
static uint32_t last_command_time = 0;
static const uint32_t DEBOUNCE_TIME_MS = 1000; // 1秒防抖时间

void CarController::HandleTextResponse(const cJSON* root) {
    // 检查是否是文本消息
    auto text = cJSON_GetObjectItem(root, "text");
    if (!cJSON_IsString(text)) {
        return;
    }
    
    // 使用静态缓冲区存储文本，避免临时字符串问题
    static char text_buffer[256];
    strncpy(text_buffer, text->valuestring, sizeof(text_buffer) - 1);
    text_buffer[sizeof(text_buffer) - 1] = '\0'; // 确保结尾
    
    std::string text_str = text_buffer;
    
    // 防抖处理，避免短时间内重复处理相同内容
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    if (text_str == last_command && (current_time - last_command_time) < DEBOUNCE_TIME_MS) {
        return;
    }
    last_command = text_str;
    last_command_time = current_time;
    
    // 解析命令类型
    CommandType cmd_type = ParseCommandType(text_str);
    
    // 根据命令类型进行处理
    switch (cmd_type) {
        case CommandType::MOVE:
            // 尝试解析命令，如果失败则跳过
            ParseMoveCommand(text_str);
            break;
        case CommandType::ARM:
            ParseArmCommand(text_str);
            break;
        case CommandType::RELAY:
            ParseRelayCommand(text_str);
            break;
        case CommandType::LED:
            ParseLedCommand(text_str);
            break;
        case CommandType::SERVO:
            ParseServoCommand(text_str);
            break;
        case CommandType::FAN:
            ParseFanCommand(text_str);
            break;
        case CommandType::RGB:
            ParseRgbCommand(text_str);
            break;
        case CommandType::UNKNOWN:
            break;
    }
}

CommandType CarController::ParseCommandType(const std::string& text) {
    std::string lower_text = text;
    // 转换为小写，以进行不区分大小写的匹配
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // 检查移动命令
    if (lower_text.find("move") != std::string::npos ||
        lower_text.find("前进") != std::string::npos ||
        lower_text.find("后退") != std::string::npos ||
        lower_text.find("左转") != std::string::npos ||
        lower_text.find("右转") != std::string::npos ||
        lower_text.find("停止") != std::string::npos) {
        return CommandType::MOVE;
    }
    
    // 检查机械臂命令
    if (lower_text.find("arm") != std::string::npos ||
        lower_text.find("机械臂") != std::string::npos ||
        lower_text.find("拿取") != std::string::npos ||
        lower_text.find("抓取") != std::string::npos ||
        lower_text.find("松开") != std::string::npos) {
        return CommandType::ARM;
    }
    
    // 检查继电器命令
    if (lower_text.find("relay") != std::string::npos ||
        lower_text.find("继电器") != std::string::npos) {
        return CommandType::RELAY;
    }
    
    // 检查WS2812彩灯命令
    if (lower_text.find("彩灯") != std::string::npos ||
        lower_text.find("ws2812") != std::string::npos ||
        lower_text.find("rgb灯") != std::string::npos) {
        return CommandType::RGB;
    }
    
    // 检查LED命令
    if (lower_text.find("led") != std::string::npos ||
        lower_text.find("灯") != std::string::npos ||
        lower_text.find("灯光") != std::string::npos ||
        lower_text.find("闪烁") != std::string::npos) {
        return CommandType::LED;
    }
    
    // 检查舵机命令
    if (lower_text.find("servo") != std::string::npos ||
        lower_text.find("舵机") != std::string::npos ||
        lower_text.find("角度") != std::string::npos) {
        return CommandType::SERVO;
    }
    
    // 检查风扇命令
    if (lower_text.find("fan") != std::string::npos ||
        lower_text.find("风扇") != std::string::npos ||
        lower_text.find("fan_on") != std::string::npos ||
        lower_text.find("fan_off") != std::string::npos) {
        return CommandType::FAN;
    }
    
    return CommandType::UNKNOWN;
}

// 静态变量用于命令去重
static std::string last_printed_cmd = "";
static uint32_t last_print_time = 0;
static const uint32_t CMD_PRINT_DEBOUNCE_MS = 3000; // 3秒内不重复打印相同命令

void CarController::SendCommand(const std::string& cmd) {
    if (!initialized_) {
        return; // UART未初始化
    }
    
    // 使用静态缓冲区减少栈使用
    static char buffer[64];
    
    // 安全地复制命令到缓冲区
    strncpy(buffer, cmd.c_str(), sizeof(buffer) - 2); // 留出\n和\0的空间
    buffer[sizeof(buffer) - 2] = '\0';
    
    // 添加换行符
    size_t cmd_len = strlen(buffer);
    buffer[cmd_len] = '\n';
    buffer[cmd_len + 1] = '\0';
    
    int len = cmd_len + 1; // +1 是因为\n
    
    // 命令去重逻辑，避免短时间内打印重复命令
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    if (cmd != last_printed_cmd || (current_time - last_print_time) > CMD_PRINT_DEBOUNCE_MS) {
        // 只有新命令或者间隔足够长的相同命令才会打印
        printf("%s\n", cmd.c_str());
        last_printed_cmd = cmd;
        last_print_time = current_time;
    }
    
    // 使用较短延迟
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 发送命令
    int written = uart_write_bytes(uart_port_, buffer, len);
    if (written != len) {
        ESP_LOGE(TAG, "错误: 命令发送失败, 预期 %d 字节, 实际发送 %d", len, written);
    }
    
    // 确保发送完成
    uart_wait_tx_done(uart_port_, pdMS_TO_TICKS(50));
}

void CarController::Move(MoveDirection direction, int speed) {
    // 将速度限制在0和100之间
    speed = std::max(0, std::min(100, speed));
    
    std::string cmd = "MOVE ";
    
    // 添加方向字符
    switch (direction) {
        case MoveDirection::FORWARD:
            cmd += "F";
            break;
        case MoveDirection::BACKWARD:
            cmd += "B";
            break;
        case MoveDirection::LEFT:
            cmd += "L";
            break;
        case MoveDirection::RIGHT:
            cmd += "R";
            break;
        case MoveDirection::STOP:
            cmd = "MOVE S"; // 停止命令不需要速度参数
            SendCommand(cmd);
            return;
    }
    
    // 为非停止命令添加速度参数
    cmd += " " + std::to_string(speed);
    SendCommand(cmd);
}

bool CarController::ParseMoveCommand(const std::string& text) {
    // 使用非阻塞方式获取互斥锁
    bool lock_acquired = false;
    if (this->command_mutex != NULL) {
        lock_acquired = (xSemaphoreTake(this->command_mutex, 0) == pdTRUE);
    }

    // 设置默认速度和结果
    int speed = 20;
    bool result = false;
    
    try {
        // 转换成小写进行匹配
        std::string lower_text = text;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                      [](unsigned char c){ return std::tolower(c); });
        
        // 查询速度参数
        size_t speed_pos = lower_text.find("速度");
        if (speed_pos != std::string::npos && speed_pos + 2 < lower_text.length()) {
            // 尝试提取速度值
            for (size_t i = speed_pos + 2; i < lower_text.length(); i++) {
                if (std::isdigit(lower_text[i])) {
                    size_t start = i;
                    while (i < lower_text.length() && std::isdigit(lower_text[i])) i++;
                    try {
                        int value = std::stoi(lower_text.substr(start, i - start));
                        if (value >= 0 && value <= 100) speed = value;
                    } catch (...) {
                        // 忽略解析错误
                    }
                    break;
                }
            }
        }
        
        // 判断前进命令
        if (lower_text.find("小车前进") != std::string::npos || 
            lower_text.find("车子前进") != std::string::npos ||
            lower_text.find("前进") != std::string::npos) {
            
            if (lock_acquired) Move(MoveDirection::FORWARD, speed);
            result = true;
        }
        // 判断后退命令
        else if (lower_text.find("小车后退") != std::string::npos || 
                 lower_text.find("车子后退") != std::string::npos ||
                 lower_text.find("后退") != std::string::npos) {
            
            if (lock_acquired) Move(MoveDirection::BACKWARD, speed);
            result = true;
        }
        // 判断左转命令
        else if (lower_text.find("小车左转") != std::string::npos || 
                 lower_text.find("车子左转") != std::string::npos ||
                 lower_text.find("左转") != std::string::npos) {
            
            if (lock_acquired) Move(MoveDirection::LEFT, speed);
            result = true;
        }
        // 判断右转命令
        else if (lower_text.find("小车右转") != std::string::npos || 
                 lower_text.find("车子右转") != std::string::npos ||
                 lower_text.find("右转") != std::string::npos) {
            
            if (lock_acquired) Move(MoveDirection::RIGHT, speed);
            result = true;
        }
        // 判断停止命令
        else if (lower_text.find("小车停") != std::string::npos || 
                 lower_text.find("车子停") != std::string::npos ||
                 lower_text.find("停止") != std::string::npos) {
            
            if (lock_acquired) Move(MoveDirection::STOP, 0);
            result = true;
        }
    } catch (...) {
        // 捕获所有异常，避免程序崩溃
        if (lock_acquired) xSemaphoreGive(this->command_mutex);
        return false;
    }
    
    // 释放互斥锁
    if (lock_acquired) xSemaphoreGive(this->command_mutex);
    
    return result;
}

void CarController::ArmControl(const std::string& action, int servo_id, int angle) {
    std::string cmd = "ARM ";
    
    cmd += action;
    
    // 如果有舵机ID和角度参数，添加到命令中
    if (servo_id > 0) {
        cmd += " " + std::to_string(servo_id);
        if (angle > 0) {
            cmd += " " + std::to_string(angle);
        }
    }
    
    SendCommand(cmd);
}

bool CarController::ParseArmCommand(const std::string& text) {
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // 支持"机械臂抓取"格式
    if (lower_text.find("机械臂抓取") != std::string::npos ||
        lower_text.find("机械臂拿取") != std::string::npos) {
        ESP_LOGI(TAG, "Detected specific ARM GRAB command");
        ArmControl("GRAB", 0, 0);
        return true;
    }
    
    // 支持"机械臂松开"格式
    if (lower_text.find("机械臂松开") != std::string::npos ||
        lower_text.find("机械臂放开") != std::string::npos) {
        ESP_LOGI(TAG, "Detected specific ARM RELEASE command");
        ArmControl("RELEASE", 0, 0);
        return true;
    }
    
    // 支持"机械臂回归初始"格式
    if (lower_text.find("机械臂回归初始") != std::string::npos ||
        lower_text.find("机械臂复位") != std::string::npos) {
        ESP_LOGI(TAG, "Detected ARM RESET command");
        ArmControl("DOWN"); // 假设向下动作为初始位置
        return true;
    }
    
    // 检查拿取命令
    if (lower_text.find("grab") != std::string::npos ||
        lower_text.find("catch") != std::string::npos ||
        lower_text.find("hold") != std::string::npos ||
        lower_text.find("拿") != std::string::npos ||
        lower_text.find("抓") != std::string::npos ||
        lower_text.find("抓取") != std::string::npos) {
        ESP_LOGI(TAG, "Detected ARM GRAB command");
        ArmControl("GRAB", 0, 0);
        return true;
    }
    
    // 检查松开命令
    if (lower_text.find("release") != std::string::npos ||
        lower_text.find("drop") != std::string::npos ||
        lower_text.find("let go") != std::string::npos ||
        lower_text.find("放下") != std::string::npos ||
        lower_text.find("松开") != std::string::npos) {
        ESP_LOGI(TAG, "Detected ARM RELEASE command");
        ArmControl("RELEASE", 0, 0);
        return true;
    }
    
    // 检查机械臂上升命令
    if ((lower_text.find("arm") != std::string::npos || 
         lower_text.find("机械臂") != std::string::npos) &&
        (lower_text.find("up") != std::string::npos ||
         lower_text.find("上捆") != std::string::npos ||
         lower_text.find("上升") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected ARM UP command");
        ArmControl("UP");
        return true;
    }
    
    // 检查机械臂下降命令
    if ((lower_text.find("arm") != std::string::npos || 
         lower_text.find("机械臂") != std::string::npos) &&
        (lower_text.find("down") != std::string::npos ||
         lower_text.find("下降") != std::string::npos ||
         lower_text.find("下移") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected ARM DOWN command");
        ArmControl("DOWN");
        return true;
    }
    
    // 检查机械臂左移命令
    if ((lower_text.find("arm") != std::string::npos || 
         lower_text.find("机械臂") != std::string::npos) &&
        (lower_text.find("left") != std::string::npos ||
         lower_text.find("左转") != std::string::npos ||
         lower_text.find("左移") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected ARM LEFT command");
        ArmControl("LEFT");
        return true;
    }
    
    // 检查机械臂右移命令
    if ((lower_text.find("arm") != std::string::npos || 
         lower_text.find("机械臂") != std::string::npos) &&
        (lower_text.find("right") != std::string::npos ||
         lower_text.find("右转") != std::string::npos ||
         lower_text.find("右移") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected ARM RIGHT command");
        ArmControl("RIGHT");
        return true;
    }
    
    return false;
}

void CarController::RelayControl(int relay_id, bool state) {
    std::string cmd = "RELAY " + std::to_string(relay_id) + " ";
    cmd += state ? "ON" : "OFF";
    
    SendCommand(cmd);
}

bool CarController::ParseRelayCommand(const std::string& text) {
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // 从"打开继电器1"或"关闭继电器2"格式中提取继电器ID
    int relay_id = 1;
    std::regex cn_relay_id_regex("继电器([0-9]+)");
    std::smatch match;
    if (std::regex_search(lower_text, match, cn_relay_id_regex)) {
        relay_id = std::stoi(match[1]);
    }
    
    // 支持"打开继电器1"格式
    if ((lower_text.find("打开继电器") != std::string::npos || 
         lower_text.find("开启继电器") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected specific RELAY %d ON command", relay_id);
        RelayControl(relay_id, true);
        return true;
    }
    
    // 支持"关闭继电器2"格式
    if ((lower_text.find("关闭继电器") != std::string::npos || 
         lower_text.find("关掉继电器") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected specific RELAY %d OFF command", relay_id);
        RelayControl(relay_id, false);
        return true;
    }
    
    // 提取继电器ID（如果存在，默认为1）
    std::regex relay_id_regex("relay\\s+([0-9]+)");
    if (std::regex_search(lower_text, match, relay_id_regex)) {
        relay_id = std::stoi(match[1]);
    }
    
    // 检查继电器开启指令
    if (lower_text.find("relay") != std::string::npos &&
        (lower_text.find("on") != std::string::npos ||
         lower_text.find("open") != std::string::npos ||
         lower_text.find("开") != std::string::npos ||
         lower_text.find("打开") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected RELAY %d ON command", relay_id);
        RelayControl(relay_id, true);
        return true;
    }
    
    // 检查继电器关闭指令
    if (lower_text.find("relay") != std::string::npos &&
        (lower_text.find("off") != std::string::npos ||
         lower_text.find("close") != std::string::npos ||
         lower_text.find("关") != std::string::npos ||
         lower_text.find("关闭") != std::string::npos)) {
        ESP_LOGI(TAG, "Detected RELAY %d OFF command", relay_id);
        RelayControl(relay_id, false);
        return true;
    }
    
    return false;
}

void CarController::LedControl(int id, bool on, int r, int g, int b, int blink_freq) {
    std::string cmd = "LED " + std::to_string(id) + " ";
    
    if (on) {
        if (blink_freq > 0) {
            cmd += "BLINK " + std::to_string(blink_freq);
        } else {
            cmd += "ON";
        }
    } else {
        cmd += "OFF";
    }
    
    // 如果指定了RGB值，添加到命令中
    if (r > 0 || g > 0 || b > 0) {
        cmd += " " + std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b);
    }
    
    SendCommand(cmd);
}

bool CarController::ParseLedCommand(const std::string& text) {
    
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // 从文本中提取LED编号
    int led_id = 1;
    std::regex cn_led_id_regex("灯([0-9]+)");
    std::smatch match;
    if (std::regex_search(lower_text, match, cn_led_id_regex)) {
        led_id = std::stoi(match[1]);
    }
    
    // 预定义RGB颜色值
    int r = 0, g = 0, b = 0;
    bool has_color = false;
    
    // 检测颜色 - 使用精确匹配方式检查每个字的存在
    
    // 检查是否是设置颜色的命令
    bool is_set_color_command = false;
    if (lower_text.find("颜色") != std::string::npos || 
        lower_text.find("设置") != std::string::npos ||
        lower_text.find("设") != std::string::npos ||
        lower_text.find("color") != std::string::npos ||
        lower_text.find("set") != std::string::npos) {
        is_set_color_command = true;
    }
    
    // 先检查文本中的RGB值
    std::regex rgb_regex("rgb\\s*[\\( ]\\s*(\\d+)\\s*[,\\s]\\s*(\\d+)\\s*[,\\s]\\s*(\\d+)");
    std::regex rgb_regex2("(\\d+)\\s*[,\\s]\\s*(\\d+)\\s*[,\\s]\\s*(\\d+)");
    
    std::smatch rgb_match;
    if (std::regex_search(lower_text, rgb_match, rgb_regex)) {
        // 匹配到全格式 "rgb(255,0,0)" 或 "rgb 255,0,0"
        r = std::stoi(rgb_match[1]);
        g = std::stoi(rgb_match[2]);
        b = std::stoi(rgb_match[3]);
        has_color = true;
    } else if (std::regex_search(lower_text, rgb_match, rgb_regex2)) {
        // 匹配到简单格式 "255,0,0"
        r = std::stoi(rgb_match[1]);
        g = std::stoi(rgb_match[2]);
        b = std::stoi(rgb_match[3]);
        has_color = true;
    }
    
    // 检测颜色关键字
    
    // 如果没有从 RGB 值提取到颜色，尝试从文本中提取颜色词
    if (!has_color) {
        if (lower_text.find("红") != std::string::npos || 
            lower_text.find("red") != std::string::npos) {
            r = 255; g = 0; b = 0;
            has_color = true;
        } else if (lower_text.find("绿") != std::string::npos || 
                  lower_text.find("green") != std::string::npos) {
            r = 0; g = 255; b = 0;
            has_color = true;
        } else if (lower_text.find("蓝") != std::string::npos || 
                  lower_text.find("blue") != std::string::npos) {
            r = 0; g = 0; b = 255;
            has_color = true;
        } else if (lower_text.find("黄") != std::string::npos || 
                  lower_text.find("yellow") != std::string::npos) {
            r = 255; g = 255; b = 0;
            has_color = true;
        } else if (lower_text.find("紫") != std::string::npos || 
                  lower_text.find("purple") != std::string::npos) {
            r = 128; g = 0; b = 128;
            has_color = true;
        } else if (lower_text.find("白") != std::string::npos || 
                  lower_text.find("white") != std::string::npos) {
            r = 255; g = 255; b = 255;
            has_color = true;
        } else {
            // 未检测到颜色
        }
    }
    
    // 支持"打开LED灯"格式
    if ((lower_text.find("打开") != std::string::npos ||
         lower_text.find("开启") != std::string::npos) &&
        (lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos)) {
        
        if (has_color) {
            // 如果有颜色信息，使用指定颜色
            LedControl(led_id, true, r, g, b);
        } else {
            // 无颜色信息，使用默认的白色
            LedControl(led_id, true);
        }
        return true;
    }
    
    // 支持"关闭LED灯"格式
    if ((lower_text.find("关闭") != std::string::npos ||
         lower_text.find("关掉") != std::string::npos) &&
        (lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos)) {
        ESP_LOGI(TAG, "检测到LED %d 关灯指令", led_id);
        LedControl(led_id, false);
        return true;
    }
    
    // 支持"LED灯闪烁"格式
    if ((lower_text.find("闪烁") != std::string::npos ||
         lower_text.find("闪亮") != std::string::npos ||
         lower_text.find("闪灯") != std::string::npos) &&
        (lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos)) {
        
        if (has_color) {
            // 如果有颜色信息，使用指定颜色闪烁
            LedControl(led_id, true, r, g, b, 1);
        } else {
            // 无颜色信息，使用默认的白色闪烁
            LedControl(led_id, true, 0, 0, 0, 1); // 闪烁频率设为1
        }
        return true;
    }
    
    // 提取LED编号（如果存在）
    std::regex led_id_regex("led\\s*([0-9]+)");
    if (std::regex_search(lower_text, match, led_id_regex)) {
        led_id = std::stoi(match[1]);
    }
    
    // 检查LED开灯指令
    if ((lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos) &&
        (lower_text.find("开") != std::string::npos ||
         lower_text.find("亮") != std::string::npos ||
         lower_text.find("启动") != std::string::npos)) {
        
        if (has_color) {
            // 如果有颜色信息，使用指定颜色
            LedControl(led_id, true, r, g, b);
        } else {
            // 无颜色信息，使用默认的白色
            LedControl(led_id, true);
        }
        return true;
    }
    
    // 检查LED关灯指令
    if ((lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos) &&
        (lower_text.find("关灯") != std::string::npos ||
         lower_text.find("灭") != std::string::npos ||
         lower_text.find("停") != std::string::npos)) {
        ESP_LOGI(TAG, "检测到LED %d 关灯指令", led_id);
        LedControl(led_id, false);
        return true;
    }
    
    // 检查LED闪烁指令
    if ((lower_text.find("灯") != std::string::npos ||
         lower_text.find("led") != std::string::npos) &&
        (lower_text.find("闪") != std::string::npos ||
         lower_text.find("闪烁") != std::string::npos ||
         lower_text.find("闪亮") != std::string::npos)) {
        
        if (has_color) {
            // 如果有颜色信息，使用指定颜色闪烁
            LedControl(led_id, true, r, g, b, 1);
        } else {
            // 无颜色信息，使用默认的白色闪烁
            LedControl(led_id, true, 0, 0, 0, 1); // 闪烁频率设为1
        }
        return true;
    }
    
    // 如果是设置颜色命令且有颜色信息，直接设置 LED 的颜色
    if (is_set_color_command && has_color) {
        SetLedColor(led_id, r, g, b); // 使用专门的颜色设置函数
        return true;
    }
    
    return false;
}

// 直接设置 LED 颜色的函数，使用格式：LED <ID> <R> <G> <B>
void CarController::SetLedColor(int id, int r, int g, int b) {
    std::string cmd = "LED " + std::to_string(id) + " " + 
                    std::to_string(r) + " " + 
                    std::to_string(g) + " " + 
                    std::to_string(b);
    SendCommand(cmd);
}

void CarController::ServoControl(int servo_id, int angle) {
    // Clamp angle between 0 and 180 degrees
    angle = std::max(0, std::min(180, angle));
    
    std::string cmd = "SERVO " + std::to_string(servo_id) + " " + std::to_string(angle);
    SendCommand(cmd);
}

bool CarController::ParseServoCommand(const std::string& text) {
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // 提取舵机ID（如果存在，默认为1）
    int servo_id = 1; 
    std::regex servo_id_regex("servo\\s+([0-9]+)");
    std::smatch match;
    if (std::regex_search(lower_text, match, servo_id_regex)) {
        servo_id = std::stoi(match[1]);
    }
    
    // 提取角度（如果存在，查找0-180之间的数字）
    int angle = 90; // Default to middle position
    std::regex angle_regex("\\b([0-9]{1,3})\\b");
    if (std::regex_search(lower_text, match, angle_regex)) {
        int extracted_angle = std::stoi(match[1]);
        if (extracted_angle >= 0 && extracted_angle <= 180) {
            angle = extracted_angle;
        }
    }
    
    // Check for servo command
    if (lower_text.find("servo") != std::string::npos ||
        lower_text.find("舵机") != std::string::npos ||
        lower_text.find("角度") != std::string::npos) {
        ESP_LOGI(TAG, "Detected SERVO %d command with angle %d", servo_id, angle);
        ServoControl(servo_id, angle);
        return true;
    }
    
    return false;
}

void CarController::FanControl(int speed) {
    // 将速度限制在0-100之间
    speed = std::max(0, std::min(100, speed));
    
    std::string cmd = "FAN_SPEED " + std::to_string(speed);
    SendCommand(cmd);
}

void CarController::FanOn() {
    SendCommand("FAN_ON");
}

void CarController::FanOff() {
    SendCommand("FAN_OFF");
}

bool CarController::ParseFanCommand(const std::string& text) {
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                    [](unsigned char c){ return std::tolower(c); });
    
    // 检查风扇开启命令
    if (lower_text.find("开启风扇") != std::string::npos || 
        lower_text.find("打开风扇") != std::string::npos || 
        lower_text.find("fan_on") != std::string::npos) {
        FanOn();
        return true;
    }
    
    // 检查风扇关闭命令
    if (lower_text.find("关闭风扇") != std::string::npos || 
        lower_text.find("关掉风扇") != std::string::npos || 
        lower_text.find("fan_off") != std::string::npos) {
        FanOff();
        return true;
    }
    
    // 检查风扇速度设置命令
    if (lower_text.find("风扇速度") != std::string::npos || 
        lower_text.find("设置风扇") != std::string::npos) {
        // 提取速度参数
        int speed = 50; // 默认速度50%
        std::regex speed_regex("\\b([0-9]{1,3})\\b");
        std::smatch match;
        if (std::regex_search(lower_text, match, speed_regex)) {
            int extracted_speed = std::stoi(match[1]);
            if (extracted_speed >= 0 && extracted_speed <= 100) {
                speed = extracted_speed;
            }
        }
        
        FanControl(speed);
        return true;
    }
    
    return false;
}

// WS2812彩灯控制函数
void CarController::RgbLedOn(int id) {
    std::string cmd = "RGB " + std::to_string(id) + " ON";
    SendCommand(cmd);
}

void CarController::RgbLedOff(int id) {
    std::string cmd = "RGB " + std::to_string(id) + " OFF";
    SendCommand(cmd);
}

void CarController::RgbLedSetBrightness(int id, int brightness) {
    // 确保亮度值在0-100之间
    brightness = std::max(0, std::min(100, brightness));
    std::string cmd = "RGB " + std::to_string(id) + " LIGHT " + std::to_string(brightness);
    SendCommand(cmd);
}

void CarController::RgbLedSetGradient(int id, int start_value, int end_value) {
    std::string cmd = "RGB " + std::to_string(id) + " GRADIENT " + 
                    std::to_string(start_value) + " " + std::to_string(end_value);
    SendCommand(cmd);
}

// 解析WS2812彩灯命令
bool CarController::ParseRgbCommand(const std::string& text) {
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), 
                  [](unsigned char c){ return std::tolower(c); });
    
    // 从文本中提取彩灯ID，默认为1
    int rgb_id = 1;
    std::regex rgb_id_regex("彩灯([0-9]+)");
    std::smatch match;
    if (std::regex_search(lower_text, match, rgb_id_regex)) {
        rgb_id = std::stoi(match[1]);
    }
    
    // 支持"打开彩灯"格式
    if (lower_text.find("打开彩灯") != std::string::npos || 
        lower_text.find("开启彩灯") != std::string::npos) {
        ESP_LOGI(TAG, "检测到彩灯 %d 开启命令", rgb_id);
        RgbLedOn(rgb_id);
        return true;
    }
    
    // 支持"关闭彩灯"格式
    if (lower_text.find("关闭彩灯") != std::string::npos || 
        lower_text.find("关掉彩灯") != std::string::npos) {
        ESP_LOGI(TAG, "检测到彩灯 %d 关闭命令", rgb_id);
        RgbLedOff(rgb_id);
        return true;
    }
    
    // 支持"设置彩灯亮度"格式
    if (lower_text.find("设置彩灯亮度") != std::string::npos || 
        lower_text.find("调整彩灯亮度") != std::string::npos || 
        lower_text.find("彩灯亮度") != std::string::npos) {
        // 提取亮度参数，默认为50%
        int brightness = 50;
        std::regex brightness_regex("\\b([0-9]{1,3})\\b");
        if (std::regex_search(lower_text, match, brightness_regex)) {
            int extracted_brightness = std::stoi(match[1]);
            if (extracted_brightness >= 0 && extracted_brightness <= 100) {
                brightness = extracted_brightness;
            }
        }
        
        ESP_LOGI(TAG, "检测到彩灯 %d 亮度设置命令: %d%%", rgb_id, brightness);
        RgbLedSetBrightness(rgb_id, brightness);
        return true;
    }
    
    // 支持"设置彩灯渐变色差"格式
    if (lower_text.find("设置彩灯渐变") != std::string::npos || 
        lower_text.find("彩灯渐变") != std::string::npos || 
        lower_text.find("调整彩灯渐变") != std::string::npos) {
        // 默认渐变值
        int start_value = 0;
        int end_value = 65535;
        
        // 尝试提取两个数值
        std::regex number_regex("\\b([0-9]+)\\b");
        std::smatch number_match;
        std::string::const_iterator search_start(lower_text.cbegin());
        std::vector<int> numbers;
        
        while (numbers.size() < 2 && 
               std::regex_search(search_start, lower_text.cend(), number_match, number_regex)) {
            numbers.push_back(std::stoi(number_match[1]));
            search_start = number_match.suffix().first;
        }
        
        // 如果提取到两个数值，使用提取到的值
        if (numbers.size() >= 2) {
            start_value = numbers[0];
            end_value = numbers[1];
        }
        
        ESP_LOGI(TAG, "检测到彩灯 %d 渐变设置命令: %d - %d", rgb_id, start_value, end_value);
        RgbLedSetGradient(rgb_id, start_value, end_value);
        return true;
    }
    
    return false;
}
