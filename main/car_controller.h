#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <string>
#include <vector>
#include <cJSON.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "boards/atk-dnesp32s3m-wifi/config.h"

// 指令类型
enum class CommandType {
    MOVE,     // 移动控制
    ARM,      // 机械臂控制
    RELAY,    // 继电器控制
    LED,      // LED灯控制
    SERVO,    // 舵机控制
    FAN,      // 风扇控制
    RGB,      // WS2812彩灯控制
    UNKNOWN   // 未知命令
};

// 移动方向
enum class MoveDirection {
    FORWARD,  // 前进
    BACKWARD, // 后退
    LEFT,     // 左转
    RIGHT,    // 右转
    STOP      // 停止
};

class CarController {
public:
    static CarController& GetInstance() {
        static CarController instance;
        return instance;
    }
    
    // 初始化UART
    void Initialize(uart_port_t uart_port = CAR_UART_PORT, int tx_pin = CAR_UART_TX_PIN, int rx_pin = CAR_UART_RX_PIN, int baud_rate = CAR_UART_BAUD_RATE);
    
    // 处理文本响应
    void HandleTextResponse(const cJSON* root);
    
    // 发送基于文本的指令
    void SendCommand(const std::string& cmd);
    
    // 移动控制命令
    void Move(MoveDirection direction, int speed = 50);
    
    // 机械臂控制命令
    void ArmControl(const std::string& action, int servo_id = 0, int angle = 0);
    
    // 继电器控制命令
    void RelayControl(int id, bool on);
    
    // LED灯控制命令
    void LedControl(int id, bool on, int r = 0, int g = 0, int b = 0, int blink_freq = 0);
    
    // 设置 LED 颜色命令，直接使用格式：LED <ID> <R> <G> <B>
    void SetLedColor(int id, int r, int g, int b);
    
    // 舵机控制命令
    void ServoControl(int id, int angle);
    
    // 风扇控制命令
    void FanControl(int speed);
    void FanOn();
    void FanOff();
    
    // WS2812彩灯控制命令
    void RgbLedOn(int id);
    void RgbLedOff(int id);
    void RgbLedSetBrightness(int id, int brightness);
    void RgbLedSetGradient(int id, int start_value, int end_value);

private:
    CarController() : initialized_(false), uart_port_(UART_NUM_1), command_mutex(NULL) {}
    ~CarController();
    
    // 解析文本中的命令类型和参数
    CommandType ParseCommandType(const std::string& text);
    
    // 解析移动命令
    bool ParseMoveCommand(const std::string& text);
    
    // 解析机械臂命令
    bool ParseArmCommand(const std::string& text);
    
    // 解析继电器命令
    bool ParseRelayCommand(const std::string& text);
    
    // 解析LED命令
    bool ParseLedCommand(const std::string& text);
    
    // 解析舵机命令
    bool ParseServoCommand(const std::string& text);
    
    // 解析风扇命令
    bool ParseFanCommand(const std::string& text);
    
    // 解析WS2812彩灯命令
    bool ParseRgbCommand(const std::string& text);
    
    // 从文本中提取关键词
    std::vector<std::string> ExtractKeywords(const std::string& text);
    
    bool initialized_;
    uart_port_t uart_port_;
    SemaphoreHandle_t command_mutex;
};

#endif // CAR_CONTROLLER_H
