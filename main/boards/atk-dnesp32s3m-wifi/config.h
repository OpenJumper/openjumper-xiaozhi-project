#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// 板载配置文件，定义各种引脚和参数
#include <driver/gpio.h>

// 音频采样率设置
#define AUDIO_INPUT_SAMPLE_RATE      24000  // 输入采样率
#define AUDIO_OUTPUT_SAMPLE_RATE     24000  // 输出采样率

// 音量控制按钮
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_48  // 音量增加按钮
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_47  // 音量减小按钮

// I2S音频接口引脚定义
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_6    // 主时钟 (MCLK)
#define AUDIO_I2S_GPIO_WS GPIO_NUM_12     // 字选择/帧同步 (LRCK/WS)
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14   // 位时钟 (SCLK/BCK)
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_13  // 数据输入 (DIN/DO)
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_11   // 数据输出 (DOUT)

// ES8311音频编解码器I2C接口
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_9  // PA功放控制引脚
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_5   // I2C数据线 (8311_SDA)
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_4   // I2C时钟线 (8311_SCL)
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR  // ES8311默认I2C地址

// 启动按钮
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// 板载LED
#define BUILTIN_LED_GPIO GPIO_NUM_1

// TFT液晶屏SPI接口引脚
#define LCD_SCLK_PIN GPIO_NUM_16   // 时钟线 (TFT_SCL)
#define LCD_MOSI_PIN GPIO_NUM_17   // 主输出从输入 (TFT_SDA)
#define LCD_MISO_PIN GPIO_NUM_NC   // 主输入从输出 (未使用)
#define LCD_DC_PIN GPIO_NUM_7      // 数据/命令控制 (TFT_DC)
#define LCD_CS_PIN GPIO_NUM_15     // 片选 (TFT_CS)
#define LCD_RST_PIN GPIO_NUM_18    // 复位 (TFT_RES)

// 音频控制引脚
#define SPK_EN_PIN  GPIO_NUM_42    // 扬声器使能控制

// 显示屏参数配置
#define DISPLAY_WIDTH    320        // 显示屏宽度
#define DISPLAY_HEIGHT   170        // 显示屏高度
#define DISPLAY_MIRROR_X false      // X轴镜像
#define DISPLAY_MIRROR_Y true       // Y轴镜像
#define DISPLAY_SWAP_XY  true       // 交换XY轴

// 显示偏移量设置
#define DISPLAY_OFFSET_X 1          // X轴偏移
#define DISPLAY_OFFSET_Y 26         // Y轴偏移
// #define DISPLAY_OFFSET_X 0       // 备用X轴偏移
// #define DISPLAY_OFFSET_Y 0       // 备用Y轴偏移

// 显示屏背光控制
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_3    // 背光控制引脚 (TFT_BL)
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT true // 背光输出是否反转

// 小车控制UART串口引脚定义
#define CAR_UART_PORT       UART_NUM_0     // 使用UART0，对应TXD0/RXD0引脚
#define CAR_UART_TX_PIN     GPIO_NUM_43    // TXD0引脚
#define CAR_UART_RX_PIN     GPIO_NUM_44    // RXD0引脚
#define CAR_UART_BAUD_RATE  9600          // 波特率115200

#endif // _BOARD_CONFIG_H_

