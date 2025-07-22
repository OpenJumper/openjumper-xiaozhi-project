// 强制使用ST7789 LCD显示屏，禁用SSD1306 OLED显示屏
#define CONFIG_BOARD_TYPE_ESP32S3_LCD 1
#define CONFIG_LCD_ST7789_170X320 1
#define CONFIG_DISPLAY_OLED_NONE 1

#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "driver/gpio.h"
#include "assets/lang_config.h"
#include "car_controller.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#define TAG "atk_dnesp32s3m_wifi"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class atk_dnesp32s3m_wifi : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    LcdDisplay* display_;

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    // Initialize spi peripheral
    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = LCD_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = LCD_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        boot_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        boot_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });

    }


    
    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 1. 初始化SPI总线的液晶屏控制IO
        ESP_LOGI(TAG, "Install ST7789 panel IO over SPI bus");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = LCD_CS_PIN;
        io_config.dc_gpio_num = LCD_DC_PIN;
        io_config.spi_mode = 0;  // ST7789使用SPI模式0
        io_config.pclk_hz = 60 * 1000 * 1000;  // 60MHz SPI时钟频率
        io_config.trans_queue_depth = 10;  // 增加传输队列深度，提高性能
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        esp_err_t ret = esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create LCD panel IO: %s", esp_err_to_name(ret));
            return;
        }

        // 2. 初始化ST7789液晶屏驱动芯片
        ESP_LOGI(TAG, "Install ST7789 LCD driver (170x320)");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = LCD_RST_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;  // BGR顺序显示更正确的颜色
        panel_config.bits_per_pixel = 16;  // 16位色深（RGB565）
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_BIG;
        ret = esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create LCD panel: %s", esp_err_to_name(ret));
            return;
        }

        // 3. 复位并初始化显示面板
        ESP_LOGI(TAG, "Resetting and initializing ST7789 panel");
        esp_lcd_panel_reset(panel);  // 硬件复位
        esp_lcd_panel_init(panel);   // 初始化

        // 4. 发送ST7789颜色校正命令
        ESP_LOGI(TAG, "Setting ST7789 color correction parameters");
        uint8_t data0[] = {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
        uint8_t data1[] = {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
        esp_lcd_panel_io_tx_param(panel_io, 0xe0, data0, 16);  // 正伽马校正
        esp_lcd_panel_io_tx_param(panel_io, 0xe1, data1, 16);  // 负伽马校正

        // 5. 配置显示方向和颜色设置
        ESP_LOGI(TAG, "Configuring display orientation and color settings");
        esp_lcd_panel_invert_color(panel, true);  // ST7789需要反转颜色
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);  // 根据方向设置交换XY
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);  // 根据方向设置镜像
        
        // 6. 创建SpiLcdDisplay对象
        ESP_LOGI(TAG, "Creating SpiLcdDisplay object with ST7789 panel");
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, 
                                    DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_16_4,
                                        .icon_font = &font_awesome_16_4,
                                        .emoji_font = font_emoji_32_init(),
                                    });
    }

    // 物联网初始化，添加对 AI 可见设备 
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Car"));
    }
    
    // 初始化小车控制器
    void InitializeCarController() {
        // 使用UART串口通信控制小车
        CarController::GetInstance().Initialize(CAR_UART_PORT, CAR_UART_TX_PIN, CAR_UART_RX_PIN, CAR_UART_BAUD_RATE);
        ESP_LOGI(TAG, "小车控制器(UART)初始化完成");
    }

public:
    atk_dnesp32s3m_wifi() : 
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        // 初始化I2C和SPI总线
        InitializeCodecI2c();
        InitializeSpi();
        
        // 初始化显示
        InitializeSt7789Display();
        
        // 初始化按钮和物联网
        InitializeButtons();
        InitializeIot();
        InitializeCarController();
        
        // 恢复背光亮度
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(atk_dnesp32s3m_wifi);
