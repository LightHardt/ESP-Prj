#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_0      // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           GPIO_NUM_1      // GPIO number used for I2C master data 
#define I2C_MASTER_NUM              0               // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip (my case there are 2 esp32-s3)
#define I2C_MASTER_FREQ_HZ          400000          // I2C master clock frequency

// SSD1306 Commands refer to data sheet
#define SSD1306_DISPLAYOFF           0xAE        // Turn off display
#define SSD1306_DISPLAYON            0xAF        // Turn off display
#define SSD1306_SETDISPLAYCLOCKRATIO 0xD5        // set display clock ratio
#define SSD1306_SETMULTIPLEXRATIO    0xA8        // set multiplex ratio
#define SSD1306_SETDISPLAYOFFSET     0xD3        // set display offset
#define SSD1306_SETSTARTLINEADDRESS  0x40        // set start line address
#define SSD1306_SETCHARGEPUMP        0x8D        // set charge pump
#define SSD1306_DISPLAYALLON_RESUME  0xA4        // Go back to normal display whats in buffer
#define SSD1306_DISPLAYALLON         0xA5        // Turn on all pixels on display
#define SSD1306_SETPAGEADRESS        0x22        // Need to set page adress range as it determines where draw on screen
#define SSD1306_SETCOLUMNADRESS      0X21        // Need to set column adress range as it determines where draw on screen
#define SSD1306_SETCONTRAST          0X81        // Need to set cONTRAST
#define SSD1306_SETADDRESSINGMODE    0X20        // Need to set cONTRAST

// Display constants
#define DISPLAY_WIDTH                 128
#define DISPLAY_HEIGHT                64

// Need these handles to communicate with devices 
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

// Buffer that will hold display data
uint8_t display_buffer[DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8)] = {0}; // Width * ((height / 8) Written this way as each page is 8 pixels high you want total height to be divided by 8
size_t display_buffer_len = DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8);

// Tag for logging functionality
static const char* TAG = "i2cLed";

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_master_port,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI(TAG, "Attempting master i2c init");
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));

    // Setup i2c device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C, // On back of device
        .scl_speed_hz = I2C_MASTER_FREQ_HZ, // 400khz
    };

    ESP_LOGI(TAG, "Attempting i2c connection to display");
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, 0x3C, -1));
}

static void ssd1306_send_command(uint8_t c) {
    uint8_t buffer[2];
    buffer[0] = 0x00; // Control byte: Co=0, D/C#=0 (command mode)
    buffer[1] = c;    // Command

    ESP_LOGI(TAG, "Attempting to send command to display");
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buffer, sizeof(buffer), -1));
}

static void ssd1306_send_data(void) {

    uint8_t bytes_per_page = 128; // 128x64 there is 128 bytes per page

    // Due to hardware limitation i can not send all at once i need to split up display buffer cant send it all at once
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_send_command(SSD1306_SETPAGEADRESS);    // set page address command)
        ssd1306_send_command(page);                     // start at 0
        ssd1306_send_command(page);                     // end at 7 (max)
        ssd1306_send_command(SSD1306_SETCOLUMNADRESS);
        ssd1306_send_command(0x00);                     // start at column 0
        ssd1306_send_command(0x7F);                     // end at 127 (max)

        // send byte telling it to recieve data
        uint8_t page_buffer[bytes_per_page + 1]; //  add one for data byte
        page_buffer[0] = 0x40; // write data
        memcpy(&page_buffer[1], &display_buffer[page * bytes_per_page], bytes_per_page);

        ESP_LOGI(TAG, "Attempting to send page %d of display buffer", page);
        // attempt to write display data
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, page_buffer, sizeof(page_buffer), -1));
    }
    ESP_LOGI(TAG, "Succesfully sent display buffer");
}

// Letting chat GPT attempt to make a cool image
// Gave it two attempts first try was pretty bad second one was better but not good either
void generate_smiley(void) {
    memset(display_buffer, 0x00, display_buffer_len); // Clear the buffer

    // Face outline (circle)
    for (int col = 30; col < 98; col++) {
        display_buffer[1 * DISPLAY_WIDTH + col] = 0x18; // Top edge of the circle
        display_buffer[6 * DISPLAY_WIDTH + col] = 0x18; // Bottom edge of the circle
    }
    for (int col = 28; col < 100; col++) {
        display_buffer[2 * DISPLAY_WIDTH + col] = 0xFF; // Middle of the top arc
        display_buffer[5 * DISPLAY_WIDTH + col] = 0xFF; // Middle of the bottom arc
    }
    for (int col = 26; col < 102; col++) {
        display_buffer[3 * DISPLAY_WIDTH + col] = 0xFF; // Side arc
        display_buffer[4 * DISPLAY_WIDTH + col] = 0xFF; // Side arc
    }

    // Eyes
    display_buffer[2 * DISPLAY_WIDTH + 50] = 0x3C; // Left eye (vertical)
    display_buffer[2 * DISPLAY_WIDTH + 78] = 0x3C; // Right eye (vertical)

    // Smile
    for (int col = 45; col < 83; col++) {
        display_buffer[5 * DISPLAY_WIDTH + col] = 0x66; // Upper smile curve
        display_buffer[6 * DISPLAY_WIDTH + col] = 0x18; // Lower smile curve
    }
}

void app_main(void)
{
    i2c_master_init();
    ESP_LOGI(TAG, "Succesfully initialized i2c connection");

    // *******INIT DISPLAY******* //
    // Step 1: Turn the display OFF
    ssd1306_send_command(SSD1306_DISPLAYOFF); // Display OFF

    // Step 2: Set Display Clock Divide Ratio and Oscillator Frequency
    ssd1306_send_command(SSD1306_SETDISPLAYCLOCKRATIO);
    ssd1306_send_command(0x80); // Default value for the divide ratio and oscillator frequency

    // Step 3: Set Multiplex Ratio
    ssd1306_send_command(SSD1306_SETMULTIPLEXRATIO);
    ssd1306_send_command(0x3F); // 0x3F for 64 rows (default for 128x64)

    // Step 4: Set Display Offset
    ssd1306_send_command(SSD1306_SETDISPLAYOFFSET);
    ssd1306_send_command(0x00); // No offset

    // Step 5: Set Start Line Address
    ssd1306_send_command(SSD1306_SETSTARTLINEADDRESS | 0x0); // Line 0

    // Step 6: Enable Charge Pump
    ssd1306_send_command(SSD1306_SETCHARGEPUMP);
    ssd1306_send_command(0x14); // Enable charge pump

    // Step 7: Set Contrast Control
    ssd1306_send_command(SSD1306_SETCONTRAST);
    ssd1306_send_command(0x7F); // Medium contrast

    // Step 8: Set Adressing
    ssd1306_send_command(SSD1306_SETADDRESSINGMODE);
    ssd1306_send_command(0x22); // Page addressing

    // Step 9: Turn the display ON
    ssd1306_send_command(SSD1306_DISPLAYON); // Display ON
    // ************************ //

    // Make sure display works and not noticeable damage
    ssd1306_send_command(SSD1306_DISPLAYALLON);
    ESP_LOGI(TAG, "Succesfully sent display all on command");

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Leave on for 5 seconds so i can visually see display all lit up

    ssd1306_send_command(SSD1306_DISPLAYALLON_RESUME); // Now use what is in buffer for display
    ESP_LOGI(TAG, "Succesfully sent display resume command");

    // manipulate display buffer to show some level of functionality

    // Expecting the bottom two bits of each "page" to be on
    memset(display_buffer, 3, display_buffer_len);
    ssd1306_send_data();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // So i have time to look at display

    // Expecting to see chatgpts botched smiley face lol
    generate_smiley();
    ssd1306_send_data();

    for(;;) {
        // Need this delay or else the freertos watchdog will get mad as no idle time
        vTaskDelay(1);
    }

}
