Documentation https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html#user-guide-s3-devkitc-1-v1-1-header-blocks
i2c api: https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-reference/peripherals/i2c.html
If i get stuck: https://github.com/adafruit/Adafruit_SSD1306/blob/master/Adafruit_SSD1306.cpp#L419

Open Terminal (made default open ESP IDF enviroment with vscode settings)

execute idf.py build

then flash it to whatever port connected to and monitor
idf.py flash -p <PORT> monitor

Also will need to make sure bread board is properly configured with pins being used