# Project

The goal of this project was to gain embedded programming experience with an ESP32 device.

## ESP IDF

I first set up the [ESP IDF environment on Windows](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html) so I could start developing. Here are some notable things I needed to do once I created the project:

1. Make sure to change the target device.  
   - This was important as the default is the regular ESP32, but I needed the ESP32-S3.

2. Go into the menuconfig to configure various settings such as the CPU clock speed and flash size.

3. Commands used:
   - `idf.py build create-project ESP-Prj .` (to create project)
   - `idf.py build` (to build the software)
   - `idf.py -p COM8 flash monitor` (to flash to the COM port the device was connected to and monitor output)
   - `idf.py menuconfig` (to modify the configuration for compilation)
   - `idf.py set-target esp32-s3` (to change the target)
   - `idf.py openocd gdbtui` (uses built in JTAG for debugging)

## IDE

I used [VSCodium](https://github.com/VSCodium/vscodium) as my IDE and used the `settings.json` to get the environment set up in the default integrated terminal for the workspace. I also used the `compile_commands.json` file generated from building with `idf.py build` for the [clangd language server](https://clangd.llvm.org/installation) that I use with the IDE.

## Devices/Tools

1. **ESP32-S3-DevKitC-1**
2. **OLED Display**  
   - SSD1306 Driver (this is the important part; I just got a cheap display with this driver)
3. **Breadboard**
4. **Micro USB Cable**
5. **2kΩ Resistor (2)**
6. **Wire (4)**

## General Notes

1. There was odd behavior initially when trying to flash to the board. I needed to keep clicking the reset and boot buttons (I believe pressing both at the same time is what worked) to successfully flash. At some point, while doing so, my computer downloaded JTAG serial drivers, the COM port of the device changed, and I was finally able to flash.  
   - I kept getting the error:  
     `A fatal error occurred: Failed to connect to ESP32: No serial data received.`

2. When I got stuck, reading some existing code for this driver from Arduino helped. Not fully understanding some terms and not reading the entire datasheet thoroughly limited me in most cases.

3. For this device, most of the pins could be used for I2C communication, so I referred to the pin layout to pick some unimportant ones.

4. In this repo, most of the documentation I referred to for information is attached.

5. Using the built-in JTAG was a nightmare. I used Zadig to flash drivers that I probably didn’t even need, which likely caused my COM port to stop working. To fix it, I flashed it with the serial drivers to get the COM port back, but then OpenOCD stopped working. I eventually downloaded these [USB JTAG drivers](https://github.com/espressif/esp-win-usb-drivers/releases) and used Device Manager to install them for the USB JTAG/Serial Debug unit. After that, everything finally started working.

6. I used external resistors that came with the breadboard just for fun. It works just fine with the internal pull-up resistors.

7. I used [LCD Image Converter](https://lcd-image-converter.riuson.com/en/about/) to display my name on the screen. It took me a little longer than I’d like to admit to write the correct `for` loop to match the driver’s expected format, but I got it to work. I’ve included the configuration in the `lcd_convert_config` folder, so if anyone would like to import and use it, they can.

## Breadboard

![Breadboard](https://github.com/LightHardt/ESP32-S3-SSD1306_Display-Prj/blob/main/images/Breadboard%20Setup.jpg)

## Pin Layout

![Pin Layout](https://github.com/LightHardt/ESP32-S3-SSD1306_Display-Prj/blob/main/images/ESP32-S3_DevKitC-1_pinlayout_v1.1.jpg)
