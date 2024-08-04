// ESP32 Frequency Meter / 2 Channel - Version 3  (use this Version) 
// ESP32 DevKit + I2C PCF8574 LCD
// Arduino IDE 2.3.2   / ESP32 Arduino V 3.02

// Gustavo Murta e Rui Viana august/2020 - 2024/07/30

// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// https://www.esp32.com/viewtopic.php?f=19&t=17018

// References:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html

// LCD I2C SDA - GPIO_21
// LCD I2C SCL - GPIO_22


///////////////////////////////////////////////////
**Arduino Compiler Output**

Sketch uses 316797 bytes (24%) of program storage space. Maximum is 1310720 bytes.
Global variables use 20856 bytes (6%) of dynamic memory, leaving 306824 bytes for local variables. Maximum is 327680 bytes.
esptool.py v4.6
Serial port COM14
Connecting....
Chip is ESP32-D0WD-V3 (revision v3.0)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: 3c:e9:0e:85:2f:1c
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 921600
Changed.
Configuring flash size...
Flash will be erased from 0x00001000 to 0x00005fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x0000e000 to 0x0000ffff...
Flash will be erased from 0x00010000 to 0x0005dfff...
Compressed 19744 bytes to 13604...
Writing at 0x00001000... (100 %)
Wrote 19744 bytes (13604 compressed) at 0x00001000 in 0.4 seconds (effective 361.1 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 146...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (146 compressed) at 0x00008000 in 0.1 seconds (effective 441.1 kbit/s)...
Hash of data verified.
Compressed 8192 bytes to 47...
Writing at 0x0000e000... (100 %)
Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.1 seconds (effective 624.7 kbit/s)...
Hash of data verified.
Compressed 317168 bytes to 179910...
Writing at 0x00010000... (9 %)
Writing at 0x0001c222... (18 %)
Writing at 0x000271b9... (27 %)
Writing at 0x0002c76e... (36 %)
Writing at 0x00032114... (45 %)
Writing at 0x00037716... (54 %)
Writing at 0x0003cd9f... (63 %)
Writing at 0x0004228b... (72 %)
Writing at 0x00047602... (81 %)
Writing at 0x0004ce90... (90 %)
Writing at 0x0005618b... (100 %)
Wrote 317168 bytes (179910 compressed) at 0x00010000 in 3.5 seconds (effective 732.7 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...

//////////////////////////////////////////////////////
**Arduino Console - COM14 - 115200 baud**

ets Jul 29 2019 12:21:46

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0030,len:1448
load:0x40078000,len:14844
ho 0 tail 12 room 4
load:0x40080400,len:4
load:0x40080404,len:3356
entry 0x4008059c
E (156) esp_cor�Eյ�}���͡� Core dump data check failed:   <<<<<<<  Please help me to solve this bug
Calculated checksum='cbb3a019'
Image chec
 Enter the Frequency - 1 to 40 MHz - First Osc 0 / Second Osc 1
 Oscillators have some accuracy limitations 

Frequency CH1: 0 Hz 

Frequency CH0: 10,000 Hz 
Frequency CH1: 20,001 Hz 

Frequency CH0: 10,000 Hz 
Frequency CH1: 20,000 Hz 

Frequency CH0: 10,000 Hz 
Frequency CH1: 20,000 Hz 

//////////////////////////////////////////
