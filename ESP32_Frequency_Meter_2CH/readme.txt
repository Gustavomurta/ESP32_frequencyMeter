ESP32 FREQUENCY METER - 2 CHANNELS: 

Best Frequency Meter ever made with ESP32 - awesome! (Arduino Version)
https://www.esp32.com/viewtopic.php?f=19&t=17018&start=80#p135103

I am posting now an upgraded version with 2 chnnels - under development!!!

There is still a problem with the program - the interrupts of Pulse counter 0 is interfering with the interrupts of Pulse counter 1.
This causes the program to crash - I am researching a solution.

I chose another library for the LCD I2C Display. Simpler and functional with the new ESP32 firmware.
I removed the option to connect directly to the LCD, as it was confusing for some.
I changed the inclusion of some ESP32 libraries, to make them compatible with the new ESP32 firmware.

Connect the oscillator outputs (0 or 1) to the frequency meter inputs (CH0 or CH1) to perform tests.
Use the serial console (Arduino IDE - 115200 bps) to change the oscillators frequency for testing (same frequency in the both channels). 
Use resistors R1 and R2 to equalize the voltage level on the I2C bus to 3.3V used in ESP32.

If you are in doubt about the I2C address of your I2C LCD module, use the I2C scanner.
ESP32 - I2C Scanner using Arduino IDE
https://www.esp32.com/viewtopic.php?f=18&t=4742

Add this library LCD-I2C using the Arduino IDE library manager
https://github.com/hasenradball/LCD-I2C
