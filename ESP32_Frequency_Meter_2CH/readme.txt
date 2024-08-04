ESP32 FREQUENCY METER - 2 CHANNELS: 

Best Frequency Meter ever made with ESP32 - awesome! (Arduino Version)
https://www.esp32.com/viewtopic.php?f=19&t=17018&p=135624#p135624

As promised, I managed to fix the interference problem between the interrupts of the two Pulse Counters 0 and 1.
The solution was to disable the Pulse Counter interrupt right after the complete count using the "pcnt_isr_unregister" function.
After printing the value of frequency 0, I enabled the interrupt of Pulse Counter 1 (function "pcnt_isr_register").
And after printing the value of frequency 1, I enabled the interrupt of Pulse Counter 0. It worked!
Every second, a counter counts the pulses and calculates the frequency. Everything is controlled with the 1-second One-shot Timers.

To test the frequency counter, you can use the signals from both oscillators (initial values ​​for Osc 0 = 10,000 Hz and Osc 1 = 20,000 Hz).
Connect the oscillator output to the frequency counter channel input.
To change the oscillator frequencies, enter one value at a time in the Arduino IDE serial console (115200 bps).
The first value will be used in Oscillator 0 and the second value will be used in Oscillator 1. Very simple.

There is only one more challenge left - when the CH0 channel input is disconnected, the frequency value varies different from 0 Hz.
But the CH1 channel, when disconnected presents a frequency of 0 Hz (which is the expected value).
Connect the CH0 channel to an oscillator circuit and the frequency will be measured accurately!

I implemented a variable for calibrating measurements (offset)- calibrator => calibrator of frequency reading (may be + or - integer numbers).
Use resistors R1 and R2 to equalize the voltage level on the I2C bus to 3.3V used in ESP32.

If you are in doubt about the I2C address of your I2C LCD module, use the I2C scanner.
ESP32 - I2C Scanner using Arduino IDE
https://www.esp32.com/viewtopic.php?f=18&t=4742

Add this library LCD-I2C using the Arduino IDE library manager
https://github.com/hasenradball/LCD-I2C

ESP32 API references used in this project:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
