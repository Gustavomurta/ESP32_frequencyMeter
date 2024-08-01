// ESP32 Frequency Meter / 2 Channel - Version 2
// ESP32 DevKit + I2C PCF8574 LCD
// Arduino IDE 2.3.2   / ESP32 Arduino V 3.02
// Gustavo Murta e Rui Viana august/2020 - 2024/07/29
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// https://www.esp32.com/viewtopic.php?f=19&t=17018
// References:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
// LCD I2C SDA - GPIO_21
// LCD I2C SCL - GPIO_22

#include "stdio.h"            // Library STDIO
#include "driver/ledc.h"      // Library ESP32 LEDC
#include "driver/pcnt.h"      // Library ESP32 PCNT
#include "soc/pcnt_struct.h"  // Library ESP32 PCNT
#include "esp32/rom/gpio.h"   // Library ESP32 GPIO

#include <LCD-I2C.h>       // Library LCD with PCF8574
LCD_I2C lcd(0x3F, 16, 2);  // Instance LCD I2C - address 0x3F

#define PCNT_COUNT_UNIT_0 PCNT_UNIT_0        // Set Pulse Counter Unit - 0
#define PCNT_COUNT_CHANNEL_0 PCNT_CHANNEL_0  // Set Pulse Counter channel - 0
#define PCNT_INPUT_SIG_IO_0 GPIO_NUM_34      // Set Pulse Counter input - Freq Meter Input GPIO 34
#define PCNT_INPUT_CTRL_IO_0 GPIO_NUM_35     // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down
#define OUTPUT_CONTROL_GPIO_0 GPIO_NUM_32    // Timer output control port - GPIO_32

#define PCNT_COUNT_UNIT_1 PCNT_UNIT_1        // Set Pulse Counter Unit - 1
#define PCNT_COUNT_CHANNEL_1 PCNT_CHANNEL_1  // Set Pulse Counter channel - 0
#define PCNT_INPUT_SIG_IO_1 GPIO_NUM_25      // Set Pulse Counter input - Freq Meter Input GPIO 25
#define PCNT_INPUT_CTRL_IO_1 GPIO_NUM_26     // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down
#define OUTPUT_CONTROL_GPIO_1 GPIO_NUM_27    // Timer output control port - GPIO_27

#define LEDC_HS_CH0_GPIO GPIO_NUM_33  // LEDC output - pulse generator 0 - GPIO_33
#define LEDC_HS_CH1_GPIO GPIO_NUM_14  // LEDC output - pulse generator 1 - GPIO_14
#define IN_BOARD_LED GPIO_NUM_2       // ESP32 native LED - GPIO 2

#define PCNT_H_LIM_VAL overflow  // Overflow of Pulse Counter

uint32_t overflow = 32000;       // Max Pulse Counter value 32000 - limit 32767
uint32_t sample_time = 1000000;  // Sample time of 1 second to count pulses (change the value to calibrate frequency meter)
uint calibrator = 1;             // calibrator of frequency reading (may be + or - integer numbers)

bool flag_0 = true;         // Flag to enable print frequency reading
int16_t pulses_0 = 0;       // Pulse Counter value
uint32_t multPulses_0 = 0;  // Number of PCNT counter overflows
float frequency_0 = 0;      // frequency value

bool flag_1 = true;         // Flag to enable print frequency reading
int16_t pulses_1 = 0;       // Pulse Counter value
uint32_t multPulses_1 = 0;  // Number of PCNT counter overflows
float frequency_1 = 0;      // frequency value

uint32_t osc_freq_0 = 38000;  // Oscillator frequency - initial 32000 Hz (may be 1 Hz to 40 MHz)
uint32_t mDuty_0 = 0;         // Duty cycle value
uint32_t resolution_0 = 0;    // Resolution value of Oscillator

uint32_t osc_freq_1 = 10000;  // Oscillator frequency - initial 16000 Hz (may be 1 Hz to 40 MHz)
uint32_t mDuty_1 = 0;         // Duty cycle value
uint32_t resolution_1 = 0;    // Resolution value of Oscillator

char buf_0[32];  // Buffer 0
char buf_1[32];  // Buffer 1

esp_timer_create_args_t timer_args_0;  // Create an esp_timer instance
esp_timer_handle_t timer_handle_0;     // Create an single timer

esp_timer_create_args_t timer_args_1;  // Create an esp_timer instance
esp_timer_handle_t timer_handle_1;     // Create an single timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // portMUX_TYPE to do synchronism

//----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                            // Serial Console Arduino 115200 Bps
  Serial.println();                                                // Print line
  Serial.println(" Input the Frequency - 1 to 40 MHz");            // Console print
  Serial.println(" Oscillators have some accuracy limitations ");  // Console print
  Serial.println();                                                // Print line

  lcd.begin();
  lcd.display();
  lcd.backlight();
  lcd.setCursor(0, 0);    // Set cursor position - column and row
  lcd.print("F0:");       // LCD print
  lcd.setCursor(0, 1);    // Set cursor position - column and row
  lcd.print("F1:");       // LCD print
  init_frequencyMeter();  // Initialize Frequency Meter
}

//----------------------------------------------------------------------------
void init_osc_freq_0()  // Initialize Oscillator to test Freq Meter
{
  resolution_0 = (log(80000000 / osc_freq_0) / log(2)) / 2;  // Calc of resolution of Oscillator
  if (resolution_0 < 1) resolution_0 = 1;                    // set min resolution
  // Serial.println(resolution);                            // Print
  mDuty_0 = (pow(2, resolution_0)) / 2;  // Calc of Duty Cycle 50% of the pulse
  // Serial.println(mDuty);                                 // Print

  ledc_timer_config_t ledc_timer = {};  // LEDC timer config instance

  ledc_timer.duty_resolution = ledc_timer_bit_t(resolution_0);  // Set resolution
  ledc_timer.freq_hz = osc_freq_0;                              // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                 // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_0;                          // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                               // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};  // LEDC Channel config instance

  ledc_channel.channel = LEDC_CHANNEL_0;           // Set HS Channel - 0
  ledc_channel.duty = mDuty_0;                     // Set Duty Cycle 50%
  ledc_channel.gpio_num = LEDC_HS_CH0_GPIO;        // LEDC Oscillator output GPIO 33
  ledc_channel.intr_type = LEDC_INTR_DISABLE;      // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;  // Set LEDC high speed mode
  ledc_channel.timer_sel = LEDC_TIMER_0;           // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);              // Config LEDC channel
}

//----------------------------------------------------------------------------
void init_osc_freq_1()  // Initialize Oscillator to test Freq Meter
{
  resolution_1 = (log(80000000 / osc_freq_1) / log(2)) / 2;  // Calc of resolution of Oscillator
  if (resolution_1 < 1) resolution_1 = 1;                    // set min resolution
  // Serial.println(resolution);                            // Print
  mDuty_1 = (pow(2, resolution_1)) / 2;  // Calc of Duty Cycle 50% of the pulse
  // Serial.println(mDuty);                                 // Print

  ledc_timer_config_t ledc_timer = {};  // LEDC timer config instance

  ledc_timer.duty_resolution = ledc_timer_bit_t(resolution_1);  // Set resolution
  ledc_timer.freq_hz = osc_freq_1;                              // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                 // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_1;                          // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                               // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};  // LEDC Channel config instance

  ledc_channel.channel = LEDC_CHANNEL_1;           // Set HS Channel - 0
  ledc_channel.duty = mDuty_1;                     // Set Duty Cycle 50%
  ledc_channel.gpio_num = LEDC_HS_CH1_GPIO;        // LEDC Oscillator output GPIO 14
  ledc_channel.intr_type = LEDC_INTR_DISABLE;      // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;  // Set LEDC high speed mode
  ledc_channel.timer_sel = LEDC_TIMER_1;           // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);              // Config LEDC channel
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler_0(void *arg)  // Counting overflow pulses
{
  multPulses_0++;                             // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT_0);  // Clear Pulse Counter interrupt bit
  pcnt_counter_resume(PCNT_COUNT_UNIT_0);     // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler_1(void *arg)  // Counting overflow pulses
{
  multPulses_1++;                             // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT_1);  // Clear Pulse Counter interrupt bit
  pcnt_counter_resume(PCNT_COUNT_UNIT_1);     // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
void init_PCNT_0(void)  // Initialize and run PCNT 0 unit
{
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO_0);                  // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO_0, GPIO_MODE_OUTPUT);  // Set GPIO 32 as Timer output
  timer_args_0.callback = read_PCNT_0;                          // Function to call when the timer expires - stop counter
  esp_timer_create(&timer_args_0, &timer_handle_0);             // Create esp-timer instance / single timer

  pcnt_config_t pcnt_config_0 = {};                    // PCNT unit instance
  pcnt_config_0.pulse_gpio_num = PCNT_INPUT_SIG_IO_0;  // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config_0.ctrl_gpio_num = PCNT_INPUT_CTRL_IO_0;  // Pulse Count Control signal - input GPIO 35
  pcnt_config_0.unit = PCNT_COUNT_UNIT_0;              // Pulse Count Unit PCNT - 0
  pcnt_config_0.channel = PCNT_COUNT_CHANNEL_0;        // Pulse Count Unit Channel - 0
  pcnt_config_0.counter_h_lim = PCNT_H_LIM_VAL;        // Maximum counter value - 32000 pulses
  pcnt_config_0.pos_mode = PCNT_COUNT_INC;             // PCNT positive edge count mode - inc
  pcnt_config_0.neg_mode = PCNT_COUNT_INC;             // PCNT negative edge count mode - inc
  pcnt_config_0.lctrl_mode = PCNT_MODE_DISABLE;        // PCNT low control mode - disable
  pcnt_config_0.hctrl_mode = PCNT_MODE_KEEP;           // PCNT high control mode - won't change counter mode  

  pcnt_unit_config(&pcnt_config_0);                       // Initialize PCNT unit
  pcnt_event_enable(PCNT_COUNT_UNIT_0, PCNT_EVT_H_LIM);   // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler_0, NULL, 0, NULL);  // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT_0);                    // Enable interrupts for PCNT unit
}

//----------------------------------------------------------------------------------
void init_PCNT_1(void)  // Initialize and run PCNT 1 unit
{
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO_1);                  // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO_1, GPIO_MODE_OUTPUT);  // Set GPIO 27 as Timer output
  timer_args_1.callback = read_PCNT_1;                          // Function to call when the timer expires - stop counter
  esp_timer_create(&timer_args_1, &timer_handle_1);             // Create esp-timer instance / single timer

  pcnt_config_t pcnt_config_1 = {};                    // PCNT unit instance
  pcnt_config_1.pulse_gpio_num = PCNT_INPUT_SIG_IO_1;  // Pulse input GPIO 25 - Freq Meter Input
  pcnt_config_1.ctrl_gpio_num = PCNT_INPUT_CTRL_IO_1;  // Pulse Count Control signal - input GPIO 26
  pcnt_config_1.unit = PCNT_COUNT_UNIT_1;              // Pulse Count Unit PCNT - 1
  pcnt_config_1.channel = PCNT_COUNT_CHANNEL_1;        // Pulse Count Unit Channel - 1
  pcnt_config_1.counter_h_lim = PCNT_H_LIM_VAL;        // Maximum counter value - 32000 pulses
  pcnt_config_1.pos_mode = PCNT_COUNT_INC;             // PCNT positive edge count mode - inc
  pcnt_config_1.neg_mode = PCNT_COUNT_INC;             // PCNT negative edge count mode - inc
  pcnt_config_1.lctrl_mode = PCNT_MODE_DISABLE;        // PCNT low control mode - disable
  pcnt_config_1.hctrl_mode = PCNT_MODE_KEEP;           // PCNT high control mode - won't change counter mode 

  pcnt_unit_config(&pcnt_config_1);                       // Initialize PCNT unit
  pcnt_event_enable(PCNT_COUNT_UNIT_1, PCNT_EVT_H_LIM);   // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler_1, NULL, 1, NULL);  // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT_1);                    // Enable interrupts for PCNT unit
}

//----------------------------------------------------------------------------------
void read_PCNT_0(void *p)  // Read Pulse Counter 0 - timeout callback
{
  pcnt_get_counter_value(PCNT_COUNT_UNIT_0, &pulses_0);  // Read Pulse Counter value
  gpio_set_level(OUTPUT_CONTROL_GPIO_0, 0);              // Stop counter - GPIO 32 - output control LOW
  flag_0 = true;                                         // Change flag 0 to enable print
}

//----------------------------------------------------------------------------------
void read_PCNT_1(void *p)  // Read Pulse Counter 1 - timeout callback
{
  pcnt_get_counter_value(PCNT_COUNT_UNIT_1, &pulses_1);  // Read Pulse Counter value
  gpio_set_level(OUTPUT_CONTROL_GPIO_1, 0);              // Stop counter - GPIO 27 - output control LOW
  flag_1 = true;                                         // Change flag 1 to enable print
}

//---------------------------------------------------------------------------------
void init_frequencyMeter() {
  init_osc_freq_0();  // Config and Initialize Oscillator 0
  init_osc_freq_1();  // Config and Initialize Oscillator 1

  init_PCNT_0();  // Config and Initialize PCNT 0 unit
  init_PCNT_1();  // Config and Initialize PCNT 1 unit

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);               // Set LED inboard as output
  gpio_matrix_in(PCNT_INPUT_SIG_IO_0, SIG_IN_FUNC226_IDX, false);   // Set GPIO matrin IN - Freq Meter CH0 input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);  // Set GPIO matrix OUT - to inboard LED
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos)  // Format an unsigned long (32 bits) into a string
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = ',';  // decimal separator
  return s;
}
//----------------------------------------------------------------------------------------
char *ltos(long val, char *s, int radix)  // Format an long (32 bits) into a string
{
  if (radix < 2 || radix > 36) {
    s[0] = 0;
  } else {
    char *p = s;
    if (radix == 10 && val < 0) {
      val = -val;
      *p++ = '-';
    }
    p = ultos_recursive(val, p, radix, 0) - 1;
    *p = 0;
  }
  return s;
}

//---------------------------------------------------------------------------------
void loop() {
  if (flag_0 == true) {  // If count 0 has ended
    flag_0 = false;      // Change flag to enable count

    frequency_0 = (pulses_0 + (multPulses_0 * (overflow + calibrator))) / 2;  // Calculation of frequency
    printf("Frequency CH0: %s", (ltos(frequency_0, buf_0, 10)));              // Print frequency with commas
    printf(" Hz \n");                                                         // Print unity Hz

    lcd.setCursor(3, 0);         // Set cursor position - column and row
    lcd.print("             ");  // Clear Field
    lcd.setCursor(3, 0);
    lcd.print((ltos(frequency_0, buf_0, 10)));  // LCD print frequency
    lcd.print(" Hz");                           // LCD print unity Hz

    multPulses_0 = 0;                                   // Clear overflow counter
    pcnt_counter_clear(PCNT_COUNT_UNIT_0);           // Clear Pulse Counter 0   pcnt_unit_clear_count
    esp_timer_start_once(timer_handle_0, sample_time);  // Start High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO_0, 1);           // Set enable PCNT count 0 - GPIO 32
  }

  if (flag_1 == true) {  // If count 1 has ended
    flag_1 = false;      // Change flag to enable count

    frequency_1 = (pulses_1 + (multPulses_1 * (overflow + calibrator))) / 2;  // Calculation of frequency
    printf("Frequency CH1: %s", (ltos(frequency_1, buf_1, 10)));              // Print frequency with commas
    printf(" Hz \n");                                                         // Print unity Hz
    Serial.println();                                                         // Print line

    lcd.setCursor(3, 1);         // Set cursor position - column and row
    lcd.print("             ");  // Clear field
    lcd.setCursor(3, 1);
    lcd.print((ltos(frequency_1, buf_1, 10)));  // LCD print frequency
    lcd.print(" Hz");                           // LCD print unity Hz

    multPulses_1 = 0;                                   // Clear overflow counter
    pcnt_counter_clear(PCNT_COUNT_UNIT_1);              // Clear Pulse Counter 1
    esp_timer_start_once(timer_handle_1, sample_time);  // Start High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO_1, 1);           // Set enable PCNT count 1 - GPIO 27
  }

  // Input of frequency value
  String inputString = "";  // clear temporary string
  osc_freq_0 = 0;           // Clear oscillator 0 frequency
  osc_freq_1 = 0;           // Clear oscillator 1 frequency
  while (Serial.available()) {
    char inChar = (char)Serial.read();  // Reads a byte on the console
    inputString += inChar;              // Add char to string
    if (inChar == '\n')                 // If new line (enter)
    {
      osc_freq_0 = inputString.toInt();  // Converts String into integer value
      osc_freq_1 = osc_freq_0;
      inputString = "";  // Clear string
    }
  }
  if (osc_freq_0 != 0)  // If some value inputted to oscillator frequency
  {
    init_osc_freq_0();  // reconfigure ledc function - oscillator 0
    init_osc_freq_1();  // reconfigure ledc function - oscillator 1
  }
}
