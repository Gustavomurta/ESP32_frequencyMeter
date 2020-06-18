/*
  Measures the frequency of a square wave appliet to a digital input
  Uses a free running timer/counter for the period measurement
  by: rodmcm
  date: 25/7/2018
  Modified by Gustavo Murta
  date : 17/jun/2020
  https://www.esp32.com/viewtopic.php?f=19&t=6533
  OBS: connect input pin only after reset, or ESP32 will enter in a loop...
*/

const byte        interruptPin = 23;              // Assign the interrupt pin
volatile uint64_t StartValue = 0;                 // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second
float             Freq;                           // frequency

hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t

//=======================================
void IRAM_ATTR handleInterrupt()
{
  uint64_t TempVal = timerRead(timer);            // value of timer at interrupt
  PeriodCount = TempVal - StartValue;             // period count between rising edges
  StartValue = TempVal;                           // puts latest reading as start for next calculation
}

//======================================
void setup()
{
  Serial.begin(115200);
  pinMode(interruptPin, INPUT);                                       // sets pin as input
  
  attachInterrupt(interruptPin, handleInterrupt, FALLING);            // attaches pin to interrupt on Falling Edge
  timer = timerBegin(0, 2, true);                                     // configure timer 
  // 0 = first timer
  // 2 is prescaler so 80 MHZ divided by 2 = 40 MHZ signal
  // true - counts up
  timerStart(timer);                                                  // starts the timer
}

void loop()
{
  Freq = 40000000.00 / PeriodCount;                                  // calculate frequency 
  Serial.print("Frequency   "); Serial.println(Freq, 0);
  delay(250);
}
