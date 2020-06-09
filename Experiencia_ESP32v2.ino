// https://github.com/espressif/arduino-esp32/blob/f32083a6d0afad8efc4d52b11091ccf55249dc29/cores/esp32/esp32-hal-timer.c

hw_timer_t *meuTempo = NULL;                          // Instancia do timer
unsigned int prescaler = 80;                          // Prescaler do contador de timer
unsigned long timeTA, timeTB, timeTC, timeTD, timeTE = 0;

//----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                               // Inicializa a serial
  meuTempo = timerBegin(0, prescaler, true);          // usa o timer 0 com o prescaler definido
  timerStart(meuTempo);
}
//----------------------------------------------------------------------------------
void loop()
{
  timerWrite(meuTempo, 0);

  timeTA = timerRead(meuTempo);

  delayMicroseconds (10000);
  timeTA = timerRead(meuTempo);

  delayMicroseconds (10000);
  timeTB = timerRead(meuTempo);

  delayMicroseconds (10000);
  timeTC = timerRead(meuTempo);

  delayMicroseconds (10000);
  timeTD = timerRead(meuTempo);

  delayMicroseconds (10000);
  timeTE = timerRead(meuTempo);

  Serial.println();
  Serial.print("TA "); Serial.println(timeTA);
  Serial.print("TB "); Serial.println(timeTB);
  Serial.print("TC "); Serial.println(timeTC);
  Serial.print("TD "); Serial.println(timeTD);
  Serial.print("TE "); Serial.println(timeTE);
}
//------------------------------------------------------------------------------------
