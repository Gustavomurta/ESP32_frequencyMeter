// Frequencimetro ESP32
// Rui Viana e Gustavo Murta - 11/junho/2020
//
// referencias  https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/pcnt.h
//              https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/soc/soc/pcnt_reg.
//              https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/soc/soc/pcnt_struct.h
//              https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/esp32/esp_intr_alloc.h
//              https://github.com/espressif/esp-idf/tree/master/examples/peripherals/pcnt
//
//              https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-timer.h
//              https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-timer.c


#include "driver/pcnt.h"                                  // Biblioteca de pulse count

#define PCNT_FREQ_UNIT      PCNT_UNIT_0                   // Unidade de Pulse Count 0 
#define PCNT_H_LIM_VAL      32767                         // Limite superior de contagem 32767 
#define PCNT_INPUT_SIG_IO   4                             // Pulse Input GPIO 4 

int16_t contador = 0;                                     // Contador de pulsos - valor max 65536
int contadorOverflow;                                     // Contador de overflow do Contador de Pulsos
float frequencia = 0;                                     // Frequencia medida
String unidade;                                           // Unidade de medida da escala
unsigned long tempo;                                      // base de tempo da medida dos pulsos
int prescaler;                                            // Divisor de frequencia do Timer
bool contadorOK = false;

pcnt_isr_handle_t user_isr_handle = NULL;                 // handler da interrupção - não usado
hw_timer_t * timer = NULL;                                // Instancia do timer

//----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                                   // Inicializa a console serial
  contadorOverflow = 0;                                   // zera o Contador de overflow
  prescaler = 8;                                          // Prescaler do contador do timer = 80 Mhz / 8 = 10 MHz
  tempo = 10000000;                                       // base de tempo da medida dos pulsos - 10 milhões
  unidade = "Hz";                                         // unidade de Hz

  iniciaContadorPulsos();                                 // Inicializa o contador de pulsos
  startTimer();                                           // Inicializa o timer
}

//------------------------------------------------------------------------------------
void IRAM_ATTR overflowContador(void *arg)                // Rotina de interrupção de overflow do Contador
{
  contadorOverflow = contadorOverflow + 1;                // soma contador de overflow
  PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                 // Limpa flag de overflow
  pcnt_counter_clear(PCNT_FREQ_UNIT);                     // Zera e reseta o Contador de Pulsos
}

//------------------------------------------------------------
void iniciaContadorPulsos ()
{
  pcnt_config_t pcntFreqConfig = { };                        // Instancia do Contador de Pulsos
  pcntFreqConfig.pulse_gpio_num = PCNT_INPUT_SIG_IO;         // Port de entrada dos pulsos  = GPIO 4
  pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // Conta na subida do pulso
  pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // Limite maximo de contagem
  pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // Unidade 0 do Contador de Pulsos
  pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // Canal 0 da Unidade 0 Contador de Pulsos
  pcnt_unit_config(&pcntFreqConfig);                         // configura os registradores do Contador de Pulsos

  pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pausa o Contador de Pulsos
  pcnt_counter_clear(PCNT_FREQ_UNIT);                        // Zera e reseta o Contador de Pulsos

  pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // Ativa evento - interrupção no limite maximo de contagem
  pcnt_isr_register(overflowContador, NULL, 0, &user_isr_handle);  // configura registrador da interrupção
  pcnt_intr_enable(PCNT_FREQ_UNIT);                          // habilita as interrupções do Contador de Pulsos

  pcnt_counter_resume(PCNT_FREQ_UNIT);                       // reinicia o Contador de Pulsos
}

//------------------------------------------------------------
void baseTempo()                                            // Rotina de leitura do contador de pulsos (Base de tempo)
{
  pcnt_get_counter_value(PCNT_FREQ_UNIT, &contador);        // obtem o valor do contador de pulsos - valor max 16 bits
  contadorOverflow = 0;                                     // zera o contador de overflow
  pcnt_counter_clear(PCNT_FREQ_UNIT);                       // Zera e reseta o Contador de Pulsos
  contadorOK = true;
}

//------------------------------------------------------------
void startTimer()                                           // Rotina de inicializacao do timer de Base de Tempo
{
  timer = timerBegin(0, prescaler, true);                   // usa o timer 0, 80 Mhz / prescaler, true = count up
  timerAttachInterrupt(timer, &baseTempo, true);            // endereço da função a ser chamada pelo timer / true gera uma interrupção
  timerAlarmWrite(timer, tempo, true);                      // Define o tempo  de contagem 64 bits / true para repetir o alarme
  timerAlarmEnable(timer);                                  // Habilita timer 0
}

void printFrequencia ()
{
  frequencia = contador + contadorOverflow + (32767 * contadorOverflow) ;          // Calcula frequencia
  Serial.print(frequencia, 0);                                  // print
  Serial.print(" Hz   ");                                     // print
  Serial.println(contadorOverflow);
}

//----------------------------------------------------------------------------------
void loop()
{
  if (contadorOK == true)
  {
    delay(1000);                                 // Aguarda milisegundos
    printFrequencia ();
    contadorOK == false;
  }
}
