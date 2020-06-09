// Frequencimetro ESP32 versão 9.0
// Rui Viana e Gustavo Murta - junho/2020
// referencias  https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/pcnt.h
//              https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/soc/soc/pcnt_reg.h
//
//              https://github.com/espressif/arduino-esp32/blob/a59eafbc9dfa3ce818c110f996eebf68d755be24/cores/esp32/esp32-hal-timer.h
//              https://github.com/espressif/arduino-esp32/blob/f32083a6d0afad8efc4d52b11091ccf55249dc29/cores/esp32/esp32-hal-timer.c


// Escala 1 de      1 MHz  a      40 MHz    tempo   1.000    prescaler 80  ???
// Escala 2 de  327670 Hz  a     XXXX KHz   tempo   10.000   prescaler 80 
// Escala 3 de   32767 Hz  a   327670 Hz    tempo  100.000   prescaler 80
// Escala 4 de       1 Hz  a    32767 Hz    tempo 1.000.000  prescaler 80

#include "driver/pcnt.h"                                  // Biblioteca de pulse count

#define PCNT_TEST_UNIT      PCNT_UNIT_0                   // Unidade de Pulse Count 0 
#define PCNT_H_LIM_VAL      32767                         // Limite superior de contagem 32767 
#define PCNT_L_LIM_VAL      -32767                        // Limite inferior de contagem -32767 
#define PCNT_INPUT_SIG_IO   4                             // Pulse Input GPIO 4 

int16_t contador = 0;                                     // Contador de pulsos - valor max 32767
float frequencia = 0;                                     // Frequencia medida
float fatorEscala = 1;                                    // Fator multiplicador da frequencia
String indicador = "Hz";                                  // Unidade de medida da escala
unsigned long tempo = 1000000;                            // base de tempo da medida dos pulsos
unsigned int prescaler = 80;                              // Prescaler do contador do timer = 80 Mhz / 80 = 1 MHz
int escala = 4;                                           // Escala de medição
bool controle = false;                                    // Controle de escala
bool overflow  = false;                                   // Flag indicativo de overflow do Pulse Counter

pcnt_isr_handle_t user_isr_handle = NULL;                 // Rotina de manuseio do servico de Interrupção
hw_timer_t * timer = NULL;                                // Instancia do timer

//----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                                   // Inicializa a console serial
  pcnt_init();                                            // Inicializa o contador de pulsos
  startTimer();                                           // Inicializa o timer
}

//------------------------------------------------------------------------------------
void IRAM_ATTR pcnt_example_intr_handler(void *arg)       // Rotina de interrupt de overflow do Pulse Counter
{
  overflow = true;                                        // Indicativo de ocorrencia de overflow
  PCNT.int_clr.val = BIT(PCNT_TEST_UNIT);                 // Limpa flag de overflow
}

//----------------------------------------------------------------------------------
void pcnt_init()                                          // Rotina de inicializacao do Contador de pulsos
{
  pcnt_config_t pcnt_config = { };                        // Instancia pulse config
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;         // Port de entrada dos pulsos  = GPIO 4
  pcnt_config.pos_mode = PCNT_COUNT_INC;                  // Conta na subida do pulso
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;             // Inverte a contagem quando o pino de controle é LOW
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                // Se pino de controle é HIGH, conta incrementando
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;             // Limite maximo de contagem
  pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;             // Limite minimo de contagem
  pcnt_unit_config(&pcnt_config);                         // Inicializa PCNT

  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);      // Enable PCNT event of PCNT unit - Habilita evento de limite maximo = 1
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);      // Enable PCNT event of PCNT unit - Habilita evento de limite minimo = 0
  pcnt_set_filter_value(PCNT_TEST_UNIT, 40);              // Configura filtro - Any pulses lasting shorter than this will be ignored
  pcnt_filter_enable(PCNT_TEST_UNIT);                     // Habilita filtro
  pcnt_counter_pause(PCNT_TEST_UNIT);                     // Pause PCNT counter of PCNT unit
  pcnt_counter_clear(PCNT_TEST_UNIT);                     // Clear and reset PCNT counter value to zero

  // Interrupt handler function, Parameter for handler function = NULL, intr_alloc_flags Flags used to allocate the interrupt = 0,
  // Pointer to return handle = &user_isr_handle. If non-NULL, a handle for the interrupt will be returned here.

  pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);    // Registrador de ISR e manuseio de interrupt PCNT
  pcnt_intr_enable(PCNT_TEST_UNIT);                       // Enable PCNT interrupt for PCNT unit
  pcnt_counter_resume(PCNT_TEST_UNIT);                    // Resume counting for PCNT counter
}

//------------------------------------------------------------
void baseTempo()                                          // Rotina de leitura do contador de pulsos (Base de tempo)
{
  pcnt_get_counter_value(PCNT_TEST_UNIT, &contador);      // obtem o valor do contador de pulsos - valor max 32767
  pcnt_counter_clear(PCNT_TEST_UNIT);                     // zera contador de pulsos
}

//------------------------------------------------------------
void startTimer()                                         // Rotina de inicializacao do timer de Base de Tempo
{
  timer = timerBegin(0, prescaler, true);                 // usa o timer 0, 80 Mhz / prescaler, true = count up
  timerAttachInterrupt(timer, &baseTempo, true);          // endereço da função a ser chamada pelo timer / true gera uma interrupção
  timerAlarmWrite(timer, tempo, true);                    // Define o tempo  de contagem 64 bits / true para repetir o alarme
  timerAlarmEnable(timer);                                // Habilita timer 0
}

//----------------------------------------------------------------------------------
void controleEscala ()
{
  if (overflow == true)                                   // Se ocorreu interrupt de overflow de pcnt
  {
    overflow = false;                                     // Desablita nova ocorrencia
    controle = true;                                      // Habilita mudanca de escala
    Serial.println(" Overflow: ");                        // Print

    escala = escala - 1;                                  // Decrementa escala
    if (escala < 1 )                                      // Se chegou no minimo
      escala = 0;                                         // escala mínima
    selecionaEscala();                                    // Chama rotina de ajuste de escala
  }

  if (contador > 32767)                                   // Se frequencia for maior que 32767
  {
    controle = true;                                      // Habilita mudanca de escala
    escala = escala - 1;                                  // Decrementa escala
    if (escala < 1)                                       // Se chegou no minimo
      escala = 4;                                         // Vai para o maximo
    selecionaEscala();                                    // Chama rotina de ajuste de escala
  }
  Serial.print(" Contador = "); Serial.print(contador);    // Print
}

//------------------------------------------------------------------------------------
void selecionaEscala()                                    // Rotina de troca de escala
{
  if (controle == true)                                   // Se troca de escala estiver habilitada
  {
    controle = false;                                     // Desabilita na troca
    switch (escala)                                       // Seleciona escala
    {
      case 1:                                             // Escala 1
        tempo = 1000;                                     // base de tempo 1
        fatorEscala = 1000;                               // fator multiplicador da escala
        prescaler = 80;                                   // Precaler 1
        indicador = "Hz       ";                          // unidade de MHz
        break;                                            // Finaliza e retorna

      case 2:                                             // Escala 2 = de   327670 Hz  a   3276700 Hz
        tempo = 10000;                                    // base de tempo 2
        fatorEscala = 0.1;                                // fator multiplicador da escala 100
        prescaler = 80;                                   // Precaler 2
        indicador = "KHz       ";                          // unidade de KHz
        break;                                            // Finaliza e retorna

      case 3:                                             // Escala 3 = de   32767 Hz  a   327670 Hz
        tempo = 100000;                                   // base de tempo 3
        prescaler = 80;                                   // Precaler 3
        fatorEscala = 10;                                 // fator multiplicador da escala
        indicador = "Hz       ";                          // unidade de KHz
        break;                                            // Finaliza e retorna

      case 4:                                             // Escala 4 =  1 Hz  a  32767 Hz
        tempo = 1000000;                                  // base de tempo 4
        prescaler = 80;                                   // valor do Prescaler 4
        fatorEscala = 1;                                  // fator multiplicador da escala
        indicador = "Hz       ";                          // unidade de Hz
        break;                                            // Finaliza e retorna
    }
  }
  startTimer();                                           // Reinicializa o timer
}

//----------------------------------------------------------------------------------
void loop()
{
  //escala = 4;
  if (contador > 0 )
  {
    controleEscala ();
    frequencia = contador * fatorEscala;                      // Calcula frequencia
    Serial.print("     Escala ");  Serial.print(escala);      // Print
    Serial.print(" : ");  Serial.print(frequencia, 1);        // Print
    Serial.print(" "); Serial.println(indicador);             // Print
  }
  delay(200);                                                 // Aguarda milisegundos
}
