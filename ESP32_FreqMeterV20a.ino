/*
      ESP32 Frequence Meter V20
      Rui Viana 08/jul/2020
      Collaborator: Gustavo Murta

      PORT de entrada do frequencíemtro PCNT_INPUT_SIG_IO (GPIO 34)
      PORT de entrada de controle PCNT_INPUT_CTRL_IO (GPIO 25) 
      PORT de saida do timer OUTPUT_CONTROL_GPIO (GPIO 2) 
      O PORT de entrada de controle (GPIO 25) deve ser ligado ao PORT de saida do timer (GPIO 2).

      Frequencimetro com ESP32
      O frequencimetro é dividido em 4 partes:
      1. Contador de pulsos;
      2. Controle de tempo de contagem;
      3. Impressão do resultado;
      4. Espaco para outras funcoes.

     O contador de pulso usa o pcnt do ESP32. O pcnt tem varios parametros:
          port de entrada;
          port de controle;
          contagem na subida do pulso;
          contagem na descida do pulso;
          contagem so com o controle em nivel elevado;
          limite maximo de contagem.

      Com o port de contagem em nivel alto, libera o contador para contar os pulsos que chegam no port de entrada de pulsos.
      Os pulsos sao contado tanto na subida quanto na descida do pulso, para melhorar a media de contagem.

      O tempo de contagem é definido pelo esp-timer, e foi definido em 1 segundo na variavel janela.
      Como o pulsos sao contados na subida e na descida a contagem e´ o dobro da frequencia.

      Se a contagem for maior que 20000 pulsos, ocorera um overflow e a cada overflow que ocorra
      será contabilizado na variavel multPulses.
      No final o valor é calculado multiplicando o numero de overflow por 20000 e somando ao numero
      de puslos restantes e dividindo por 2, pois contou 2 vezes.
      
      Referências: 
      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
*/

#include "driver/pcnt.h"                                                  // Pulse count library

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Unidade do pcnt
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Canal do pcnt
#define PCNT_INPUT_SIG_IO     34                                          // Freq Meter Input GPIO 34
#define PCNT_INPUT_CTRL_IO    25                                          // Count Control GPIO HIGH=count up, LOW=count down GPIO 25
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_2                                  // Saida do timer GPIO 2

#define PCNT_H_LIM_VAL        overflow                                    // Limite superior de contagem

esp_timer_create_args_t create_args;                                      // Argumentos do timer
esp_timer_handle_t timer_handle;                                          // Instancia de timer

unsigned long overflow = 20000;                                           // Valor maximo para overflow de pcnt
bool flag = true;                                                         // Indicador de fim de contagem libera impressao
int16_t pulses = 0;                                                       // Contador de pulsos de entrada
unsigned long multPulses = 0;                                             // Contador de overflows de pcnt
unsigned long  janela = 1000000;                                          // Janela de 1 segundo para a contagem de pulsos

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//----------------------------------------------------------------------------------
void tempo_controle(void *p)                                              // Fim de tempo de leitura de pulsos
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, LOW);                               // Controle do PCount - stop count
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Obtem o valor contado
  flag = true;                                                            // Informa que ocorreu interrupt de controle
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Overflow de contagem de pulsos
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // Desabilita interrupção ?
  multPulses++;                                                           // Incrementa contador de overflow  
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Limpa indicador de interrupt        
  portEXIT_CRITICAL_ISR(&timerMux);                                       // Libera novo interrupt
}

//----------------------------------------------------------------------------------
void pcnt_init(void)                                                      // Rotina de inicializacao do pulse count
{
  pcnt_config_t pcnt_config = { };                                        // Instancia pulse config
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Port de entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Controle da contagem
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // Canal de contagem
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Limite maximo de contagem
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // Conta na subida do pulso
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // Conta na descida do pulso
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // Nao usado
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // Se HIGH conta incrementando
  pcnt_unit_config(&pcnt_config);                                         // Inicializa PCNT

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Inicializa o contador PCNT
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Zera o contador PCNT
 
  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Limite superior de contagem
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Rotina de Interrupt de pcnt
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Habilita interrup de pcnt
  
  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // inicia a contagem
}

//----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                                                   // Inicializa a serial
  pcnt_init();                                                            // Inicializa o pulse count
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Define o port decontrole
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Define o port de controle como saida

  create_args.callback = tempo_controle;                                  // Instancia o tempo de controle
  esp_timer_create(&create_args, &timer_handle);                          // Cria parametros do timer 
}

//---------------------------------------------------------------------------------
void loop()
{
  if (flag == true)                                                       // Se a contagem tiver terminado
  {
    flag = false;                                                         // Impede nova impresao
    float frequencia = 0;                                                 // Variavel para calculo de frequencia
    frequencia = (pulses + (multPulses * overflow))  ;                    // Calcula qtos pulsos ocorreram
    Serial.print("  Frequencia: ");                                       // Print
    Serial.println ((frequencia / 2 ), 0);                                // Print
    multPulses = 0;                                                       // Zera contador de overflow

    // Aqui pode rodar qq funcao                                          // Espaco para qualquer função
    delay(50);                                                            // Espaco para qualquer função
    // Aqui pode rodar qq funcao                                          // Espaco para qualquer função

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                  // Zera o contador PCNT
    esp_timer_start_once(timer_handle, janela);                           // Inicia contador de tempo de 1 segundo 
    gpio_set_level(OUTPUT_CONTROL_GPIO, HIGH);                            // Porta de controle habilita contagem dos pulsos 
  }
}
