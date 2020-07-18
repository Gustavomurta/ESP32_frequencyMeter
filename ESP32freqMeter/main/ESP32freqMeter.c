// ESP32 HIGH ACCURACY FREQUENCY METER version 0.33.0

/* DEVELOPER : Rui Viana
   CONTRIBUITOR: Gustavo Murta   
   DATE: 17/jul/2020
   
   ESP32 Dev Kit - ESP-IDF V4.01 or ARDUINO IDE 1.8.12

   The project:
   A high accuracy frequency meter using ESP32, without scales and showing up to 8 digits,
   measuring up to 20MHz or more. Very stable. 
   This project can be compiled with the arduino IDE or the IDF!

  Definitions: 

  GPIO_34 =  Freq Meter Input
  GPIO_25 = #define LEDC_HS_CH0_GPIO      GPIO_NUM_25

  #define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Count Control GPIO HIGH=count up, LOW=count down GPIO 25
  #define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Saida do timer GPIO 2

  PORT de entrada do frequencímetro PCNT_INPUT_SIG_IO (GPIO 34)
  PORT de entrada de controle PCNT_INPUT_CTRL_IO      (GPIO 35)
  PORT de saída do timer OUTPUT_CONTROL_GPIO          (GPIO 32)
  O PORT de entrada de controle (GPIO 35) deve ser ligado ao PORT de saída do timer (GPIO 32).
  Estes são os ports usados no projeto, mas podem ser modificados para sua melhor conveniência.

  O frequencímetro é dividido em 5 partes:
    1. Contador de pulsos;
    2. Controle de tempo de contagem;
    3. Impressão do resultado;
    4. Espaço para outras funções.
    5. Gerador de sinais programado para 2  Hz  (ou 50.000)

  1. O contador de pulso usa o pcnt do ESP32.
     O pcnt tem os seguintes parâmetros:
      a. port de entrada;
      b. canal de entrada;
      c. port de controle;
      d. contagem na subida do pulso;
      e. contagem na descida do pulso;
      f. contagem só com o controle em nível elevado;
      g. limite máximo de contagem.

  2. O Controle de tempo de contagem usa o esp-timer.
     O esp-timer tem o seguinte parâmetro:
      a. controle do tempo;

  5. Gerador de frequncias para testes usa o ledc
     O ledc tem o seguinte parâmetro:
      a. port de saida;
      b. canal de lcd;
      c. frequencia;
      d. resolucao do ledc;
      e. duty cycle em 5;


  Funcionamento:
    O port de controle de contagem em nível alto, libera o contador para contar os pulsos que chegam no port de entrada de pulsos.
  Os pulsos são contado tanto na subida quanto na descida do pulso, para melhorar a media de contagem.
  O tempo de contagem é definido pelo esp-timer, e esta' definido em 1 segundo, na variável janela.
  Se a contagem for maior que 20000 pulsos durante o tempo de contagem, ocorre overflow e a cada overflow que ocorre
  e' contabilizado na variável multPulses, e o contador de pulso retorna a zero continuando a contar.
    Quando o tempo de leitura termina, uma rotina é chamada e o valor do no contador de pulsos e' lido e salvo,
    um flag e' ligado indicando que terminou a leitura dos pulsos

    No loop, ao verificar que o flag indica que terminou a leitura dos pulsos, o valor é calculado multiplicando
  o numero de overflow por 20000 e somando ao numero de pulsos restantes e dividindo por 2, pois contou 2 vezes.
  Como o pulsos são contados na subida e na descida, a contagem e´ o dobro da frequência.
    Na frequência é insserido pontos e impressa no serial monitor.
  Os registradores são resetados e o port de controle de entrada é novamente elevado para nível alto e a contagem de
  pulsos se inicia.

  Tem também um gerador de sinais que gera 50.000 Hz, e pode ser usado para testes.
  Este gerador pode ser alterado para gerar frequencias até 40 MHz.
  Usamos o recurso ledc do ESP32 para gerar frequencia que pode ser usada como teste.
    O valor da frequencia base é 2 (ou 50.000) Hz, mas pode ser digitavo outo valor no serial monitor
    O duty foi fixado como 50%
    A resulução é calculada.
  O Port de saida deste gerador é definido na linha #define LEDC_GPIO.
  Atualmente está definido como GPIO 25.

  Internamente usando GPIO matrix,o pulso de entrada foi direcionado para o LED nativo do ESP32, 
  assim o LED piscara na frequencia de entrada.

  O compilador usa as diretivas de compilacaoo para selecionar:
   Compilador Arduino ou IDF    automatico
   Uso de LCD                   LCD_ON ou LCD_OFF
   Uso de LCD I2C               LCD_I2C_ON ou LCD_I2C_OFF

  Referências:
  author=krzychb https://github.com/espressif/esp-idf/tree/master/examples/peripherals/pcnt
  resposta by Deouss » Thu May 17, 2018 3:07 pm no tópico https://esp32.com/viewtopic.php?t=5734
  Gerador de sinais Gustavo https://github.com/Gustavomurta/ESP32_frequenceMeter/blob/master/ESP32OscilatorV03.ino
  Formatação de numero https://arduino.stackexchange.com/questions/28603/the-most-effective-way-to-format-numbers-on-arduino
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
  Agradecemos muito também ao CELSO ITO pelas sugestões e pelos incansáveis teste com IDF.
*/

#define LCD_OFF                                                           // Define LCD_ON, para usar LCD, se não, defina LCD_OFF
#define LCD_I2C_OFF                                                       // Define LCD_I2C_ON, para usar LCD I2C, se não, defina LCD_I2C_OFF

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "math.h"

#ifdef LCD_ON                                                             // Se tem LCD

#ifdef LCD_I2C_ON                                                         // Se tem LCD I2C
#include <LiquidCrystal_I2C.h>                                            // Inclue a bibliotea do LCD I2C
LiquidCrystal_I2C lcd(0x3F, 16, 2);                                       // Instancia e define I2C addr
#else
#include <LiquidCrystal.h>                                                // Inclue a bibliotea do LCD
LiquidCrystal lcd(5, 18, 19, 21, 22, 23);                                 // Instancia e define port
#endif

#endif

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Unidade 0 do pcnt
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Canal 0 do pcnt

//                 Para usar teste jumper GPIO 25 com GPIO 34
//                 Port de entrada do frequencimetro  GPIO 34
#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_25                                 // Saida do ledc gerador de pulsos

//                  Necessario jumper entre GPIO 32 e GPIO 35
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Count Control GPIO HIGH = count up, LOW = count down 
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Saida do timer GPIO 32 Controla a contagem

#define IN_BOARD_LED          (gpio_num_t)2                                // LED nativo ESP32 GPIO 2

#define LEDC_HS_CH0_CHANNEL   LEDC_CHANNEL_0                              // LEDC no canal 0
#define LEDC_HS_MODE          LEDC_HIGH_SPEED_MODE                        // LEDC em high speed
#define LEDC_HS_TIMER         LEDC_TIMER_0                                // Usar timer0 do ledc

uint32_t         overflow  =  20000;                                      // Valor maximo para overflow de pcnt
#define PCNT_H_LIM_VAL        overflow                                    // Limite superior de contagem

esp_timer_create_args_t create_args;                                      // Argumentos do esp-timer
esp_timer_handle_t timer_handle;                                          // Instancia de esp-timer

//  Calculo do ajustes para cada faixa de frequencia
//  Resolucao = log2(Clock(80MHz)/f) + 1   ex: 50.000 HZ = 80.0000/50.000 = 1.600 log2(1600) = 10 + 1 = 11
//  Duty 50%  = (2**Resolucao)/2       ex: 2**11 = 2048   2048/2 = 1024

bool            flag          = true;                                     // Indicador de fim de contagem libera impressao
int16_t         pulses        = 0;                                        // Contador de pulsos de entrada
uint32_t        multPulses    = 0;                                        // Contador de overflows de pcnt
uint32_t        janela        = 1000000;                                  // Janela de 1 segundo para a contagem de pulsos
uint32_t        oscilator     = 2;                                        // Frequencia em Hz
uint32_t        mDuty         = 0;                                        // Valor calculado do duty
uint32_t        resolucao         = 0;                                    // Valor calculado da resolucao

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // variavel tipo portMUX_TYPE para sincronismo
//----------------------------------------------------------------------------------------
// Sem comentarios originais  https://arduino.stackexchange.com/questions/28603/the-most-effective-way-to-format-numbers-on-arduino
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos)
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = '.';
  return s;
}
//----------------------------------------------------------------------------------------
// Sem comentarios originais  https://arduino.stackexchange.com/questions/28603/the-most-effective-way-to-format-numbers-on-arduino
char *ltos(long val, char *s, int radix)
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
//----------------------------------------------------------------------------
void ledcInit ()
{
  resolucao = log((80000000 / oscilator) + 1);                            // Calculo da resolucao para ledc
  //  Serial.println(resolucao);
  mDuty = (pow(2, resolucao)) / 2;                                        // Calculo do duty para ledc
  //  Serial.println(mDuty);

  ledc_timer_config_t ledc_timer = {};                                    // Instancia a configuracao do timer do ledc

  ledc_timer.duty_resolution = (ledc_timer_bit_t) + resolucao;            // Configura resolucao
  ledc_timer.freq_hz    = oscilator;                                      // Frequencia de oscilacao
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Mode de operacao em high speed
  ledc_timer.timer_num = LEDC_TIMER_0;                                    // Usar timer0 do ledc
  ledc_timer_config(&ledc_timer);                                         // Configurar o timer do ledc

  ledc_channel_config_t ledc_channel = {};                                // Instancia a configuracao canal do ledc

  ledc_channel.channel    = LEDC_HS_CH0_CHANNEL;                          // Configura canal0
  ledc_channel.duty       = mDuty;                                        // Valor calculado do duty em %
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // Saida no GPIO defino no inicio
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // Desabilita interrupt de ledc
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Mode de operacao do canal em high speed
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Usar timer0 do ledc

  ledc_channel_config(&ledc_channel);                                     // Configurar o canal do ledc
}
//----------------------------------------------------------------------------------
void tempo_controle(void *p)                                              // Fim de tempo de leitura de pulsos
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Controle do PCount - stop count
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
void myInit()
{
#ifdef LCD_ON                                                             // Se tem LCD
  lcd.begin(16, 2);                                                       // Inicializa lcd
  lcd.print("Frequencimetro");                                            // Print
#endif

  ledcInit();                                                             // Inicializa o ledc
  pcnt_init();                                                            // Inicializa o pulse count
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Define o port decontrole
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Define o port de controle como saida

  create_args.callback = tempo_controle;                                  // Instancia o tempo de controle
  esp_timer_create(&create_args, &timer_handle);                          // Cria parametros do timer

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Port LED como saida

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Direciona a entrada de pulsos
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Para o LED do ESP32
}
//---------------------------------------------------------------------------------
void app_main(void)
{
#ifndef ARDUINO                                                           // IDF
  myInit();                                                               // IDF
  while (1)                                                               // IDF
  {
#endif
    if (flag == true)                                                     // Se a contagem tiver terminado
    {
      flag = false;                                                       // Impede nova impresao
      float frequencia = 0;                                               // Variavel para calculo de frequencia
      frequencia = (pulses + (multPulses * overflow)) / 2  ;              // Calcula qtos pulsos ocorreram
      char buf[32];                                                       // Buffer para guardar a pontuacao
      printf("frequencia: %s", (ltos(frequencia, buf, 10)));              // Imprime pontuado
      printf(" Hz \n");                                                   // Sufixo
#ifdef LCD_ON                                                             // LCD
      lcd.setCursor(2, 1);                                                // Posiciona cusros na posicao 3 da linha 2
      lcd.print((ltos(frequencia, buf, 10)));                             // Print
      lcd.print(" Hz              ");                                     // Sufixo
#endif
      multPulses = 0;                                                     // Zera contador de overflow
      // Aqui pode rodar qq funcao                                        // Espaco para qualquer função
      vTaskDelay(1);
      // Aqui pode rodar qq funcao                                        // Espaco para qualquer função
      pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Zera o contador PCNT
      esp_timer_start_once(timer_handle, janela);                         // Inicia contador de tempo de 1 segundo
      gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             // Porta de controle habilita contagem dos pulsos
    }
#ifndef ARDUINO                                                           // IDF
  }                                                                       // IDF
#endif
}
//---------------------------------------------------------------------------------
#ifdef ARDUINO                                                            // Arduino
void setup()
{
  Serial.begin(115200);                                                   // Inicializa a serial
  myInit();                                                               // Chaama inicializacao
}
//---------------------------------------------------------------------------------
void loop()
{
  app_main();                                                             // Roda rotina principal
  String inputString = "";                                                // Temporaria transformar o valor da frequencia
  oscilator = 0;                                                          // Limpa valor original
  while (Serial.available())                                              // Enquanto tiver dados na serial
  {
    char inChar = (char)Serial.read();                                    // Le um byte:
    inputString += inChar;                                                // Adicione na string:
    if (inChar == '\n')                                                   // Se new line:
    {
      oscilator = inputString.toInt();                                    // Transforma a string em inteiro
      inputString = "";                                                   // Limpa a string
    }
  }
  if (oscilator != 0)                                                     // Se foi digitado algum valor
  {
    ledcInit();                                                           // Reconfigura ledc
  }
}
#endif
