// BLOG Eletrogate
// ESP32 Frequencimetro
// ESP32 DevKit 38 pinos + LCD
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// Rui Viana e Gustavo Murta agosto/2020

#include "stdio.h"                                                        // Biblioteca STDIO
#include "driver/ledc.h"                                                  // Biblioteca ESP32 LEDC
#include "driver/pcnt.h"                                                  // Biblioteca ESP32 PCNT
#include "soc/pcnt_struct.h"                                              // <<<<<<<<<<<<<<<<<<<<<<<< Novidade 

#define LCD_OFF                                                           // Defina LCD_ON, para usar LCD, se não, defina LCD_OFF
#define LCD_I2C_OFF                                                       // Defina LCD_I2C_ON, para usar LCD I2C, se não, defina LCD_I2C_OFF

#ifdef LCD_I2C_ON                                                         // Se habilitar LCD I2C
#define I2C_SDA 21                                                        // LCD I2C SDA - GPIO_21
#define I2C_SCL 22                                                        // LCD I2C SCL - GPIO_22
#include <Wire.h>                                                         // Biblioteca para I2C
#include <LiquidCrystal_PCF8574.h>                                        // Biblioteca para LCD com PCF8574
LiquidCrystal_PCF8574 lcd(0x3F);                                          // Instancia LCD I2C com endereço x3F
#endif                                                                    // LCD I2C

#ifdef LCD_ON                                                             // Se habilitar LCD com interface 4 bits
#include <LiquidCrystal.h>                                                // Biblioteca para LCD
LiquidCrystal lcd(4, 16, 17, 5, 18, 19);                                  // Instancia e define os ports
#endif                                                                    // LCD

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Unidade 0 do Contador de pulso PCNT do ESP32
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Canal 0 do Contador de pulso PCNT do ESP32

#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Entrada do Frequencimetro -  GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Pino de controle do PCNT - HIGH = count up, LOW = count down 
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Saida do timer - Controla a contagem - GPIO_32
#define PCNT_H_LIM_VAL        overflow                                    // Limite superior de contagem

#define IN_BOARD_LED          GPIO_NUM_2                                  // LED nativo ESP32 - GPIO 2

bool            flag          = true;                                     // Indicador de fim de contagem - libera impressão
uint32_t        overflow      = 20000;                                    // Valor maximo para overflow do contador PCNT
int16_t         pulses        = 0;                                        // Quantidade de pulsos contados
uint32_t        multPulses    = 0;                                        // Quantidade de overflows do contador PCNT
uint32_t        janela        = 1000000;                                  // Tempo de amostragem  de 1 segundo para a contagem de pulsos
uint32_t        oscilador     = 19200;                                    // Frequencia inicial do oscilador - 12543 Hz
uint32_t        mDuty         = 0;                                        // Valor calculado do ciclo de trabalho
uint32_t        resolucao     = 0;                                        // Valor calculado da resolucao
float           frequencia    = 0;                                        // Variavel para calculo de frequencia
char            buf[32];                                                  // Buffer para guardar a pontuacao

esp_timer_create_args_t create_args;                                      // Argumentos do ESP-Timer
esp_timer_handle_t timer_handle;                                          // Instancia de ESP-Timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // variavel tipo portMUX_TYPE para sincronismo

//----------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                                                   // Inicializa a serial 115200 Bps
  Serial.println(" Digite uma frequencia - 1 a 40 MHz");                  // Print na console

#ifdef LCD_I2C_ON                                                         // Se estiver usando LCD I2C 
  Wire.begin(I2C_SDA, I2C_SCL);                                           // Inicializa Interface I2C
  lcd.setBacklight(255);                                                  // Ativa leds do backlight do LCD
#endif

#if defined LCD_ON || defined LCD_I2C_ON                                  // Se estiver usando LCD ou LCD I2C      
  lcd.begin(16, 2);                                                       // Inicializa LCD 16 colunas 2 linhas
  lcd.print("  Frequencia:");                                             // Print no LCD
#endif

  inicializa_frequencimetro();                                            // Inicializa o frequencimetro
}

//----------------------------------------------------------------------------
void inicializa_oscilador ()                                              // Inicializa gerador de pulsos
{
  resolucao = (log (80000000 / oscilador)  / log(2)) / 2 ;                // Calculo da resolucao para o oscilador
  if (resolucao < 1) resolucao = 1;                                       // Resoluçao mínima
  // Serial.println(resolucao);                                           // Print
  mDuty = (pow(2, resolucao)) / 2;                                        // Calculo do ciclo de trabalho 50% do pulso
  // Serial.println(mDuty);                                               // Print

  ledc_timer_config_t ledc_timer = {};                                    // Instancia a configuracao do timer do LEDC

  ledc_timer.duty_resolution =  ledc_timer_bit_t(resolucao);              // Configura resolucao
  ledc_timer.freq_hz    = oscilador;                                      // Configura a frequencia do oscilador
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Modo de operacao em alta velocidade
  ledc_timer.timer_num = LEDC_TIMER_0;                                    // Usar timer0 do LEDC
  ledc_timer_config(&ledc_timer);                                         // Configura o timer do LEDC

  ledc_channel_config_t ledc_channel = {};                                // Instancia a configuracao canal do LEDC

  ledc_channel.channel    = LEDC_CHANNEL_0;                               // Configura canal 0
  ledc_channel.duty       = mDuty;                                        // Configura o ciclo de trabalho
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // Configura GPIO da saida do LEDC - oscilador
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // Desabilita interrupção do LEDC
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Modo de operacao do canal em alta velocidade
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Seleciona timer 0 do LEDC
  ledc_channel_config(&ledc_channel);                                     // Configura o canal do LEDC
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Contagem do contador de Overflow
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // Bloqueia nova interrupção
  multPulses++;                                                           // Incrementa contador de overflow
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Limpa indicador de interrupção
  portEXIT_CRITICAL_ISR(&timerMux);                                       // Libera nova interrupção
}

//----------------------------------------------------------------------------------
void inicializa_contador(void)                                            // Inicializacao do contador de pulsos
{
  pcnt_config_t pcnt_config = { };                                        // Instancia PCNT config

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Configura GPIO para entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Configura GPIO para controle da contagem
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // Canal de contagem PCNT - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Limite maximo de contagem - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // Incrementa contagem na subida do pulso
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // Incrementa contagem na descida do pulso
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT - modo lctrl desabilitado
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT - modo hctrl - se HIGH conta incrementando
  pcnt_unit_config(&pcnt_config);                                         // Configura o contador PCNT

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pausa o contador PCNT
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Zera o contador PCNT

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Configura limite superior de contagem
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Conigura rotina de interrupção do PCNT
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Habilita interrupções do PCNT

  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Reinicia a contagem no contador PCNT
}

//----------------------------------------------------------------------------------
void tempo_controle(void *p)                                              // Fim de tempo de leitura de pulsos
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Controle do PCNT - para o contador
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Obtem o valor contado no PCNT
  flag = true;                                                            // Informa que ocorreu interrupção de controle
}

//---------------------------------------------------------------------------------
void inicializa_frequencimetro()
{
  inicializa_oscilador ();                                                // Inicia a geração de pulsos no oscilador
  inicializa_contador();                                                  // Inicializa o contador de pulsos PCNT

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Define o port decontrole
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Define o port de controle como saida

  create_args.callback = tempo_controle;                                  // Instancia o tempo de controle
  esp_timer_create(&create_args, &timer_handle);                          // Cria parametros do timer

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Port LED como saida

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Direciona a entrada de pulsos
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Para o LED do ESP32
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos) // Formata um número longo de 32 bits com pontos
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
char *ltos(long val, char *s, int radix)    // Formata um número longo de 32 bits com pontos
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
void loop()
{
  if (flag == true)                                                       // Se a contagem tiver terminado
  {
    flag = false;                                                         // Impede nova impressao
    frequencia = (pulses + (multPulses * overflow)) / 2  ;                // Calcula a soma dos pulsos contados no PCNT
    printf("Frequencia : %s", (ltos(frequencia, buf, 10)));               // Print frequencia com pontos
    printf(" Hz \n");                                                     // Print unidade Hz

#if defined LCD_ON || defined LCD_I2C_ON                                  // Se estiver usando LCD ou LCD I2C  
    lcd.setCursor(2, 1);                                                  // Posiciona cursor na posicao 2 da linha 1
    lcd.print((ltos(frequencia, buf, 10)));                               // Print frequencia no LCD
    lcd.print(" Hz              ");                                       // Print unidade Hz no LCD
#endif

    multPulses = 0;                                                       // Zera contador de overflow
    // Espaco para qualquer função
    delay (100);                                                          // Delay 100 ms
    // Espaco para qualquer função

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                  // Zera o contador PCNT
    esp_timer_start_once(timer_handle, janela);                           // Inicia contador de tempo de 1 segundo
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                               // Porta de controle - habilita contagem dos pulsos
  }

  String inputString = "";                                                // Limpa string para entrada de dados
  oscilador = 0;                                                          // Zera o valor da frequencia
  while (Serial.available())                                              // Enquanto tiver dados na serial
  {
    char inChar = (char)Serial.read();                                    // Le um byte:
    inputString += inChar;                                                // Adicione na string:
    if (inChar == '\n')                                                   // Se pressionar ENTER:
    {
      oscilador = inputString.toInt();                                    // Transforma a string em inteiro
      inputString = "";                                                   // Limpa a string
    }
  }
  if (oscilador != 0)                                                     // Se foi digitado algum valor
  {
    inicializa_oscilador ();                                              // Reconfigura a frequencia do oscilador
  }
}
