/*
  ESP32 Frequence_Meter_0.0.23a
  Desenvolvedores: Rui Viana e Gustavo Murta   13/jul/2020
  Display LCD I2C

  Para desenvolver este projeto, foram aproveitadas partes de códigos dos seguintes desenvolvedores.

  author=krzychb https://github.com/espressif/esp-idf/tree/master/examples/peripherals/pcnt
  resposta by Deouss » Thu May 17, 2018 3:07 pm no tópico https://esp32.com/viewtopic.php?t=5734
  Gerador de sinais Gustavo https://github.com/Gustavomurta/ESP32_frequenceMeter/blob/master/ESP32OscilatorV03.ino

  O Projeto:
  Um frequencímetro usando ESP32, sem necessidade de escalas e mostrando até 7 dígitos,
  atingindo com precisão até 20MHz ou mais.

  Definições:
  PORT de entrada do frequencímetro PCNT_INPUT_SIG_IO (GPIO 34)
  PORT de entrada de controle PCNT_INPUT_CTRL_IO (GPIO 35)
  PORT de saída do timer OUTPUT_CONTROL_GPIO (GPIO 32)
  O PORT de entrada de controle (GPIO 35) deve ser ligado ao PORT de saída do timer (GPIO 32).
  Estes são os ports usados no projeto, mas podem ser modificados para sua melhor conveniência.

  O frequencímetro é dividido em 5 partes:
    1. Contador de pulsos;
    2. Controle de tempo de contagem;
    3. Impressão do resultado;
    4. Espaço para outras funções.
    5. Gerador de sinais programado para 10 KHz

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

  Funcionamento:
    O port de controle de contagem em nível alto, libera o contador para contar os pulsos que chegam no port de entrada de pulsos.
  Os pulsos são contado tanto na subida quanto na descida do pulso, para melhorar a media de contagem.
  O tempo de contagem é definido pelo esp-timer, e esta' definido em 1 segundo, na variável janela.
  Se a contagem for maior que 20000 pulsos durante o tempo de contagem, ocorra overflow e a cada overflow que ocorre
  e' contabilizado na variável multPulses, e o contador de pulso retorna a zero continuando a contar.
    Quando o tempo de leitura termina, uma rotina é chamada e o valor do no contador de pulsos e' lido e salvo,
    um flag e' ligado indicando que terminou a leitura dos pulsos

    No loop, ao verificar que o flag indica que terminou a leitura dos pulsos, o valor é calculado multiplicando
  o numero de overflow por 20000 e somando ao numero de pulsos restantes e dividindo por 2, pois contou 2 vezes.
  Como o pulsos são contados na subida e na descida, a contagem e´ o dobro da frequência.
    A frequência é impressa no serial monitor.
  Os registradores são resetados e o port de controle de entrada é novamente elevado para nível alto e a contagem de
  pulsos se inicia.

  Tem também um gerador de sinais que gera 10 KHz, e pode ser usado para testes.
  Este gerador pode ser alterado para gerar frequencias até 40 MHz.
  O Port de saida deste gerador é definido na linha #define LEDC_GPIO.
  Atualmente está definido como GPIO 25.

  Para o uso da Serial e de um LCD, existem 2 definições que pode ser usadas juntas ou bloqueadas.
  Este uso e bolqueio pode ser comitantemente ou individualmente.

  Referências:
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html

*/

#define LCD_ON                                                          // Se tem LCD defina LCD_ON, se não tem LCD, defina LCD_OFF
#define Ser_ON                                                            // Se quer imprimir resultados na serial defina Ser_ON, se não, defina  Ser_OFF
unsigned long overflow = 20000;                                           // Valor maximo para overun de pcnt
#include "driver/pcnt.h"                                                  // Pulse count library

#ifdef LCD_ON                                                             // Se tem LCD
#include <LiquidCrystal_I2C.h>                                            // Inclue a bibliotea do LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);                                       // Instancia e define port
#endif

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Unidade de pcnt
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Canal pcnt
#define PCNT_INPUT_SIG_GPIO   34                                          // Pulse Input GPIO
#define PCNT_INPUT_CTRL_GPIO  35                                          // Control GPIO HIGH=count up, LOW=count down
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Saida de controle
#define LEDC_GPIO             25                                          // Saida da frequencia gerada pelo LEDc

#define PCNT_H_LIM_VAL        overflow                                    // Limite superior de contagem

esp_timer_create_args_t create_args;                                      // Argumentos do timer
esp_timer_handle_t timer_handle;                                          // Instancia de timer

bool flag = true;                                                         // Indicador de fim de contagem libera impressao
int16_t pulses = 0;                                                       // Contador de pulsos de entrada
unsigned long multPulses = 0;                                             // Contador de overflows de pcnt
unsigned long  janela = 999999;                                           // Janela de contagem de pulsos
uint32_t dig[8];

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Parametro do ledc
long  freque = 10000;                                                     // Frequencia em Hz
long  canal = 1;                                                          // Canal gerador de sinal
long  resolBits = 1;                                                      // Bits de resolucao
float dutyCycle = 1;                                                      // Valor divisor de Duty Cycle
//----------------------------------------------------------------------------
void freqAdjust ()
{
  pinMode(LEDC_GPIO, OUTPUT);                                             // GPIO_ as Output
  ledcAttachPin(LEDC_GPIO, canal);                                        // GPIO_ attached to PWM Channel
  ledcSetup(canal, freque, resolBits);                                    // Channel  , freq  ,  bit resolution
  ledcWrite(canal, dutyCycle);                                            // Enable frequency with dutty cycle
}
//----------------------------------------------------------------------------------
void tempo_controle(void *p)                                              // Fim de tempo de leitura de puslos
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Desliga o controle do pcnt
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Obtem o valor contado
  flag = true;                                                            // Informa que ocorreu interrupt de controle
}
//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Overflow de contagem de pulsos
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // Bloqueia novo interrupt
  multPulses++;                                                           // Incrementa contador de overflow
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Limpa indicador de interrupt
  portEXIT_CRITICAL_ISR(&timerMux);                                       // Libera novo interrupt
}
//----------------------------------------------------------------------------------
void pcnt_init(void)                                                      // Rotina de inicializacao do pulse count
{
  pcnt_config_t pcnt_config = { };                                        // Instancia pulse config
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_GPIO;                       // Port de entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_GPIO;                       // Controle da contagem
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
//---------------------------------------------------------------------------------
void inserePt(uint32_t frequencia)                                        // Separação da frequencia em digitos
{
  uint32_t cTemp;                                                         // Temporaria para calculo
  cTemp = (uint32_t)(frequencia / 2);                                     // Obtem o valor da frequencia
  for (int i = 0 ; i < 8; i++)                                            // Separa 8 digitos
  {
    dig[i] = (cTemp % 10);                                                // Salva digito separado na mattriz
    cTemp /= 10;                                                          // Divide por 10 para separar o proximo digito
  }
}
//---------------------------------------------------------------------------------
void rotinaSerial()                                                       // Imprime na serial com ponto milhar
{
#ifdef Ser_ON                                                             // Se está definido para imprimir na serial
  bool nTemp = false;                                                     // Temporario para limpeza de zeros a esquerda
  for (int i = 7 ; i > -1 ; i--)                                          // Identifica o digito
  {
    if (nTemp == false)                                                   // Se não encontrou digito significativo ainda
    {
      if (dig[i] == 0)                                                    // Testa se o digito é zero
      {
        Serial.print  ("" );                                              // Se for Print nullo
      }
      else                                                                // Se não for
      {
        Serial.print  (dig[i]);                                           // Print o digito
        if (i == 6)                                                       // Se for a 6a posição
          Serial.print  (".");                                            // Print imprime ponto
        if (i == 3)                                                       // Se for a 3a posição
          Serial.print  (".");                                            // Imprime ponto
        nTemp = true;                                                     // Informa que tem numero significativo
      }
    }
    else                                                                  // Se tem numero significativo
    {
      Serial.print  (dig[i] );                                            // Imprime o digito
      if (i == 6)                                                         // Se for a 6a posição
        Serial.print  (".");                                              // Print imprime ponto
      if (i == 3)                                                         // Se for a 3a posição
        Serial.print  (".");                                              // Imprime ponto
    }
  }
  Serial.println  (" Hz" );                                               // Print
#endif
}
//---------------------------------------------------------------------------------
void rotinaLCD()                                                          // Imprime LCD com ponto milhar
{
#ifdef LCD_ON                                                             // Se está definido para imprimir no LCD
  bool nTemp = false;                                                     // Temporario para limpeza de zeros a esquerda
  lcd.setCursor(2, 1);                                                    //´Vai para posicao 3 Linha 1
  for (int i = 7 ; i > -1 ; i--)                                          // Identifica o digito
  {
    if (nTemp == false)                                                   // Se não encontrou digito significativo ainda
    {
      if (dig[i] == 0)                                                    // Testa se o digito é zero
      {
        lcd.print("");                                                    // Se for Print nullo
      }
      else                                                                // Se não for
      {
        lcd.print(dig[i]);                                                // Print o digito
        if (i == 6)                                                       // Se for a 6a posição
          lcd.print(".");                                                 // Print imprime ponto
        if (i == 3)                                                       // Se for a 3a posição
          lcd.print(".");                                                 // Imprime ponto
        nTemp = true;                                                     // Informa que tem numero significativo
      }
    }
    else                                                                  // Se tem numero significativo
    {
      lcd.print(dig[i]);                                                  // Imprime o digito
      if (i == 6)                                                         // Se for a 6a posição
        lcd.print(".");                                                   // Print imprime ponto
      if (i == 3)                                                         // Se for a 3a posição
        lcd.print(".");                                                   // Imprime ponto
    }
  }
  lcd.print(" Hz         ");                                              // Print
#endif
}
//----------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                                                   // Inicializa serial
#ifdef LCD_ON                                                             // Se tem LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Frequencimetro");                                            // Print
#endif
  //freqAdjust();
  pcnt_init();                                                            // Inicializa o pulse count
  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Define o port decontrole
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Define como saida

  create_args.callback = tempo_controle;                                  // Instancia o tempo de controel
  esp_timer_create(&create_args, &timer_handle);                          // Cria parametros do timer
}
//---------------------------------------------------------------------------------
void loop()
{
  if (flag == true)                                                       // Se impressa estiver liberada
  {
    flag = false;                                                         // Impede nova impresao
    float frequencia = 0;                                                 // Variavel para calculo de frequencia
    frequencia = (pulses + (multPulses * overflow))  ;                    // Calcula qtos pulsos ocorram
    inserePt((uint32_t) frequencia);
    rotinaSerial();
    rotinaLCD();
    multPulses = 0;                                                       // Zera contador de overflow
    // Aqui pode rodar qq funcao                                          // Espaco para qualquer função
    delay(50);                                                            // Espaco para qualquer função
    // Aqui pode rodar qq funcao                                          // Espaco para qualquer função

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                  // Zera o contador PCNT
    esp_timer_start_once(timer_handle, janela);                           // Disparo unico de tempo de finido na variavel Janela
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                               // Libera port de contagem
  }
}
