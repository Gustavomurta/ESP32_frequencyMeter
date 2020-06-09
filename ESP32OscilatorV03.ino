// ESP32 DEVKIT / Arduino IDE 1.8.12  / ESP32 versao 1.0.4
// Based on  https://portal.vidadesilicio.com.br/controle-de-potencia-via-pwm-esp32/
//           https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
//           https://techtutorialsx.com/2017/06/15/esp32-arduino-led-pwm-fading/
// Max Freq = 40 MHz / bit resolution   (Bit resolution = 1 to 16 bit)
// Duty cycle = decimal * 100 / (2 ^ bit resolution)
// Reference  https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-ledc.c
//            https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/ledc.html
// Exemplo: para um duty cycle de 50% com 10bits (2^10 = 1024) de resolução, devemos escrever 512.
// Gustavo Murta e Rui Viana - maio/2020

String inputString = "";          // a String to hold incoming data
int controlSeq = 0;               // whether the string is complete
bool input = false;

long  valor = 0;                  // valor digitado na console serial
long  frequencia = 100000;        // frequencia em Hz
long  canal = 1;                  // canal gerador de sinal
long  resolBits = 5;              // bits de resolucao
long  resolValor;                 // valor de resolucao em decimal
long  duty = 50;                  // Duty Cycle em porcentagem
float dutyCycle;                  // valor divisor de Duty Cycle

//----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println(); Serial.println();
  Serial.print(" Clock do ESP32 = "); Serial.print(getCpuFrequencyMhz()); Serial.println(" MHz");
  Serial.print(" Frequencia do Cristal = "); Serial.print(getXtalFrequencyMhz()); Serial.println(" MHz");
  Serial.print(" Frequencia do APB Clock = "); Serial.println(getApbFrequency());
  Serial.println();
  Serial.println("Digite a frequencia em Hz - 1 a 40.000.000:");
}
//----------------------------------------------------------------------------
void sequencia ()
{
  controlSeq ++;                               // incrementa a sequencia 
  if (controlSeq > 3) freqAdjust();            // so ajusta a frequencia apos todos parametros
  input = false;
  if (controlSeq >= 4)                         // se a sequencia esta completa 
    controlSeq = 0;                            // volta para a sequencia inicial 
}
//----------------------------------------------------------------------------
void freqAdjust ()
{
  pinMode(18, OUTPUT);                          // GPIO_18 as Output
  ledcAttachPin(18, canal);                     // GPIO_18 attached to PWM Channel
  ledcSetup(canal, frequencia, resolBits);      // Channel  , freq  ,  bit resolution
  ledcWrite(canal, dutyCycle);                  // Enable frequency with dutty cycle
}
//----------------------------------------------------------------------------
void loop()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();    // le um byte:
    inputString += inChar;                // Adicione na string:
    if (inChar == '\n')                   // Se new line:
    {
      valor = inputString.toInt();
      inputString = "";
      input = true;
    }
  }
  if (input == true)
  {
    if (controlSeq == 0)
    {
      frequencia = valor;
      Serial.print("Frequencia = "); Serial.print(frequencia); Serial.println(" Hz");  // imprime a frequencia em Hz 
      Serial.println();
      Serial.println("Digite o Canal - 0 a 15: ");
      sequencia ();                                                                    // avanca a sequencia 
      return;
    }
    if (controlSeq == 1)
    {
      canal = valor;
      Serial.print("canal = "); Serial.println(canal);                                  // imprime o canal gerador de sinal 
      Serial.println();
      Serial.println("Digite a resolucao 1 a 16 bits - 2 elevado a X. Por exemplo 2^5 = 32 ");
      sequencia ();                                                                     // avanca a sequencia
      return;
    }
    if (controlSeq == 2)
    {
      resolBits = valor;
      Serial.print("resolucao = "); Serial.print(resolBits); Serial.print(" bits");
      resolValor = pow(2, resolBits);                                                  // calcula 2 elevado a resolucao
      Serial.print(" >>>> "); Serial.println(resolValor);                              // imprime valor calculado da resolucao
      Serial.print("Frequencia maxima = "); Serial.print(80000000 / resolValor);
      Serial.println(" Hz");
      Serial.println();
      Serial.println("Digite o Duty cycle em %  - 1 a 99: ");
      sequencia ();                                                                    // avanca a sequencia
      return;
    }
    if (controlSeq == 3)
    {
      duty = valor;                                                                    // Duty Cycle em porcentagem
      dutyCycle = (resolValor * duty) / 100;                                           // valor divisor de Duty Cycle
      Serial.print("Duty cycle = "); Serial.print(duty); Serial.println(" %");         // imprime Duty Cycle
      Serial.println();
      Serial.println("Digite a frequencia em Hz - 1 a 40.000.000:");
      sequencia ();                                                                    // avanca a sequencia
      return;
    }
  }
  delay(1000);
}
