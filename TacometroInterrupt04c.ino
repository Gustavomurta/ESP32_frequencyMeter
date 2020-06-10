/* Tacomentro usando Interrupções
   Arduino Uno - IDE 1.8.9
   Gustavo Murta - 13/jun/2019
   https://github.com/simonmonk/ArduinoNextSteps/blob/master/ArduinoNextSteps/sketch_13_01_averaging/sketch_13_01_averaging.ino
*/


int SinalA = 2;                                    // Porta digital D02 - Sinal A do Encoder
int SinalB = 3;                                    // Porta digital D03 - Sinal B do Encoder
int pino7 = 7;                                     // pino de teste D7
int pino8 = 8;                                     // pino de teste D8

unsigned long pulsoNegativo = 0;                             // Periodo negativo do pulso em microsegundos
unsigned long pulsoPositivo = 0;                             // Periodo positivo do pulso em microsegundos
float pulsoPositivoAVG = 0;
float pulsoNegativoAVG = 0;
float Periodo = 0;                                   // Periodo corrigido do Pulso de passo

unsigned long currentMicros2;
unsigned long previousMicros2;
unsigned long currentMicros3;
unsigned long previousMicros3;
unsigned long currentMillis;
unsigned long previousMillis;

long PPR = 0;                                      // Numero de pulsos por volta
long Pulsos;                                       // Pulsos para o driver do motor
float PPS = 0;                                     // Pulsos por segundo
int Voltas = 0;                                    // voltas do motor
float RPS;                                         // Rotacoes por segundo
float RPM;                                         // Rotacoes por minuto
float dutyCycle = 0;

volatile int long counterA = 0;                    // contador do Encoder
volatile int long counterB = 0;                    // contador do Encoder

int bufferSize = 50;
int index2 = 0 ;
int index3 = 0;
long sum2 = 0;
long sum3 = 0;

void setup()
{
  pinMode(SinalA, INPUT);                          // Configura pino SinalA como entrada
  pinMode(SinalB, INPUT);                          // Configura pino SinalB como entrada
  pinMode(pino7, OUTPUT);                          // Configura pino D7 como saída
  pinMode(pino8, OUTPUT);                          // Configura pino D8 como saída
  digitalWrite(pino7, LOW);                        // pino D7 low
  digitalWrite(pino8, LOW);                        // pino D8 low

  Serial.begin(9600);
  attachInterruptPins ();
}

void attachInterruptPins ()
{
  attachInterrupt(digitalPinToInterrupt(2), int0, CHANGE);         // Interrupção INT0(D2) = sinal A Change
  attachInterrupt(digitalPinToInterrupt(3), int1, CHANGE);         // Interrupção INT1(D3) = sinal B Change
}

void dettachInterruptPins ()
{
  detachInterrupt(digitalPinToInterrupt(2));
  detachInterrupt(digitalPinToInterrupt(3));
}

void PulsoPino7 ()                                  // Pulso de teste pino D7
{
  digitalWrite(pino7, HIGH);                        // pino D7 High
  //delayMicroseconds(5);                           // atraso 2 microsegundos - Total 4,5 us
  digitalWrite(pino7, LOW);                         // pino D7 low
}

void PulsoPino8 ()                                  // Pulso de teste pino D8
{
  digitalWrite(pino8, HIGH);                        // pino D8 High
  //delayMicroseconds(5);                           // atraso 2 microsegundos - Total 4,5 us
  digitalWrite(pino8, LOW);                         // pino D8 low
}

void pulsoPosPrint ()
{
  if ((currentMicros2 > previousMicros2) && (digitalRead(3) == HIGH))
  {
    pulsoPositivo = (currentMicros2 - previousMicros2);
    // Serial.print ("  P+ = "); Serial.print (pulsoPositivo);
    // Serial.println();
    sum2 = sum2 + pulsoPositivo;
    index2 ++;
    // PulsoPino7 ();
  }
}

void pulsoNegPrint ()
{
  if ((currentMicros3 > previousMicros3) && (digitalRead(2) == LOW))
  {
    pulsoNegativo = (currentMicros3 - previousMicros3);
    // Serial.print ("  P- = "); Serial.print (pulsoNegativo);
    // Serial.println();
    sum3 = sum3 + pulsoNegativo;
    index3 ++;
    // PulsoPino8 ();
  }
}

void pulsoPosAVG ()
{
  if (index2 >= bufferSize)
  {
    pulsoPositivoAVG = (sum2 / index2);
    index2 = 0;
    sum2 = 0;
    PulsoPino7 ();
    Serial.print ("  P+ avg = "); Serial.print (pulsoPositivoAVG, 0); Serial.print (" us ");
  }
}

void pulsoNegAVG ()
{
  if (index3 >= bufferSize)
  {
    pulsoNegativoAVG = (sum3 / index3);
    index3 = 0;
    sum3 = 0;
    PulsoPino8 ();
    Serial.print ("  P- avg = "); Serial.print (pulsoNegativoAVG, 0); Serial.print (" us ");
    Frequencia();
  }
}

void Frequencia()                                   // calcula Pulsos, PPS e RPM
{
  //Pulsos = PPR * Voltas;                            // Quantidade total de Pulsos (PPR = pulsos por volta)
  Periodo = pulsoPositivoAVG + pulsoNegativoAVG;       // Correção do periodo do Pulso de passo
  PPS = 1000000 / (Periodo);                           // frequencia - Pulsos por segundo
  dutyCycle = (pulsoPositivoAVG / Periodo) * 100;

  Serial.print ("  Periodo = ");
  Serial.print (Periodo, 0);
  Serial.print (" ms");
  Serial.print ("  PPS = ");
  Serial.print (PPS, 1);
  Serial.print (" Hz");
  Serial.print("  Duty Cycle = ");
  Serial.print (dutyCycle, 1);
  Serial.println(" % ");
  // Serial.println();
  // RPS = PPS / PPR;                                  // rotações por segundo
  // RPM = RPS * 60;                                   // Calculo do RPM
}

void int0()                                   // Interrupção INT0(D2) = sinal A do Encoder
{
  if (digitalRead(2) == HIGH)                  // se o estado do sinal A for alto
  {
    previousMicros2 = micros();
  }
  else                                        // senão
  {
    currentMicros2 = micros();
    //PulsoPino7 ();
    //pulsoPosPrint ();
  }
}

void int1()                                   // Interrupção INT1(D3) = sinal B do Encoder
{
  if (digitalRead(3) == LOW)                  // se o estado do sinal B for baixo
  {
    previousMicros3 = micros();
  }
  else                                        // senão
  {
    currentMicros3 = micros();
    //PulsoPino8 ();
    //pulsoNegPrint ();
  }
}

void loop()
{
  pulsoPosPrint ();
  pulsoNegPrint ();
  pulsoPosAVG ();
  pulsoNegAVG ();
  bufferSize = PPS / 10;
}
