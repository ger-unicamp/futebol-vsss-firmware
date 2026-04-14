#include "Encoder.h"

void inicializarEncoder(Encoder *enc, uint8_t pA, uint8_t pB)
{
  enc->pinA = pA;
  enc->pinB = pB;
  enc->ticks = 0;
  enc->lastTicks = 0;
  enc->delta_ticks = 0;

  // Configura os pinos como entrada com pull-up interno
  pinMode(enc->pinA, INPUT_PULLUP);
  pinMode(enc->pinB, INPUT_PULLUP);
}

// A lógica de leitura usando 1 pino de interrupção e 1 de direção.
// Presumindo que a interrupção seja RISING no canal A.
void IRAM_ATTR logicaEncoder(Encoder *enc)
{
  // Lê o canal B para determinar a direção
  if (digitalRead(enc->pinB) == HIGH)
    enc->ticks++;
  else
    enc->ticks--;
}

// Essa função deve ser chamada em uma frequência fixa (ex: a cada 10ms ou 20ms)
void atualizarDeltaTicks(Encoder *enc)
{
  // Pausa as interrupções rapidamente para copiar a variável volatile com segurança
  // (evita que a variável mude exatamente no meio da leitura de 32 bits)
  noInterrupts();
  long currentTicks = enc->ticks;
  interrupts();

  // Calcula a diferença de passos desde a última leitura
  enc->delta_ticks = currentTicks - enc->lastTicks;

  // Atualiza a memória
  enc->lastTicks = currentTicks;
}